# Controller firmware protocol

Everything here was recovered by disassembling a flash dump of the STM32F439
controller (`test.bin`, 2 MiB image, 58092 bytes of content,
`sha256:09f992b1b9cf3f289523ecea434ddc676761b4b30b7465343f688523895a77a9` over
the content). **No source for this firmware is available.** Anything the dump
did not answer is marked "unknown" rather than guessed.

Written down because the pinout, the timer setup and the per-axis output
ceilings existed nowhere else, and the app has to respect all three.

## Identification

| | |
| --- | --- |
| Core | ARM Cortex-M4F (hardware float, `vcvt`/`vmul.f32` throughout) |
| Initial SP | `0x20030000` |
| Reset vector | `0x080023d0` |
| `main()` | `0x08001154` |
| Framework | STM32 HAL (`HAL_TIM_*`, `HAL_GPIO_WritePin`, `HAL_RCC_OscConfig`) |
| `.data` init image | flash `0x0800df64` → RAM `0x20000000`..`0x20000388` |
| `.bss` | `0x20000388`..`0x20002004` |

## Wire format

15 bytes, host → controller, one packet per command. No response except on
error (below).

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 1 | Header. The app sends `0xA0`; see "header is not checked". |
| 1 | 4 | X output, `f32` little-endian |
| 5 | 4 | Y output, `f32` little-endian |
| 9 | 4 | Z output, `f32` little-endian |
| 13 | 1 | CRC high byte |
| 14 | 1 | CRC low byte |

CRC is Modbus RTU (poly `0xA001`, init `0xFFFF`) over bytes 0..12, transmitted
high byte first. Confirmed against the firmware's own implementation at
`0x08001018`: `eor #0xa000` followed by `eor #1` is `0xA001`, and the tail does
`lsr #8` into byte 13.

`igrf-core/src/packet.rs` matches this exactly.

### Header is not checked

At `0x080011da` the firmware loads the receive pointer, stores it to
`0x200003e0`, then compares `*rx_ptr` against `*rx_ptr` - the same byte through
two pointers. The comparison is always true, so **byte 0 is not validated**.
The CRC is the only integrity check on the link.

### Error response

A CRC mismatch (`0x0800197c`) or the byte-0 comparison failing (`0x08001972`)
sends the 5-byte string at `0x20000000`, `"Error\r"`, back to the host and
restarts the receive loop. The app currently never reads the return path, so
rejected packets are invisible; reading it would give a live error rate.

## Receive loop

```text
0x08001190  while (rx_count <= 14) ;      // busy-wait, no timeout
            rx_count = 0
            crc16(rx_buf, 13, &computed)
            if (rx_buf[13] != computed[0]) goto error
            if (rx_buf[14] != computed[1]) goto error
            ... decode and drive ...
            goto 0x08001190
```

**There is no receive timeout.** If the host stops sending, the loop spins and
every CCR keeps its last value indefinitely. The coils stay energised at
whatever was last commanded. This is why the app has to send an explicit zero
before disconnecting rather than just closing the port.

## Per-axis drive

Each axis float is multiplied by a scale factor, then range-checked, then
written to a capture/compare register with a direction GPIO.

| Axis | Packet offset | Scale factor | Scaled var | Timer | ARR | Ceiling | Ceiling / ARR |
| --- | --- | --- | --- | --- | --- | --- | --- |
| X ch1 | +1 | `0x20000008` | `0x200003b0` | TIM1 CCR1 | 55960 | 42000 | 75.1% |
| X ch2 | +1 | `0x2000000c` | `0x200003b4` | TIM1 CCR2 | 55960 | 42000 | 75.1% |
| Y ch1 | +5 | `0x20000010` | `0x200003b8` | TIM3 CCR1 | 58360 | 17700 | 30.3% |
| Y ch2 | +5 | `0x20000014` | `0x200003bc` | TIM3 CCR2 | 58360 | 17700 | 30.3% |
| Z ch1 | +9 | `0x20000018` | `0x200003c0` | TIM2 CCR1 | 97270 | 69000 | 70.9% |
| Z ch2 | +9 | `0x2000001c` | `0x200003c4` | TIM2 CCR2 | 97270 | 69000 | 70.9% |

Direction pins, from the `HAL_GPIO_WritePin` calls:

| Channel | Pin | High when |
| --- | --- | --- |
| X ch1 | PF14 | value > 0 |
| X ch2 | PF3 | value > 0 |
| Y ch1 | PB9 | value > 0 |
| Y ch2 | PF12 | value > 0 |
| Z ch1 | PD11 | value > 0 |
| Z ch2 | PB12 | value > 0 |

**TIM1 and TIM3 are 16-bit. TIM2 is 32-bit.** That distinction matters below.

### Scale factors are all 1.0

The six scale factors ship as `1.0f` in the `.data` init image
(`0x0800df6c`..`0x0800df80`). The per-axis calibration hook exists but has
never been used. Correcting coil gain differences or cross-axis coupling in
firmware would go here.

### The range check does not clamp

All six channels have the same three-arm structure:

```c
v = packet_float * scale;
if (v > 0 && v <=  CEILING) { CCR = (uint32_t)v;  DIR = 1; }
else if (v < 0 && v >= -CEILING) { CCR = (uint32_t)-v; DIR = 0; }
else                             { CCR = (uint32_t)v; }   // no abs, no clamp, DIR untouched
```

The `else` arm is reached by anything outside `[-CEILING, CEILING]`, and it
writes the raw value. Consequences, per axis:

- **X and Y (16-bit timers)** truncate. Commanding 83940 on X writes CCR 18404:
  33% duty where 100% was asked for. Commanding 200000 gives 6%. The drive goes
  *down* as the command goes up.
- **A negative value past the ceiling** hits `vcvt.u32.f32` on a negative
  float, which saturates to 0 on ARM. The coil drops to zero output while the
  direction pin holds its previous state.
- **Z (32-bit timer)** does not wrap, but any value above ARR 97270 is simply
  100% duty.

Nothing is reported back in any of these cases.

Because of this the app clamps in `build_controller_packet`
(`igrf-core/src/packet.rs`), which every path to the coils goes through, and
`AppConfig::sanitize` pulls configured `MaxOutput`/`MinOutput` inside the
ceilings and reports when it had to.

## Known firmware defect: X ch1 negative branch reads the wrong variable

At `0x08001264`, the X ch1 negative-range check tests `0x200003b0` (X ch1) for
sign, then compares **`0x200003b8` (Y ch1)** against X's `-42000.0` limit:

```text
08001264  ldr   r3, =0x200003b0   ; X ch1
08001266  vldr  s15, [r3]
0800126a  vcmpe.f32 s15, #0       ; is X ch1 negative?
08001272  bpl   0x80012d4
08001274  ldr   r3, =0x200003b8   ; <-- Y ch1, should be X ch1
08001276  vldr  s15, [r3]
0800127a  vldr  s14, =-42000.0
0800127e  vcmpe.f32 s15, s14      ; compares Y ch1 against X's limit
08001282  blt   0x80012d4
```

Every other channel loads the same variable in both halves of the check - X ch2
at `0x0800136e` reads `0x200003b4` twice, and so on. Counting references into
the literal pool makes it unambiguous: each axis variable is referenced 7
times, while the slot holding `0x200003b8` at `0x08001404` is referenced
**exactly once**, from this instruction. That is a source-level typo the
compiler faithfully reproduced, not a disassembly artefact.

Effect: X's negative direction is gated on Y's value. A negative X command
passes or fails the range check depending on where Y happens to be. With the
app clamping to ±42000 the `else` arm is unreachable from this build, so the
practical impact is bounded - but the firmware is wrong and should be fixed at
the source.

**A binary patch is possible** - the literal at `0x08001404` would change from
`B8 03 00 20` to `B0 03 00 20`, four bytes, one reference, no other effect.
This has *not* been done: it needs the flash checksum situation confirmed
first, and it would produce a binary matching no source anywhere. Recovering
the source is the better path.

## Clock

`HAL_RCC_OscConfig` at `0x08001a2c` is called with:

| Field | Value |
| --- | --- |
| `OscillatorType` | 1 (HSE) |
| `HSEState` | `0x10000` (BYPASS - external oscillator, not a crystal) |
| `PLL.PLLState` | 2 (ON) |
| `PLL.PLLSource` | `0x400000` (HSE) |
| `PLLM` | 4 |
| `PLLN` | 168 |
| `PLLP` | 2 |
| `PLLQ` | 7 |

`SYSCLK = HSE / PLLM * PLLN / PLLP = HSE * 21`.

**HSE frequency is unknown** - it is a board property, not in the dump. If HSE
is 8 MHz this gives the expected 168 MHz. Note `PLLM = 4` puts the PLL input at
HSE/4, which must land in 1-2 MHz; 8 MHz HSE gives exactly 2 MHz, at the top of
the allowed range.

`SystemCoreClock` initialises to 16000000 (`0x20000020`), the HSI default,
before `SystemClock_Config` runs.

PWM frequency cannot be derived without the HSE frequency and the APB
prescalers. Unknown.

## Open questions

- HSE frequency, to pin down SYSCLK and the PWM carrier frequency.
- Whether Y's 30% ceiling is deliberate (the Y coil pair is the smallest, at
  `HALF_SIDE` 0.72) or a leftover.
- Whether the board or bootloader checks a flash checksum, which decides
  whether a binary patch is viable.
- Where the firmware source is.
