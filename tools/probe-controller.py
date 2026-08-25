#!/usr/bin/env python3
"""Verify on real hardware what the flash dump says the controller firmware does.

The dump (see docs/controller-protocol.md) says the firmware does not clamp:
past its per-axis ceiling it writes the raw value into a capture/compare
register, which truncates on the 16-bit timers. This sends packets that
straddle each boundary and asks you to record the measured field, so the claim
is settled by measurement rather than by disassembly.

This drives the coils directly, bypassing the PID and the app's clamp. Run it
with someone watching the cage.

    ./tools/probe-controller.py --port /dev/ttyACM0 --axis x --dry-run
    ./tools/probe-controller.py --port /dev/ttyACM0 --axis x

Requires pyserial for anything but --dry-run.
"""

import argparse
import struct
import sys
import time

# Per docs/controller-protocol.md.
CEILING = {"x": 42000.0, "y": 17700.0, "z": 69000.0}
ARR = {"x": 55960, "y": 58360, "z": 97270}
BITS = {"x": 16, "y": 16, "z": 32}
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def crc16(data: bytes) -> bytes:
    """Modbus RTU CRC, high byte first, matching the firmware at 0x08001018."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc.to_bytes(2, "big")


def packet(x: float, y: float, z: float) -> bytes:
    body = b"\xa0" + struct.pack("<fff", x, y, z)
    return body + crc16(body)


def predict(axis: str, value: float) -> str:
    """What the dump says the firmware will do with this command."""
    ceiling, arr, bits = CEILING[axis], ARR[axis], BITS[axis]
    if value == 0:
        # Zero fails both signed tests and lands in the else arm, which writes
        # (uint32_t)0.0 == 0. Same result, different path.
        return "CCR      0  duty   0.0%  DIR unchanged"
    if 0 < value <= ceiling:
        return f"CCR {int(value):>6}  duty {value / arr * 100:5.1f}%  DIR high"
    if -ceiling <= value < 0:
        return f"CCR {int(-value):>6}  duty {-value / arr * 100:5.1f}%  DIR low"
    if value < 0:
        # vcvt.u32.f32 saturates a negative float to 0 on ARM.
        return "CCR      0  duty   0.0%  DIR unchanged   <-- drops out"
    written = int(value) & ((1 << bits) - 1)
    duty = min(written / arr, 1.0) * 100
    wrapped = " WRAPPED" if int(value) >= (1 << bits) else " above ARR"
    return f"CCR {written:>6}  duty {duty:5.1f}%  DIR unchanged  <--{wrapped}"


def steps(axis: str) -> list[tuple[float, str]]:
    ceiling, arr = CEILING[axis], ARR[axis]
    return [
        (0.0, "baseline, coils off"),
        (ceiling * 0.25, "quarter scale, inside the range"),
        (ceiling * 0.50, "half scale, inside the range"),
        (ceiling - 1, "one below the ceiling"),
        (ceiling + 1, "one above: does the field jump or hold?"),
        (float(arr), "at ARR: should be full duty"),
        (
            arr * 1.5,
            "past ARR: X/Y wrap and DROP here, Z holds full duty"
            if BITS[axis] == 16
            else "past ARR: 32-bit, holds full duty",
        ),
        (-(ceiling - 1), "one below the negative ceiling"),
        (-(ceiling + 1), "one past it: field should collapse to zero"),
        (0.0, "back to zero"),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="controller serial port")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--axis", choices=["x", "y", "z"], required=True)
    parser.add_argument(
        "--dwell",
        type=float,
        default=3.0,
        help="seconds to hold each step (default 3)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the plan and the predicted behaviour, send nothing",
    )
    args = parser.parse_args()

    axis = args.axis
    index = AXIS_INDEX[axis]
    plan = steps(axis)

    print(f"Axis {axis.upper()}: ceiling {CEILING[axis]:.0f}, "
          f"{BITS[axis]}-bit timer, ARR {ARR[axis]}")
    print()
    print(f"{'command':>10}  {'predicted':<52} note")
    for value, note in plan:
        print(f"{value:>10.0f}  {predict(axis, value):<52} {note}")
    print()

    if args.dry_run:
        print("Dry run: nothing sent. Drop --dry-run to drive the cage.")
        return 0

    if not args.port:
        print("--port is required unless --dry-run", file=sys.stderr)
        return 2

    try:
        import serial
    except ImportError:
        print("pyserial is required: pip install pyserial", file=sys.stderr)
        return 2

    print("This drives the coils directly, with no PID and no app-side clamp.")
    if input("Someone watching the cage? [yes/N] ").strip().lower() != "yes":
        print("Aborted.")
        return 1

    outputs = [0.0, 0.0, 0.0]
    port = serial.Serial(args.port, args.baud, timeout=0.5)
    try:
        print()
        print(f"{'command':>10}  {'reply':<10} measured |B| (record by hand)")
        for value, note in plan:
            outputs[index] = value
            port.write(packet(*outputs))
            port.flush()
            # The firmware answers "Error\r" on a CRC mismatch and is silent
            # otherwise, so anything here means the packet was rejected.
            time.sleep(0.2)
            reply = port.read(port.in_waiting or 0)
            status = reply.decode("ascii", "replace").strip() if reply else "ok"
            print(f"{value:>10.0f}  {status:<10} ______________  ({note})")
            time.sleep(max(0.0, args.dwell - 0.2))
    finally:
        # Whatever happened, leave the coils at zero: the firmware has no
        # receive timeout and holds its last command forever.
        port.write(packet(0.0, 0.0, 0.0))
        port.flush()
        time.sleep(0.2)
        port.close()
        print()
        print("Coils commanded to zero.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
