# IGRF control

Rust desktop application for geomagnetic calculation, sensor monitoring, PID
control, and CSV logging.

## Run

Install the stable Rust toolchain, then:

```bash
cp SystemConfig.example.json SystemConfig.json
cargo run --release --package igrf-app
```

Edit `SystemConfig.json` for the serial ports, Magson address, PID values,
sensor calibration and setpoint source. The local config and generated logs are
intentionally ignored by Git.

## Units

Every field value in the app, the plots and the CSV is **nanotesla**. Raw ADC
counts become nT at `Calibration.CountToNt` (HMR2300: +-2 G over +-30000 counts
= 6.667 nT/count) and stay there through the calibration, the Kalman filter, the
PID and the log.

## Controller firmware

The STM32 controller's firmware source is not available. What the protocol is,
which pin drives which coil, and what each axis' real output ceiling is were
all recovered from a flash dump and written up in
[docs/controller-protocol.md](docs/controller-protocol.md).

Two findings shape this app:

- **The firmware does not clamp.** Past its per-axis ceiling (X 42000,
  Y 17700, Z 69000) it writes the raw value into a capture/compare register,
  which truncates on the 16-bit timers - so commanding 83940 on X produces 33%
  duty, not 100%. `build_controller_packet` clamps every command, and
  `AppConfig::sanitize` pulls configured limits in and reports when it had to.
- **The firmware has no receive timeout.** It holds the last command forever,
  which is why every path that stops the loop sends an explicit zero before
  closing the port.

`tools/probe-controller.py` drives one axis across each boundary so the
behaviour can be confirmed against a magnetometer rather than trusted from the
disassembly. Run it with `--dry-run` first; it bypasses the PID and the
app-side clamp.

## CSV columns

The first 28 columns are the C# row, unchanged, so existing analysis scripts
keep working. This build appends four:

| Column | Meaning |
| --- | --- |
| `CmdX/Y/Z` | The commanded setpoint, before the slew limiter. `SetX/Y/Z` is where the ramp has reached this tick, so the pair tells a slow ramp from a small command. |
| `TickMs` | The real PID interval for that row. A nominal 100 ms tick lands anywhere from 100 to 133 ms. |

## Sensor calibration

`Calibration` in `SystemConfig.json` holds the scale, hard-iron offset and
soft-iron matrix for the magnetometer mounted in the cage, and the app edits
them under "Sensor calibration". Re-fitting the ellipsoid no longer needs a
rebuild.

An ellipsoid fit produces a symmetric soft-iron matrix, so the app warns when
one is not: at 50000 nT on one axis, an asymmetry of 0.045 leaks 2250 nT into
another, which reads as a cage uniformity problem rather than a config typo.

## Setpoint sources

The commanded field can come from three places, chosen under "Setpoint command":

- **Manual** - a magnitude in nT applied along the declination/inclination the
  WMM2025 panel reports, or a per-axis value on each axis card.
- **CSV profile** - `time_s,bx_nt,by_nt,bz_nt` rows, interpolated between
  samples and held flat past the end. This is the seam for an external orbit
  propagator: run SGP4 and the attitude model wherever they already run and drop
  the resulting field series here. See `setpoint_profile.example.csv`.
- **UDP socket** - one datagram is one command, `bx,by,bz` in nT:

  ```bash
  echo "39858,-619,20583" | nc -u -w0 127.0.0.1 5005
  ```

  The newest datagram wins; a lost one is superseded rather than retried. If no
  datagram arrives for 10 s the field ramps back to zero, so a propagator that
  dies mid-run does not leave the coils holding its last command.

Whichever source is live, the command goes through a slew limiter
(`SetpointSlewNtPerSecond`, default 5000 nT/s) that ramps along the commanded
vector so the direction holds steady while the magnitude moves. A step straight
into the 48 V / 1500 W drivers is never issued.

## Check and build

```bash
cargo fmt --all -- --check
cargo test --workspace
cargo build --release --package igrf-app
```

The release binary is `target/release/igrf-app`.

## Automatic GitHub releases

Every push to `main` runs the checks and creates a GitHub Release containing
the Linux binary and its SHA-256 file. Tags use
`v<igrf-app-version>-main.<run-id>`; for example, `v0.1.0-main.123456789`.

Change the version in `igrf-app/Cargo.toml` when you want the next release
line, then push to `main`.

## Release versioning

- `v0.x.y`: bug fixes and safe hardware-compatible changes
- `v0.x.0`: new features or configuration fields
- `v1.0.0`: stable hardware/protocol contract

`v0.3.0` counts the packets the controller rejects and shows them on the
controller panel. Nothing about the control path changed.

`v0.2.0` adds the setpoint sources, moves sensor calibration into the config,
and clamps output to what the controller firmware acts on. Field values from
its WMM2025 calculation differ from `v0.1.0` by roughly 4 nT, because the
expansion now uses the model's own 6371.2 km reference radius; logs from the
two releases are not directly comparable.

Create a release locally after the checks pass:

```bash
git tag -a v0.1.0 -m "Release v0.1.0"
git push origin main --follow-tags
```

`origin` must be configured first if this repository is going to be hosted
remotely.
