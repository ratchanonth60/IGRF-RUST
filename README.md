# IGRF control

Rust desktop application for geomagnetic calculation, sensor monitoring, PID
control, and CSV logging.

## Run

Install the stable Rust toolchain, then:

```bash
cp SystemConfig.example.json SystemConfig.json
cargo run --release --package igrf-app
```

Edit `SystemConfig.json` for the serial ports, Magson address, and PID values.
The local config and generated logs are intentionally ignored by Git.

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

The first release is `v0.1.0`.

- `v0.1.x`: bug fixes and safe hardware-compatible changes
- `v0.x.0`: new features or configuration fields
- `v1.0.0`: stable hardware/protocol contract

Create a release locally after the checks pass:

```bash
git tag -a v0.1.0 -m "Release v0.1.0"
git push origin master --follow-tags
```

`origin` must be configured first if this repository is going to be hosted
remotely.
