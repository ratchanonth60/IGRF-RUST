# Local compile check only. The binary this produces links GLIBC_2.39
# (cross-rs's base is Ubuntu 24.04) and will NOT run on Raspberry Pi OS
# Bookworm's 2.36 - "GLIBC_2.39 not found". Matching the Pi's glibc here
# isn't practical: the image's own arm64 libc6 is already 2.39 and apt
# refuses to downgrade it, and an older base can't run the host rustc that
# `cross` mounts in (needs a newer glibc itself). The Pi-runnable binary
# comes from the `ubuntu-22.04-arm` job in .github/workflows/release.yml,
# which compiles natively - no cross toolchain, no glibc mismatch.
#
# This image just proves the code links against real X11/Wayland/GL/udev
# for aarch64, faster than waiting on CI.
FROM ghcr.io/cross-rs/aarch64-unknown-linux-gnu:main

RUN dpkg --add-architecture arm64 && \
    apt-get update && \
    apt-get install --assume-yes \
        libudev-dev:arm64 \
        libx11-dev:arm64 \
        libwayland-dev:arm64 \
        libxkbcommon-dev:arm64 \
        libgl1-mesa-dev:arm64 \
        pkg-config
