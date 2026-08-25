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

There is a second mode, --measure-gain, which reads the HMR2300 itself and
fits how many nT of field one output count buys. Nothing in the app knows that
number today, which is why the entire operating point has to be built by the
integrator instead of commanded directly:

    ./tools/probe-controller.py --port /dev/ttyACM0 --sensor-port /dev/ttyUSB0 \
        --axis x --measure-gain

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

# HMR2300: +-2 G over +-30000 counts.
COUNT_TO_NT = 20.0 / 3.0
SENSOR_HANDSHAKE = b"*00WE\r"
SENSOR_PACKET_SIZE = 7


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


def read_sensor(port, samples: int, timeout: float = 5.0) -> tuple[float, float, float]:
    """Mean of `samples` HMR2300 readings, in nT, as raw scaled counts.

    Hard- and soft-iron terms are deliberately not applied: the gain is a
    slope, and a constant offset cancels out of one. The soft-iron scale does
    not, but it is within a percent of unity and the number this produces is a
    starting point for a bench measurement, not a calibration.
    """
    buffer = b""
    readings = []
    deadline = time.time() + timeout
    while len(readings) < samples and time.time() < deadline:
        buffer += port.read(port.in_waiting or 1)
        while len(buffer) >= SENSOR_PACKET_SIZE:
            if buffer[SENSOR_PACKET_SIZE - 1] != 0x0D:
                buffer = buffer[1:]
                continue
            frame = buffer[:SENSOR_PACKET_SIZE]
            buffer = buffer[SENSOR_PACKET_SIZE:]
            readings.append(
                tuple(
                    int.from_bytes(frame[i : i + 2], "big", signed=True) * COUNT_TO_NT
                    for i in (0, 2, 4)
                )
            )
    if not readings:
        raise TimeoutError("no HMR2300 packets; is the sensor port right?")
    return tuple(sum(axis) / len(readings) for axis in zip(*readings))


def fit_line(x: list[float], y: list[float]) -> tuple[float, float]:
    """Least-squares slope and intercept. Slope is nT per output count."""
    n = len(x)
    mean_x = sum(x) / n
    mean_y = sum(y) / n
    denominator = sum((value - mean_x) ** 2 for value in x)
    if denominator == 0:
        return 0.0, mean_y
    slope = sum((a - mean_x) * (b - mean_y) for a, b in zip(x, y)) / denominator
    return slope, mean_y - slope * mean_x


def gain_steps(axis: str) -> list[float]:
    """Commands for the gain fit: symmetric, and well inside the ceiling.

    Nothing here goes near the range check - this measures the linear region,
    not the boundary the main mode probes. The zero at each end is a baseline,
    and repeating it catches drift over the run.
    """
    ceiling = CEILING[axis]
    return [ceiling * f for f in (0.0, 0.25, 0.5, -0.25, -0.5, 0.0)]


def measure_gain(args) -> int:
    """Fit nT of field per output count, plus the ambient field.

    The app has no such number, which is why every standing current has to be
    built by the integrator: at Ki = 0.068 an output of -7900 is an integral of
    about -116000, so the whole operating point lives in the integrator and any
    reset drops the field. With a gain the command can be issued directly and
    the loop left to correct the remainder.

    The intercept is the ambient field at the sensor, which feedforward needs
    as well: the coils have to produce (setpoint - ambient), not the setpoint.
    """
    import serial

    axis = args.axis
    index = AXIS_INDEX[axis]
    plan = gain_steps(axis)

    print(f"Axis {axis.upper()}: fitting nT per output count over "
          f"{min(plan):.0f}..{max(plan):.0f}, ceiling {CEILING[axis]:.0f}.")
    print("This drives the coils directly, with no PID and no app-side clamp.")
    if input("Someone watching the cage? [yes/N] ").strip().lower() != "yes":
        print("Aborted.")
        return 1

    controller = serial.Serial(args.port, args.baud, timeout=0.5)
    sensor = serial.Serial(args.sensor_port, args.sensor_baud, timeout=0.5)
    outputs = [0.0, 0.0, 0.0]
    commands: list[float] = []
    fields: list[tuple[float, float, float]] = []
    try:
        sensor.write(SENSOR_HANDSHAKE)
        sensor.flush()
        time.sleep(0.5)
        sensor.reset_input_buffer()

        print()
        print(f"{'command':>10}  {'Bx':>10} {'By':>10} {'Bz':>10}  (nT)")
        for value in plan:
            outputs[index] = value
            controller.write(packet(*outputs))
            controller.flush()
            time.sleep(args.dwell)
            sensor.reset_input_buffer()
            reading = read_sensor(sensor, args.samples)
            commands.append(value)
            fields.append(reading)
            print(f"{value:>10.0f}  {reading[0]:>10.1f} {reading[1]:>10.1f} "
                  f"{reading[2]:>10.1f}")
    finally:
        # The firmware has no receive timeout and holds its last command
        # forever, so the coils are left at zero whatever happened above.
        controller.write(packet(0.0, 0.0, 0.0))
        controller.flush()
        time.sleep(0.2)
        controller.close()
        sensor.close()

    print()
    print(f"{'sensor axis':>12}  {'nT / count':>12}  {'ambient nT':>12}  note")
    for sensor_axis in range(3):
        slope, intercept = fit_line(commands, [field[sensor_axis] for field in fields])
        note = "driven axis" if sensor_axis == index else "cross-coupling"
        print(f"{'XYZ'[sensor_axis]:>12}  {slope:>12.5f}  {intercept:>12.1f}  {note}")

    slope, _ = fit_line(commands, [field[index] for field in fields])
    if slope != 0:
        print()
        print(f"Full scale on this axis: {CEILING[axis] * slope:,.0f} nT at the "
              f"firmware ceiling of {CEILING[axis]:.0f}.")
        print(f"To command 50000 nT: {50000 / slope:,.0f} counts.")
    print()
    print("Cross-coupling slopes are one column of the coil-to-field matrix.")
    print("Run all three axes to get the whole thing.")
    return 0


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
    parser.add_argument(
        "--measure-gain",
        action="store_true",
        help="fit nT of field per output count, reading the HMR2300 directly",
    )
    parser.add_argument("--sensor-port", help="HMR2300 serial port, for --measure-gain")
    parser.add_argument("--sensor-baud", type=int, default=9600)
    parser.add_argument(
        "--samples",
        type=int,
        default=20,
        help="sensor readings to average per step (default 20)",
    )
    args = parser.parse_args()

    if args.measure_gain:
        if not args.port or not args.sensor_port:
            print("--measure-gain needs both --port and --sensor-port", file=sys.stderr)
            return 2
        try:
            import serial  # noqa: F401
        except ImportError:
            print("pyserial is required: pip install pyserial", file=sys.stderr)
            return 2
        return measure_gain(args)

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
