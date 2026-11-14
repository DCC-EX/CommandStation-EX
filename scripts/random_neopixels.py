#!/usr/bin/env python3
"""Send randomized NeoPixel color and on/off commands to a serial port."""

import argparse
import math
import random
import time


VPINS = tuple(range(1000, 1256)) + tuple(range(2000, 2005))


def positive_integer(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("value must be at least 1")
    return number


def nonnegative_delay(value):
    delay = float(value)
    if not math.isfinite(delay) or delay < 0:
        raise argparse.ArgumentTypeError("delay must be a finite nonnegative number")
    return delay


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Randomly set color, brightness, and on/off state on the configured "
            "VPIN 1000 and 2000 strips."
        )
    )
    parser.add_argument("port", help="serial port, for example /dev/ttyUSB0 or COM3")
    parser.add_argument(
        "--count", type=positive_integer, default=500, help="number of commands (default: 500)"
    )
    parser.add_argument(
        "--baud", type=positive_integer, default=115200, help="serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--min-delay",
        type=nonnegative_delay,
        default=0.02,
        help="minimum delay in seconds (default: 0.02)",
    )
    parser.add_argument(
        "--max-delay",
        type=nonnegative_delay,
        default=0.15,
        help="maximum delay in seconds (default: 0.15)",
    )
    parser.add_argument(
        "--startup-delay",
        type=nonnegative_delay,
        default=10.0,
        help="seconds to wait after opening the port (default: 10.0)",
    )
    args = parser.parse_args()

    if args.max_delay < args.min_delay:
        parser.error("--max-delay must be greater than or equal to --min-delay")

    try:
        import serial
    except ImportError:
        parser.error("pyserial is required; install it with: python3 -m pip install pyserial")

    rng = random.Random()
    with serial.Serial(args.port, args.baud, timeout=1) as connection:
        print(f"Waiting for start {args.startup_delay:g} seconds...", flush=True)
        time.sleep(args.startup_delay)
        for index in range(args.count):
            vpin = rng.choice(VPINS)
            signed_vpin = vpin if rng.choice((True, False)) else -vpin
            red, green, blue, brightness = (rng.randint(0, 255) for _ in range(4))
            led_count = rng.randint(1, 5)
            command = f"<o {signed_vpin} {red} {green} {blue} {led_count} {brightness}>"
            connection.write((command + "\n").encode("ascii"))
            connection.flush()
            print(command, flush=True)
            if index + 1 < args.count:
                time.sleep(rng.uniform(args.min_delay, args.max_delay))


if __name__ == "__main__":
    main()
