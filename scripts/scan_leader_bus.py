#!/usr/bin/env python3
"""Read-only scan of a Feetech motor bus: lists whatever IDs actually
respond, at every supported baud rate. No writes to hardware -- safe to
run before calibration to diagnose "motor check failed" / empty motor list
errors (power not connected, daisy-chain not seated, wrong port, etc).

Usage:
    ..\.venv\Scripts\python.exe scripts\scan_leader_bus.py --port COM10
"""

import argparse

from lerobot.motors.feetech import FeetechMotorsBus


def main():
    ap = argparse.ArgumentParser(description="Read-only Feetech bus scan.")
    ap.add_argument("--port", required=True, help="e.g. COM10")
    args = ap.parse_args()

    print(f"Scanning {args.port} at every supported baud rate...")
    print("(no writes to hardware -- this only pings)\n")

    found = FeetechMotorsBus.scan_port(args.port)

    if not found:
        print("No motors responded at ANY baud rate.")
        print("Check: arm power supply connected + switched on, data cable")
        print("seated into the first servo, correct port.")
    else:
        print("\nSummary:")
        for baudrate, ids in found.items():
            print(f"  baudrate={baudrate}: IDs found = {sorted(ids)}")


if __name__ == "__main__":
    main()
