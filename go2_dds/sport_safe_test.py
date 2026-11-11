"""Guarded high-level SportClient test; never imports or publishes LowCmd."""

from __future__ import annotations

import argparse
import json
import math
import time


CONFIRMATION = "PHYSICAL_REMOTE_READY"


def validate_request(enable_send: bool, confirmation: str, speed: float, duration: float) -> None:
    if not enable_send or confirmation != CONFIRMATION:
        raise ValueError("sending is locked; use both --enable-send and the exact confirmation phrase")
    if not math.isfinite(speed) or abs(speed) > 0.10:
        raise ValueError("speed must be finite and within ±0.10 m/s")
    if not math.isfinite(duration) or duration <= 0 or duration > 0.50:
        raise ValueError("duration must be within (0, 0.50] seconds")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=0.25)
    parser.add_argument("--enable-send", action="store_true")
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    try:
        validate_request(args.enable_send, args.confirm, args.speed, args.duration)
    except ValueError as error:
        print(json.dumps({"status": "LOCKED", "reason": str(error)}))
        return 2

    # Import and initialize the RPC client only after both safety gates pass.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    ChannelFactoryInitialize(args.domain, args.interface)
    client = SportClient()
    client.SetTimeout(2.0)
    client.Init()
    try:
        result = client.Move(args.speed, 0.0, 0.0)
        time.sleep(args.duration)
    finally:
        stop_result = client.StopMove()
    print(json.dumps({"status": "SENT_AND_STOPPED", "move_result": result, "stop_result": stop_result}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
