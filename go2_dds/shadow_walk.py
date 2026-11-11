"""Run the walking policy from real LowState samples without publishing commands."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time

import numpy as np

from policy_runtime.selftest import DEFAULT_MODEL
from policy_runtime.walk_policy import WalkPolicyRunner
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


class LatestState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.sequence = 0
        self.message: LowState_ | None = None

    def receive(self, message: LowState_) -> None:
        with self.lock:
            self.sequence += 1
            self.message = message

    def read(self) -> tuple[int, LowState_ | None]:
        with self.lock:
            return self.sequence, self.message


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--topic", default="rt/lowstate", choices=("rt/lowstate", "rt/lf/lowstate"))
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    output = args.output or Path.home() / (
        "go2-shadow-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".jsonl"
    )
    output = output.expanduser().resolve()
    runner = WalkPolicyRunner(args.model)
    latest = LatestState()

    ChannelFactoryInitialize(args.domain, args.interface)
    subscriber = ChannelSubscriber(args.topic, LowState_)
    subscriber.Init(latest.receive, 10)

    period = runner.config.control_dt
    previous = np.zeros(12, dtype=np.float32)
    last_sequence = 0
    frames = 0
    stale_cycles = 0
    started = time.monotonic()
    deadline = started
    with output.open("w", encoding="utf-8") as stream:
        stream.write(
            json.dumps(
                {
                    "event": "start",
                    "mode": "shadow_only",
                    "publisher_created": False,
                    "topic": args.topic,
                    "interface": args.interface,
                    "model_sha256": runner.config.model_sha256,
                }
            )
            + "\n"
        )
        try:
            while time.monotonic() - started < args.seconds:
                deadline += period
                sequence, message = latest.read()
                if message is None or sequence == last_sequence:
                    stale_cycles += 1
                else:
                    last_sequence = sequence
                    motors = list(message.motor_state[:12])
                    observation = runner.assemble_observation(
                        angular_velocity=message.imu_state.gyroscope,
                        quaternion_wxyz=message.imu_state.quaternion,
                        command_xyz=(args.vx, args.vy, args.yaw),
                        joint_position_hardware=[motor.q for motor in motors],
                        joint_velocity_hardware=[motor.dq for motor in motors],
                        previous_action_policy=previous,
                    )
                    previous = runner.infer(observation)
                    targets = runner.action_to_hardware_targets(previous)
                    stream.write(
                        json.dumps(
                            {
                                "event": "frame",
                                "monotonic_seconds": time.monotonic(),
                                "lowstate_tick": int(message.tick),
                                "sequence": sequence,
                                "action_policy": previous.tolist(),
                                "target_hardware": targets.tolist(),
                            },
                            separators=(",", ":"),
                        )
                        + "\n"
                    )
                    frames += 1
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            subscriber.Close()
            stream.write(
                json.dumps(
                    {
                        "event": "stop",
                        "mode": "shadow_only",
                        "publisher_created": False,
                        "frames": frames,
                        "stale_cycles": stale_cycles,
                    }
                )
                + "\n"
            )
    print(json.dumps({"status": "PASS" if frames else "NO_DATA", "frames": frames, "output": str(output)}))
    return 0 if frames else 2


if __name__ == "__main__":
    raise SystemExit(main())
