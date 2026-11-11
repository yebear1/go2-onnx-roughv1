"""Subscribe to candidate Go2 state topics without creating any publisher."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_


CANDIDATE_TOPICS = (
    ("rt/lowstate", LowState_),
    ("rt/lf/lowstate", LowState_),
    ("rt/sportmodestate", SportModeState_),
    ("rt/lf/sportmodestate", SportModeState_),
)


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class Probe:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.counts: Counter[str] = Counter()
        self.first_at: dict[str, float] = {}
        self.last_at: dict[str, float] = {}
        self.snapshots: dict[str, dict[str, object]] = {}

    def callback(self, topic: str):
        def receive(message) -> None:
            now = time.monotonic()
            if isinstance(message, LowState_):
                motors = list(message.motor_state[:12])
                snapshot = {
                    "kind": "lowstate",
                    "tick": int(message.tick),
                    "level_flag": int(message.level_flag),
                    "crc": int(message.crc),
                    "imu_quaternion": [float(value) for value in message.imu_state.quaternion],
                    "imu_gyroscope": [float(value) for value in message.imu_state.gyroscope],
                    "motor_q": [float(motor.q) for motor in motors],
                    "motor_dq": [float(motor.dq) for motor in motors],
                    "finite": _finite(message.imu_state.quaternion)
                    and _finite(message.imu_state.gyroscope)
                    and _finite([motor.q for motor in motors])
                    and _finite([motor.dq for motor in motors]),
                }
            else:
                snapshot = {
                    "kind": "sportmodestate",
                    "mode": int(message.mode),
                    "gait_type": int(message.gait_type),
                    "error_code": int(message.error_code),
                    "velocity": [float(value) for value in message.velocity],
                    "yaw_speed": float(message.yaw_speed),
                    "finite": _finite(message.velocity) and math.isfinite(float(message.yaw_speed)),
                }
            with self.lock:
                self.counts[topic] += 1
                self.first_at.setdefault(topic, now)
                self.last_at[topic] = now
                self.snapshots[topic] = snapshot

        return receive

    def report(self, interface: str, duration: float) -> dict[str, object]:
        with self.lock:
            topics = {}
            for topic, _ in CANDIDATE_TOPICS:
                count = self.counts[topic]
                elapsed = max(0.0, self.last_at.get(topic, 0.0) - self.first_at.get(topic, 0.0))
                topics[topic] = {
                    "messages": count,
                    "observed_hz": (count - 1) / elapsed if count > 1 and elapsed > 0 else 0.0,
                    "snapshot": self.snapshots.get(topic),
                }
        return {
            "mode": "read_only",
            "publisher_created": False,
            "interface": interface,
            "duration_seconds": duration,
            "topics": topics,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=15.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    ChannelFactoryInitialize(args.domain, args.interface)
    probe = Probe()
    subscribers = []
    for topic, data_type in CANDIDATE_TOPICS:
        subscriber = ChannelSubscriber(topic, data_type)
        subscriber.Init(probe.callback(topic), 10)
        subscribers.append(subscriber)
    try:
        time.sleep(args.seconds)
    finally:
        for subscriber in subscribers:
            subscriber.Close()

    report = probe.report(args.interface, args.seconds)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if any(item["messages"] for item in report["topics"].values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
