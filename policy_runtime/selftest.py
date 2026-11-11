"""Offline model validation and Raspberry Pi latency benchmark."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import time

import numpy as np

from .walk_policy import WalkPolicyConfig, WalkPolicyRunner


DEFAULT_MODEL = Path(__file__).resolve().parent / "models" / "go2_blind_rough_v1.onnx"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument("--warmup", type=int, default=50)
    args = parser.parse_args()
    if args.iterations < 1 or args.warmup < 0:
        parser.error("iterations must be positive and warmup must not be negative")

    runner = WalkPolicyRunner(args.model)
    cfg = WalkPolicyConfig()
    defaults = np.asarray(cfg.default_position_hardware, dtype=np.float32)
    zero = np.zeros(12, dtype=np.float32)

    neutral = runner.assemble_observation(
        angular_velocity=(0.0, 0.0, 0.0),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        command_xyz=(0.0, 0.0, 0.0),
        joint_position_hardware=defaults,
        joint_velocity_hardware=zero,
        previous_action_policy=zero,
    )
    if not np.allclose(neutral[9:21], 0.0):
        raise RuntimeError("neutral joint-position observation is not zero")

    previous = zero
    for _ in range(args.warmup):
        warmup_observation = runner.assemble_observation(
            angular_velocity=(0.01, -0.02, 0.03),
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            command_xyz=(0.2, 0.0, 0.1),
            joint_position_hardware=defaults,
            joint_velocity_hardware=zero,
            previous_action_policy=previous,
        )
        previous = runner.infer(warmup_observation)

    durations_ms: list[float] = []
    final_action = previous
    for index in range(args.iterations):
        phase = index * 0.013
        disturbance = np.sin(np.arange(12, dtype=np.float32) + phase) * 0.01
        velocity = np.cos(np.arange(12, dtype=np.float32) + phase) * 0.05
        observation = runner.assemble_observation(
            angular_velocity=(0.01, -0.02, 0.03),
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            command_xyz=(0.2, 0.0, 0.1),
            joint_position_hardware=defaults + disturbance,
            joint_velocity_hardware=velocity,
            previous_action_policy=final_action,
        )
        started = time.perf_counter_ns()
        final_action = runner.infer(observation)
        durations_ms.append((time.perf_counter_ns() - started) / 1_000_000.0)

    targets = runner.action_to_hardware_targets(final_action)
    ordered = sorted(durations_ms)
    p95_index = min(len(ordered) - 1, int(0.95 * len(ordered)))
    report = {
        "status": "PASS",
        "hardware_io": False,
        "model": str(args.model.resolve()),
        "model_sha256": cfg.model_sha256,
        "input_name": runner.input_name,
        "input_shape": runner.input_shape,
        "output_name": runner.output_name,
        "output_shape": runner.output_shape,
        "iterations": args.iterations,
        "latency_ms": {
            "mean": statistics.fmean(durations_ms),
            "median": statistics.median(durations_ms),
            "p95": ordered[p95_index],
            "maximum": max(durations_ms),
        },
        "action": runner.finite_summary(final_action),
        "target": runner.finite_summary(targets),
        "control_period_ms": cfg.control_dt * 1000.0,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
