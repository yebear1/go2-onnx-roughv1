"""Verify every archived Go2 model and run hardware-free zero-input inference."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import statistics
import time

import numpy as np


MODEL_DIR = Path(__file__).resolve().parent / "models"
MODELS = {
    "walk_v1": {
        "file": "go2_blind_rough_v1.onnx",
        "bytes": 758_549,
        "sha256": "dac692d44da6ecef5aa10ab1007217ed60c150a0b27f28b6d392ad1be20c4e30",
        "input": (1, 45),
        "outputs": {"actions": (1, 12)},
    },
    "terrain": {
        "file": "go2_terrain.onnx",
        "bytes": 5_191_324,
        "sha256": "bc08253fd68f661bf6be5ce5ca70cec555c3fde2c397d23fbe9814a998e056f5",
        "input": (1, 225),
        "outputs": {"actions": (1, 12), "weights": (1, 8), "latent": (1, 32)},
    },
    "wave": {
        "file": "go2_wave.onnx",
        "bytes": 227_346,
        "sha256": "0f379a4fa44e4189281e6657894ec6525b07b15db44e1c9d832e501cc4eae22f",
        "input": (1, 56),
        "outputs": {"actions": (1, 12)},
    },
    "sit": {
        "file": "go2_sit.onnx",
        "bytes": 227_346,
        "sha256": "2614d0e07a4a6c8d68ac664738780da1c11062d4c34e41b95a54002a28afb567",
        "input": (1, 56),
        "outputs": {"actions": (1, 12)},
    },
    "forward_jump": {
        "file": "go2_forward_jump.onnx",
        "bytes": 808_493,
        "sha256": "3956b5c7202512744c9ff7feb54a37f56efbb3cdfce7f8b0abd47530e4d8e32b",
        "input": (1, 69),
        "outputs": {"actions": (1, 12)},
    },
}


def main() -> int:
    import onnxruntime as ort

    report: dict[str, object] = {"status": "PASS", "hardware_io": False, "models": {}}
    for name, expected in MODELS.items():
        path = MODEL_DIR / str(expected["file"])
        if not path.is_file():
            raise FileNotFoundError(path)
        size = path.stat().st_size
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if size != expected["bytes"] or digest != expected["sha256"]:
            raise RuntimeError(f"{name}: model integrity check failed")

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        session = ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])
        model_input = session.get_inputs()[0]
        input_shape = tuple(model_input.shape)
        outputs = {output.name: tuple(output.shape) for output in session.get_outputs()}
        if input_shape != expected["input"] or outputs != expected["outputs"]:
            raise RuntimeError(f"{name}: ONNX contract mismatch: input={input_shape}, outputs={outputs}")

        sample = np.zeros(input_shape, dtype=np.float32)
        for _ in range(10):
            session.run(None, {model_input.name: sample})
        durations: list[float] = []
        inferred = []
        for _ in range(100):
            started = time.perf_counter_ns()
            inferred = session.run(None, {model_input.name: sample})
            durations.append((time.perf_counter_ns() - started) / 1_000_000.0)
        if not all(np.isfinite(value).all() for value in inferred):
            raise RuntimeError(f"{name}: non-finite zero-input output")

        ordered = sorted(durations)
        report["models"][name] = {
            "file": path.name,
            "sha256": digest,
            "input_shape": input_shape,
            "output_shapes": outputs,
            "latency_ms": {
                "mean": statistics.fmean(durations),
                "median": statistics.median(durations),
                "p95": ordered[95],
                "maximum": max(durations),
            },
        }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
