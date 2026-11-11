"""Go2 flat-walking policy contract without any robot I/O.

This module intentionally contains no DDS, ROS, socket, or Unitree imports.  It
only converts already-sampled robot state into the ONNX observation layout and
maps policy actions back to Go2 hardware joint order.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class WalkPolicyConfig:
    """Pinned contract from spacemit-robotics/unitree-go2 commit ec9a4fe1059b."""

    model_bytes: int = 758_549
    model_sha256: str = "dac692d44da6ecef5aa10ab1007217ed60c150a0b27f28b6d392ad1be20c4e30"
    observation_dim: int = 45
    action_dim: int = 12
    control_dt: float = 0.02
    action_scale: float = 0.25
    angular_velocity_scale: float = 0.2
    joint_position_scale: float = 1.0
    joint_velocity_scale: float = 0.05
    command_limits: tuple[float, ...] = (-1.0, 1.0, -0.4, 0.4, -1.0, 1.0)
    # Policy action index -> Go2 hardware index. Hardware order is
    # FR, FL, RR, RL, with hip/thigh/calf inside each leg.
    policy_to_hardware: tuple[int, ...] = (3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8)
    default_position_hardware: tuple[float, ...] = (
        -0.1,
        0.8,
        -1.5,
        0.1,
        0.8,
        -1.5,
        -0.1,
        1.0,
        -1.5,
        0.1,
        1.0,
        -1.5,
    )
    kp_policy: tuple[float, ...] = (25.0,) * 12
    kd_policy: tuple[float, ...] = (0.5,) * 12


def _finite_vector(name: str, values: Sequence[float], size: int) -> np.ndarray:
    result = np.asarray(values, dtype=np.float32)
    if result.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {result.shape}")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} contains NaN or infinity")
    return result


def projected_gravity(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    """Return world-down gravity projected into the robot body frame."""

    q = _finite_vector("quaternion_wxyz", quaternion_wxyz, 4).astype(np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-6:
        raise ValueError("quaternion norm is zero")
    w, x, y, z = q / norm
    return np.asarray(
        [2.0 * (w * y - x * z), -2.0 * (y * z + w * x), -(1.0 - 2.0 * (x * x + y * y))],
        dtype=np.float32,
    )


class WalkPolicyRunner:
    """Validated CPU ONNX runner for the pinned flat-walking policy."""

    def __init__(self, model_path: str | Path, config: WalkPolicyConfig | None = None):
        self.config = config or WalkPolicyConfig()
        self.model_path = Path(model_path).expanduser().resolve()
        self._verify_model_file()

        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError("onnxruntime is required for policy inference") from error

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.session = ort.InferenceSession(
            str(self.model_path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or len(outputs) < 1:
            raise RuntimeError(f"unexpected ONNX I/O count: {len(inputs)} input(s), {len(outputs)} output(s)")
        self.input_name = inputs[0].name
        self.output_name = outputs[0].name
        self.input_shape = tuple(inputs[0].shape)
        self.output_shape = tuple(outputs[0].shape)
        if not self._shape_has_width(self.input_shape, self.config.observation_dim):
            raise RuntimeError(f"model input is not {self.config.observation_dim} values: {self.input_shape}")
        if not self._shape_has_width(self.output_shape, self.config.action_dim):
            raise RuntimeError(f"model output is not {self.config.action_dim} values: {self.output_shape}")

    @staticmethod
    def _shape_has_width(shape: tuple[object, ...], width: int) -> bool:
        return bool(shape) and shape[-1] in (width, str(width))

    def _verify_model_file(self) -> None:
        if not self.model_path.is_file():
            raise FileNotFoundError(self.model_path)
        size = self.model_path.stat().st_size
        if size != self.config.model_bytes:
            raise RuntimeError(f"model size mismatch: expected {self.config.model_bytes}, got {size}")
        digest = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
        if digest != self.config.model_sha256:
            raise RuntimeError(f"model SHA-256 mismatch: expected {self.config.model_sha256}, got {digest}")

    def assemble_observation(
        self,
        *,
        angular_velocity: Sequence[float],
        quaternion_wxyz: Sequence[float],
        command_xyz: Sequence[float],
        joint_position_hardware: Sequence[float],
        joint_velocity_hardware: Sequence[float],
        previous_action_policy: Sequence[float],
    ) -> np.ndarray:
        """Assemble [ang_vel, gravity, command, q-default, qd, last_action]."""

        cfg = self.config
        angular_velocity = _finite_vector("angular_velocity", angular_velocity, 3)
        command = _finite_vector("command_xyz", command_xyz, 3).copy()
        joint_position = _finite_vector("joint_position_hardware", joint_position_hardware, 12)
        joint_velocity = _finite_vector("joint_velocity_hardware", joint_velocity_hardware, 12)
        previous_action = _finite_vector("previous_action_policy", previous_action_policy, 12)

        limits = cfg.command_limits
        command[0] = np.clip(command[0], limits[0], limits[1])
        command[1] = np.clip(command[1], limits[2], limits[3])
        command[2] = np.clip(command[2], limits[4], limits[5])

        mapping = np.asarray(cfg.policy_to_hardware, dtype=np.int64)
        defaults = np.asarray(cfg.default_position_hardware, dtype=np.float32)
        position_policy = joint_position[mapping]
        velocity_policy = joint_velocity[mapping]
        default_policy = defaults[mapping]

        observation = np.concatenate(
            (
                angular_velocity * cfg.angular_velocity_scale,
                projected_gravity(quaternion_wxyz),
                command,
                (position_policy - default_policy) * cfg.joint_position_scale,
                velocity_policy * cfg.joint_velocity_scale,
                previous_action,
            )
        ).astype(np.float32, copy=False)
        if observation.shape != (cfg.observation_dim,):
            raise RuntimeError(f"assembled observation has shape {observation.shape}")
        if not np.isfinite(observation).all():
            raise RuntimeError("assembled observation contains NaN or infinity")
        return observation

    def infer(self, observation: Sequence[float]) -> np.ndarray:
        observation = _finite_vector("observation", observation, self.config.observation_dim)
        result = self.session.run([self.output_name], {self.input_name: observation.reshape(1, -1)})[0]
        action = np.asarray(result, dtype=np.float32).reshape(-1)
        if action.shape != (self.config.action_dim,):
            raise RuntimeError(f"policy returned shape {action.shape}")
        if not np.isfinite(action).all():
            raise RuntimeError("policy returned NaN or infinity")
        return action

    def action_to_hardware_targets(self, action_policy: Sequence[float]) -> np.ndarray:
        action = _finite_vector("action_policy", action_policy, self.config.action_dim)
        targets = np.asarray(self.config.default_position_hardware, dtype=np.float32).copy()
        mapping = np.asarray(self.config.policy_to_hardware, dtype=np.int64)
        targets[mapping] += action * self.config.action_scale
        if not np.isfinite(targets).all():
            raise RuntimeError("mapped targets contain NaN or infinity")
        return targets

    @staticmethod
    def finite_summary(values: Sequence[float]) -> dict[str, float]:
        array = np.asarray(values, dtype=np.float32)
        if not np.isfinite(array).all():
            raise ValueError("summary input contains NaN or infinity")
        return {
            "minimum": float(array.min()),
            "maximum": float(array.max()),
            "l2_norm": float(math.sqrt(float(np.dot(array, array)))),
        }
