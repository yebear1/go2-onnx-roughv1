"""Run the pinned walking policy from Go2 LowState with guarded LowCmd output.

The default mode is shadow-only: no DDS publisher is constructed.  Low-level
actuation requires three independent startup gates intended for a physically
suspended robot test.  Browser arming controls velocity commands; it cannot
enable low-level actuation by itself.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time
from typing import Callable, Sequence

import numpy as np

from policy_runtime.selftest import DEFAULT_MODEL
from policy_runtime.walk_policy import WalkPolicyRunner


ACTUATION_CONFIRMATION = "ROBOT_SUSPENDED_REMOTE_READY"
POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0


def validate_actuation_gates(enable_send: bool, test_mode: str, confirmation: str) -> None:
    """Reject low-level output unless all suspended-test gates are present."""

    if not enable_send:
        raise ValueError("low-level output is disabled")
    if test_mode != "suspended":
        raise ValueError("test mode must be 'suspended'")
    if confirmation != ACTUATION_CONFIRMATION:
        raise ValueError(f"confirmation must exactly match {ACTUATION_CONFIRMATION}")


@dataclass(frozen=True)
class SafetyConfig:
    state_timeout: float = 0.10
    command_timeout: float = 0.35
    warmup_frames: int = 10
    max_target_step: float = 0.04
    max_abs_joint_velocity: float = 35.0
    joint_lower: tuple[float, ...] = (
        -1.02625588, -1.52018493, -2.70385065,
        -1.02625588, -1.52018493, -2.70385065,
        -1.02625588, -0.47298494, -2.70385065,
        -1.02625588, -0.47298494, -2.70385065,
    )
    joint_upper: tuple[float, ...] = (
        1.02625588, 3.44008479, -0.85660931,
        1.02625588, 3.44008479, -0.85660931,
        1.02625588, 4.48728473, -0.85660931,
        1.02625588, 4.48728473, -0.85660931,
    )


class CommandMailbox:
    """Thread-safe latest web command with an independent dead-man timeout."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._command = np.zeros(3, dtype=np.float32)
        self._updated_at = 0.0

    def set(self, forward: float, lateral: float, yaw: float) -> None:
        command = np.asarray((forward, lateral, yaw), dtype=np.float32)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("command must contain three finite values")
        with self._lock:
            self._command = command
            self._updated_at = time.monotonic()

    def zero(self) -> None:
        self.set(0.0, 0.0, 0.0)

    def read(self, timeout: float, now: float | None = None) -> tuple[np.ndarray, bool]:
        current = time.monotonic() if now is None else now
        with self._lock:
            command = self._command.copy()
            updated_at = self._updated_at
        fresh = updated_at > 0.0 and current - updated_at <= timeout
        return (command if fresh else np.zeros(3, dtype=np.float32)), fresh


class LatestLowState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sequence = 0
        self._message = None
        self._received_at = 0.0

    def receive(self, message) -> None:
        with self._lock:
            self._sequence += 1
            self._message = message
            self._received_at = time.monotonic()

    def read(self):
        with self._lock:
            return self._sequence, self._message, self._received_at


class TargetSafetySupervisor:
    """Validate feedback, clamp policy targets, and enforce per-frame slew."""

    def __init__(self, config: SafetyConfig | None = None) -> None:
        self.config = config or SafetyConfig()
        self._last_target: np.ndarray | None = None
        self._valid_frames = 0

    @property
    def ready(self) -> bool:
        return self._valid_frames >= self.config.warmup_frames

    def reset(self) -> None:
        self._last_target = None
        self._valid_frames = 0

    def validate_state(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        received_at: float,
        now: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        current = time.monotonic() if now is None else now
        q = np.asarray(positions, dtype=np.float32)
        dq = np.asarray(velocities, dtype=np.float32)
        if q.shape != (12,) or dq.shape != (12,):
            self.reset()
            raise ValueError("LowState must contain 12 joint positions and velocities")
        if not np.isfinite(q).all() or not np.isfinite(dq).all():
            self.reset()
            raise ValueError("LowState contains NaN or infinity")
        if received_at <= 0.0 or current - received_at > self.config.state_timeout:
            self.reset()
            raise ValueError("LowState is stale")
        if np.max(np.abs(dq)) > self.config.max_abs_joint_velocity:
            self.reset()
            raise ValueError("joint velocity exceeds safety limit")
        lower = np.asarray(self.config.joint_lower, dtype=np.float32)
        upper = np.asarray(self.config.joint_upper, dtype=np.float32)
        if np.any(q < lower - 0.10) or np.any(q > upper + 0.10):
            self.reset()
            raise ValueError("joint feedback exceeds configured range")
        self._valid_frames += 1
        return q, dq

    def prepare_target(self, desired: Sequence[float], measured: Sequence[float]) -> np.ndarray:
        target = np.asarray(desired, dtype=np.float32)
        measured_q = np.asarray(measured, dtype=np.float32)
        if target.shape != (12,) or measured_q.shape != (12,) or not np.isfinite(target).all():
            self.reset()
            raise ValueError("target must contain 12 finite joint positions")
        lower = np.asarray(self.config.joint_lower, dtype=np.float32)
        upper = np.asarray(self.config.joint_upper, dtype=np.float32)
        target = np.clip(target, lower, upper)
        origin = measured_q if self._last_target is None else self._last_target
        target = origin + np.clip(
            target - origin, -self.config.max_target_step, self.config.max_target_step
        )
        target = np.clip(target, lower, upper).astype(np.float32, copy=False)
        self._last_target = target.copy()
        return target


class LowCmdBuilder:
    """Create official-format Go2 LowCmd frames and calculate their CRC."""

    def __init__(self) -> None:
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self.message = unitree_go_msg_dds__LowCmd_()
        self.crc = CRC()
        self.message.head[0] = 0xFE
        self.message.head[1] = 0xEF
        self.message.level_flag = 0xFF
        self.message.gpio = 0
        for motor in self.message.motor_cmd:
            motor.mode = 0x01
            motor.q = POS_STOP_F
            motor.dq = VEL_STOP_F
            motor.tau = 0.0
            motor.kp = 0.0
            motor.kd = 0.0

    def build(
        self,
        targets: Sequence[float],
        kp: Sequence[float],
        kd: Sequence[float],
    ):
        q = np.asarray(targets, dtype=np.float32)
        p = np.asarray(kp, dtype=np.float32)
        d = np.asarray(kd, dtype=np.float32)
        if q.shape != (12,) or p.shape != (12,) or d.shape != (12,):
            raise ValueError("targets, kp and kd must each contain 12 values")
        if not np.isfinite(q).all() or not np.isfinite(p).all() or not np.isfinite(d).all():
            raise ValueError("LowCmd values must be finite")
        if np.any(p < 0.0) or np.any(p > 25.0) or np.any(d < 0.0) or np.any(d > 0.5):
            raise ValueError("LowCmd gains exceed pinned walking-policy limits")
        for index in range(12):
            motor = self.message.motor_cmd[index]
            motor.mode = 0x01
            motor.q = float(q[index])
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(p[index])
            motor.kd = float(d[index])
        self.message.crc = self.crc.Crc(self.message)
        return self.message


class PolicyController:
    """50 Hz policy loop shared by the web adapter and standalone diagnostics."""

    def __init__(
        self,
        *,
        interface: str = "eth0",
        domain: int = 0,
        state_topic: str = "rt/lowstate",
        command_topic: str = "rt/lowcmd",
        model: str | Path = DEFAULT_MODEL,
        enable_send: bool = False,
        test_mode: str = "",
        confirmation: str = "",
        status_callback: Callable[[dict], None] | None = None,
        safety: SafetyConfig | None = None,
    ) -> None:
        self.interface = interface
        self.domain = domain
        self.state_topic = state_topic
        self.command_topic = command_topic
        self.enable_send = enable_send
        self.test_mode = test_mode
        self.confirmation = confirmation
        self.status_callback = status_callback
        self.runner = WalkPolicyRunner(model)
        self.safety = TargetSafetySupervisor(safety)
        self.commands = CommandMailbox()
        self.latest = LatestLowState()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._subscriber = None
        self._publisher = None
        self._builder: LowCmdBuilder | None = None
        self._status_lock = threading.Lock()
        self.status = {
            "mode": "shadow_only",
            "publisher_created": False,
            "robot_seen": False,
            "frames": 0,
            "writes": 0,
            "fault": "",
            "command_fresh": False,
        }

    def start(self) -> None:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_

        if self.enable_send:
            validate_actuation_gates(self.enable_send, self.test_mode, self.confirmation)
        ChannelFactoryInitialize(self.domain, self.interface)
        self._subscriber = ChannelSubscriber(self.state_topic, LowState_)
        self._subscriber.Init(self.latest.receive, 10)
        if self.enable_send:
            self._builder = LowCmdBuilder()
            self._publisher = ChannelPublisher(self.command_topic, LowCmd_)
            self._publisher.Init()
            self._update_status(mode="suspended_test", publisher_created=True)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="go2-policy-50hz", daemon=True)
        self._thread.start()

    def set_command(self, forward: float, lateral: float, yaw: float) -> None:
        self.commands.set(forward, lateral, yaw)

    def stop_command(self) -> None:
        self.commands.zero()

    def _update_status(self, **values) -> None:
        with self._status_lock:
            self.status.update(values)
            snapshot = dict(self.status)
        if self.status_callback is not None:
            self.status_callback(snapshot)

    def _run(self) -> None:
        period = self.runner.config.control_dt
        previous_action = np.zeros(12, dtype=np.float32)
        last_sequence = 0
        next_deadline = time.monotonic()
        last_report = 0.0
        while not self._stop.is_set():
            next_deadline += period
            now = time.monotonic()
            sequence, message, received_at = self.latest.read()
            if message is None or sequence == last_sequence:
                if received_at and now - received_at > self.safety.config.state_timeout:
                    self.safety.reset()
                    self._update_status(fault="LowState timeout")
            else:
                last_sequence = sequence
                try:
                    motors = list(message.motor_state[:12])
                    positions, velocities = self.safety.validate_state(
                        [motor.q for motor in motors],
                        [motor.dq for motor in motors],
                        received_at,
                        now,
                    )
                    command, command_fresh = self.commands.read(
                        self.safety.config.command_timeout, now
                    )
                    observation = self.runner.assemble_observation(
                        angular_velocity=message.imu_state.gyroscope,
                        quaternion_wxyz=message.imu_state.quaternion,
                        command_xyz=command,
                        joint_position_hardware=positions,
                        joint_velocity_hardware=velocities,
                        previous_action_policy=previous_action,
                    )
                    previous_action = self.runner.infer(observation)
                    desired = self.runner.action_to_hardware_targets(previous_action)
                    targets = self.safety.prepare_target(desired, positions)
                    writes = int(self.status["writes"])
                    if self._publisher is not None and self.safety.ready:
                        assert self._builder is not None
                        kp = np.full(12, 25.0, dtype=np.float32)
                        kd = np.full(12, 0.5, dtype=np.float32)
                        self._publisher.Write(self._builder.build(targets, kp, kd))
                        writes += 1
                    frames = int(self.status["frames"]) + 1
                    values = {
                        "robot_seen": True,
                        "frames": frames,
                        "writes": writes,
                        "fault": "",
                        "command_fresh": command_fresh,
                        "velocity_command": [float(value) for value in command],
                        "battery": int(message.bms_state.soc),
                        "voltage": round(float(message.power_v), 1),
                    }
                    if now - last_report >= 0.2:
                        self._update_status(**values)
                        last_report = now
                    else:
                        with self._status_lock:
                            self.status.update(values)
                except Exception as exc:
                    self.safety.reset()
                    self._update_status(fault=str(exc))
            remaining = next_deadline - time.monotonic()
            if remaining > 0:
                self._stop.wait(remaining)
            elif remaining < -period:
                next_deadline = time.monotonic()

    def close(self) -> None:
        self.stop_command()
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._subscriber is not None:
            self._subscriber.Close()
        self._publisher = None
        self._builder = None
