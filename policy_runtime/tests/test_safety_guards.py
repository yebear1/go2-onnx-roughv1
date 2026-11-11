from pathlib import Path
import time
import unittest

import numpy as np

from go2_dds.policy_controller import (
    ACTUATION_CONFIRMATION,
    CommandMailbox,
    LowCmdBuilder,
    SafetyConfig,
    TargetSafetySupervisor,
    validate_actuation_gates,
)
from go2_dds.sport_safe_test import CONFIRMATION, validate_request


ROOT = Path(__file__).resolve().parents[2]


class SafetyGuardTest(unittest.TestCase):
    def test_read_only_sources_have_no_publish_api(self):
        for relative in ("go2_dds/read_only_probe.py", "go2_dds/shadow_walk.py"):
            source = (ROOT / relative).read_text()
            self.assertNotIn("ChannelPublisher", source)
            self.assertNotIn("LowCmd_", source)

    def test_sport_send_requires_both_gates(self):
        with self.assertRaises(ValueError):
            validate_request(False, CONFIRMATION, 0.05, 0.25)
        with self.assertRaises(ValueError):
            validate_request(True, "wrong", 0.05, 0.25)
        validate_request(True, CONFIRMATION, 0.05, 0.25)

    def test_sport_limits(self):
        with self.assertRaises(ValueError):
            validate_request(True, CONFIRMATION, 0.11, 0.25)
        with self.assertRaises(ValueError):
            validate_request(True, CONFIRMATION, 0.05, 0.51)

    def test_lowcmd_requires_all_three_gates(self):
        with self.assertRaises(ValueError):
            validate_actuation_gates(False, "suspended", ACTUATION_CONFIRMATION)
        with self.assertRaises(ValueError):
            validate_actuation_gates(True, "ground", ACTUATION_CONFIRMATION)
        with self.assertRaises(ValueError):
            validate_actuation_gates(True, "suspended", "wrong")
        validate_actuation_gates(True, "suspended", ACTUATION_CONFIRMATION)

    def test_command_mailbox_deadman_returns_zero(self):
        mailbox = CommandMailbox()
        mailbox.set(0.2, -0.1, 0.3)
        command, fresh = mailbox.read(1.0)
        self.assertTrue(fresh)
        np.testing.assert_allclose(command, (0.2, -0.1, 0.3))
        command, fresh = mailbox.read(0.01, now=time.monotonic() + 1.0)
        self.assertFalse(fresh)
        np.testing.assert_allclose(command, (0.0, 0.0, 0.0))

    def test_target_clamp_and_slew(self):
        config = SafetyConfig(warmup_frames=1, max_target_step=0.04)
        safety = TargetSafetySupervisor(config)
        measured = np.asarray(
            (-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5),
            dtype=np.float32,
        )
        safety.validate_state(measured, np.zeros(12), time.monotonic())
        target = safety.prepare_target(np.full(12, 100.0), measured)
        self.assertTrue(np.all(np.abs(target - measured) <= 0.040001))
        self.assertTrue(np.all(target <= np.asarray(config.joint_upper) + 1e-6))

    def test_stale_state_resets_ready(self):
        config = SafetyConfig(warmup_frames=1)
        safety = TargetSafetySupervisor(config)
        q = np.asarray(
            (-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5),
            dtype=np.float32,
        )
        safety.validate_state(q, np.zeros(12), time.monotonic())
        self.assertTrue(safety.ready)
        with self.assertRaisesRegex(ValueError, "stale"):
            safety.validate_state(q, np.zeros(12), time.monotonic() - 1.0)
        self.assertFalse(safety.ready)

    def test_lowcmd_frame_layout_and_crc_when_sdk_available(self):
        try:
            builder = LowCmdBuilder()
        except ModuleNotFoundError as error:
            self.skipTest(f"Unitree/CycloneDDS SDK unavailable: {error}")
        targets = (-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5)
        message = builder.build(targets, (25.0,) * 12, (0.5,) * 12)
        self.assertEqual(message.head, [0xFE, 0xEF])
        self.assertEqual(message.level_flag, 0xFF)
        self.assertNotEqual(message.crc, 0)
        for motor in message.motor_cmd[12:]:
            self.assertEqual(motor.q, 2.146e9)
            self.assertEqual(motor.kp, 0.0)


if __name__ == "__main__":
    unittest.main()
