import unittest

import numpy as np

from policy_runtime.walk_policy import WalkPolicyConfig, WalkPolicyRunner, projected_gravity


class WalkPolicyLayoutTest(unittest.TestCase):
    def test_identity_gravity(self):
        np.testing.assert_allclose(projected_gravity((1, 0, 0, 0)), (0, 0, -1), atol=1e-7)

    def test_policy_mapping_is_permutation(self):
        config = WalkPolicyConfig()
        self.assertEqual(sorted(config.policy_to_hardware), list(range(12)))

    def test_action_mapping(self):
        runner = object.__new__(WalkPolicyRunner)
        runner.config = WalkPolicyConfig()
        action = np.arange(12, dtype=np.float32) / 10.0
        targets = runner.action_to_hardware_targets(action)
        defaults = np.asarray(runner.config.default_position_hardware, dtype=np.float32)
        for policy_index, hardware_index in enumerate(runner.config.policy_to_hardware):
            self.assertAlmostEqual(
                float(targets[hardware_index]),
                float(defaults[hardware_index] + action[policy_index] * runner.config.action_scale),
                places=6,
            )


if __name__ == "__main__":
    unittest.main()
