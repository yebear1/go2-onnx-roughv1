from types import SimpleNamespace
import unittest

from bridge.app import (
    SDK_SPORT_CONFIRMATION,
    SdkSportAdapter,
    validate_sdk_sport_gates,
)


class FakeClient:
    def __init__(self):
        self.moves = []
        self.stops = 0

    def Move(self, forward, lateral, yaw):
        self.moves.append((forward, lateral, yaw))
        return 0

    def StopMove(self):
        self.stops += 1
        return 0


def config():
    return SimpleNamespace(
        sdk_sport_enable=True,
        sdk_sport_confirmation=SDK_SPORT_CONFIRMATION,
        net_iface="eth0",
        dds_domain=0,
    )


class SdkSportAdapterTest(unittest.TestCase):
    def test_process_level_send_gate(self):
        with self.assertRaises(RuntimeError):
            validate_sdk_sport_gates(False, SDK_SPORT_CONFIRMATION)
        with self.assertRaises(RuntimeError):
            validate_sdk_sport_gates(True, "wrong")
        validate_sdk_sport_gates(True, SDK_SPORT_CONFIRMATION)

    def test_zero_transition_stops_once(self):
        adapter = SdkSportAdapter(config())
        adapter.client = FakeClient()
        adapter.move(0.05, 0.0, 0.0)
        adapter.move(0.0, 0.0, 0.0)
        adapter.move(0.0, 0.0, 0.0)
        self.assertEqual(adapter.client.moves, [(0.05, 0.0, 0.0)])
        self.assertEqual(adapter.client.stops, 1)

    def test_controlled_stop_always_calls_stopmove(self):
        adapter = SdkSportAdapter(config())
        adapter.client = FakeClient()
        adapter.controlled_stop()
        self.assertEqual(adapter.client.stops, 1)


if __name__ == "__main__":
    unittest.main()
