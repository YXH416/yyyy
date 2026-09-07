"""Headless regression for startup RX loss and cumulative key release."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location(
    "debugger", Path(__file__).parents[1] / "pc_tools/ball_keyboard_debugger/app.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Harness:
    process_line = module.BallDebugger.process_line
    step_angle = module.BallDebugger.step_angle
    commit_key_up = module.BallDebugger.commit_key_up
    start_manual = module.BallDebugger.start_manual

    def __init__(self):
        self.manual_state = "STARTING"
        self.auto_enable_allowed = True
        self.sent = []
        self.recorder = SimpleNamespace(event=lambda *args: None)
        self.append_log = lambda *args: None
        self.send = lambda command, **kwargs: self.sent.append(command)
        self.current_manual_angle = 12.0
        self.mechanical_pos_limit = 15.0
        self.mechanical_neg_limit = 7.0
        self.firmware_command_limit = 15.0
        self.release_jobs = {}
        self.pressed = {"Right"}
        self.repeat_key = "Right"

    def cancel_repeat(self):
        self.repeat_key = None

    def command_manual_angle(self, value, event):
        self.sent.append(value)
        self.current_manual_angle = value


class Regression(unittest.TestCase):
    def test_rx_loss_does_not_cancel_start_or_heartbeat_state(self):
        h = Harness()
        h.process_line("[ERR] RX_LOST resend_after_newline")
        self.assertEqual(h.manual_state, "STARTING")
        self.assertTrue(h.auto_enable_allowed)
        self.assertEqual(h.sent, ["STATUS"])

    def test_release_holds_without_zero_command(self):
        h = Harness()
        h.commit_key_up("Right")
        h.commit_key_up("Right")
        self.assertEqual(h.sent, [])
        self.assertEqual(h.current_manual_angle, 12.0)
        self.assertIsNone(h.repeat_key)

    def test_asymmetric_limit_reverse_is_one_degree(self):
        h = Harness()
        h.step_angle("Left", "test")
        self.assertEqual(h.sent, [11.0])
        for _ in range(40):
            h.step_angle("Left", "test")
        self.assertEqual(h.current_manual_angle, -7.0)

    def test_stop_cancels_delayed_auto_restart(self):
        h = Harness()
        h.auto_enable_allowed = False
        h.start_manual()
        self.assertEqual(h.sent, [])


if __name__ == "__main__":
    unittest.main()
