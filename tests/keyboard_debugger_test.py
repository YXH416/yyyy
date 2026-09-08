"""Headless regression for startup RX loss and cumulative key release."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import queue
from unittest.mock import patch

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
        self.rx_loss_count = 0
        self.last_rx_warning = float("inf")
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
    def test_inbox_flood_returns_control_to_gui(self):
        h = Harness()
        h.inbox = queue.Queue()
        for _ in range(1000):
            h.inbox.put(("line", "noise"))
        processed, scheduled = [], []
        h.process_line = processed.append
        h.process_inbox = lambda: None
        h.after = lambda *args: scheduled.append(args)
        with patch.object(module.time, "monotonic", return_value=0.0):
            module.BallDebugger.process_inbox(h)
        self.assertEqual(len(processed), 100)
        self.assertEqual(h.inbox.qsize(), 900)
        self.assertEqual(scheduled[0][0], 20)

    def test_zero_confirmation_timeout_requests_stop(self):
        h = Harness()
        h.manual_state = "ZEROING"
        h.zero_request_pending = True
        h.zero_deadline = -1.0
        stopped = []
        h.stop_manual = lambda: stopped.append(True)
        h.set_manual_state = lambda state, detail: setattr(h, "manual_state", state)
        h.manual_state_guard = lambda: None
        h.after = lambda *args: None
        module.BallDebugger.manual_state_guard(h)
        self.assertEqual(stopped, [True])
        self.assertEqual(h.manual_state, "STOPPED")

    def test_rx_loss_does_not_cancel_start_or_heartbeat_state(self):
        h = Harness()
        for _ in range(8406):
            h.process_line("[ERR] RX_LOST resend_after_newline")
        self.assertEqual(h.manual_state, "STARTING")
        self.assertTrue(h.auto_enable_allowed)
        self.assertEqual(h.sent, [])
        self.assertEqual(h.rx_loss_count, 8406)

    def test_corrupt_frame_is_not_start_failure(self):
        h = Harness()
        h.process_line("[ERR] BAD_COMMAND_OR_RANGE use_HELP")
        self.assertEqual(h.manual_state, "STARTING")
        self.assertEqual(h.sent, [])

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
