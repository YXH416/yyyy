"""Headless regression for the ROUND-047 minimal keyboard remote."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location(
    "remote", Path(__file__).parents[1] / "pc_tools/ball_keyboard_debugger/app.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Harness:
    step_angle = module.BallRemote.step_angle
    send_relative = module.BallRemote.send_relative
    commit_key_up = module.BallRemote.commit_key_up
    process_line = module.BallRemote.process_line

    def __init__(self):
        self.target_angle = 0.0
        self.actual_angle = None
        self.sent = []
        self.pressed = set()
        self.release_jobs = {}
        self.repeat_key = None
        self.repeat_job = None
        self.link = SimpleNamespace(connected=True)
        self.link.send = lambda cmd: (self.sent.append(cmd), True)[1]
        self.update_display = lambda: None

    def send(self, command):
        return self.link.send(command)

    def cancel_repeat(self, clear_key=True):
        self.repeat_job = None
        if clear_key:
            self.repeat_key = None

    def after(self, ms, fn):
        return f"job-{ms}"


class Regression(unittest.TestCase):
    def test_right_increments_and_commands(self):
        h = Harness()
        h.step_angle("Right")
        self.assertEqual(h.target_angle, 1.0)
        self.assertEqual(h.sent, ["ANGLE_REL,+1.0"])

    def test_left_decrements_and_commands(self):
        h = Harness()
        h.target_angle = 2.0
        h.step_angle("Left")
        self.assertEqual(h.target_angle, 1.0)
        self.assertEqual(h.sent, ["ANGLE_REL,+1.0"])

    def test_upper_limit_blocks_and_stops_repeat(self):
        h = Harness()
        h.target_angle = 15.0
        h.repeat_key = "Right"
        h.step_angle("Right")
        self.assertEqual(h.target_angle, 15.0)
        self.assertEqual(h.sent, [])
        self.assertIsNone(h.repeat_key)

    def test_lower_limit_blocks(self):
        h = Harness()
        h.target_angle = -15.0
        h.step_angle("Left")
        self.assertEqual(h.target_angle, -15.0)
        self.assertEqual(h.sent, [])

    def test_release_holds_target_without_zero(self):
        h = Harness()
        h.target_angle = 12.0
        h.pressed = {"Right"}
        h.repeat_key = "Right"
        h.commit_key_up("Right")
        h.commit_key_up("Right")
        self.assertEqual(h.sent, [])
        self.assertEqual(h.target_angle, 12.0)
        self.assertIsNone(h.repeat_key)

    def test_status_parses_actual_angle(self):
        h = Harness()
        h.process_line("[STATUS] ms=1 motor_deg=3.25 fault=NONE fb=PWM")
        self.assertEqual(h.actual_angle, 3.25)

    def test_status_invalid_actual_angle_is_none(self):
        h = Harness()
        h.process_line("[STATUS] ms=1 motor_deg=-9999.00 fault=NO_ENCODER")
        self.assertIsNone(h.actual_angle)

    def test_relative_command_format(self):
        h = Harness()
        h.send_relative(3.0)
        h.send_relative(-2.0)
        self.assertEqual(h.sent, ["ANGLE_REL,+3.0", "ANGLE_REL,-2.0"])

    def test_relative_not_sent_when_disconnected(self):
        h = Harness()
        h.link.connected = False
        h.send_relative(3.0)
        self.assertEqual(h.sent, [])


if __name__ == "__main__":
    unittest.main()
