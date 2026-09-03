"""Minimal touch editor for the traditional ball L MAX threshold (MOD-026).

The touch handling follows Yahboom's official ``TOUCH(0).read(1)`` example.
Large tap buttons are used; a timeout synthesizes release on firmware versions
that occasionally omit EVENT_UP.
"""

import time

import image
from machine import TOUCH


REPEAT_DELAY_MS = 300
REPEAT_INTERVAL_MS = 50
REPEAT_FAST_AFTER_MS = 1000
REPEAT_FAST_STEP = 5
RELEASE_GAP_MS = 120
TOUCH_DEBOUNCE_MS = 80

TUNE_X = 512
TUNE_Y = 4
TUNE_W = 120
TUNE_H = 56

PANEL_Y = 240
ROW_Y = 292
ROW_H = 92
MINUS_X = 8
MINUS_W = 180
VALUE_X = 196
VALUE_W = 248
PLUS_X = 452
PLUS_W = 180

SAVE_X = 8
SAVE_Y = 404
SAVE_W = 304
SAVE_H = 68
RESET_X = 320
RESET_Y = 404
RESET_W = 312
RESET_H = 68

OSD_BLACK = (255, 0, 0, 0)
OSD_WHITE = (255, 255, 255, 255)
OSD_BLUE = (255, 35, 110, 190)
OSD_YELLOW = (255, 255, 190, 0)

RGB_BLACK = (0, 0, 0)
RGB_WHITE = (255, 255, 255)
RGB_MUTED = (175, 182, 192)
RGB_BLUE = (35, 110, 190)
RGB_GREEN = (25, 150, 80)
RGB_DARK = (30, 34, 40)
RGB_YELLOW = (255, 190, 0)


def _bounded_l_max(value):
    try:
        value = int(value)
    except Exception:
        value = 72
    return max(0, min(100, value))


class BallThresholdUI:
    """One-value editor with binary preview and large touch targets."""

    def __init__(self, current_l_max, default_l_max=72):
        self.default_l_max = _bounded_l_max(default_l_max)
        self.l_max = _bounded_l_max(current_l_max)
        self.touch = TOUCH(0)
        self.tuning = False
        self.pending_save = False
        self.canvas = image.Image(640, 480, image.RGB565)

        self.press_started_ms = None
        self.last_point_ms = None
        self.last_repeat_ms = 0
        self.last_action_ms = None
        self.hold_delta = 0
        self.ignore_until_release = False
        self.touch_x = 0
        self.touch_y = 0
        self.touch_error_count = 0
        print(
            "#BALL_TOUCH ready control=L_MAX repeat=%s/%sms "
            "release_gap=%sms mod=MOD-026"
            % (REPEAT_DELAY_MS, REPEAT_INTERVAL_MS, RELEASE_GAP_MS)
        )

    @staticmethod
    def _inside(x, y, left, top, width, height):
        return (
            left <= x <= left + width
            and top <= y <= top + height
        )

    def _action_ready(self, now):
        return (
            self.last_action_ms is None
            or time.ticks_diff(now, self.last_action_ms)
            >= TOUCH_DEBOUNCE_MS
        )

    def _mark_action(self, now):
        self.last_action_ms = now

    def _adjust(self, delta):
        self.l_max = _bounded_l_max(self.l_max + delta)

    def _button_delta(self, x, y):
        if not (ROW_Y <= y <= ROW_Y + ROW_H):
            return 0
        if MINUS_X <= x <= MINUS_X + MINUS_W:
            return -1
        if PLUS_X <= x <= PLUS_X + PLUS_W:
            return 1
        return 0

    def _release(self):
        self.press_started_ms = None
        self.last_point_ms = None
        self.last_repeat_ms = 0
        self.hold_delta = 0
        self.ignore_until_release = False

    def _repeat_hold(self, now):
        if not self.hold_delta or self.press_started_ms is None:
            return
        if (
            time.ticks_diff(now, self.press_started_ms)
            >= REPEAT_DELAY_MS
            and time.ticks_diff(now, self.last_repeat_ms)
            >= REPEAT_INTERVAL_MS
        ):
            elapsed = time.ticks_diff(now, self.press_started_ms)
            step = (
                REPEAT_FAST_STEP
                if elapsed >= REPEAT_FAST_AFTER_MS
                else 1
            )
            self._adjust(self.hold_delta * step)
            self.last_repeat_ms = now

    def _enter_tuning(self):
        self.tuning = True
        self.ignore_until_release = True
        print("#BALL_THRESHOLD mode=edit l_max=%s" % self.l_max)

    def _save_exit(self):
        self.pending_save = True
        self.tuning = False
        self.ignore_until_release = True
        print("#BALL_THRESHOLD save_requested l_max=%s" % self.l_max)

    def _reset(self):
        self.l_max = self.default_l_max
        self.ignore_until_release = True
        print("#BALL_THRESHOLD reset l_max=%s" % self.l_max)

    def consume_save_request(self):
        requested = self.pending_save
        self.pending_save = False
        return requested

    def poll_touch(self):
        now = time.ticks_ms()
        try:
            points = self.touch.read(1)
        except Exception as error:
            self.touch_error_count += 1
            if (
                self.touch_error_count == 1
                or self.touch_error_count % 30 == 0
            ):
                print(
                    "#BALL_TOUCH read_error count=%s detail=%s"
                    % (self.touch_error_count, error)
                )
            return

        if not points:
            if (
                self.press_started_ms is not None
                and self.last_point_ms is not None
            ):
                gap = time.ticks_diff(now, self.last_point_ms)
                if gap >= RELEASE_GAP_MS:
                    self._release()
                else:
                    self._repeat_hold(now)
            return

        point = points[0]
        if point.event == TOUCH.EVENT_UP:
            self._release()
            return
        if point.event not in (0, TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE):
            return

        self.touch_x = point.x
        self.touch_y = point.y
        self.last_point_ms = now
        if self.ignore_until_release:
            return

        if self.press_started_ms is None:
            self.press_started_ms = now
            print(
                "#BALL_TOUCH down x=%s y=%s mode=%s"
                % (
                    point.x,
                    point.y,
                    "edit" if self.tuning else "run",
                )
            )
            if not self.tuning:
                if (
                    self._inside(
                        point.x,
                        point.y,
                        TUNE_X,
                        TUNE_Y,
                        TUNE_W,
                        TUNE_H,
                    )
                    and self._action_ready(now)
                ):
                    self._mark_action(now)
                    self._enter_tuning()
                return

            if (
                self._inside(
                    point.x,
                    point.y,
                    SAVE_X,
                    SAVE_Y,
                    SAVE_W,
                    SAVE_H,
                )
                and self._action_ready(now)
            ):
                self._mark_action(now)
                self._save_exit()
                return

            if (
                self._inside(
                    point.x,
                    point.y,
                    RESET_X,
                    RESET_Y,
                    RESET_W,
                    RESET_H,
                )
                and self._action_ready(now)
            ):
                self._mark_action(now)
                self._reset()
                return

            self.hold_delta = self._button_delta(point.x, point.y)
            self.last_repeat_ms = now
            if self.hold_delta and self._action_ready(now):
                self._adjust(self.hold_delta)
                self._mark_action(now)

        if self.hold_delta:
            if (
                self._button_delta(point.x, point.y)
                != self.hold_delta
            ):
                self._release()
                return
            self._repeat_hold(now)

    @staticmethod
    def _center_text(target, x, y, width, text, size, color):
        text_x = x + max(
            4,
            (width - len(text) * max(7, size // 2)) // 2,
        )
        target.draw_string_advanced(
            text_x,
            y,
            size,
            text,
            color=color,
        )

    def _draw_touch_marker(self, target, color):
        if self.press_started_ms is None:
            return
        target.draw_circle(
            self.touch_x,
            self.touch_y,
            12,
            color=color,
            thickness=3,
        )

    def draw_normal(self, target):
        target.draw_rectangle(
            TUNE_X,
            TUNE_Y,
            TUNE_W,
            TUNE_H,
            color=OSD_BLUE,
            fill=True,
        )
        self._center_text(
            target,
            TUNE_X,
            TUNE_Y + 13,
            TUNE_W,
            "TUNE",
            24,
            OSD_WHITE,
        )
        self._draw_touch_marker(target, OSD_YELLOW)

    def render_tuner(self, frame):
        binary = frame.copy()
        binary.binary(
            [(0, self.l_max, -128, 127, -128, 127)]
        )

        canvas = self.canvas
        canvas.clear()
        canvas.draw_rectangle(
            0, 0, 640, 480, color=RGB_BLACK, fill=True
        )
        canvas.draw_image(binary, 160, 0)
        canvas.draw_rectangle(
            0, 0, 640, 36, color=RGB_BLACK, fill=True
        )
        canvas.draw_string_advanced(
            12,
            8,
            20,
            "BINARY PREVIEW  L:0-%s" % self.l_max,
            color=RGB_WHITE,
        )
        canvas.draw_rectangle(
            0,
            PANEL_Y,
            640,
            480 - PANEL_Y,
            color=RGB_BLACK,
            fill=True,
        )
        canvas.draw_string_advanced(
            12, 252, 22, "L MAX", color=RGB_MUTED
        )

        held_minus = self.hold_delta == -1
        held_plus = self.hold_delta == 1
        canvas.draw_rectangle(
            MINUS_X,
            ROW_Y,
            MINUS_W,
            ROW_H,
            color=RGB_YELLOW if held_minus else RGB_BLUE,
            fill=True,
        )
        self._center_text(
            canvas,
            MINUS_X,
            ROW_Y + 25,
            MINUS_W,
            "-",
            40,
            RGB_BLACK if held_minus else RGB_WHITE,
        )
        canvas.draw_rectangle(
            VALUE_X,
            ROW_Y,
            VALUE_W,
            ROW_H,
            color=RGB_DARK,
            fill=True,
        )
        self._center_text(
            canvas,
            VALUE_X,
            ROW_Y + 27,
            VALUE_W,
            str(self.l_max),
            36,
            RGB_WHITE,
        )
        canvas.draw_rectangle(
            PLUS_X,
            ROW_Y,
            PLUS_W,
            ROW_H,
            color=RGB_YELLOW if held_plus else RGB_GREEN,
            fill=True,
        )
        self._center_text(
            canvas,
            PLUS_X,
            ROW_Y + 25,
            PLUS_W,
            "+",
            40,
            RGB_BLACK if held_plus else RGB_WHITE,
        )

        canvas.draw_rectangle(
            SAVE_X,
            SAVE_Y,
            SAVE_W,
            SAVE_H,
            color=RGB_GREEN,
            fill=True,
        )
        self._center_text(
            canvas,
            SAVE_X,
            SAVE_Y + 19,
            SAVE_W,
            "SAVE EXIT",
            28,
            RGB_WHITE,
        )
        canvas.draw_rectangle(
            RESET_X,
            RESET_Y,
            RESET_W,
            RESET_H,
            color=RGB_BLUE,
            fill=True,
        )
        self._center_text(
            canvas,
            RESET_X,
            RESET_Y + 19,
            RESET_W,
            "RESET 72",
            28,
            RGB_WHITE,
        )
        self._draw_touch_marker(canvas, RGB_YELLOW)
        binary = None
        return canvas

    def close(self):
        try:
            self.touch.deinit()
        except Exception:
            pass
        self.canvas = None


