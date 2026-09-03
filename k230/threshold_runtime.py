"""Yahboom K230 direct-frame display and tap-button LAB threshold editor."""

import time

from machine import TOUCH
from media.display import Display

from utils.threshold_config import (
    DEFAULT_THRESHOLD,
    VALUE_RANGES,
    load_profile,
    save_profile,
)


REPEAT_DELAY_MS = 300
REPEAT_INTERVAL_MS = 40
REPEAT_FAST_AFTER_MS = 1200
REPEAT_FAST_STEP = 5
TOUCH_DEBOUNCE_MS = 180

TUNE_X = 440
TUNE_Y = 8
TUNE_W = 192
TUNE_H = 64

SAVE_X = 8
SAVE_Y = 8
SAVE_W = 210
SAVE_H = 64

RESET_X = 220
RESET_Y = 8
RESET_W = 210
RESET_H = 64

PREVIEW_H = 240

ROW_Y0 = 246
ROW_STEP = 38
ROW_H = 36
LABEL_X = 8
MINUS_X = 64
MINUS_W = 170
VALUE_X = 240
VALUE_W = 190
PLUS_X = 436
PLUS_W = 196

LABELS = ("L MIN", "L MAX", "A MIN", "A MAX", "B MIN", "B MAX")
FULL_RANGE_THRESHOLD = (0, 100, -128, 127, -128, 127)

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
MUTED = (170, 176, 188)
BLUE = (34, 115, 188)
GREEN = (38, 142, 83)
DARK = (31, 34, 40)
YELLOW = (255, 190, 0)


class ThresholdRuntime:
    """Display annotated frames and own the touch/threshold state."""

    def __init__(self, screen_width=640, screen_height=480):
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.touch = TOUCH(0)

        profile = load_profile(default_threshold=DEFAULT_THRESHOLD)
        self.values = list(profile["threshold"])
        self.invert = bool(profile["invert"])
        self.tuning = False

        self.press_started_ms = None
        self.touch_x = 0
        self.touch_y = 0
        self.hold_row = None
        self.hold_delta = 0
        self.last_repeat_ms = 0
        self.last_action_ms = None
        print(
            "#TOUCH mode=tap_buttons repeat=%s/%sms fast_step=%s "
            "reset=full_range mod=MOD-011"
            % (REPEAT_DELAY_MS, REPEAT_INTERVAL_MS, REPEAT_FAST_STEP)
        )

    def threshold(self):
        return tuple(self.values)

    def inverted(self):
        return self.invert

    def _button_hit(self, x, y):
        for row in range(6):
            row_y = ROW_Y0 + row * ROW_STEP
            if row_y <= y <= row_y + ROW_H:
                if MINUS_X <= x <= MINUS_X + MINUS_W:
                    return row, -1
                if PLUS_X <= x <= PLUS_X + PLUS_W:
                    return row, 1
        return None

    @staticmethod
    def _inside(x, y, left, top, width, height):
        return (
            left <= x <= left + width
            and top <= y <= top + height
        )

    def _tune_button_hit(self, x, y):
        return self._inside(x, y, TUNE_X, TUNE_Y, TUNE_W, TUNE_H)

    def _save_button_hit(self, x, y):
        return self._inside(x, y, SAVE_X, SAVE_Y, SAVE_W, SAVE_H)

    def _reset_button_hit(self, x, y):
        return self._inside(
            x, y, RESET_X, RESET_Y, RESET_W, RESET_H
        )

    def _action_ready(self, now):
        return (
            self.last_action_ms is None
            or time.ticks_diff(now, self.last_action_ms)
            >= TOUCH_DEBOUNCE_MS
        )

    def _mark_action(self, now):
        self.last_action_ms = now

    def _adjust(self, row, delta):
        low, high = VALUE_RANGES[row]
        value = max(low, min(high, self.values[row] + delta))
        if row % 2 == 0:
            value = min(value, self.values[row + 1])
        else:
            value = max(value, self.values[row - 1])
        self.values[row] = value

    def _reset_full_range(self):
        self.values = list(FULL_RANGE_THRESHOLD)
        self.invert = False
        print("#THRESHOLD reset=full_range values=%s" % (self.threshold(),))

    def _release(self):
        self.press_started_ms = None
        self.hold_row = None
        self.hold_delta = 0
        self.last_repeat_ms = 0

    def _toggle_mode(self):
        self.tuning = not self.tuning
        self.hold_row = None
        if self.tuning:
            profile = load_profile(default_threshold=DEFAULT_THRESHOLD)
            self.values = list(profile["threshold"])
            self.invert = bool(profile["invert"])
            print("#THRESHOLD mode=edit")
        else:
            save_profile(self.threshold(), self.invert)
            print("#THRESHOLD mode=run values=%s" % (self.threshold(),))

    def _repeat_hold(self, now):
        if self.hold_row is None:
            return
        if (
            time.ticks_diff(now, self.press_started_ms) >= REPEAT_DELAY_MS
            and time.ticks_diff(now, self.last_repeat_ms)
            >= REPEAT_INTERVAL_MS
        ):
            elapsed = time.ticks_diff(now, self.press_started_ms)
            step = (
                REPEAT_FAST_STEP
                if elapsed >= REPEAT_FAST_AFTER_MS
                else 1
            )
            self._adjust(self.hold_row, self.hold_delta * step)
            self.last_repeat_ms = now
            self._mark_action(now)

    def poll_touch(self):
        # Yahboom's UI example performs actions immediately on EVENT_DOWN.
        # Mode switching therefore uses large tap buttons, not press duration.
        points = self.touch.read(1)
        now = time.ticks_ms()

        if not points:
            if self.press_started_ms is not None and self.hold_row is not None:
                self._repeat_hold(now)
            return

        point = points[0]
        if point.event == TOUCH.EVENT_UP:
            self._release()
            return
        # Yahboom 的 02.Basic/16.touch.py 还把事件值 0 作为有效接触。
        if point.event not in (0, TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE):
            return

        self.touch_x = point.x
        self.touch_y = point.y
        if self.press_started_ms is None:
            self.press_started_ms = now
            print(
                "#TOUCH event=down x=%s y=%s mode=%s"
                % (
                    point.x,
                    point.y,
                    "edit" if self.tuning else "run",
                )
            )

            if not self.tuning:
                if (
                    self._tune_button_hit(point.x, point.y)
                    and self._action_ready(now)
                ):
                    self._mark_action(now)
                    self._toggle_mode()
                return

            if (
                self._save_button_hit(point.x, point.y)
                and self._action_ready(now)
            ):
                self._mark_action(now)
                self._toggle_mode()
                return

            if (
                self._reset_button_hit(point.x, point.y)
                and self._action_ready(now)
            ):
                self._mark_action(now)
                self._reset_full_range()
                return

            hit = self._button_hit(point.x, point.y)
            if hit:
                self.hold_row, self.hold_delta = hit
                self.last_repeat_ms = now
                if self._action_ready(now):
                    self._adjust(self.hold_row, self.hold_delta)
                    self._mark_action(now)

        if self.hold_row is not None:
            hit = self._button_hit(point.x, point.y)
            if hit != (self.hold_row, self.hold_delta):
                self._release()
                return
            self._repeat_hold(now)
            return

    @staticmethod
    def _center_text(target, x, y, width, text, size, color):
        text_x = x + max(4, (width - len(text) * max(7, size // 2)) // 2)
        target.draw_string_advanced(text_x, y, size, text, color=color)

    def _draw_adjust_button(self, target, row, delta, x, width, color):
        y = ROW_Y0 + row * ROW_STEP
        held = self.hold_row == row and self.hold_delta == delta
        background = YELLOW if held else color
        foreground = BLACK if held else WHITE
        target.draw_rectangle(
            x, y, width, ROW_H, color=background, fill=True
        )
        self._center_text(
            target,
            x,
            y + 5,
            width,
            "-" if delta < 0 else "+",
            22,
            foreground,
        )

    def _draw_controls(self, target):
        for row in range(6):
            y = ROW_Y0 + row * ROW_STEP
            target.draw_string_advanced(
                LABEL_X, y + 10, 14, LABELS[row], color=MUTED
            )
            self._draw_adjust_button(
                target, row, -1, MINUS_X, MINUS_W, BLUE
            )
            target.draw_rectangle(
                VALUE_X, y, VALUE_W, ROW_H, color=DARK, fill=True
            )
            self._center_text(
                target,
                VALUE_X,
                y + 7,
                VALUE_W,
                str(self.values[row]),
                20,
                WHITE,
            )
            self._draw_adjust_button(
                target, row, 1, PLUS_X, PLUS_W, GREEN
            )

    def _draw_touch_marker(self, target):
        if self.press_started_ms is None:
            return
        target.draw_circle(
            self.touch_x,
            self.touch_y,
            12,
            color=YELLOW,
            thickness=3,
        )

    def _draw_tune_button(self, target):
        target.draw_rectangle(
            TUNE_X,
            TUNE_Y,
            TUNE_W,
            TUNE_H,
            color=BLUE,
            fill=True,
        )
        self._center_text(
            target,
            TUNE_X,
            TUNE_Y + 15,
            TUNE_W,
            "TUNE",
            28,
            WHITE,
        )

    def _draw_save_button(self, target):
        target.draw_rectangle(
            SAVE_X,
            SAVE_Y,
            SAVE_W,
            SAVE_H,
            color=GREEN,
            fill=True,
        )
        self._center_text(
            target,
            SAVE_X,
            SAVE_Y + 16,
            SAVE_W,
            "SAVE EXIT",
            24,
            WHITE,
        )

    def _draw_reset_button(self, target):
        target.draw_rectangle(
            RESET_X,
            RESET_Y,
            RESET_W,
            RESET_H,
            color=BLUE,
            fill=True,
        )
        self._center_text(
            target,
            RESET_X,
            RESET_Y + 16,
            RESET_W,
            "FULL RESET",
            22,
            WHITE,
        )

    def show_normal(self, frame):
        # 官方 640x480 RGB888 cv_lite 例程使用的原生直显路径。
        # 不再把摄像头帧缩放绘制到另一张 RGB888 画布。
        self._draw_tune_button(frame)
        self._draw_touch_marker(frame)
        Display.show_image(frame)

    def show_tuner(self, frame):
        # 保持原生 640x480 尺寸：上半屏显示实时二值结果，下半屏为
        # 不透明大按钮。整个运行时不再使用 draw_image、缩放或视频层合成。
        binary = frame.copy()
        binary.binary([self.threshold()], invert=self.invert)
        binary.draw_rectangle(
            0,
            PREVIEW_H,
            self.screen_width,
            self.screen_height - PREVIEW_H,
            color=BLACK,
            fill=True,
        )
        binary.draw_rectangle(440, 0, 200, 36, color=BLACK, fill=True)
        binary.draw_string_advanced(
            456,
            8,
            16,
            "BINARY",
            color=WHITE,
        )
        self._draw_save_button(binary)
        self._draw_reset_button(binary)
        self._draw_controls(binary)
        self._draw_touch_marker(binary)
        Display.show_image(binary)
        del binary

    def close(self):
        try:
            self.touch.deinit()
        except Exception:
            pass


