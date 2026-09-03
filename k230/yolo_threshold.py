"""Touch UI and persistent confidence/NMS thresholds for K230 YOLO."""

import os
import time

try:
    import ujson as json
except ImportError:
    import json

from machine import TOUCH


CONFIG_PATH = "/sdcard/configs/yolo_threshold_config.json"
BACKUP_PATH = "/sdcard/configs/yolo_threshold_config.bak.json"
TEMP_PATH = "/sdcard/configs/yolo_threshold_config.tmp.json"
CONFIG_DIR = "/sdcard/configs"
PROFILE_VERSION = 3

MIN_VALUE = 0.05
MAX_VALUE = 0.95
SHORT_STEP = 0.05
FAST_STEP = 0.10
REPEAT_DELAY_MS = 300
REPEAT_INTERVAL_MS = 60
REPEAT_FAST_AFTER_MS = 1000
TOUCH_DEBOUNCE_MS = 80
RELEASE_GAP_MS = 120

TUNE_X = 440
TUNE_Y = 8
TUNE_W = 192
TUNE_H = 64

PANEL_Y = 250
ROW_Y = (292, 352)
ROW_H = 52
MINUS_X = 90
MINUS_W = 140
VALUE_X = 238
VALUE_W = 160
PLUS_X = 406
PLUS_W = 226

SAVE_X = 8
SAVE_Y = 414
SAVE_W = 304
SAVE_H = 58
RESET_X = 320
RESET_Y = 414
RESET_W = 312
RESET_H = 58

LABELS = ("CONF", "NMS")

# ARGB8888 colors used by PipeLine.osd_img.
BLACK = (255, 0, 0, 0)
WHITE = (255, 255, 255, 255)
MUTED = (255, 175, 182, 192)
BLUE = (255, 35, 110, 190)
GREEN = (255, 25, 150, 80)
DARK = (255, 30, 34, 40)
YELLOW = (255, 255, 190, 0)


def _clamp(value):
    return round(max(MIN_VALUE, min(MAX_VALUE, float(value))), 2)


def load_yolo_thresholds(default_confidence, default_nms):
    confidence = _clamp(default_confidence)
    nms = _clamp(default_nms)
    try:
        with open(CONFIG_PATH, "r") as config_file:
            profile = json.load(config_file)
        if int(profile.get("version", 0)) == PROFILE_VERSION:
            confidence = _clamp(
                profile.get("confidence_threshold", confidence)
            )
            nms = _clamp(profile.get("nms_threshold", nms))
        else:
            print(
                "#YOLO_THRESHOLD old_profile_ignored version=%s "
                "new_default_confidence=%.2f mod=MOD-024"
                % (profile.get("version", 0), confidence)
            )
    except Exception:
        pass
    return confidence, nms


def _remove_if_exists(path):
    try:
        os.remove(path)
    except OSError:
        pass


def save_yolo_thresholds(confidence, nms):
    profile = {
        "version": PROFILE_VERSION,
        "confidence_threshold": _clamp(confidence),
        "nms_threshold": _clamp(nms),
        "modified_by": "MOD-024",
    }
    try:
        os.stat(CONFIG_DIR)
    except OSError:
        os.mkdir(CONFIG_DIR)
    with open(TEMP_PATH, "w") as config_file:
        config_file.write(json.dumps(profile))

    had_previous = False
    try:
        os.stat(CONFIG_PATH)
        had_previous = True
    except OSError:
        pass

    if had_previous:
        _remove_if_exists(BACKUP_PATH)
        os.rename(CONFIG_PATH, BACKUP_PATH)

    try:
        os.rename(TEMP_PATH, CONFIG_PATH)
    except Exception:
        if had_previous:
            try:
                os.rename(BACKUP_PATH, CONFIG_PATH)
            except Exception:
                pass
        raise
    return profile


class YoloThresholdUI:
    """Large-button editor for YOLO confidence and NMS thresholds."""

    def __init__(self, default_confidence, default_nms):
        self.default_confidence = _clamp(default_confidence)
        self.default_nms = _clamp(default_nms)
        self.confidence, self.nms = load_yolo_thresholds(
            self.default_confidence,
            self.default_nms,
        )
        self.touch = TOUCH(0)
        self.tuning = False

        self.press_started_ms = None
        self.last_point_ms = None
        self.ignore_until_release = False
        self.hold_row = None
        self.hold_delta = 0
        self.last_repeat_ms = 0
        self.last_action_ms = None
        self.touch_x = 0
        self.touch_y = 0
        self.touch_error_count = 0
        print(
            "#YOLO_TOUCH tap=TUNE/SAVE repeat=%s/%sms release_gap=%sms "
            "mod=MOD-024"
            % (REPEAT_DELAY_MS, REPEAT_INTERVAL_MS, RELEASE_GAP_MS)
        )

    def values(self):
        return self.confidence, self.nms

    def set_values(self, confidence, nms):
        self.confidence = _clamp(confidence)
        self.nms = _clamp(nms)

    @staticmethod
    def _inside(x, y, left, top, width, height):
        return (
            left <= x <= left + width
            and top <= y <= top + height
        )

    def _button_hit(self, x, y):
        for row in range(2):
            if ROW_Y[row] <= y <= ROW_Y[row] + ROW_H:
                if MINUS_X <= x <= MINUS_X + MINUS_W:
                    return row, -1
                if PLUS_X <= x <= PLUS_X + PLUS_W:
                    return row, 1
        return None

    def _action_ready(self, now):
        return (
            self.last_action_ms is None
            or time.ticks_diff(now, self.last_action_ms)
            >= TOUCH_DEBOUNCE_MS
        )

    def _mark_action(self, now):
        self.last_action_ms = now

    def _adjust(self, row, delta):
        if row == 0:
            self.confidence = _clamp(self.confidence + delta)
        else:
            self.nms = _clamp(self.nms + delta)

    def _release(self):
        self.press_started_ms = None
        self.last_point_ms = None
        self.ignore_until_release = False
        self.hold_row = None
        self.hold_delta = 0
        self.last_repeat_ms = 0

    def _save_exit(self):
        save_yolo_thresholds(self.confidence, self.nms)
        self.tuning = False
        # Keep the current contact gated until a real/synthesized release.
        # Otherwise the MOVE belonging to SAVE can become a new TUNE press.
        self.ignore_until_release = True
        print(
            "#YOLO_THRESHOLD saved confidence=%.2f nms=%.2f"
            % self.values()
        )

    def _reset(self):
        self.confidence = self.default_confidence
        self.nms = self.default_nms
        print(
            "#YOLO_THRESHOLD reset confidence=%.2f nms=%.2f"
            % self.values()
        )

    def _repeat_hold(self, now):
        if self.hold_row is None:
            return
        if (
            time.ticks_diff(now, self.press_started_ms)
            >= REPEAT_DELAY_MS
            and time.ticks_diff(now, self.last_repeat_ms)
            >= REPEAT_INTERVAL_MS
        ):
            elapsed = time.ticks_diff(now, self.press_started_ms)
            step = (
                FAST_STEP
                if elapsed >= REPEAT_FAST_AFTER_MS
                else SHORT_STEP
            )
            self._adjust(self.hold_row, self.hold_delta * step)
            self.last_repeat_ms = now

    def poll_touch(self):
        now = time.ticks_ms()
        try:
            points = self.touch.read(1)
        except Exception as error:
            self.touch_error_count += 1
            if self.touch_error_count == 1:
                print("#YOLO_TOUCH read_error=%s" % error)
            return

        if not points:
            if (
                self.press_started_ms is not None
                and self.last_point_ms is not None
            ):
                gap = time.ticks_diff(now, self.last_point_ms)
                if gap >= RELEASE_GAP_MS:
                    # Some Yahboom touch firmware revisions do not always
                    # emit EVENT_UP. Without this timeout the first contact
                    # remains latched and every later TUNE tap is ignored.
                    self._release()
                elif self.hold_row is not None:
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
                "#YOLO_TOUCH down x=%s y=%s mode=%s"
                % (
                    point.x,
                    point.y,
                    "edit" if self.tuning else "run",
                )
            )

            if not self.tuning:
                if (
                    self._inside(
                        point.x, point.y,
                        TUNE_X, TUNE_Y, TUNE_W, TUNE_H
                    )
                    and self._action_ready(now)
                ):
                    self._mark_action(now)
                    self.tuning = True
                    self.ignore_until_release = True
                    print(
                        "#YOLO_THRESHOLD mode=edit confidence=%.2f nms=%.2f"
                        % self.values()
                    )
                return

            if (
                self._inside(
                    point.x, point.y,
                    SAVE_X, SAVE_Y, SAVE_W, SAVE_H
                )
                and self._action_ready(now)
            ):
                self._mark_action(now)
                self._save_exit()
                return

            if (
                self._inside(
                    point.x, point.y,
                    RESET_X, RESET_Y, RESET_W, RESET_H
                )
                and self._action_ready(now)
            ):
                self._mark_action(now)
                self._reset()
                self.ignore_until_release = True
                return

            hit = self._button_hit(point.x, point.y)
            if hit:
                self.hold_row, self.hold_delta = hit
                self.last_repeat_ms = now
                if self._action_ready(now):
                    self._adjust(
                        self.hold_row,
                        self.hold_delta * SHORT_STEP,
                    )
                    self._mark_action(now)

        if self.hold_row is not None:
            hit = self._button_hit(point.x, point.y)
            if hit != (self.hold_row, self.hold_delta):
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
            text_x, y, size, text, color=color
        )

    def _draw_button(
        self, target, row, delta, x, width, color
    ):
        y = ROW_Y[row]
        held = self.hold_row == row and self.hold_delta == delta
        background = YELLOW if held else color
        foreground = BLACK if held else WHITE
        target.draw_rectangle(
            x, y, width, ROW_H, color=background, fill=True
        )
        self._center_text(
            target,
            x,
            y + 10,
            width,
            "-" if delta < 0 else "+",
            28,
            foreground,
        )

    def draw(self, target, detection_count):
        target.draw_rectangle(
            0, 0, 432, 72, color=BLACK, fill=True
        )
        target.draw_string_advanced(
            8,
            10,
            20,
            "DET:%s CONF:%.2f NMS:%.2f"
            % (detection_count, self.confidence, self.nms),
            color=WHITE,
        )

        if not self.tuning:
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
                TUNE_Y + 13,
                TUNE_W,
                "TUNE",
                26,
                WHITE,
            )
            self._draw_touch_marker(target)
            return

        target.draw_rectangle(
            0, PANEL_Y, 640, 480 - PANEL_Y,
            color=BLACK, fill=True
        )
        target.draw_string_advanced(
            8, 260, 20, "YOLO THRESHOLD", color=WHITE
        )

        values = self.values()
        for row in range(2):
            y = ROW_Y[row]
            target.draw_string_advanced(
                8, y + 15, 18, LABELS[row], color=MUTED
            )
            self._draw_button(
                target, row, -1, MINUS_X, MINUS_W, BLUE
            )
            target.draw_rectangle(
                VALUE_X,
                y,
                VALUE_W,
                ROW_H,
                color=DARK,
                fill=True,
            )
            self._center_text(
                target,
                VALUE_X,
                y + 14,
                VALUE_W,
                "%.2f" % values[row],
                24,
                WHITE,
            )
            self._draw_button(
                target, row, 1, PLUS_X, PLUS_W, GREEN
            )

        target.draw_rectangle(
            SAVE_X, SAVE_Y, SAVE_W, SAVE_H,
            color=GREEN, fill=True
        )
        self._center_text(
            target,
            SAVE_X,
            SAVE_Y + 15,
            SAVE_W,
            "SAVE EXIT",
            24,
            WHITE,
        )
        target.draw_rectangle(
            RESET_X, RESET_Y, RESET_W, RESET_H,
            color=BLUE, fill=True
        )
        self._center_text(
            target,
            RESET_X,
            RESET_Y + 15,
            RESET_W,
            "RESET",
            24,
            WHITE,
        )
        self._draw_touch_marker(target)

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

    def close(self):
        try:
            self.touch.deinit()
        except Exception:
            pass


