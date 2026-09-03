"""High-rate traditional steel-ball tracker for Yahboom K230 (MOD-046).

The detector uses the official CanMV RGB565 ``image.find_blobs`` API:

1. periodically find the long bright pipe and cache its ROI;
2. find dark blobs only inside that ROI;
3. reject edge, size, aspect and fill-ratio mismatches;
4. select one top-scoring candidate and confirm it temporally;
5. draw/send only that target.

Human-readable metrics use USB serial ``print``. UART1 sends the existing
MSPM0 function-ID-16 position packet with frame interval in its second value.
"""

import gc
import os
import time
import ujson

from ball_stream_pipeline import BallStreamPipeline

try:
    from ball_threshold_ui import BallThresholdUI
    _TOUCH_IMPORT_ERROR = None
except Exception as touch_import_error:
    BallThresholdUI = None
    _TOUCH_IMPORT_ERROR = str(touch_import_error)

try:
    from ybUtils.YbUart import YbUart
    from yb_ascii_protocol import send_relative_x
    _UART_IMPORT_ERROR = None
except Exception as uart_import_error:
    YbUart = None
    send_relative_x = None
    _UART_IMPORT_ERROR = str(uart_import_error)


CONFIG_PATH = "/sdcard/traditional_ball_config.json"
CONFIG_TEMP_PATH = "/sdcard/traditional_ball_config.tmp.json"
CONFIG_BACKUP_PATH = "/sdcard/traditional_ball_config.bak.json"
DISPLAY_SIZE = (640, 480)
TARGET_FRAME_MS = 20
LOST_EVENT_FRAMES = 3

PIPE_COLOR = (255, 0, 220, 255)
CANDIDATE_COLOR = (255, 255, 200, 0)
TARGET_COLOR = (255, 0, 255, 0)
CENTER_COLOR = (255, 255, 0, 0)
TEXT_COLOR = (255, 255, 255, 255)

DEFAULT_CONFIG = {
    "analysis_width": 320,
    "analysis_height": 240,
    "sensor_fps": 60,
    "search_roi": [8, 12, 304, 216],
    "fallback_pipe_roi": [2, 40, 316, 38],
    "auto_pipe": False,
    "pipe_refresh_frames": 30,
    "pipe_hold_frames": 300,
    "pipe_l_min": 75,
    "pipe_min_pixels": 500,
    "pipe_min_area": 1400,
    "pipe_min_long": 100,
    "pipe_min_short": 18,
    "pipe_aspect_min_x100": 250,
    "pipe_merge_margin": 8,
    "pipe_inset": 5,
    "ball_l_min": 0,
    "ball_l_max": 72,
    "ball_min_pixels": 16,
    "ball_min_area": 25,
    "ball_merge": False,
    "ball_merge_margin": 2,
    "ball_min_size": 5,
    "ball_max_size": 28,
    "ball_aspect_min_x100": 60,
    "ball_fill_min_x100": 18,
    "ball_axis_tolerance": 12,
    "expected_diameter": 10,
    "minimum_score": 55,
    "maximum_jump": 30,
    "confirm_frames": 2,
    "smoothing_percent": 75,
    "uart_enabled": True,
    "uart_interval_ms": 50,
    "metric_interval_ms": 1000,
    "osd_every_n_frames": 3,
    "gc_interval_frames": 120,
    "test_scene": "MOVING",
}


def _bounded_int(value, default, low, high):
    try:
        value = int(value)
    except Exception:
        value = default
    return max(low, min(high, value))


def _bounded_roi(value, width, height, default):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        value = default
    try:
        x, y, w, h = [int(item) for item in value]
    except Exception:
        x, y, w, h = default
    x = max(0, min(width - 2, x))
    y = max(0, min(height - 2, y))
    w = max(2, min(width - x, w))
    h = max(2, min(height - y, h))
    return (x, y, w, h)


def load_config(path=CONFIG_PATH):
    config = dict(DEFAULT_CONFIG)
    try:
        with open(path, "r") as stream:
            saved = ujson.load(stream)
        if isinstance(saved, dict):
            config.update(saved)
    except OSError:
        print("#BALL_CONFIG default reason=file_missing path=%s" % path)
    except Exception as error:
        print("#BALL_CONFIG default reason=invalid detail=%s" % error)

    width = _bounded_int(
        config.get("analysis_width"), 320, 160, 640
    )
    width = ((width + 15) // 16) * 16
    height = _bounded_int(
        config.get("analysis_height"), 240, 120, 480
    )
    config["analysis_width"] = width
    config["analysis_height"] = height
    config["sensor_fps"] = _bounded_int(
        config.get("sensor_fps"), 60, 30, 60
    )
    config["search_roi"] = _bounded_roi(
        config.get("search_roi"),
        width,
        height,
        DEFAULT_CONFIG["search_roi"],
    )
    config["fallback_pipe_roi"] = _bounded_roi(
        config.get("fallback_pipe_roi"),
        width,
        height,
        DEFAULT_CONFIG["fallback_pipe_roi"],
    )
    config["auto_pipe"] = bool(
        config.get("auto_pipe", DEFAULT_CONFIG["auto_pipe"])
    )

    integer_limits = {
        "pipe_refresh_frames": (30, 1, 600),
        "pipe_hold_frames": (300, 1, 3600),
        "pipe_l_min": (75, 0, 100),
        "pipe_min_pixels": (500, 1, width * height),
        "pipe_min_area": (1400, 1, width * height),
        "pipe_min_long": (100, 2, max(width, height)),
        "pipe_min_short": (18, 2, min(width, height)),
        "pipe_aspect_min_x100": (250, 100, 1000),
        "pipe_merge_margin": (8, 0, 40),
        "pipe_inset": (5, 0, 40),
        "ball_l_min": (0, 0, 100),
        "ball_l_max": (72, 0, 100),
        "ball_min_pixels": (16, 1, width * height),
        "ball_min_area": (25, 1, width * height),
        "ball_merge_margin": (2, 0, 20),
        "ball_min_size": (5, 2, min(width, height)),
        "ball_max_size": (28, 3, max(width, height)),
        "ball_aspect_min_x100": (60, 10, 100),
        "ball_fill_min_x100": (18, 1, 100),
        "ball_axis_tolerance": (12, 1, min(width, height)),
        "expected_diameter": (10, 2, max(width, height)),
        "minimum_score": (55, 0, 100),
        "maximum_jump": (30, 1, max(width, height)),
        "confirm_frames": (2, 1, 10),
        "smoothing_percent": (75, 1, 100),
        "uart_interval_ms": (50, 10, 1000),
        "metric_interval_ms": (1000, 250, 10000),
        "osd_every_n_frames": (2, 1, 30),
        "gc_interval_frames": (120, 10, 1000),
    }
    for key, limits in integer_limits.items():
        config[key] = _bounded_int(
            config.get(key), limits[0], limits[1], limits[2]
        )
    if config["ball_l_min"] > config["ball_l_max"]:
        config["ball_l_min"], config["ball_l_max"] = (
            config["ball_l_max"],
            config["ball_l_min"],
        )
    if config["ball_min_size"] > config["ball_max_size"]:
        config["ball_min_size"], config["ball_max_size"] = (
            config["ball_max_size"],
            config["ball_min_size"],
        )
    config["ball_merge"] = bool(config.get("ball_merge", False))
    config["uart_enabled"] = bool(config.get("uart_enabled", True))
    scene = str(config.get("test_scene", "MOVING")).upper()
    if scene not in ("POS_STATIC", "NEG_EMPTY", "MOVING"):
        scene = "MOVING"
    config["test_scene"] = scene
    return config


def _remove_if_exists(path):
    try:
        os.remove(path)
    except OSError:
        pass


def save_config(config, path=CONFIG_PATH):
    """Atomically persist the current MOD-046 configuration."""
    with open(CONFIG_TEMP_PATH, "w") as stream:
        stream.write(ujson.dumps(config))

    had_previous = False
    try:
        os.stat(path)
        had_previous = True
    except OSError:
        pass

    if had_previous:
        _remove_if_exists(CONFIG_BACKUP_PATH)
        os.rename(path, CONFIG_BACKUP_PATH)
    try:
        os.rename(CONFIG_TEMP_PATH, path)
    except Exception:
        if had_previous:
            try:
                os.rename(CONFIG_BACKUP_PATH, path)
            except Exception:
                pass
        raise
    print(
        "#BALL_CONFIG saved path=%s ball_l_max=%s mod=MOD-046"
        % (path, config["ball_l_max"])
    )


def _blob_values(blob):
    """Return x, y, w, h, selected_pixels, cx, cy or None."""
    try:
        if hasattr(blob, "rect"):
            x, y, w, h = blob.rect()
        else:
            x, y, w, h = blob[0:4]
        pixels = int(blob[4])
        cx = int(blob[5])
        cy = int(blob[6])
        return (
            int(x),
            int(y),
            int(w),
            int(h),
            pixels,
            cx,
            cy,
        )
    except Exception:
        return None


def choose_pipe_roi(blobs, search_roi, config):
    """Select one long bright blob and return its inset global ROI."""
    best = None
    best_pixels = -1
    for blob in blobs:
        values = _blob_values(blob)
        if values is None:
            continue
        x, y, w, h, pixels, _, _ = values
        short_side = min(w, h)
        long_side = max(w, h)
        if (
            short_side < config["pipe_min_short"]
            or long_side < config["pipe_min_long"]
            or long_side * 100
            < short_side * config["pipe_aspect_min_x100"]
        ):
            continue
        if pixels > best_pixels:
            best = (x, y, w, h)
            best_pixels = pixels

    if best is None:
        return None
    x, y, w, h = best
    inset = min(
        config["pipe_inset"],
        max(0, w // 4 - 1),
        max(0, h // 4 - 1),
    )
    x += inset
    y += inset
    w -= inset * 2
    h -= inset * 2
    search_x, search_y, search_w, search_h = search_roi
    left = max(search_x, x)
    top = max(search_y, y)
    right = min(search_x + search_w, x + w)
    bottom = min(search_y + search_h, y + h)
    if right - left < 4 or bottom - top < 4:
        return None
    return (left, top, right - left, bottom - top)


def select_ball_candidate(blobs, roi, config, last_center=None):
    """Return (selected, raw_count, valid_count, axis_rejected, raw_probe).

    selected is ``(score, x, y, w, h, cx, cy, pixels, fill_x100)``.
    raw_probe is the closest-to-axis raw blob represented as
    ``(w, h, pixels, aspect_x100, fill_x100, cy)``.
    """
    try:
        raw_count = len(blobs)
    except Exception:
        raw_count = 0
    best = None
    valid_count = 0
    axis_rejected = 0
    raw_probe = None
    raw_probe_rank = None
    roi_axis_y = int(roi[1]) + int(roi[3]) // 2
    axis_tolerance = config["ball_axis_tolerance"]
    expected = config["expected_diameter"]
    maximum_jump = config["maximum_jump"]
    max_jump_sq = maximum_jump * maximum_jump

    for blob in blobs:
        values = _blob_values(blob)
        if values is None:
            continue
        x, y, w, h, pixels, _, _ = values
        if w <= 0 or h <= 0:
            continue
        short_side = min(w, h)
        long_side = max(w, h)
        aspect_x100 = short_side * 100 // max(1, long_side)
        area = w * h
        fill_x100 = min(100, pixels * 100 // max(1, area))
        cx = x + w // 2
        cy = y + h // 2
        diameter = (w + h) // 2
        probe_rank = (
            abs(cy - roi_axis_y),
            abs(diameter - expected),
            -pixels,
        )
        if raw_probe_rank is None or probe_rank < raw_probe_rank:
            raw_probe_rank = probe_rank
            raw_probe = (
                w,
                h,
                pixels,
                aspect_x100,
                fill_x100,
                cy,
            )
        # MOD-028 deliberately does not reject contact with either long pipe
        # wall. With a fixed ROI fitted to the real pipe, the steel-ball blob
        # is commonly clipped by that boundary. The size, aspect, fill and
        # Top-1 score gates below still reject long wall segments.
        if (
            short_side < config["ball_min_size"]
            or long_side > config["ball_max_size"]
        ):
            continue
        if aspect_x100 < config["ball_aspect_min_x100"]:
            continue
        if fill_x100 < config["ball_fill_min_x100"]:
            continue

        if abs(cy - roi_axis_y) > axis_tolerance:
            axis_rejected += 1
            continue
        size_score = max(
            0,
            100
            - abs(diameter - expected) * 100 // max(1, expected),
        )
        fill_score = min(100, fill_x100 * 100 // 55)
        motion_score = 50
        if last_center is not None:
            dx = cx - last_center[0]
            dy = cy - last_center[1]
            distance_sq = dx * dx + dy * dy
            if distance_sq >= max_jump_sq:
                # While locked, a far-away round screw/mark must not replace
                # the ball. _update_track clears last_center after LOST, at
                # which point global reacquisition is allowed again.
                continue
            motion_score = (
                100 - distance_sq * 100 // max(1, max_jump_sq)
            )
        score = (
            aspect_x100 * 45
            + fill_score * 20
            + size_score * 25
            + motion_score * 10
        ) // 100
        valid_count += 1
        candidate = (
            score,
            x,
            y,
            w,
            h,
            cx,
            cy,
            pixels,
            fill_x100,
        )
        if best is None or candidate[0] > best[0]:
            best = candidate

    if best is not None and best[0] < config["minimum_score"]:
        best = None
    return best, raw_count, valid_count, axis_rejected, raw_probe


def _scale_value(value, output_size, input_size):
    result = (int(value) * int(output_size) + input_size // 2) // input_size
    return max(0, min(output_size - 1, result))


def map_center(center, analysis_size, display_size):
    if center is None:
        return None
    return (
        _scale_value(center[0], display_size[0], analysis_size[0]),
        _scale_value(center[1], display_size[1], analysis_size[1]),
    )


def map_pipe_relative_x(center_x, pipe_roi, analysis_size, display_size):
    """Map ball X to signed display pixels around the pipe ROI midpoint."""
    center_x = int(center_x)
    roi_x, _, roi_w, _ = pipe_roi
    # Keep half-pixel ROI centres exact by performing the calculation doubled.
    delta_twice = center_x * 2 - (int(roi_x) * 2 + int(roi_w))
    numerator = delta_twice * int(display_size[0])
    denominator = max(1, int(analysis_size[0]) * 2)
    if numerator < 0:
        return -((-numerator + denominator // 2) // denominator)
    return (numerator + denominator // 2) // denominator


def map_rect(candidate, analysis_size, display_size):
    if candidate is None:
        return None
    x1 = _scale_value(candidate[1], display_size[0], analysis_size[0])
    y1 = _scale_value(candidate[2], display_size[1], analysis_size[1])
    x2 = _scale_value(
        candidate[1] + candidate[3],
        display_size[0],
        analysis_size[0],
    )
    y2 = _scale_value(
        candidate[2] + candidate[4],
        display_size[1],
        analysis_size[1],
    )
    return x1, y1, max(1, x2 - x1), max(1, y2 - y1)


class CvMetricWindow:
    """One-second, integer-only serial metrics with negligible log load."""

    def __init__(self, interval_ms, scene):
        self.interval_ms = interval_ms
        self.scene = scene
        self.sequence = 0
        self.last_frame_start = None
        self.current_miss_run = 0
        self.global_max_miss_run = 0
        self.reset(time.ticks_ms())

    def reset(self, now):
        self.window_start = now
        self.attempts = 0
        self.frames = 0
        self.frame_errors = 0
        self.detect_errors = 0
        self.hits = 0
        self.cv_calls = 0
        self.pipe_calls = 0
        self.pipe_sum = 0
        self.raw_sum = 0
        self.valid_sum = 0
        self.axis_reject_sum = 0
        self.raw_probe = None
        self.candidate_max = 0
        self.score_sum = 0
        self.relative_x_last = None
        self.relative_x_min = None
        self.relative_x_max = None
        self.raw_jitter_count = 0
        self.raw_jitter_sum = 0
        self.raw_jitter_max = 0
        self.output_jitter_count = 0
        self.output_jitter_sum = 0
        self.output_jitter_max = 0
        self.detect_ms_sum = 0
        self.detect_ms_max = 0
        self.work_ms_sum = 0
        self.work_ms_max = 0
        self.stream_ms_sum = 0
        self.stream_ms_max = 0
        self.misses = 0
        self.window_max_miss_run = 0
        self.lost_events = 0
        self.reacquired = 0
        self.late = 0
        self.skip_estimate = 0
        self.uart_sent = 0
        self.rtsp_sent = 0

    def _record_gap(self, frame_start):
        if self.last_frame_start is not None:
            gap = time.ticks_diff(frame_start, self.last_frame_start)
            if gap > 30:
                self.late += 1
            self.skip_estimate += max(
                0, (gap + TARGET_FRAME_MS // 2) // TARGET_FRAME_MS - 1
            )
        self.last_frame_start = frame_start

    def _record_miss(self):
        self.misses += 1
        self.current_miss_run += 1
        self.window_max_miss_run = max(
            self.window_max_miss_run, self.current_miss_run
        )
        self.global_max_miss_run = max(
            self.global_max_miss_run, self.current_miss_run
        )

    def record(
        self,
        frame_start,
        hit,
        pipe_call,
        pipe_count,
        raw_count,
        raw_probe,
        valid_count,
        axis_reject_count,
        score,
        relative_x,
        raw_jitter,
        output_jitter,
        detect_ms,
        work_ms,
        stream_ms,
        detect_error,
        lost_event,
        reacquired,
        uart_sent,
        rtsp_sent,
    ):
        self._record_gap(frame_start)
        self.attempts += 1
        self.frames += 1
        self.cv_calls += 1
        self.detect_errors += 1 if detect_error else 0
        if pipe_call:
            self.pipe_calls += 1
            self.pipe_sum += pipe_count
        self.raw_sum += raw_count
        if raw_probe is not None:
            self.raw_probe = raw_probe
        self.valid_sum += valid_count
        self.axis_reject_sum += axis_reject_count
        self.candidate_max = max(self.candidate_max, valid_count)
        self.detect_ms_sum += detect_ms
        self.detect_ms_max = max(self.detect_ms_max, detect_ms)
        self.work_ms_sum += work_ms
        self.work_ms_max = max(self.work_ms_max, work_ms)
        self.stream_ms_sum += stream_ms
        self.stream_ms_max = max(self.stream_ms_max, stream_ms)
        self.lost_events += 1 if lost_event else 0
        self.reacquired += 1 if reacquired else 0
        self.uart_sent += 1 if uart_sent else 0
        self.rtsp_sent += 1 if rtsp_sent else 0

        if hit:
            self.hits += 1
            self.score_sum += max(0, score)
            if relative_x is not None:
                relative_x = int(relative_x)
                self.relative_x_last = relative_x
                if self.relative_x_min is None:
                    self.relative_x_min = relative_x
                    self.relative_x_max = relative_x
                else:
                    self.relative_x_min = min(
                        self.relative_x_min, relative_x
                    )
                    self.relative_x_max = max(
                        self.relative_x_max, relative_x
                    )
            self.current_miss_run = 0
            if raw_jitter is not None:
                self.raw_jitter_count += 1
                self.raw_jitter_sum += raw_jitter
                self.raw_jitter_max = max(
                    self.raw_jitter_max, raw_jitter
                )
            if output_jitter is not None:
                self.output_jitter_count += 1
                self.output_jitter_sum += output_jitter
                self.output_jitter_max = max(
                    self.output_jitter_max, output_jitter
                )
        else:
            self._record_miss()

    def record_frame_error(
        self,
        frame_start,
        work_ms,
        stream_ms=0,
        lost_event=False,
        rtsp_sent=False,
    ):
        self._record_gap(frame_start)
        self.attempts += 1
        self.frame_errors += 1
        self.work_ms_sum += work_ms
        self.work_ms_max = max(self.work_ms_max, work_ms)
        self.stream_ms_sum += stream_ms
        self.stream_ms_max = max(self.stream_ms_max, stream_ms)
        self.lost_events += 1 if lost_event else 0
        self.rtsp_sent += 1 if rtsp_sent else 0
        self._record_miss()

    def report_if_due(self, now, pipe_roi, pipe_source):
        elapsed = time.ticks_diff(now, self.window_start)
        if elapsed < self.interval_ms:
            return False
        attempts = max(1, self.attempts)
        frames = max(1, self.frames)
        cv_calls = max(1, self.cv_calls)
        pipe_calls = max(1, self.pipe_calls)
        hits = max(1, self.hits)
        raw_jitter_count = max(1, self.raw_jitter_count)
        output_jitter_count = max(1, self.output_jitter_count)
        raw_probe_text = "NA"
        if self.raw_probe is not None:
            raw_probe_text = "%sx%s:%s:%s:%s:%s" % self.raw_probe
        self.sequence += 1
        print(
            "#CVSTAT,v=1,mod=MOD-046,scene=%s,seq=%s,win_ms=%s,"
            "try=%s,frm=%s,frame_err=%s,detect_err=%s,"
            "hit=%s,hit_pm=%s,cv_call=%s,pipe_call=%s,"
            "pipe_n10=%s,raw_n10=%s,raw_box=%s,"
            "gate_n10=%s,axis_n10=%s,"
            "gate_pm=%s,cand_max=%s,"
            "score10=%s,raw_jit_n=%s,raw_jit_avg10=%s,"
            "raw_jit_max=%s,out_jit_n=%s,out_jit_avg10=%s,"
            "out_jit_max=%s,det_avg_ms=%s,det_max_ms=%s,"
            "work_avg_ms=%s,work_max_ms=%s,stream_avg_ms=%s,"
            "stream_max_ms=%s,miss=%s,miss_run_win=%s,"
            "miss_streak=%s,miss_run_max=%s,lost_evt=%s,reacq=%s,"
            "late=%s,"
            "skip_est=%s,uart=%s,rtsp=%s,"
            "x_last=%s,x_min=%s,x_max=%s,fps10=%s,"
            "roi=%s:%s:%s:%s,pipe=%s"
            % (
                self.scene,
                self.sequence,
                elapsed,
                self.attempts,
                self.frames,
                self.frame_errors,
                self.detect_errors,
                self.hits,
                self.hits * 1000 // attempts,
                self.cv_calls,
                self.pipe_calls,
                self.pipe_sum * 10 // pipe_calls,
                self.raw_sum * 10 // cv_calls,
                raw_probe_text,
                self.valid_sum * 10 // cv_calls,
                self.axis_reject_sum * 10 // cv_calls,
                self.valid_sum * 1000 // max(1, self.raw_sum),
                self.candidate_max,
                self.score_sum * 10 // hits,
                self.raw_jitter_count,
                self.raw_jitter_sum * 10 // raw_jitter_count,
                self.raw_jitter_max,
                self.output_jitter_count,
                self.output_jitter_sum * 10 // output_jitter_count,
                self.output_jitter_max,
                self.detect_ms_sum // cv_calls,
                self.detect_ms_max,
                self.work_ms_sum // attempts,
                self.work_ms_max,
                self.stream_ms_sum // attempts,
                self.stream_ms_max,
                self.misses,
                self.window_max_miss_run,
                self.current_miss_run,
                self.global_max_miss_run,
                self.lost_events,
                self.reacquired,
                self.late,
                self.skip_estimate,
                self.uart_sent,
                self.rtsp_sent,
                "NA" if self.relative_x_last is None
                else self.relative_x_last,
                "NA" if self.relative_x_min is None
                else self.relative_x_min,
                "NA" if self.relative_x_max is None
                else self.relative_x_max,
                self.frames * 10000 // max(1, elapsed),
                pipe_roi[0],
                pipe_roi[1],
                pipe_roi[2],
                pipe_roi[3],
                pipe_source,
            )
        )
        self.reset(now)
        return True


class BallTracker:
    def __init__(self, config_path=CONFIG_PATH):
        self.config_path = config_path
        self.config = None
        self.pipeline = None
        self.threshold_ui = None
        self.uart = None
        self.clock = None
        self.metrics = None
        self.analysis_size = None
        self.display_size = None

        self.frame_count = 0
        self.frame_error_count = 0
        self.detect_error_count = 0
        self.uart_send_count = 0
        self.uart_error_count = 0
        self.last_uart_ms = None

        self.pipe_roi = None
        self.pipe_source = "FALLBACK"
        self.last_pipe_frame = -1000000
        self.last_candidate_center = None
        self.confirm_count = 0
        self.smooth_center = None
        self.last_raw_display_center = None
        self.last_display_center = None
        self.miss_streak = 0
        self.track_active = False
        self.ever_acquired = False
        self.tuning_was_active = False
        self.tuning_error_count = 0

    def start(self):
        if self.pipeline is not None:
            return
        self.config = load_config(self.config_path)
        self.analysis_size = (
            self.config["analysis_width"],
            self.config["analysis_height"],
        )
        self.pipe_roi = self.config["fallback_pipe_roi"]
        self.pipe_source = (
            "AUTO_WAIT" if self.config["auto_pipe"] else "FIXED"
        )
        self.pipeline = BallStreamPipeline(
            analysis_size=self.analysis_size,
            display_size=DISPLAY_SIZE,
        )
        try:
            self.pipeline.create(sensor_fps=self.config["sensor_fps"])
            self.display_size = tuple(self.pipeline.get_display_size())
            self._start_uart()
            self.clock = time.clock()
            self.metrics = CvMetricWindow(
                self.config["metric_interval_ms"],
                self.config["test_scene"],
            )
            if BallThresholdUI is not None:
                try:
                    self.threshold_ui = BallThresholdUI(
                        self.config["ball_l_max"],
                        DEFAULT_CONFIG["ball_l_max"],
                    )
                except Exception as error:
                    self.threshold_ui = None
                    print(
                        "#BALL_TOUCH disabled reason=init_error detail=%s"
                        % error
                    )
            else:
                print(
                    "#BALL_TOUCH disabled reason=import_error detail=%s"
                    % _TOUCH_IMPORT_ERROR
                )
        except BaseException:
            self.stop()
            raise

        print(
            "#CVCFG,v=1,mod=MOD-046,engine=rgb565_lab_blob,"
            "proc=%sx%s,disp=%sx%s,scene=%s,report_ms=%s,"
            "target_fps=50,jitter=raw_and_smoothed_L1_display,"
            "sensor_fps_req=%s,"
            "auto_pipe=%s,pipe_l=%s:100,ball_l=%s:%s,"
            "size=%s:%s,axis_tol=%s,merge=%s,margin=%s,"
            "score_min=%s,confirm=%s,smooth=%s,"
            "lost_after=%s,pipe_every=%s,osd_every=%s,gc_every=%s,"
            "uart_ms=%s,uart_x=pipe_center_signed"
            % (
                self.analysis_size[0],
                self.analysis_size[1],
                self.display_size[0],
                self.display_size[1],
                self.config["test_scene"],
                self.config["metric_interval_ms"],
                self.config["sensor_fps"],
                self.config["auto_pipe"],
                self.config["pipe_l_min"],
                self.config["ball_l_min"],
                self.config["ball_l_max"],
                self.config["ball_min_size"],
                self.config["ball_max_size"],
                self.config["ball_axis_tolerance"],
                "ON" if self.config["ball_merge"] else "OFF",
                self.config["ball_merge_margin"],
                self.config["minimum_score"],
                self.config["confirm_frames"],
                self.config["smoothing_percent"],
                LOST_EVENT_FRAMES,
                self.config["pipe_refresh_frames"],
                self.config["osd_every_n_frames"],
                self.config["gc_interval_frames"],
                self.config["uart_interval_ms"],
            )
        )
        print(
            "#CVGUIDE POS_STATIC=hit_pm+raw_jit/out_jit "
            "NEG_EMPTY=hit_is_false_positive "
            "MOVING=hit_pm+axis_n10+gate_pm+x_min:x_max+miss_run_win"
        )
        roi_display = map_rect(
            (
                0,
                self.pipe_roi[0],
                self.pipe_roi[1],
                self.pipe_roi[2],
                self.pipe_roi[3],
                0,
                0,
                0,
                0,
            ),
            self.analysis_size,
            self.display_size,
        )
        relative_left = map_pipe_relative_x(
            self.pipe_roi[0],
            self.pipe_roi,
            self.analysis_size,
            self.display_size,
        )
        relative_right = map_pipe_relative_x(
            self.pipe_roi[0] + self.pipe_roi[2],
            self.pipe_roi,
            self.analysis_size,
            self.display_size,
        )
        print(
            "#BALL_ROI_CAL mod=MOD-046 source=photo_20260730_fixed_inner "
            "roi_analysis=%s:%s:%s:%s roi_display=%s:%s:%s:%s "
            "origin_x=%s relative_range=%s:%s"
            % (
                self.pipe_roi[0],
                self.pipe_roi[1],
                self.pipe_roi[2],
                self.pipe_roi[3],
                roi_display[0],
                roi_display[1],
                roi_display[2],
                roi_display[3],
                roi_display[0] + roi_display[2] // 2,
                relative_left,
                relative_right,
            )
        )

    def _start_uart(self):
        if not self.config["uart_enabled"]:
            print("#BALL_UART disabled reason=configuration")
            return
        if YbUart is None or send_relative_x is None:
            print(
                "#BALL_UART disabled reason=import_error detail=%s"
                % _UART_IMPORT_ERROR
            )
            return
        try:
            uart = YbUart(baudrate=115200)
            if getattr(uart, "uart", None) is None:
                raise RuntimeError("YbUart did not create UART1")
            self.uart = uart
            print(
                "#BALL_UART ready baud=115200 pins=IO9_TX/IO10_RX "
                "protocol=$len,16,relative_x,frame_dt_ms,# "
                "origin=pipe_roi_long_axis_center unit=display_pixel"
            )
        except Exception as error:
            self.uart = None
            print("#BALL_UART disabled reason=init_error detail=%s" % error)

    def _set_pipe_roi(self, roi, source, candidate_count):
        changed = roi != self.pipe_roi or source != self.pipe_source
        self.pipe_roi = roi
        self.pipe_source = source
        if changed or self.frame_count == 0:
            print(
                "#BALL_PIPE source=%s frame=%s candidates=%s "
                "roi=%s:%s:%s:%s"
                % (
                    source,
                    self.frame_count,
                    candidate_count,
                    roi[0],
                    roi[1],
                    roi[2],
                    roi[3],
                )
            )

    def _refresh_pipe_roi(self, frame):
        if not self.config["auto_pipe"]:
            self._set_pipe_roi(
                self.config["fallback_pipe_roi"],
                "FIXED",
                0,
            )
            return False, 0
        due = (
            self.frame_count == 0
            or self.frame_count % self.config["pipe_refresh_frames"] == 0
        )
        if not due:
            return False, 0
        try:
            blobs = frame.find_blobs(
                [
                    (
                        self.config["pipe_l_min"],
                        100,
                        -128,
                        127,
                        -128,
                        127,
                    )
                ],
                roi=self.config["search_roi"],
                x_stride=4,
                y_stride=2,
                pixels_threshold=self.config["pipe_min_pixels"],
                area_threshold=self.config["pipe_min_area"],
                merge=True,
                margin=self.config["pipe_merge_margin"],
            )
            pipe_count = len(blobs)
            selected = choose_pipe_roi(
                blobs, self.config["search_roi"], self.config
            )
            if selected is not None:
                self._set_pipe_roi(selected, "AUTO", pipe_count)
                self.last_pipe_frame = self.frame_count
            elif (
                self.frame_count - self.last_pipe_frame
                > self.config["pipe_hold_frames"]
            ):
                self._set_pipe_roi(
                    self.config["fallback_pipe_roi"],
                    "FALLBACK",
                    pipe_count,
                )
            return True, pipe_count
        except Exception as error:
            self.detect_error_count += 1
            if (
                self.detect_error_count == 1
                or self.detect_error_count % 30 == 0
            ):
                print(
                    "#BALL pipe_detect_error count=%s detail=%s"
                    % (self.detect_error_count, error)
                )
            self._set_pipe_roi(
                self.config["fallback_pipe_roi"],
                "FALLBACK",
                0,
            )
            return True, 0

    def _detect_candidate(self, frame):
        pipe_call, pipe_count = self._refresh_pipe_roi(frame)
        try:
            blobs = frame.find_blobs(
                [
                    (
                        self.config["ball_l_min"],
                        self.config["ball_l_max"],
                        -128,
                        127,
                        -128,
                        127,
                    )
                ],
                roi=self.pipe_roi,
                x_stride=1,
                y_stride=1,
                pixels_threshold=self.config["ball_min_pixels"],
                area_threshold=self.config["ball_min_area"],
                merge=self.config["ball_merge"],
                margin=self.config["ball_merge_margin"],
            )
            (
                selected,
                raw_count,
                valid_count,
                axis_reject_count,
                raw_probe,
            ) = select_ball_candidate(
                blobs,
                self.pipe_roi,
                self.config,
                self.smooth_center,
            )
            return (
                selected,
                raw_count,
                raw_probe,
                valid_count,
                axis_reject_count,
                pipe_call,
                pipe_count,
            )
        except Exception as error:
            self.detect_error_count += 1
            if (
                self.detect_error_count == 1
                or self.detect_error_count % 30 == 0
            ):
                print(
                    "#BALL blob_detect_error count=%s detail=%s"
                    % (self.detect_error_count, error)
                )
            return None, 0, None, 0, 0, pipe_call, pipe_count

    def _register_track_miss(self, clear_candidate):
        lost_event = False
        if clear_candidate:
            self.confirm_count = 0
            self.last_candidate_center = None
        self.miss_streak += 1
        if (
            self.track_active
            and self.miss_streak >= LOST_EVENT_FRAMES
        ):
            self.track_active = False
            self.smooth_center = None
            lost_event = True
            print(
                "#BALL_STATE state=LOST frame=%s miss_frames=%s"
                % (self.frame_count, self.miss_streak)
            )
        return None, lost_event, False, None

    def _update_track(self, candidate):
        lost_event = False
        reacquired = False
        if candidate is None:
            return self._register_track_miss(True)

        raw_center = (candidate[5], candidate[6])
        same_track = False
        if self.last_candidate_center is not None:
            dx = raw_center[0] - self.last_candidate_center[0]
            dy = raw_center[1] - self.last_candidate_center[1]
            same_track = (
                dx * dx + dy * dy
                <= self.config["maximum_jump"]
                * self.config["maximum_jump"]
        )
        self.confirm_count = self.confirm_count + 1 if same_track else 1
        self.last_candidate_center = raw_center
        if self.confirm_count < self.config["confirm_frames"]:
            # A raw candidate is not a valid target until confirmation.
            return self._register_track_miss(False)

        self.miss_streak = 0
        previous_smooth = self.smooth_center
        if previous_smooth is None:
            self.smooth_center = raw_center
        else:
            alpha = self.config["smoothing_percent"]
            self.smooth_center = (
                (
                    raw_center[0] * alpha
                    + previous_smooth[0] * (100 - alpha)
                    + 50
                )
                // 100,
                (
                    raw_center[1] * alpha
                    + previous_smooth[1] * (100 - alpha)
                    + 50
                )
                // 100,
            )
        jitter = (
            abs(raw_center[0] - self.smooth_center[0])
            + abs(raw_center[1] - self.smooth_center[1])
        )
        if not self.track_active:
            reacquired = self.ever_acquired
            self.track_active = True
            self.ever_acquired = True
            print(
                "#BALL_STATE state=ACQUIRED frame=%s px=%s py=%s "
                "score=%s reacquired=%s"
                % (
                    self.frame_count,
                    self.smooth_center[0],
                    self.smooth_center[1],
                    candidate[0],
                    reacquired,
                )
            )
        return (
            (self.smooth_center[0], self.smooth_center[1], candidate[0]),
            lost_event,
            reacquired,
            jitter,
        )

    def _draw_osd(self, candidate, target):
        osd = self.pipeline.osd_img
        osd.clear()
        pipe_rect = map_rect(
            (
                0,
                self.pipe_roi[0],
                self.pipe_roi[1],
                self.pipe_roi[2],
                self.pipe_roi[3],
                0,
                0,
                0,
                0,
            ),
            self.analysis_size,
            self.display_size,
        )
        osd.draw_rectangle(
            pipe_rect[0],
            pipe_rect[1],
            pipe_rect[2],
            pipe_rect[3],
            color=PIPE_COLOR,
            thickness=2,
        )

        if candidate is not None:
            rect = map_rect(
                candidate, self.analysis_size, self.display_size
            )
            color = TARGET_COLOR if target is not None else CANDIDATE_COLOR
            osd.draw_rectangle(
                rect[0],
                rect[1],
                rect[2],
                rect[3],
                color=color,
                thickness=4 if target is not None else 2,
            )
        display_center = None
        if target is not None:
            display_center = map_center(
                target, self.analysis_size, self.display_size
            )
            osd.draw_circle(
                display_center[0],
                display_center[1],
                7,
                color=CENTER_COLOR,
                fill=True,
                thickness=3,
            )

        osd.draw_rectangle(
            0,
            0,
            self.display_size[0],
            64,
            color=(190, 0, 0, 0),
            fill=True,
        )
        score = target[2] if target is not None else (
            candidate[0] if candidate is not None else 0
        )
        fps = int(self.clock.fps()) if self.clock is not None else 0
        osd.draw_string_advanced(
            8,
            4,
            22,
            "BALL:%s FPS:%s SCORE:%s PIPE:%s"
            % (
                1 if target is not None else 0,
                fps,
                score,
                self.pipe_source,
            ),
            color=TEXT_COLOR,
        )
        relative_x = None
        if target is not None:
            relative_x = map_pipe_relative_x(
                target[0],
                self.pipe_roi,
                self.analysis_size,
                self.display_size,
            )
        osd.draw_string_advanced(
            8,
            34,
            18,
            "X:%s  L:%s-%s  RTSP:%s"
            % (
                "--" if relative_x is None else relative_x,
                self.config["ball_l_min"],
                self.config["ball_l_max"],
                self.pipeline.stream_short_status(),
            ),
            color=TEXT_COLOR,
        )
        if self.threshold_ui is not None:
            self.threshold_ui.draw_normal(osd)
        self.pipeline.show_osd()
        return display_center

    def _send_uart(self, relative_x, score):
        if self.uart is None or relative_x is None:
            return False
        now = time.ticks_ms()
        frame_dt_ms = 0
        if self.last_uart_ms is not None:
            frame_dt_ms = time.ticks_diff(now, self.last_uart_ms)
        if (
            self.last_uart_ms is not None
            and frame_dt_ms >= 0
            and frame_dt_ms < self.config["uart_interval_ms"]
        ):
            return False
        if frame_dt_ms < 0:
            frame_dt_ms = 0
        elif frame_dt_ms > 60000:
            frame_dt_ms = 60000
        try:
            packet = send_relative_x(
                self.uart, relative_x, frame_dt_ms
            )
            self.last_uart_ms = now
            self.uart_send_count += 1
            if self.uart_send_count == 1:
                print(
                    "#BALL_UART sent count=%s relative_x=%s dt_ms=%s "
                    "score=%s packet=%s"
                    % (
                        self.uart_send_count,
                        relative_x,
                        frame_dt_ms,
                        score,
                        packet,
                    )
                )
            return True
        except Exception as error:
            self.uart_error_count += 1
            if (
                self.uart_error_count == 1
                or self.uart_error_count % 20 == 0
            ):
                print(
                    "#BALL_UART send_error count=%s detail=%s"
                    % (self.uart_error_count, error)
                )
            return False

    def _reset_track_after_tuning(self):
        self.last_candidate_center = None
        self.confirm_count = 0
        self.smooth_center = None
        self.last_raw_display_center = None
        self.last_display_center = None
        self.miss_streak = 0
        self.track_active = False

    def _poll_threshold_ui(self):
        ui = self.threshold_ui
        if ui is None:
            return False
        ui.poll_touch()
        self.config["ball_l_max"] = ui.l_max
        if ui.consume_save_request():
            try:
                save_config(self.config, self.config_path)
            except Exception as error:
                print("#BALL_CONFIG save_error=%s" % error)
        return ui.tuning

    def _run_tuner_once(self):
        frame = self.pipeline.get_analysis_image()
        if frame is None:
            time.sleep_ms(10)
            return None
        try:
            tune_canvas = self.threshold_ui.render_tuner(frame)
            frame = None
            self.pipeline.show_tune_image(tune_canvas)
        except Exception as error:
            self.tuning_error_count += 1
            if (
                self.tuning_error_count == 1
                or self.tuning_error_count % 30 == 0
            ):
                print(
                    "#BALL_THRESHOLD render_error count=%s detail=%s"
                    % (self.tuning_error_count, error)
                )
        time.sleep_ms(10)
        return None

    def run_once(self):
        if self.pipeline is None:
            raise RuntimeError("BallTracker.start() must be called first")
        if self._poll_threshold_ui():
            if not self.tuning_was_active:
                self.tuning_was_active = True
                self._reset_track_after_tuning()
                print(
                    "#BALL_THRESHOLD paused=vision_uart_rtsp "
                    "binary_preview=True"
                )
            return self._run_tuner_once()
        if self.tuning_was_active:
            self.tuning_was_active = False
            self.metrics.last_frame_start = None
            self.metrics.reset(time.ticks_ms())
            print(
                "#BALL_THRESHOLD resumed=vision_uart_rtsp "
                "l_max=%s" % self.config["ball_l_max"]
            )
        frame_start = time.ticks_ms()
        frame = self.pipeline.get_analysis_image()
        if frame is None:
            self.frame_error_count += 1
            if (
                self.frame_error_count == 1
                or self.frame_error_count % 30 == 0
            ):
                print(
                    "#BALL frame_error count=%s"
                    % self.frame_error_count
                )
            _, lost_event, _, _ = self._update_track(None)
            self.last_raw_display_center = None
            self.last_display_center = None
            stream_start = time.ticks_ms()
            rtsp_sent = self.pipeline.send_stream_if_due()
            stream_ms = time.ticks_diff(time.ticks_ms(), stream_start)
            time.sleep_ms(2)
            self.frame_count += 1
            work_ms = time.ticks_diff(time.ticks_ms(), frame_start)
            self.metrics.record_frame_error(
                frame_start=frame_start,
                work_ms=work_ms,
                stream_ms=stream_ms,
                lost_event=lost_event,
                rtsp_sent=rtsp_sent,
            )
            self.metrics.report_if_due(
                time.ticks_ms(), self.pipe_roi, self.pipe_source
            )
            return None

        self.clock.tick()
        detect_error_before = self.detect_error_count
        (
            candidate,
            raw_count,
            raw_probe,
            valid_count,
            axis_reject_count,
            pipe_call,
            pipe_count,
        ) = self._detect_candidate(frame)
        target, lost_event, reacquired, _ = self._update_track(
            candidate
        )
        detect_error = self.detect_error_count > detect_error_before
        # Include the CH2 snapshot in traditional-vision latency.
        detect_ms = time.ticks_diff(time.ticks_ms(), frame_start)

        display_center = None
        if (
            self.frame_count
            % self.config["osd_every_n_frames"]
            == 0
        ):
            display_center = self._draw_osd(candidate, target)
        elif target is not None:
            display_center = map_center(
                target, self.analysis_size, self.display_size
            )

        uart_sent = False
        relative_x = None
        if target is not None:
            relative_x = map_pipe_relative_x(
                target[0],
                self.pipe_roi,
                self.analysis_size,
                self.display_size,
            )
            uart_sent = self._send_uart(relative_x, target[2])

        raw_jitter = None
        output_jitter = None
        if display_center is not None:
            raw_display_center = map_center(
                (candidate[5], candidate[6]),
                self.analysis_size,
                self.display_size,
            )
            if self.last_raw_display_center is not None:
                raw_jitter = (
                    abs(
                        raw_display_center[0]
                        - self.last_raw_display_center[0]
                    )
                    + abs(
                        raw_display_center[1]
                        - self.last_raw_display_center[1]
                    )
                )
            if self.last_display_center is not None:
                output_jitter = (
                    abs(display_center[0] - self.last_display_center[0])
                    + abs(display_center[1] - self.last_display_center[1])
                )
            self.last_raw_display_center = raw_display_center
            self.last_display_center = display_center
        else:
            # Do not count the reacquisition jump as frame-to-frame jitter.
            self.last_raw_display_center = None
            self.last_display_center = None

        # Release the CH2 buffer before acquiring a CH1 encoder frame.
        frame = None
        stream_start = time.ticks_ms()
        rtsp_sent = self.pipeline.send_stream_if_due()
        stream_ms = time.ticks_diff(time.ticks_ms(), stream_start)
        self.frame_count += 1
        if self.frame_count % self.config["gc_interval_frames"] == 0:
            gc.collect()
        work_ms = time.ticks_diff(time.ticks_ms(), frame_start)

        self.metrics.record(
            frame_start=frame_start,
            hit=target is not None,
            pipe_call=pipe_call,
            pipe_count=pipe_count,
            raw_count=raw_count,
            raw_probe=raw_probe,
            valid_count=valid_count,
            axis_reject_count=axis_reject_count,
            score=target[2] if target is not None else 0,
            relative_x=relative_x,
            raw_jitter=raw_jitter,
            output_jitter=output_jitter,
            detect_ms=detect_ms,
            work_ms=work_ms,
            stream_ms=stream_ms,
            detect_error=detect_error,
            lost_event=lost_event,
            reacquired=reacquired,
            uart_sent=uart_sent,
            rtsp_sent=rtsp_sent,
        )
        self.metrics.report_if_due(
            time.ticks_ms(), self.pipe_roi, self.pipe_source
        )
        return target

    def run_forever(self):
        self.start()
        try:
            while True:
                os.exitpoint()
                self.run_once()
        except BaseException as error:
            print("#BALL exit_request=%s" % error)
        finally:
            try:
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            except BaseException:
                pass
            self.stop()

    def stop(self):
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except BaseException:
            pass
        print("#BALL cleanup=start")
        uart = self.uart
        pipeline = self.pipeline
        threshold_ui = self.threshold_ui
        self.uart = None
        self.pipeline = None
        self.threshold_ui = None
        if uart is not None:
            try:
                uart.deinit()
            except BaseException as error:
                print("#BALL uart_cleanup_error=%s" % error)
        if threshold_ui is not None:
            try:
                threshold_ui.close()
            except BaseException as error:
                print("#BALL touch_cleanup_error=%s" % error)
        if pipeline is not None:
            try:
                pipeline.destroy()
            except BaseException as error:
                print("#BALL pipeline_cleanup_error=%s" % error)
        gc.collect()
        print("#BALL cleanup=done")


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    BallTracker().run_forever()


if __name__ == "__main__":
    main()


