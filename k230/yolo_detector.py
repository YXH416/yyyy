"""Config-driven K230 YOLO + RTSP runtime (MOD-024).

This module is based on Yahboom/Canaan's deploy_det_video.py. It owns the
three-channel camera pipeline, Kmodel detector, LCD video layer, H.264 RTSP,
box OSD, threshold UI and the existing MSPM0 target-center UART transport.
"""

import gc
import os
import time

from libs.PlatTasks import DetectionApp
from libs.Utils import read_json
from vision_stream_pipeline import VisionStreamPipeline
from yolo_threshold import YoloThresholdUI

try:
    from ybUtils.YbUart import YbUart
    from yb_ascii_protocol import send_center
    _UART_IMPORT_ERROR = None
except Exception as uart_import_error:
    # Camera and YOLO must remain usable even if the optional transport file
    # was not copied.  The startup log identifies the missing dependency.
    YbUart = None
    send_center = None
    _UART_IMPORT_ERROR = str(uart_import_error)


MODEL_ROOT = "/sdcard/mp_deployment_source"
DISPLAY_MODE = "lcd"

# Keep the AI channel and the 640x480 Yahboom LCD at the same aspect ratio.
# DetectionApp can then map its boxes directly onto the bound camera image.
DISPLAY_SIZE = [640, 480]
RGB888P_SIZE = [640, 480]

GC_INTERVAL_FRAMES = 10
DEBUG_MODE = 0
UART_SEND_INTERVAL_MS = 50
BOX_COLOR = (255, 0, 255, 0)
DIAG_COLOR = (255, 255, 0, 255)
CENTER_COLOR = (255, 255, 0, 0)
STATUS_COLOR = (255, 255, 255, 255)


def _join_path(root, name):
    return root.rstrip("/") + "/" + name.lstrip("/")


def load_deploy_config(model_root=MODEL_ROOT):
    config_path = _join_path(model_root, "deploy_config.json")
    config = read_json(config_path)

    required = (
        "kmodel_path",
        "categories",
        "confidence_threshold",
        "nms_threshold",
        "img_size",
        "nms_option",
        "model_type",
    )
    for key in required:
        if key not in config:
            raise RuntimeError("deploy_config.json missing: " + key)

    model_path = _join_path(model_root, config["kmodel_path"])
    try:
        os.stat(model_path)
    except OSError:
        raise RuntimeError("Kmodel not found: " + model_path)

    anchors = []
    if config["model_type"] == "AnchorBaseDet":
        if "anchors" not in config or len(config["anchors"]) != 3:
            raise RuntimeError("AnchorBaseDet requires 3 anchor groups")
        anchors = (
            config["anchors"][0]
            + config["anchors"][1]
            + config["anchors"][2]
        )

    return config, model_path, anchors


def detection_count(result):
    if result is None:
        return 0
    try:
        return min(
            len(result["boxes"]),
            len(result["scores"]),
            len(result["idx"]),
        )
    except Exception:
        return 0


def select_best_detection(result, source_size, label_count):
    """Return one valid highest-score steel-ball detection.

    DetectionApp may return many low-confidence anchors. This single-object
    task must never draw every candidate because that caused MOD-023's green
    box flood and first-detection freeze.
    """
    count = detection_count(result)
    best = None
    for index in range(count):
        try:
            score = float(result["scores"][index])
            class_id = int(result["idx"][index])
            box = result["boxes"][index]
            x1 = max(0, min(source_size[0] - 1, int(box[0])))
            y1 = max(0, min(source_size[1] - 1, int(box[1])))
            x2 = max(0, min(source_size[0] - 1, int(box[2])))
            y2 = max(0, min(source_size[1] - 1, int(box[3])))
        except Exception:
            continue
        if score != score:
            continue
        if class_id < 0 or class_id >= label_count:
            continue
        if x2 <= x1 or y2 <= y1:
            continue
        candidate = (index, class_id, score, x1, y1, x2, y2)
        if best is None or score > best[2]:
            best = candidate
    return best


def best_detection_center(detection, source_size, display_size):
    """Map one selected detection center to display coordinates."""
    if detection is None:
        return None
    source_x = (detection[3] + detection[5]) // 2
    source_y = (detection[4] + detection[6]) // 2
    center_x = int(source_x * display_size[0] // source_size[0])
    center_y = int(source_y * display_size[1] // source_size[1])
    center_x = max(0, min(display_size[0] - 1, center_x))
    center_y = max(0, min(display_size[1] - 1, center_y))
    return center_x, center_y, detection[2], detection[0]


def draw_detection_boxes(
    target,
    detection,
    labels,
    source_size,
    display_size,
    color=BOX_COLOR,
):
    """Draw exactly one selected box on the local ARGB OSD."""
    if detection is None:
        return 0
    class_id = detection[1]
    score = detection[2]
    x1 = detection[3] * display_size[0] // source_size[0]
    y1 = detection[4] * display_size[1] // source_size[1]
    x2 = detection[5] * display_size[0] // source_size[0]
    y2 = detection[6] * display_size[1] // source_size[1]
    target.draw_rectangle(
        x1,
        y1,
        x2 - x1,
        y2 - y1,
        color=color,
        thickness=4,
    )
    label = labels[class_id]
    target.draw_string_advanced(
        x1,
        max(74, y1 - 26),
        22,
        "%s %.2f" % (label, score),
        color=color,
    )
    return 1


class YoloDetector:
    """Reusable owner of one K230 camera-to-YOLO-to-LCD pipeline."""

    def __init__(
        self,
        model_root=MODEL_ROOT,
        display_size=DISPLAY_SIZE,
        rgb888p_size=RGB888P_SIZE,
        debug_mode=DEBUG_MODE,
        diagnostic_mode=False,
        enable_uart=True,
    ):
        self.model_root = model_root
        self.display_size = list(display_size)
        self.rgb888p_size = list(rgb888p_size)
        self.debug_mode = debug_mode
        self.diagnostic_mode = diagnostic_mode
        self.enable_uart = enable_uart

        self.pipeline = None
        self.detector = None
        self.threshold_ui = None
        self.uart = None
        self.config = None
        self.labels = []
        self.frame_count = 0
        self.frame_error_count = 0
        self.uart_send_count = 0
        self.uart_error_count = 0
        self.last_uart_send_ms = None
        self.raw_shapes_logged = False
        self.last_result = None
        self.last_detection_count = 0
        self.last_raw_count = 0
        self.start_ms = None

    def start(self):
        """Load the deployment config/model and start camera plus LCD."""
        if self.pipeline is not None:
            return

        config, model_path, anchors = load_deploy_config(self.model_root)
        self.config = config
        self.labels = config["categories"]
        try:
            self.pipeline = VisionStreamPipeline(
                rgb888p_size=self.rgb888p_size,
                display_size=self.display_size,
                display_mode=DISPLAY_MODE,
                debug_mode=self.debug_mode,
            )
            self.pipeline.create()

            self.detector = DetectionApp(
                "video",
                model_path,
                config["categories"],
                config["img_size"],
                anchors,
                config["model_type"],
                config["confidence_threshold"],
                config["nms_threshold"],
                self.rgb888p_size,
                self.pipeline.get_display_size(),
                debug_mode=self.debug_mode,
            )
            # The stock DetectionApp defaults to class-wise NMS. Preserve the
            # generated deployment configuration for future replacement models.
            self.detector.nms_option = bool(config["nms_option"])
            self.detector.config_preprocess()
            self.threshold_ui = YoloThresholdUI(
                config["confidence_threshold"],
                config["nms_threshold"],
            )
            self._apply_thresholds()
            self._start_uart()
            self.start_ms = time.ticks_ms()
        except Exception:
            self.stop()
            raise

        print(
            "#YOLO ready model=%s labels=%s ai=%sx%s display=%sx%s "
            "confidence=%.2f nms=%.2f deploy_confidence=%.2f "
            "selection=top1 stream=%s mod=MOD-024"
            % (
                config["kmodel_path"],
                len(config["categories"]),
                self.rgb888p_size[0],
                self.rgb888p_size[1],
                self.pipeline.get_display_size()[0],
                self.pipeline.get_display_size()[1],
                self.detector.confidence_threshold,
                self.detector.nms_threshold,
                config["confidence_threshold"],
                self.pipeline.streamer.active,
            )
        )

    def _start_uart(self):
        if not self.enable_uart:
            print("#YOLO_UART disabled reason=configuration")
            return
        if YbUart is None or send_center is None:
            print(
                "#YOLO_UART disabled reason=import_error detail=%s"
                % _UART_IMPORT_ERROR
            )
            return
        try:
            uart = YbUart(baudrate=115200)
            if getattr(uart, "uart", None) is None:
                raise RuntimeError("YbUart did not create UART1")
            self.uart = uart
            print(
                "#YOLO_UART ready baud=115200 pins=IO9_TX/IO10_RX "
                "protocol=TARGET_CENTER interval_ms=%s"
                % UART_SEND_INTERVAL_MS
            )
        except Exception as error:
            self.uart = None
            print("#YOLO_UART disabled reason=init_error detail=%s" % error)

    def _apply_thresholds(self):
        if self.detector is None or self.threshold_ui is None:
            return
        confidence, nms = self.threshold_ui.values()
        self.detector.confidence_threshold = confidence
        self.detector.nms_threshold = nms

    def set_thresholds(self, confidence, nms):
        """Set runtime thresholds without writing the persistent profile."""
        if self.threshold_ui is None:
            raise RuntimeError("YoloDetector.start() must be called first")
        self.threshold_ui.set_values(confidence, nms)
        self._apply_thresholds()

    def show_osd_self_test(self):
        """Draw a fixed box before inference to prove the OSD path works."""
        if self.pipeline is None or self.threshold_ui is None:
            raise RuntimeError("YoloDetector.start() must be called first")
        osd = self.pipeline.osd_img
        osd.clear()
        osd.draw_rectangle(
            80, 100, 480, 260, color=DIAG_COLOR, thickness=6
        )
        osd.draw_string_advanced(
            190, 190, 36, "OSD TEST", color=DIAG_COLOR
        )
        self.threshold_ui.draw(osd, 0)
        self.pipeline.show_image()
        print("#YOLO_DIAG osd_test=shown")

    def _log_raw_shapes(self):
        if (
            not self.diagnostic_mode
            or self.raw_shapes_logged
            or self.detector is None
        ):
            return
        shapes = []
        for output in self.detector.results:
            try:
                shapes.append(str(output.shape))
            except Exception:
                shapes.append("unknown")
        print("#YOLO_DIAG raw_output_shapes=%s" % shapes)
        self.raw_shapes_logged = True

    def _send_target_center(self, target):
        if self.uart is None or target is None:
            return
        now = time.ticks_ms()
        if (
            self.last_uart_send_ms is not None
            and time.ticks_diff(now, self.last_uart_send_ms)
            < UART_SEND_INTERVAL_MS
        ):
            return
        try:
            if self.uart_send_count == 0 and self.uart_error_count == 0:
                print(
                    "#YOLO_UART first_send_begin x=%s y=%s score=%.3f"
                    % (target[0], target[1], target[2])
                )
            packet = send_center(self.uart, target[0], target[1])
            self.last_uart_send_ms = now
            self.uart_send_count += 1
            if (
                self.uart_send_count == 1
                or self.uart_send_count % 20 == 0
            ):
                print(
                    "#YOLO_UART sent count=%s x=%s y=%s "
                    "score=%.3f packet=%s"
                    % (
                        self.uart_send_count,
                        target[0],
                        target[1],
                        target[2],
                        packet,
                    )
                )
        except Exception as error:
            self.uart_error_count += 1
            if (
                self.uart_error_count == 1
                or self.uart_error_count % 20 == 0
            ):
                print(
                    "#YOLO_UART send_error count=%s detail=%s"
                    % (self.uart_error_count, error)
                )

    def _stream_camera_frame(self):
        """Send a clean CH1 frame without modifying the YUV buffer.

        The local LCD keeps the selected YOLO box. Drawing every candidate
        and advanced text directly onto CH1 doubled the work and caused the
        first-detection freeze in MOD-023.
        """
        frame = self.pipeline.get_stream_frame()
        if frame is None:
            return
        try:
            self.pipeline.send_stream_frame(frame)
        except BaseException as error:
            print("#RTSP raw_frame_error=%s" % error)
        finally:
            frame = None

    def _poll_thresholds(self):
        self.threshold_ui.poll_touch()
        self._apply_thresholds()
        return self.threshold_ui.tuning

    def _service_tuning_ui(self):
        """Pause KPU/RTSP and service the touch editor at about 50 Hz."""
        osd = self.pipeline.osd_img
        osd.clear()
        self.threshold_ui.draw(osd, self.last_detection_count)
        osd.draw_string_advanced(
            8,
            228,
            16,
            "TUNING: AI/RTSP PAUSED",
            color=STATUS_COLOR,
        )
        self.pipeline.show_image()
        time.sleep_ms(20)
        return self.last_result

    def infer_once(self):
        """Run one frame and return DetectionApp's boxes/scores/class IDs."""
        if self.pipeline is None or self.detector is None:
            raise RuntimeError("YoloDetector.start() must be called first")

        if self._poll_thresholds():
            return self._service_tuning_ui()

        frame = self.pipeline.get_frame()
        if frame is None:
            self.frame_error_count += 1
            if (
                self.frame_error_count == 1
                or self.frame_error_count % 30 == 0
            ):
                print(
                    "#YOLO frame_error count=%s sensor_frame=None"
                    % self.frame_error_count
                )
            return None

        # A second sample bounds the normal touch interval around snapshot.
        if self._poll_thresholds():
            return self._service_tuning_ui()

        result = self.detector.run(frame)
        self._log_raw_shapes()
        raw_count = detection_count(result)
        selected = select_best_detection(
            result,
            self.rgb888p_size,
            len(self.labels),
        )
        selected_score = selected[2] if selected is not None else -1.0
        if (
            raw_count > 1
            and (
                self.frame_count < 3
                or self.frame_count % 30 == 0
            )
        ):
            # This is intentionally before any OSD/UART/RTSP work. If the
            # device later stalls, the last line identifies candidate load.
            print(
                "#YOLO_CANDIDATES raw=%s selected=%s top_score=%.3f "
                "stage=before_draw"
                % (
                    raw_count,
                    1 if selected is not None else 0,
                    selected_score,
                )
            )

        # A third sample occurs immediately after the blocking KPU call.
        if self._poll_thresholds():
            self.last_result = result
            self.last_raw_count = raw_count
            self.last_detection_count = 1 if selected is not None else 0
            return self._service_tuning_ui()

        # Clear only the transparent OSD; the camera VIDEO1 layer remains.
        osd = self.pipeline.osd_img
        osd.clear()
        count = draw_detection_boxes(
            osd,
            selected,
            self.labels,
            self.rgb888p_size,
            self.pipeline.get_display_size(),
        )
        target = best_detection_center(
            selected,
            self.rgb888p_size,
            self.pipeline.get_display_size(),
        )
        if target is not None:
            osd.draw_circle(
                target[0],
                target[1],
                7,
                color=CENTER_COLOR,
                fill=True,
                thickness=3,
            )
        if self.diagnostic_mode:
            osd.draw_rectangle(
                8, 72, 110, 70, color=DIAG_COLOR, thickness=4
            )
            osd.draw_string_advanced(
                18, 92, 20, "OSD OK", color=DIAG_COLOR
            )
        self.threshold_ui.draw(osd, count)
        osd.draw_string_advanced(
            8,
            self.pipeline.get_display_size()[1] - 22,
            16,
            self.pipeline.stream_status(),
            color=STATUS_COLOR,
        )
        self.pipeline.show_image()

        self.last_result = result
        self.last_raw_count = raw_count
        self.last_detection_count = count
        self.frame_count += 1

        # Let TUNE take ownership before UART or the blocking encoder path.
        if self._poll_thresholds():
            return self._service_tuning_ui()

        if target is not None:
            self._send_target_center(target)
        self._stream_camera_frame()

        # Sample once more after GetStream returns. Tuning mode then pauses
        # all KPU/RTSP work and runs this loop every 20 ms.
        self._poll_thresholds()

        if (
            self.frame_count == 1
            or self.frame_count % (30 if self.diagnostic_mode else 60) == 0
        ):
            print(
                "#YOLO_RUN frame=%s raw=%s selected=%s top_score=%.3f "
                "confidence=%.2f nms=%.2f rtsp_frames=%s"
                % (
                    self.frame_count,
                    raw_count,
                    count,
                    selected_score,
                    self.detector.confidence_threshold,
                    self.detector.nms_threshold,
                    self.pipeline.streamer.frame_count,
                )
            )
        if self.frame_count % GC_INTERVAL_FRAMES == 0:
            gc.collect()
        return result

    def run_forever(self):
        """Start the runtime and keep detecting until IDE interruption."""
        self.start()
        try:
            while True:
                os.exitpoint()
                self.infer_once()
        except BaseException as error:
            print("#YOLO exit_request=%s" % error)
        finally:
            # Official K230 AI examples disable further IDE interrupts before
            # deinitializing KPU/Sensor/Display/Media.
            try:
                os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
            except BaseException:
                pass
            self.stop()

    def stop(self):
        """Release KPU first, then stop the camera/display pipeline."""
        # This must happen before DetectionApp.deinit(). AIBase.deinit() runs
        # GC, shrinks the KPU pool and sleeps; leaving EXITPOINT_ENABLE active
        # lets the IDE interrupt that cleanup a second time.
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except BaseException:
            pass
        print("#YOLO cleanup=start")

        detector = self.detector
        pipeline = self.pipeline
        threshold_ui = self.threshold_ui
        uart = self.uart
        self.detector = None
        self.pipeline = None
        self.threshold_ui = None
        self.uart = None
        self.last_result = None
        self.last_detection_count = 0
        self.last_raw_count = 0

        if uart is not None:
            try:
                uart.deinit()
            except BaseException as error:
                print("#YOLO uart_cleanup_error=%s" % error)
        if threshold_ui is not None:
            try:
                threshold_ui.close()
            except BaseException as error:
                print("#YOLO touch_cleanup_error=%s" % error)
        if detector is not None:
            try:
                detector.deinit()
            except BaseException as error:
                print("#YOLO detector_cleanup_error=%s" % error)
        if pipeline is not None:
            try:
                pipeline.destroy()
            except BaseException as error:
                print("#YOLO pipeline_cleanup_error=%s" % error)

        detector = None
        pipeline = None
        threshold_ui = None
        uart = None
        try:
            gc.collect()
        except BaseException as error:
            print("#YOLO gc_cleanup_error=%s" % error)
        print("#YOLO cleanup=done")


def main():
    # Official Yahboom interactive examples enable exit points before loops.
    # This lets CanMV IDE stop main.py and reach the finally cleanup.
    os.exitpoint(os.EXITPOINT_ENABLE)
    YoloDetector().run_forever()


if __name__ == "__main__":
    main()


