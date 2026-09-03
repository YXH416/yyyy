"""Shared LCD + YOLO + RTSP camera pipeline for Yahboom K230 (MOD-023)."""

import gc
import os
import time

import image
import nncase_runtime as nn
from media.display import *
from media.media import *
from media.sensor import *

try:
    from wifi_rtsp import WifiRtsp
    _STREAM_IMPORT_ERROR = None
except BaseException as stream_import_error:
    WifiRtsp = None
    _STREAM_IMPORT_ERROR = str(stream_import_error)


class _NullStreamer:
    """Keep LCD, KPU and UART alive when networking is unavailable."""

    width = 640
    height = 480
    active = False
    network_ready = False
    prepared = False
    frame_count = 0

    def connect_network(self):
        print("#RTSP disabled reason=import_error:%s" % _STREAM_IMPORT_ERROR)
        return False

    def acquire_image(self, sensor):
        return None

    def send_image(self, frame):
        return False

    def status_text(self):
        return "RTSP ERR"

    def service_diagnostics(self):
        return False

    def diagnostic_lines(self):
        return [
            "MOD-023 WIFI IMPORT ERROR",
            str(_STREAM_IMPORT_ERROR)[:58],
        ]

    def stop(self):
        pass


class VisionStreamPipeline:
    """Own one Sensor and split it across display, RTSP and inference."""

    def __init__(
        self,
        rgb888p_size=(640, 480),
        display_size=(640, 480),
        display_mode="lcd",
        osd_layer_num=1,
        debug_mode=0,
    ):
        self.rgb888p_size = [
            ALIGN_UP(rgb888p_size[0], 16),
            rgb888p_size[1],
        ]
        self.display_size = list(display_size)
        self.display_mode = display_mode
        self.osd_layer_num = osd_layer_num
        self.debug_mode = debug_mode
        self.sensor = None
        self.osd_img = None
        self.streamer = (
            WifiRtsp()
            if WifiRtsp is not None
            else _NullStreamer()
        )
        self.media_initialized = False
        self.display_initialized = False
        self.sensor_running = False

    def create(self, sensor=None, hmirror=None, vflip=None, fps=30):
        nn.shrink_memory_pool()
        self.streamer.connect_network()
        try:
            self.sensor = Sensor(fps=fps) if sensor is None else sensor
            self.sensor.reset()
            if hmirror is not None:
                self.sensor.set_hmirror(bool(hmirror))
            if vflip is not None:
                self.sensor.set_vflip(bool(vflip))

            # CH0: hardware-bound camera background for the local LCD.
            self.sensor.set_framesize(
                width=self.display_size[0],
                height=self.display_size[1],
                chn=CAM_CHN_ID_0,
            )
            self.sensor.set_pixformat(
                Sensor.YUV420SP,
                chn=CAM_CHN_ID_0,
            )

            # CH1: H.264 RTSP input. Failure disables only wireless output.
            if self.streamer.network_ready:
                self.streamer.configure_sensor_channel(self.sensor)

            # CH2: RGB planar input for KPU/AI2D.
            self.sensor.set_framesize(
                width=self.rgb888p_size[0],
                height=self.rgb888p_size[1],
                chn=CAM_CHN_ID_2,
            )
            self.sensor.set_pixformat(
                PIXEL_FORMAT_RGB_888_PLANAR,
                chn=CAM_CHN_ID_2,
            )

            # Canaan's RTSP example reserves all VENC output buffers before
            # MediaManager.init(). Keep this before Display/Media allocation
            # so the combined LCD + KPU + VENC pool is deterministic.
            if self.streamer.network_ready:
                self.streamer.reserve_encoder_buffers()

            self.osd_img = image.Image(
                self.display_size[0],
                self.display_size[1],
                image.ARGB8888,
            )
            bind_info = self.sensor.bind_info(
                x=0,
                y=0,
                chn=CAM_CHN_ID_0,
            )
            Display.bind_layer(
                **bind_info,
                layer=Display.LAYER_VIDEO1
            )
            if self.display_mode == "virt":
                Display.init(
                    Display.VIRT,
                    width=self.display_size[0],
                    height=self.display_size[1],
                    osd_num=self.osd_layer_num,
                    to_ide=True,
                )
            else:
                # Offline competition mode avoids the IDE JPEG path.
                Display.init(
                    Display.ST7701,
                    width=self.display_size[0],
                    height=self.display_size[1],
                    osd_num=self.osd_layer_num,
                    to_ide=False,
                )
            self.display_initialized = True
            self.display_size = [Display.width(), Display.height()]

            MediaManager.init()
            self.media_initialized = True
            if self.streamer.prepared:
                self.streamer.start_media()

            self.sensor.run()
            self.sensor_running = True
            self.show_startup_status()
            print(
                "#PIPELINE ready display=%sx%s ai=%sx%s stream=%s "
                "mod=MOD-023"
                % (
                    self.display_size[0],
                    self.display_size[1],
                    self.rgb888p_size[0],
                    self.rgb888p_size[1],
                    self.streamer.active,
                )
            )
        except BaseException:
            self.destroy()
            raise

    def get_frame(self):
        self.streamer.service_diagnostics()
        try:
            frame = self.sensor.snapshot(chn=CAM_CHN_ID_2)
            return frame.to_numpy_ref()
        except Exception as error:
            print("#PIPELINE ai_frame_error=%s" % error)
            return None

    def get_display_size(self):
        return self.display_size

    def show_image(self):
        if self.osd_img is None:
            return None
        try:
            Display.show_image(
                self.osd_img,
                0,
                0,
                Display.LAYER_OSD3,
            )
            return self.osd_img
        except Exception as error:
            print("#PIPELINE osd_error=%s" % error)
            return None

    def get_stream_frame(self):
        if self.sensor is None:
            return None
        return self.streamer.acquire_image(self.sensor)

    def send_stream_frame(self, frame):
        return self.streamer.send_image(frame)

    def stream_status(self):
        return self.streamer.status_text()

    def show_startup_status(self):
        """Keep the network result visible while the Kmodel is loading."""
        if self.osd_img is None:
            return
        try:
            self.osd_img.clear()
            self.osd_img.draw_rectangle(
                0,
                0,
                self.display_size[0],
                160,
                color=(230, 0, 0, 0),
                fill=True,
            )
            lines = self.streamer.diagnostic_lines()
            for index in range(min(6, len(lines))):
                self.osd_img.draw_string_advanced(
                    12,
                    8 + index * 25,
                    20,
                    lines[index],
                    color=(255, 255, 255, 255),
                )
            self.show_image()
            time.sleep_ms(600)
        except Exception as error:
            print("#PIPELINE startup_status_error=%s" % error)

    def destroy(self):
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except BaseException:
            pass

        if self.sensor is not None and self.sensor_running:
            try:
                self.sensor.stop()
            except BaseException as error:
                print("#PIPELINE sensor_stop_error=%s" % error)
        self.sensor_running = False

        try:
            self.streamer.stop()
        except BaseException as error:
            print("#PIPELINE stream_cleanup_error=%s" % error)

        if self.display_initialized:
            try:
                Display.deinit()
            except BaseException as error:
                print("#PIPELINE display_cleanup_error=%s" % error)
        self.display_initialized = False
        time.sleep_ms(50)

        if self.media_initialized:
            try:
                MediaManager.deinit()
            except BaseException as error:
                print("#PIPELINE media_cleanup_error=%s" % error)
        self.media_initialized = False
        self.sensor = None
        self.osd_img = None
        gc.collect()


