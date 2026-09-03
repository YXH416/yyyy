"""LCD + traditional vision with optional RTSP (ROUND-035 fast boot).

Channel ownership:
  CH0: 640x480 YUV420SP -> LCD VIDEO1 background
  CH1: optional H.264 / RTSP; not configured in fast-boot mode
  CH2: 320x240 RGB565   -> traditional image.find_blobs()

The initialization order follows the working Yahboom/Canaan media examples:
configure Sensor, reserve VENC buffers, initialize Display/Media, start
Encoder/RTSP, then run Sensor.
"""

import gc
import os
import time
from runtime_options import WIRELESS_STREAM_ENABLED

import image
from media.display import *
from media.media import *
from media.sensor import *

class _NullStreamer:
    width = 512
    height = 288
    active = False
    network_ready = False
    prepared = False
    frame_count = 0

    def __init__(self, reason="fast_boot"):
        self.reason = reason

    def connect_network(self):
        print("#RTSP disabled reason=%s" % self.reason)
        return False

    def configure_sensor_channel(self, sensor):
        return False

    def reserve_encoder_buffers(self):
        return False

    def start_media(self):
        return False

    def acquire_image(self, sensor):
        return None

    def send_image(self, frame):
        return False

    def status_text(self):
        return "RTSP OFF" if self.reason == "fast_boot" else "RTSP ERR"

    def service_diagnostics(self):
        return False

    def diagnostic_lines(self):
        return [
            self.status_text(),
            self.reason[:58],
        ]

    def stop(self):
        pass


def _create_streamer():
    # Avoid importing the networking, video encoder and RTSP dependencies at
    # all when local vision is requested, even if SD Wi-Fi config says enabled.
    if not WIRELESS_STREAM_ENABLED:
        return _NullStreamer()
    try:
        from wifi_rtsp import WifiRtsp
        return WifiRtsp()
    except BaseException as error:
        return _NullStreamer("import_error:%s" % error)


class BallStreamPipeline:
    """Own the only Sensor, Display, MediaManager and RTSP instances."""

    def __init__(
        self,
        analysis_size=(320, 240),
        display_size=(640, 480),
        display_mode="lcd",
        osd_layer_num=2,
    ):
        self.analysis_size = [
            ALIGN_UP(int(analysis_size[0]), 16),
            int(analysis_size[1]),
        ]
        self.display_size = [
            int(display_size[0]),
            int(display_size[1]),
        ]
        self.display_mode = display_mode
        # MOD-032 retains the MOD-027 ARGB backdrop plus RGB565 tuner layer.
        self.osd_layer_num = max(2, int(osd_layer_num))
        self.sensor = None
        self.osd_img = None
        self.tune_backdrop = None
        self.tune_backdrop_visible = False
        self.streamer = _create_streamer()
        self.media_initialized = False
        self.display_initialized = False
        self.sensor_running = False
        self.stream_channel_ready = False

    def create(self, sensor=None, sensor_fps=60, hmirror=None, vflip=None):
        self.streamer.connect_network()
        try:
            if sensor is None:
                # GC2093 officially pairs 1280x960 with up to 60 FPS.
                self.sensor = Sensor(
                    id=2,
                    width=1280,
                    height=960,
                    fps=int(sensor_fps),
                )
            else:
                self.sensor = sensor
            self.sensor.reset()
            if hmirror is not None:
                self.sensor.set_hmirror(bool(hmirror))
            if vflip is not None:
                self.sensor.set_vflip(bool(vflip))

            self.sensor.set_framesize(
                width=self.display_size[0],
                height=self.display_size[1],
                chn=CAM_CHN_ID_0,
            )
            self.sensor.set_pixformat(
                Sensor.YUV420SP,
                chn=CAM_CHN_ID_0,
            )

            if self.streamer.network_ready:
                self.stream_channel_ready = bool(
                    self.streamer.configure_sensor_channel(self.sensor)
                )

            self.sensor.set_framesize(
                width=self.analysis_size[0],
                height=self.analysis_size[1],
                chn=CAM_CHN_ID_2,
            )
            self.sensor.set_pixformat(
                Sensor.RGB565,
                chn=CAM_CHN_ID_2,
            )

            if self.stream_channel_ready:
                self.streamer.reserve_encoder_buffers()

            self.osd_img = image.Image(
                self.display_size[0],
                self.display_size[1],
                image.ARGB8888,
            )
            self.tune_backdrop = image.Image(
                self.display_size[0],
                self.display_size[1],
                image.ARGB8888,
            )
            self.tune_backdrop.clear()
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
            self._show_startup_status()
            print(
                "#BALL_PIPELINE ready display=%sx%s analysis=%sx%s "
                "format=RGB565 requested_sensor_fps=%s stream=%s "
                    "build=ROUND-035_FAST_BOOT"
                % (
                    self.display_size[0],
                    self.display_size[1],
                    self.analysis_size[0],
                    self.analysis_size[1],
                    sensor_fps,
                    self.streamer.active,
                )
            )
        except BaseException:
            self.destroy()
            raise

    def get_analysis_image(self):
        self.streamer.service_diagnostics()
        try:
            frame = self.sensor.snapshot(
                chn=CAM_CHN_ID_2,
                timeout=100,
            )
            if frame is None or frame == -1:
                return None
            return frame
        except Exception as error:
            print("#BALL_PIPELINE analysis_frame_error=%s" % error)
            return None

    def get_display_size(self):
        return self.display_size

    def _set_tune_backdrop(self, visible):
        """Show or clear the opaque black layer below the RGB565 tuner."""
        if self.tune_backdrop is None:
            return False
        visible = bool(visible)
        if self.tune_backdrop_visible == visible:
            return True
        try:
            if visible:
                self.tune_backdrop.draw_rectangle(
                    0,
                    0,
                    self.display_size[0],
                    self.display_size[1],
                    color=(255, 0, 0, 0),
                    fill=True,
                )
            else:
                self.tune_backdrop.clear()
            Display.show_image(
                self.tune_backdrop,
                0,
                0,
                Display.LAYER_OSD2,
                alpha=255,
            )
            self.tune_backdrop_visible = visible
            return True
        except Exception as error:
            print("#BALL_PIPELINE tune_backdrop_error=%s" % error)
            return False

    def show_osd(self):
        if self.osd_img is None:
            return False
        try:
            self._set_tune_backdrop(False)
            Display.show_image(
                self.osd_img,
                0,
                0,
                Display.LAYER_OSD3,
                alpha=255,
            )
            return True
        except Exception as error:
            print("#BALL_PIPELINE osd_error=%s" % error)
            return False

    def show_tune_image(self, tune_image):
        if tune_image is None:
            return False
        try:
            backdrop_ready = self._set_tune_backdrop(True)
            Display.show_image(
                tune_image,
                0,
                0,
                Display.LAYER_OSD3,
                alpha=255,
            )
            return backdrop_ready
        except Exception as error:
            print("#BALL_PIPELINE tune_display_error=%s" % error)
            return False

    def send_stream_if_due(self):
        if self.sensor is None:
            return False
        frame = self.streamer.acquire_image(self.sensor)
        if frame is None:
            return False
        try:
            return bool(self.streamer.send_image(frame))
        finally:
            frame = None

    def stream_status(self):
        return self.streamer.status_text()

    def stream_short_status(self):
        return "ON" if self.streamer.active else "OFF"

    def _show_startup_status(self):
        # Skip the RTSP diagnostic splash and its 500 ms pause in local mode.
        if not WIRELESS_STREAM_ENABLED or self.osd_img is None:
            return
        try:
            self.osd_img.clear()
            self.osd_img.draw_rectangle(
                0,
                0,
                self.display_size[0],
                150,
                color=(230, 0, 0, 0),
                fill=True,
            )
            self.osd_img.draw_string_advanced(
                12,
                8,
                24,
                "MOD-032 TRADITIONAL BALL",
                color=(255, 0, 255, 255),
            )
            lines = self.streamer.diagnostic_lines()
            for index in range(min(4, len(lines))):
                self.osd_img.draw_string_advanced(
                    12,
                    40 + index * 24,
                    18,
                    lines[index],
                    color=(255, 255, 255, 255),
                )
            self.show_osd()
            time.sleep_ms(500)
        except Exception as error:
            print("#BALL_PIPELINE startup_osd_error=%s" % error)

    def destroy(self):
        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except BaseException:
            pass

        if self.sensor is not None:
            try:
                self.sensor.stop()
            except BaseException as error:
                print("#BALL_PIPELINE sensor_stop_error=%s" % error)
        self.sensor_running = False

        try:
            self.streamer.stop()
        except BaseException as error:
            print("#BALL_PIPELINE stream_cleanup_error=%s" % error)

        if self.display_initialized:
            try:
                Display.deinit()
            except BaseException as error:
                print("#BALL_PIPELINE display_cleanup_error=%s" % error)
        self.display_initialized = False
        time.sleep_ms(50)

        if self.media_initialized:
            try:
                MediaManager.deinit()
            except BaseException as error:
                print("#BALL_PIPELINE media_cleanup_error=%s" % error)
        self.media_initialized = False
        self.stream_channel_ready = False
        self.sensor = None
        self.osd_img = None
        self.tune_backdrop = None
        self.tune_backdrop_visible = False
        gc.collect()

