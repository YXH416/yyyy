"""Fail-open Wi-Fi H.264/RTSP transport for CanMV K230 (MOD-025).

This module does not own Sensor, Display or MediaManager. The caller uses:
connect_network(), configure_sensor_channel(), reserve_encoder_buffers(),
MediaManager.init(), start_media(), then sensor.run().

Network startup state is written to the SD card and repeated on the USB serial
console after 30 seconds, so a monitor reconnected after a reboot can still
obtain the full AP/RTSP state.
"""

import gc
import os
import time
import uctypes
import ujson

import multimedia as mm
import network
from media.sensor import *
from media.vencoder import *


CONFIG_PATH = "/sdcard/wifi_stream_config.json"
STATUS_LOG_PATH = "/sdcard/wifi_stream_status.log"
SERIAL_REPORT_DELAY_MS = 30000
SERIAL_REPORT_INTERVAL_MS = 30000
DEFAULT_CONFIG = {
    "enabled": True,
    "wifi_mode": "AP",
    "ssid": "YAHBOOM-K230",
    "password": "12345678",
    "connect_timeout_s": 15,
    "stream_width": 512,
    "stream_height": 288,
    "stream_fps": 10,
    "bitrate_kbps": 600,
    "rtsp_port": 8554,
    "rtsp_session": "video",
}


def _bounded_int(value, default, low, high):
    try:
        value = int(value)
    except Exception:
        value = default
    return max(low, min(high, value))


def _safe_text(value, limit=160):
    try:
        text = str(value)
    except Exception:
        text = "unprintable"
    text = text.replace("\r", " ").replace("\n", " ")
    if len(text) > limit:
        return text[:limit]
    return text


def load_stream_config(path=CONFIG_PATH):
    config = dict(DEFAULT_CONFIG)
    try:
        with open(path, "r") as stream:
            saved = ujson.load(stream)
        if isinstance(saved, dict):
            config.update(saved)
    except OSError:
        print("#RTSP config=default reason=file_missing path=%s" % path)
    except Exception as error:
        print("#RTSP config=default reason=invalid detail=%s" % error)

    config["enabled"] = bool(config.get("enabled", True))
    mode = str(config.get("wifi_mode", "AP")).upper()
    config["wifi_mode"] = mode if mode in ("AP", "STA") else "AP"
    config["ssid"] = str(config.get("ssid", "YAHBOOM-K230"))
    config["password"] = str(config.get("password", "12345678"))
    # WPA AP passwords require at least eight characters. STA mode keeps an
    # empty password valid so an open test network can be used as a fallback.
    if config["wifi_mode"] == "AP" and len(config["password"]) < 8:
        config["password"] = "12345678"
    config["connect_timeout_s"] = _bounded_int(
        config.get("connect_timeout_s"), 15, 3, 60
    )
    config["stream_width"] = _bounded_int(
        config.get("stream_width"), 512, 320, 1280
    )
    config["stream_width"] = ((config["stream_width"] + 15) // 16) * 16
    config["stream_height"] = _bounded_int(
        config.get("stream_height"), 288, 180, 720
    )
    if config["stream_height"] & 1:
        config["stream_height"] += 1
    config["stream_fps"] = _bounded_int(
        config.get("stream_fps"), 10, 3, 25
    )
    config["bitrate_kbps"] = _bounded_int(
        config.get("bitrate_kbps"), 600, 200, 4000
    )
    config["rtsp_port"] = _bounded_int(
        config.get("rtsp_port"), 8554, 1024, 65535
    )
    session = str(config.get("rtsp_session", "video"))
    config["rtsp_session"] = session if session else "video"
    return config


class WifiRtsp:
    """H.264 RTSP output that disables itself on error, not the vision task."""

    def __init__(self, config_path=CONFIG_PATH):
        self.config = load_stream_config(config_path)
        self.enabled = self.config["enabled"]
        self.network_ready = False
        self.prepared = False
        self.active = False
        self.error = ""
        self.ip = "0.0.0.0"
        self.url = ""
        self.nic = None
        self.encoder = None
        self.rtsp = None
        self.encoder_created = False
        self.encoder_started = False
        self.venc_chn = VENC_CHN_ID_0
        self.last_frame_ms = None
        self.frame_count = 0
        self.send_error_count = 0
        self.snapshot_error_count = 0
        self.boot_ms = time.ticks_ms()
        self.last_serial_report_ms = None
        self.ap_active = False
        self.ap_status = "not_started"
        self.ap_config_ssid = ""
        self.ap_config_channel = "OFFICIAL_DEFAULT"
        self.ap_ip_source = "bootstrap"
        self.client_count = 0
        self.status_log_path = STATUS_LOG_PATH
        self._reset_status_log()

    def _reset_status_log(self):
        try:
            with open(self.status_log_path, "w") as stream:
                stream.write(
                    "# K230 Wi-Fi startup log MOD-025\n"
                    "# Password is intentionally not recorded.\n"
                )
        except Exception as error:
            print("#WIFI log_reset_error=%s" % _safe_text(error))
        self._log_status(
            "boot",
            "mode=%s ssid=%s ap_config_api=ssid+key enabled=%s"
            % (
                self.config["wifi_mode"],
                self.config["ssid"],
                self.enabled,
            ),
        )

    def _log_status(self, stage, detail=""):
        line = "#WIFI_LOG t_ms=%s stage=%s detail=%s" % (
            time.ticks_diff(time.ticks_ms(), self.boot_ms),
            _safe_text(stage, 40),
            _safe_text(detail),
        )
        print(line)
        try:
            with open(self.status_log_path, "a") as stream:
                stream.write(line + "\n")
        except Exception as error:
            print("#WIFI log_write_error=%s" % _safe_text(error))

    @property
    def width(self):
        return self.config["stream_width"]

    @property
    def height(self):
        return self.config["stream_height"]

    @property
    def fps(self):
        return self.config["stream_fps"]

    def _disable(self, reason):
        self.error = _safe_text(reason)
        self.active = False
        print("#RTSP disabled reason=%s" % self.error)
        self._log_status("disabled", self.error)
        return False

    def _read_ap_diagnostics(self):
        if self.nic is None:
            return
        try:
            self.ap_active = bool(self.nic.active())
        except Exception as error:
            self.ap_active = False
            self.ap_status = "active_error:%s" % _safe_text(error, 80)
        try:
            self.ap_status = _safe_text(self.nic.status(), 80)
        except Exception as error:
            self.ap_status = "status_error:%s" % _safe_text(error, 60)
        try:
            stations = self.nic.status("stations")
            self.client_count = len(stations) if stations is not None else 0
        except Exception:
            self.client_count = -1

    def _read_interface_ip(self, nic):
        info = nic.ifconfig()
        if info is None or len(info) < 1:
            return "0.0.0.0"
        return str(info[0])

    def connect_network(self):
        if not self.enabled:
            return self._disable("configuration")
        try:
            if self.config["wifi_mode"] == "AP":
                # main.py starts this exact Yahboom AP bootstrap before any
                # YOLO/Display/Media imports. Direct module runs fall back to
                # the same bootstrap here.
                from wifi_bootstrap import start_ap
                nic = start_ap()
                self.nic = nic
                self.ap_config_ssid = "YAHBOOM-K230"
                self.ip = self._read_interface_ip(nic)
                self._read_ap_diagnostics()
                self._log_status(
                    "ap_bootstrap",
                    "reused=True active=%s ip=%s"
                    % (self.ap_active, self.ip),
                )
                if not self.ap_active:
                    return self._disable("ap_not_active")
                if not self.ip or self.ip == "0.0.0.0":
                    return self._disable("ap_no_ip")
                self.network_ready = True
                self._log_status(
                    "wifi_ready",
                    "mode=AP configured_ssid=%s channel=AUTO "
                    "link_status=%s ip=%s ip_source=bootstrap clients=%s"
                    % (
                        self.ap_config_ssid,
                        self.ap_status,
                        self.ip,
                        self.client_count,
                    ),
                )
                print(
                    "#WIFI ready mode=AP ssid=%s ip=%s channel=OFFICIAL_DEFAULT"
                    % (
                        self.config["ssid"],
                        self.ip,
                    )
                )
                return True

            nic = network.WLAN(network.STA_IF)
            self.nic = nic
            self._log_status("sta_object", "created=True")
            if not nic.active():
                nic.active(True)
            try:
                nic.config(auto_reconnect=True)
            except Exception:
                pass
            if not nic.isconnected():
                nic.connect(self.config["ssid"], self.config["password"])
            start_ms = time.ticks_ms()
            timeout_ms = self.config["connect_timeout_s"] * 1000
            while not nic.isconnected():
                os.exitpoint()
                if time.ticks_diff(time.ticks_ms(), start_ms) >= timeout_ms:
                    return self._disable("wifi_timeout")
                time.sleep_ms(200)
            self.nic = nic
            self.ip = nic.ifconfig()[0]
            self.network_ready = True
            self._log_status(
                "wifi_ready",
                "mode=STA ssid=%s ip=%s status=%s"
                % (self.config["ssid"], self.ip, nic.status()),
            )
            print(
                "#WIFI ready mode=STA ssid=%s ip=%s"
                % (self.config["ssid"], self.ip)
            )
            return True
        except BaseException as error:
            return self._disable("wifi_error:%s" % error)

    def configure_sensor_channel(self, sensor):
        if not self.network_ready:
            return False
        try:
            sensor.set_framesize(
                width=self.width,
                height=self.height,
                chn=CAM_CHN_ID_1,
                alignment=12,
            )
            sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_1)
            return True
        except Exception as error:
            return self._disable("sensor_channel:%s" % error)

    def reserve_encoder_buffers(self):
        """Reserve VENC buffers before MediaManager.init()."""
        if not self.network_ready:
            return False
        try:
            self.encoder = Encoder()
            self.encoder.SetOutBufs(
                self.venc_chn, 8, self.width, self.height
            )
            self.prepared = True
            self._log_status(
                "encoder_buffers",
                "reserved=True size=%sx%s"
                % (self.width, self.height),
            )
            return True
        except Exception as error:
            self.encoder = None
            return self._disable("encoder_buffers:%s" % error)

    def start_media(self):
        """Create VENC first, then RTSP, following Canaan's example."""
        if not self.prepared:
            return False
        try:
            attr = ChnAttrStr(
                self.encoder.PAYLOAD_TYPE_H264,
                self.encoder.H264_PROFILE_MAIN,
                self.width,
                self.height,
                bit_rate=self.config["bitrate_kbps"],
                dst_frame_rate=self.fps,
                # Frames are manually submitted at stream_fps. Matching the
                # two rates avoids VENC dropping a submitted frame while the
                # following blocking GetStream waits for another one.
                src_frame_rate=self.fps,
            )
            self.encoder.Create(self.venc_chn, attr)
            self.encoder_created = True
            self._log_status(
                "encoder_created",
                "size=%sx%s fps=%s"
                % (self.width, self.height, self.fps),
            )

            # Official rtsp_no_ai.py creates the encoder channel before it
            # initializes and starts the RTSP server. The previous reversed
            # order could leave the LCD/YOLO alive while port 8554 was never
            # listening.
            self.rtsp = mm.rtsp_server()
            self.rtsp.rtspserver_init(self.config["rtsp_port"])
            self.rtsp.rtspserver_createsession(
                self.config["rtsp_session"],
                mm.multi_media_type.media_h264,
                False,
            )
            self.rtsp.rtspserver_start()
            self._log_status(
                "rtsp_listening",
                "port=%s session=%s"
                % (
                    self.config["rtsp_port"],
                    self.config["rtsp_session"],
                ),
            )

            self.encoder.Start(self.venc_chn)
            self.encoder_started = True
            # Use the verified AP address instead of a possibly stale URL
            # reported by the RTSP object during early startup.
            self.url = "rtsp://%s:%s/%s" % (
                self.ip,
                self.config["rtsp_port"],
                self.config["rtsp_session"],
            )
            self.active = True
            self._log_status(
                "rtsp_ready",
                "url=%s fps=%s bitrate_kbps=%s"
                % (
                    self.url,
                    self.fps,
                    self.config["bitrate_kbps"],
                ),
            )
            print(
                "#RTSP ready url=%s size=%sx%s fps=%s bitrate_kbps=%s "
                "mod=MOD-025"
                % (
                    self.url,
                    self.width,
                    self.height,
                    self.fps,
                    self.config["bitrate_kbps"],
                )
            )
            return True
        except BaseException as error:
            self.stop_media()
            return self._disable("media_start:%s" % error)

    def due(self):
        if not self.active:
            return False
        now = time.ticks_ms()
        interval_ms = max(1, 1000 // self.fps)
        if (
            self.last_frame_ms is not None
            and time.ticks_diff(now, self.last_frame_ms) < interval_ms
        ):
            return False
        self.last_frame_ms = now
        return True

    def acquire_image(self, sensor):
        if not self.due():
            return None
        try:
            frame = sensor.snapshot(
                chn=CAM_CHN_ID_1,
                timeout=100,
            )
            if frame is None or frame == -1:
                return self._snapshot_failure("empty_frame")
            self.snapshot_error_count = 0
            return frame
        except Exception as error:
            return self._snapshot_failure(error)

    def _snapshot_failure(self, detail):
        self.snapshot_error_count += 1
        if (
            self.snapshot_error_count == 1
            or self.snapshot_error_count % 5 == 0
        ):
            print(
                "#RTSP snapshot_error count=%s detail=%s"
                % (self.snapshot_error_count, _safe_text(detail))
            )
        if self.snapshot_error_count >= 10:
            count = self.snapshot_error_count
            reason = "snapshot_repeated:%s:%s" % (
                count,
                _safe_text(detail, 80),
            )
            self.stop_media()
            self._disable(reason)
        return None

    def _uv_offset(self, width, height):
        pixels = width * height
        # MOD-025 keeps the official no-AI example's 512x288 size, whose
        # Y and UV planes are contiguous. This avoids the contradictory
        # 640x480 padding branches found in different bundled demos.
        if width == 1920 and height == 1080:
            return pixels + 3072
        if width == 640 and height == 360:
            return pixels + 3072
        return pixels

    def send_image(self, frame):
        if not self.active or frame is None:
            return False

        frame_info = k_video_frame_info()
        frame_info.v_frame.width = frame.width()
        frame_info.v_frame.height = frame.height()
        frame_info.v_frame.pixel_format = Sensor.YUV420SP
        frame_info.pool_id = frame.poolid()
        frame_info.v_frame.phys_addr[0] = frame.phyaddr()
        frame_info.v_frame.phys_addr[1] = (
            frame_info.v_frame.phys_addr[0]
            + self._uv_offset(frame.width(), frame.height())
        )

        stream_data = StreamData()
        stream_acquired = False
        try:
            result = self.encoder.SendFrame(self.venc_chn, frame_info)
            if result not in (None, 0):
                raise RuntimeError("SendFrame=%s" % result)
            # Firmware v1.4 defaults to an infinite wait. A finite timeout
            # prevents one encoder fault from stopping vision and UART forever.
            result = self.encoder.GetStream(
                self.venc_chn,
                stream_data,
                50,
            )
            if result not in (None, 0):
                raise RuntimeError("GetStream=%s" % result)
            stream_acquired = True
            timestamp = self.frame_count * 1000 // self.fps
            for index in range(stream_data.pack_cnt):
                size = stream_data.data_size[index]
                packet = bytes(
                    uctypes.bytearray_at(stream_data.data[index], size)
                )
                self.rtsp.rtspserver_sendvideodata(
                    self.config["rtsp_session"],
                    packet,
                    size,
                    timestamp,
                )
            self.frame_count += 1
            self.send_error_count = 0
            if self.frame_count == 1 or self.frame_count % 120 == 0:
                print("#RTSP frame=%s url=%s" % (self.frame_count, self.url))
            return True
        except BaseException as error:
            self.send_error_count += 1
            if self.send_error_count == 1 or self.send_error_count % 20 == 0:
                print(
                    "#RTSP send_error count=%s detail=%s"
                    % (self.send_error_count, error)
                )
            if self.send_error_count >= 10:
                self.active = False
                self._log_status(
                    "rtsp_send_disabled",
                    "count=%s detail=%s"
                    % (self.send_error_count, _safe_text(error)),
                )
                print(
                    "#RTSP disabled reason=repeated_send_error "
                    "vision_continues=True"
                )
            return False
        finally:
            if stream_acquired:
                try:
                    self.encoder.ReleaseStream(
                        self.venc_chn, stream_data
                    )
                except Exception as error:
                    print("#RTSP release_error=%s" % error)
            if self.frame_count and self.frame_count % 120 == 0:
                gc.collect()

    def service_diagnostics(self):
        """Repeat a compact report after 30 s so USB reconnects can see it."""
        now = time.ticks_ms()
        if self.last_serial_report_ms is None:
            if time.ticks_diff(now, self.boot_ms) < SERIAL_REPORT_DELAY_MS:
                return False
        elif (
            time.ticks_diff(now, self.last_serial_report_ms)
            < SERIAL_REPORT_INTERVAL_MS
        ):
            return False

        self.last_serial_report_ms = now
        if self.config["wifi_mode"] == "AP" and self.nic is not None:
            self._read_ap_diagnostics()
        print(
            "#WIFI_REPORT mod=MOD-025 elapsed_s=%s enabled=%s mode=%s "
            "ssid=%s ap_active=%s link_status=%s configured_ssid=%s "
            "channel=%s ip=%s ip_source=%s clients=%s network_ready=%s "
            "rtsp_active=%s frames=%s error=%s"
            % (
                time.ticks_diff(now, self.boot_ms) // 1000,
                self.enabled,
                self.config["wifi_mode"],
                self.config["ssid"],
                self.ap_active,
                self.ap_status,
                self.ap_config_ssid,
                self.ap_config_channel,
                self.ip,
                self.ap_ip_source,
                self.client_count,
                self.network_ready,
                self.active,
                self.frame_count,
                self.error if self.error else "none",
            )
        )
        print("#WIFI_REPORT log=%s url=%s" % (self.status_log_path, self.url))
        return True

    def status_text(self):
        if self.active:
            if self.config["wifi_mode"] == "AP":
                return "AP %s  %s:%s/%s" % (
                    self.config["ssid"],
                    self.ip,
                    self.config["rtsp_port"],
                    self.config["rtsp_session"],
                )
            return "STA %s:%s/%s" % (
                self.ip,
                self.config["rtsp_port"],
                self.config["rtsp_session"],
            )
        if not self.enabled:
            return "RTSP OFF"
        if self.network_ready:
            return "RTSP ERR %s" % _safe_text(self.error, 42)
        return "WIFI ERR %s" % _safe_text(self.error, 42)

    def diagnostic_lines(self, rtsp_expected=True):
        lines = [
            "MOD-025 WIFI %s" % self.config["wifi_mode"],
            "SSID: %s" % self.config["ssid"],
        ]
        if self.network_ready:
            if self.config["wifi_mode"] == "AP":
                lines.append(
                    "IP: %s  %s" % (self.ip, self.ap_ip_source.upper())
                )
                lines.append(
                    "AP:%s CLIENTS:%s"
                    % (self.ap_active, self.client_count)
                )
            else:
                lines.append("STA READY  IP: %s" % self.ip)
            if rtsp_expected:
                lines.append(
                    "RTSP: %s"
                    % (self.url if self.active else "starting")
                )
            else:
                lines.append("NETWORK TEST ONLY")
        else:
            lines.append("NETWORK FAILED")
            lines.append(_safe_text(self.error, 58))
        lines.append("LOG: /sdcard/wifi_stream_status.log")
        return lines

    def stop_media(self):
        self.active = False
        if self.encoder is not None and self.encoder_started:
            try:
                self.encoder.Stop(self.venc_chn)
            except BaseException as error:
                print("#RTSP encoder_stop_error=%s" % error)
        self.encoder_started = False
        if self.encoder is not None and self.encoder_created:
            try:
                self.encoder.Destroy(self.venc_chn)
            except BaseException as error:
                print("#RTSP encoder_destroy_error=%s" % error)
        self.encoder_created = False
        self.encoder = None

        if self.rtsp is not None:
            try:
                self.rtsp.rtspserver_stop()
            except BaseException:
                pass
            try:
                self.rtsp.rtspserver_deinit()
            except BaseException as error:
                print("#RTSP server_deinit_error=%s" % error)
        self.rtsp = None
        self.prepared = False

    def stop_network(self):
        if self.nic is None:
            return
        try:
            if self.config["wifi_mode"] == "STA":
                self.nic.disconnect()
            else:
                try:
                    self.nic.stop()
                except Exception:
                    self.nic.active(False)
        except BaseException as error:
            print("#WIFI cleanup_error=%s" % error)
        self.nic = None
        self.network_ready = False

    def stop(self):
        self.stop_media()
        self.stop_network()


