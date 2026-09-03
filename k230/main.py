"""K230 competition launcher (ROUND-035 fast boot).

The normal task owns one three-channel camera pipeline:
  CH0 -> local LCD camera background + one traditional-vision ball box
  CH1 -> optional H.264 / RTSP, disabled by runtime_options by default
  CH2 -> 320x240 RGB565 LAB/blob tracking and MSPM0 UART

Flag priority:
  IDE_DIAGNOSTIC_MODE.flag -> wait for a temporary CanMV IDE script
  WIFI_AP_DIAGNOSTIC.flag  -> diagnostics only if wireless is enabled
  CAMERA_MODE.flag         -> standalone photo capture
  otherwise                -> traditional ball tracking + LCD + UART

WIRELESS_STREAM_ENABLED is the shared master switch. With it disabled, even
an old WIFI_AP_DIAGNOSTIC.flag cannot delay vision by starting the network.
WIFI_STREAM_MODE.flag is intentionally ignored.
"""

import gc
import os
import time
from runtime_options import WIRELESS_STREAM_ENABLED


DEFAULT_MODULE = "ball_tracker"
CAMERA_MODULE = "camera_capture"
WIFI_DIAGNOSTIC_MODULE = "wifi_ap_diagnostic"

IDE_DIAGNOSTIC_FLAG = "/sdcard/IDE_DIAGNOSTIC_MODE.flag"
WIFI_DIAGNOSTIC_FLAG = "/sdcard/WIFI_AP_DIAGNOSTIC.flag"
CAMERA_MODE_FLAG = "/sdcard/CAMERA_MODE.flag"
OLD_WIFI_FLAG = "/sdcard/WIFI_STREAM_MODE.flag"


def _flag_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def _wait_for_ide():
    print("#IDE_IDLE ready flag=%s mod=MOD-032" % IDE_DIAGNOSTIC_FLAG)
    try:
        while True:
            os.exitpoint()
            time.sleep_ms(20)
    finally:
        print("#IDE_IDLE released")


def main():
    if _flag_exists(IDE_DIAGNOSTIC_FLAG):
        _wait_for_ide()
        return

    if _flag_exists(OLD_WIFI_FLAG):
        print(
            "#LAUNCHER warning=obsolete_WIFI_STREAM_MODE_flag "
            "action=ignored mod=MOD-032"
        )

    wifi_diagnostic = _flag_exists(WIFI_DIAGNOSTIC_FLAG)
    if wifi_diagnostic and not WIRELESS_STREAM_ENABLED:
        print("#LAUNCHER wifi_diagnostic_flag=ignored reason=wireless_disabled")
    if wifi_diagnostic and WIRELESS_STREAM_ENABLED:
        module_name = WIFI_DIAGNOSTIC_MODULE
    elif _flag_exists(CAMERA_MODE_FLAG):
        module_name = CAMERA_MODULE
    else:
        module_name = DEFAULT_MODULE
    # The user's unmodified Yahboom AP example succeeds only when it runs
    # before Sensor/Display/Media imports. Preserve that startup order.
    if module_name == DEFAULT_MODULE and WIRELESS_STREAM_ENABLED:
        try:
            from wifi_bootstrap import start_ap
            start_ap()
        except BaseException as error:
            print("#AP_BOOTSTRAP failed detail=%s mod=MOD-032" % error)

    print("#LAUNCHER module=%s wireless=%s build=ROUND-035_FAST_BOOT"
          % (module_name, WIRELESS_STREAM_ENABLED))

    program = __import__(module_name)
    entry = getattr(program, "main", None)
    if entry is None:
        raise RuntimeError(module_name + ".main() is required")
    try:
        entry()
    finally:
        gc.collect()


if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    main()

