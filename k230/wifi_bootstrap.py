"""Yahboom K230 AP bootstrap that runs before all multimedia imports (MOD-022).

The body of start_ap() intentionally follows the user's verified Yahboom
course example after a boot grace period: AP_IF -> active(True) ->
config(ssid, key) -> sleep(3) -> ifconfig(). Do not add Display, MediaManager,
Sensor, VENC or RTSP imports here.
"""

import network
import time


AP_SSID = "YAHBOOM-K230"
AP_KEY = "12345678"
BOOT_WAIT_SECONDS = 15

_ap = None
_ip_info = ("0.0.0.0", "0.0.0.0", "0.0.0.0", "0.0.0.0")


def start_ap():
    global _ap, _ip_info
    if _ap is not None:
        return _ap

    # A manual IDE run succeeds after the board has settled, while immediate
    # main.py startup returns 0.0.0.0. Reproduce that timing before AP_IF.
    print(
        "#AP_BOOTSTRAP boot_wait=%ss mod=MOD-022"
        % BOOT_WAIT_SECONDS
    )
    time.sleep(BOOT_WAIT_SECONDS)

    ap = network.WLAN(network.AP_IF)
    if not ap.active():
        ap.active(True)
    print("#AP_BOOTSTRAP active=%s mod=MOD-022" % ap.active())

    ap.config(ssid=AP_SSID, key=AP_KEY)
    print("#AP_BOOTSTRAP configured ssid=%s" % AP_SSID)

    time.sleep(3)
    _ip_info = ap.ifconfig()
    _ap = ap
    print(
        "#AP_BOOTSTRAP ready ssid=%s ip=%s mask=%s gateway=%s dns=%s"
        % (
            AP_SSID,
            _ip_info[0],
            _ip_info[1],
            _ip_info[2],
            _ip_info[3],
        )
    )
    return _ap


def get_ip_info():
    return _ip_info


