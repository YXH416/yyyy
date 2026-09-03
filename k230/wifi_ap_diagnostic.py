"""Yahboom official AP example used directly at boot (MOD-022).

Only one deliberate addition is placed before the verified example:
a 15-second boot grace period. No image, Display, MediaManager, Sensor, VENC,
RTSP, YOLO, JSON configuration or wifi_bootstrap module is imported here.
"""

import network
import time


AP_SSID = "YAHBOOM-K230"
AP_KEY = "12345678"
BOOT_WAIT_SECONDS = 15


def CREATE_AP(AP_SSID, AP_KEY):
    """Create the hotspot using the Yahboom course-example sequence."""

    # Initialize AP mode exactly as in the verified Yahboom example.
    ap = network.WLAN(network.AP_IF)

    if not ap.active():
        ap.active(True)
    print("AP mode activation status:", ap.active())

    # The current Yahboom firmware accepts exactly ssid + key here.
    ap.config(ssid=AP_SSID, key=AP_KEY)
    print("Hotspot created:")
    print("SSID:", AP_SSID)
    print("KEY:", AP_KEY)

    # Keep the official three-second post-configuration wait.
    time.sleep(3)

    ip_info = ap.ifconfig()
    print("AP network configuration:")
    print("IP address:", ip_info[0])
    print("Subnet mask:", ip_info[1])
    print("Gateway:", ip_info[2])
    print("DNS server:", ip_info[3])
    print(
        "#AP_OFFICIAL_TEMPLATE mod=MOD-022 ssid=%s ip=%s "
        "active=%s"
        % (AP_SSID, ip_info[0], ap.active())
    )

    # This loop is intentionally the same station-monitoring structure as the
    # official example. It also guarantees useful output after USB reconnect.
    while True:
        clients = ap.status("stations")
        print(
            "#AP_OFFICIAL_TEMPLATE mod=MOD-022 ssid=%s ip=%s "
            "clients=%s active=%s"
            % (AP_SSID, ip_info[0], len(clients), ap.active())
        )
        time.sleep(1)


def main():
    print(
        "#AP_OFFICIAL_TEMPLATE boot_wait=%ss mod=MOD-022"
        % BOOT_WAIT_SECONDS
    )
    time.sleep(BOOT_WAIT_SECONDS)
    CREATE_AP(AP_SSID, AP_KEY)


if __name__ == "__main__":
    main()


