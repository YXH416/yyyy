"""ROUND-035: shared launcher/pipeline options for local vision experiments."""

# False skips AP bootstrap, network/RTSP imports, CH1 and encoder allocation.
# LCD (CH0), ball detection (CH2), touch UI and MSPM0 UART remain available.
# Set True and reboot to restore the wifi_stream_config.json settings.
WIRELESS_STREAM_ENABLED = False
