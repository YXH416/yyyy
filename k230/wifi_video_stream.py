"""Compatibility entry for the obsolete standalone stream mode (MOD-018).

Streaming no longer owns a second camera task. It is integrated into the
normal YOLO pipeline so LCD, inference, MSPM0 UART and RTSP share one Sensor
and one MediaManager.
"""


def main():
    print(
        "#WIFI compatibility=integrated_yolo_pipeline "
        "config=/sdcard/wifi_stream_config.json mod=MOD-018"
    )
    from yolo_detector import main as run_vision
    run_vision()


if __name__ == "__main__":
    main()


