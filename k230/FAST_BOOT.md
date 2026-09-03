# 第35次修改：本地视觉快速启动

将这三个文件覆盖到 K230 SD 卡的 /sdcard/ 根目录，然后重启：

- main.py
- ball_stream_pipeline.py
- runtime_options.py（新增，必须一起复制）

默认 WIRELESS_STREAM_ENABLED = False：

- 跳过开机 AP 热点初始化和网络连接等待；
- 不加载 Wi-Fi/RTSP模块，不配置图传CH1，不分配视频编码器缓冲；
- 跳过图传启动状态页及其500毫秒等待；
- 保留本地LCD、钢球识别、触摸调阈值、向MSPM0输出UART位置数据。

启动日志包含 ROUND-035_FAST_BOOT、wireless=False，以及 RTSP disabled reason=fast_boot。
旧wifi_stream_config.json即使enabled为true，也不会启动图传。
旧WIFI_AP_DIAGNOSTIC.flag在此模式下被忽略，文件没有删除。
IDE_DIAGNOSTIC_MODE.flag和CAMERA_MODE.flag仍按原含义工作；正常跑球识别时不要启用这两种模式。

以后恢复图传：将runtime_options.py中的WIRELESS_STREAM_ENABLED设为True并重启，
图传参数继续使用原wifi_stream_config.json。

本次只调整K230启动及图传路径，MSPM0串口实验代码无需重新烧录。
实际从上电到识别到球的时间需在板上测量；传感器和LCD初始化仍需要时间。
