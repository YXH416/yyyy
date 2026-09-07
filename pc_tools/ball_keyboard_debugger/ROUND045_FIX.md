# ROUND-045 遥控退出修正

覆盖本目录app.py；同时覆盖mspm0/empty.c和Hardware/experiment_console.c，在CCS重新编译。入口标识为ROUND-045_REMOTE_RX_RECOVERY_V1。

启动期间RX_LOST曾被当作启动失败，从而停止心跳，可能引起主控500 ms通信超时回零。现在启动期间也发送心跳，RX_LOST只记录并查询STATUS，真正故障/超时仍需确认重新启用。心跳看门狗仍为500 ms。

删去目标位置按钮、目标数值和目标线；保留实际球位置。松开左右键只停止累加，保持当前目标；空格回零。S在回零期间也可以停止脉冲，并取消延迟自动启用。

串口初始化显式启用FIFO，避免依赖旧CCS生成配置；RX_LOST增加hw_errors、overflow、bits，区分硬件接收错误和软件缓冲溢出。该改动不能证明电气链路丢包已经消失，需要实机日志验证。

无球连接后轻按右键再松手，确认目标保持+1°至少3秒。若仍自动回零，保存完整串口日志，从MANUAL STARTED到SAFE_ZERO/STOP，连同STATUS和新RX_LOST行；reason字段标明是否为LINK_TIMEOUT、MOTOR_ANGLE_LIMIT或BALL_POSITION_LIMIT。
