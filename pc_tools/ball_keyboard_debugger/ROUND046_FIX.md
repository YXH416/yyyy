# ROUND-046 接收错误风暴修正

20260907_211724记录中，主控成功回复MANUAL STARTED，随后以LINK_TIMEOUT退出。约75秒出现8406条RX_LOST。第45轮每收到一次RX_LOST就发送STATUS，导致请求/报错循环；这次删除该行为，STATUS仅由原有1秒周期查询。

主控RX_LOST日志最多每500 ms输出一次，累计错误仍保留。上位机每次处理消息最多100条或8 ms，将时间交还给按键和定时器；心跳发送也写入events.csv。回零等待超过12秒时请求停止并明确显示确认超时，避免永久停在启用流程。

日志bits=0xA是UART帧错误与BREAK，rx_overflow=0。软件修正不能证明电气链路已正常。若新版本仍持续出现framing=1或break=1且hw_errors递增，请检查USB-TTL TX接主控PA11、RX接PA10、共同GND、115200/8N1/3.3V，并确认PA11没有另一块板的输出同时驱动。bits是历史累计标志，应结合hw_errors是否增加判断。

同时复制app.py、mspm0/empty.c、mspm0/Hardware/experiment_console.c及experiment_console.h；主控自行编译烧录，启动标识ROUND-046_RX_STORM_FIX_V1。目标位置设置已删除；右/左每次±1度，长按累加、松手保持，空格回零，S停止。

先无球连接并回零，轻按右键后松手观察3秒。如果仍退出，保存含MANUAL STARTED、SAFE_ZERO/STOP、STATUS及RX_LOST的完整日志。本轮未做硬件实测，保留500 ms通信看门狗和反馈保护。
