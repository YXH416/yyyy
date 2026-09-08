# 当前复制包：ROUND-046 接收错误风暴修正

本轮必须一起覆盖app.py、mspm0/empty.c、mspm0/Hardware/experiment_console.c及experiment_console.h。删除RX_LOST逐条触发STATUS的请求风暴，限制日志处理时间并记录心跳。详细原因和实机验证见pc_tools/ball_keyboard_debugger/ROUND046_FIX.md。启动标识为ROUND-046_RX_STORM_FIX_V1。以下为历史改动说明。

本轮必须同时更新pc_tools/ball_keyboard_debugger/app.py、mspm0/empty.c、mspm0/Hardware/experiment_console.c。修正启动时RX_LOST导致心跳停止的问题，显式开启UART FIFO，并细分接收错误日志。删除目标位置控件；松键保持当前目标。实机问题若继续出现，请记录MANUAL STARTED之后的完整日志和STATUS。详情见Python目录ROUND045_FIX.md。

固定目录“待复制文件”每轮更新覆盖，不含可烧录文件。

将mspm0内文件按相同相对路径覆盖到现有CCS工程：
empty.c放工程根目录，Control和Hardware中的文件放进对应目录，不要把所有.c堆在根目录。
新增motor_balance_config.h要一起复制。新增的experiment_protocol.c和experiment_console.c只编译一份。
此次实测值已写入Control/motor_balance_config.h，VALID=1、PWM_MDEG=176861；重新编译烧录后生效。
本轮修改了Hardware/demo_config.h的电机到位容差，该文件必须一起覆盖，否0.2°小步进可能不动。
重启后发BALANCE,SHOW应显示valid=1、pwm_mdeg=176861。不会自动转到这个位置。
发送BALANCE,ZERO（末尾加换行）后，才会主动转到平衡零点；STOP可中止。
完成三点视觉标定后，发BREAKAWAY,POS或BREAKAWAY,NEG自动测启动角；每0.2°保持2秒，速度连续超过5 mm/s达200 ms时记录。
ROUND-040已修复BALANCE,ZERO显示REACHED后仍被旧10秒计时触发MOTOR_TIMEOUT的问题。如当前已在零点±0.30°内，会直接输出event=REACHED mode=NO_MOTION，不启动运动超时。
ROUND-041根据fault=DIRECTION以及QEI/PWM符号相反的实测，将重新建零后的电机闭环优先反馈改为PB20绝对PWM。Control/closed_loop.c必须一起覆盖。
新增FAULT,CLEAR、SINE,START和SINE,STATUS。SINE,START只执行1.0 Hz、±1.3°、8秒的单频正弦，不扫其他频率；球超过±40 mm或电机超过±2°时停止并回零。
ROUND-042首次加入MANUAL协议和500 ms通信看门狗；当时的±2.0°/±2.3°限制已被ROUND-043的新配置取代，不再生效。PWM反馈的NO_ENCODER确认窗口仍为256脉冲。

ROUND-043将MANUAL协议统一为相对平衡零点±15°。固件的正/负机械限位在Control/motor_balance_config.h中独立配置，当前均为15°且CONFIRMED=0（未实测），首次必须无球从±0.5°开始。
本轮修改了Hardware/experiment_console.c/.h：接收缓冲增大、发送帧增大，并增加RX字节/整行/溢出统计；这两个文件必须一起覆盖。
键盘工具的手动启动需要收到固件确认，所有写串口操作已改为单队列，心跳独立运行。空格退出手动并调用BALANCE,ZERO，S只停止脉冲。

ROUND-044取消“直接倾斜/微调”切换。轻按→目标+1°，轻按←目标-1°；长按300 ms后每100 ms继续累加，松开保持已下发的角度，不自动回零。连接后自动检查PWM、Fault和当前相对角；零点内自动待命，否则明确提示先回零。故障、LINK_TIMEOUT或S停止后必须点击“确认重新启用”。
新建pc_tools/ball_keyboard_debugger；安装requirements.txt后双击run.bat。Python必须独占串口，运行前关闭VOFA。
在CCS中Clean后Build并自行烧录。Python用法见pc_tools/ball_keyboard_debugger/README.md。

本轮K230没有变化，不用重新复制K230文件。
这里包含当前入口需要的串口模块依赖，避免漏复制前一轮新增模块。
下一轮会覆盖同名文件，并移除清单中已不需要的旧复制件；原始源码及Git历史保留。
请不要把自己的独立修改只保存在这个自动更新目录里。
