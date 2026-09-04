# 当前复制包：ROUND-040 修复到位后误超时

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
在CCS中Clean后Build并自行烧录。启动角实验步骤见mspm0/BREAKAWAY_MEASUREMENT.md。

本轮K230没有变化，不用重新复制K230文件。
这里包含当前入口需要的串口模块依赖，避免漏复制前一轮新增模块。
下一轮会覆盖同名文件，并移除清单中已不需要的旧复制件；原始源码及Git历史保留。
请不要把自己的独立修改只保存在这个自动更新目录里。
