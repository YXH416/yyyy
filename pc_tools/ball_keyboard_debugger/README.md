# 平衡球键盘调试器

用于 Windows + Python 3，只实现键盘手动倾斜、实时状态、安全停止和CSV记录。不做PID、不做扫频、不转发VOFA。

## 安装与运行

```powershell
py -3 -m pip install -r requirements.txt
py -3 app.py
```

如果系统没有 `py` 启动器，把上述 `py -3` 换成 `python`。也可双击 `run.bat`，它会自动尝试两种启动方式。Python必须独占串口，先关闭VOFA和其他串口助手。

对应固件构建标识：

```text
ROUND-046_RX_STORM_FIX_V1
```

## 快速使用

第46轮修正RX_LOST引起的STATUS请求风暴，详细日志结论和复制步骤见 [第46轮修正](ROUND046_FIX.md)。

第45轮修正启动期间RX_LOST导致停发心跳的问题，删除目标位置设置。请同时覆盖Python与主控串口文件，详见 [本轮修正](ROUND045_FIX.md)。

1. 选择COM口，点击“连接”。工具自动读取PWM、故障和当前相对角。
2. 若PWM有效、Fault=NONE且电机在平衡零点±0.30°内，自动进入“遥控待命”，无需再点击进入手动。
3. 若不在零点，界面显示“需要回零”；点击“回零并启用”，到位后自动进入遥控待命。
4. 轻按→使目标角+1°，轻按←使目标角-1°。长按300 ms后，每100 ms继续增减1°。
5. 松开方向键只停止继续累加，保持当前目标角，不会自动回0°。
6. 空格退出当前手动目标并调用 `BALANCE,ZERO`，回零成功后自动恢复遥控待命。`S`只停止脉冲，`Esc`关闭窗口。

键盘焦点离开窗口时，工具等同于松开方向键：停止累加并保持当前目标。Python崩溃或USB断开时，固件500 ms心跳看门狗会主动回零。

故障、心跳超时或按S后不会自动重新运动。处理故障后，需要点击“确认重新启用”。

## 串口命令

```text
MANUAL,START[,SEQ]
MANUAL,ANGLE,-15.000..15.000[,SEQ]
MANUAL,HEARTBEAT[,SEQ]
MANUAL,STOP[,SEQ]
BALANCE,ZERO
FAULT,CLEAR
STATUS
```

上位机和固件的手动协议目标均为相对平衡零点±15°。Python所有写串口操作由一个发送线程串行化，心跳由独立线程定时；手动命令携带序号，`STATUS` 会返回最后接收序号、年龄、接收字节、整行数、硬件错误和缓冲溢出计数。

## 机械安全限位配置

配置位于 `mspm0/Control/motor_balance_config.h`：

```c
#define MANUAL_COMMAND_LIMIT_MDEG          (15000L)
#define MANUAL_MECHANICAL_POS_LIMIT_MDEG  (15000L)
#define MANUAL_MECHANICAL_NEG_LIMIT_MDEG  (15000L)
#define MANUAL_MECHANICAL_LIMIT_CONFIRMED (0U)
#define MANUAL_ACTUAL_OVERSHOOT_MDEG       (500L)
```

当前正向和负向默认都是15°，`CONFIRMED=0`表示该机械行程尚未实测，不代表±15°已经安全。无球测得正向安全行程为9°、负向为7°时，改为 `9000L`、`7000L`，再把 `CONFIRMED` 改为 `1U`。`OVERSHOOT` 是实际角越界余量，默认0.5°。

固件仍保留PWM反馈失效停止、500 ms通信看门狗回零、电机故障锁存和已标定球位置越界保护。首次必须拿掉球，只轻按一次→到+1°，再轻按一次←回0°，确认方向和反馈后才继续。

## 记录

“开始记录”会在本目录的 `records/日期_时间/` 下创建：

- `telemetry.csv`：真实相对时间、球位置、速度、电机目标/实际角和健康状态。
- `events.csv`：墙上时间、发送序号、队列延迟、方向键按下/松开、故障和固件日志。

时间使用Python单调时钟，不受Windows系统时间校准影响。
