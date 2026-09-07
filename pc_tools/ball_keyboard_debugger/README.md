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
ROUND-043_KEYBOARD_MANUAL_V2
```

## 快速使用

1. 选择COM口，点击“连接”。
2. 确认界面显示 `PWM：有效`、`Fault：NONE`。
3. 先在无球状态测试“回平衡零点”。
4. 点击“进入手动”，界面先显示“正在启动”，收到固件 `MANUAL event=STARTED` 后才显示“手动已启动”并接受方向键。
5. 默认直接倾斜模式和0.5°起始幅值：按住→给正角，按住←给负角，反向按键立即更新目标，松开方向键命令0°。
6. 空格退出手动并调用 `BALANCE,ZERO`，`S`只停止脉冲，`Esc`安全关闭窗口。

键盘焦点离开窗口时，工具会释放方向键并发0°。Python崩溃或USB断开时，固件500 ms心跳看门狗会主动回零。
微调模式可选0.1°、0.2°、0.5°或1.0°步长；按下先执行一次，持续350 ms后每120 ms连续执行一次，不依赖Windows自动重复事件。

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

固件仍保畐PWM反馈失效停止、500 ms通信看门狗回零、电机故障锁存和已标定球位置越界保护。首次必须无球从±0.5°往返开始，不要直接输入15°。

## 记录

“开始记录”会在本目录的 `records/日期_时间/` 下创建：

- `telemetry.csv`：真实相对时间、球位置、速度、电机目标/实际角和健康状态。
- `events.csv`：墙上时间、发送序号、队列延迟、方向键按下/松开、故障和固件日志。

时间使用Python单调时钟，不受Windows系统时间校准影响。
