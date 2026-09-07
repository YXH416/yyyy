# 平衡球键盘调试器

用于 Windows + Python 3，第一版只实现键盘手动倾斜、实时状态、安全停止和CSV记录。不做PID、不做扫频、不转发VOFA。

## 安装与运行

```powershell
py -3 -m pip install -r requirements.txt
py -3 app.py
```

如果系统没有 `py` 启动器，把上述 `py -3` 换成 `python`。也可双击 `run.bat`，它会自动尝试两种启动方式。Python必须独占串口，先关闭VOFA和其他串口助手。

对应固件构建标识：

```text
ROUND-042_KEYBOARD_MANUAL_V1
```

## 快速使用

1. 选择COM口，点击“连接”。
2. 确认界面显示 `PWM：有效`、`Fault：NONE`。
3. 先在无球状态测试“回平衡零点”。
4. 点击“进入手动”，等日志显示 `MANUAL event=STARTED`。
5. 默认直接倾斜模式和0.5°安全起始幅值：按住→给正角，按住←给负角，松开方向键立即命令0°。
6. 空格命令0°，`S`停止脉冲，`Esc`安全关闭窗口。

键盘焦点离开窗口时，工具会释放方向键并发0°。Python崩溃或USB断开时，固件500 ms心跳看门狗会主动回零。

## 串口命令

```text
MANUAL,START
MANUAL,ANGLE,-2.000..2.000
MANUAL,HEARTBEAT
MANUAL,STOP
BALANCE,ZERO
FAULT,CLEAR
STATUS
```

上位机和固件均限制目标角为±2.0°；固件还会在实际角超过±2.3°或已标定球位置超过±60 mm时结束手动模式并回零。

## 记录

“开始记录”会在本目录的 `records/日期_时间/` 下创建：

- `telemetry.csv`：球位置、速度、电机目标/实际角和健康状态。
- `events.csv`：方向键按下/松开、命令、故障和关键固件日志。

时间使用Python单调时钟，不受Windows系统时间校准影响。
