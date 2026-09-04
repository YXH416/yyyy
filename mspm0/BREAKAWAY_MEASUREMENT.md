# ROUND-039 启动角自动测量

本轮测量的是“电机相对已保存平衡姿态的角度”，不是水管的绝对斜率。已保存的平衡零点是 PWM 绝对角 `176.861°`。

## 测正向

1. 用 VOFA FireWater 连接 UART0，115200、8N1，命令末尾保留换行。
2. 先完成 `CAL,CENTER`、`CAL,LEFT`、`CAL,RIGHT`三点标定。这一步是为了将 K230 像素换算为 mm 和 mm/s。
3. 发送 `BALANCE,ZERO`，等待 `event=REACHED`。
4. 手动把球中心放在中心标记，松手，不要再碰球。
5. 发送 `BREAKAWAY,POS`。

程序先确认球位置在 ±10 mm 内，且速度不超过3 mm/s持续500 ms；然后从 `+0.2°` 开始，每次增加 `0.2°`，到位后保持2 s。

当 `|ball_vel| > 5 mm/s` 在新视觉帧上连续满足200 ms，程序停止发脉冲，保持电机使能，并输出：

```text
[BREAKAWAY_RESULT] ... result=DETECTED direction=MOTOR_POS theta_break_deg=...
```

`theta_break_deg` 是检测时的电机实际相对角，应优先记录它；`target_deg` 是当时命令角。

## 测负向

1. 发送 `BALANCE,ZERO`。
2. 重新手动把球放回中心，松手。
3. 发送 `BREAKAWAY,NEG`。

负向使用相同判据，输出的方向是 `MOTOR_NEG`。发送 `BREAKAWAY,STATUS` 可查看本次上电后已测得的两个结果。

## 停止与限制

- 任何时候发送 `STOP` 或按 PB7 都会立即中止扫描。
- 视觉丢失、PWM反馈丢失或电机故障会自动中止。
- 本轮为第一次受控实验，扫描上限暂定为 ±3.0°。到上限仍未启动会输出 `result=NO_START_AT_LIMIT`，不会继续加大。
- 自动结束后电机保持当前位置，不会自动回零。下一次实验前发送 `BALANCE,ZERO`。

请把两条完整的 `[BREAKAWAY_RESULT]` 和它们前面的 `[BREAKAWAY_STEP]` 日志发回，下一轮再把实测的正负启动角写入控制参数。
