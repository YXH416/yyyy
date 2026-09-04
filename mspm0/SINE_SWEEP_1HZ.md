# ROUND-041：1 Hz 单频正弦试扫

本轮只测一个工况：

```text
motor_cmd_deg = 1.3 * sin(2*pi*1.0*t)
持续8秒
```

不会自动扫其他频率。主要辨识输入是 `motor_real_deg`，输出是 `ball_pos_mm`。

## 实验前

1. 断电后先把机构恢复到安全位置。
2. 覆盖“待复制文件”内全部源码，CCS Clean、Build并烧录。
3. 重启后确认 `BUILD_ID=ROUND-041_PWM_SINE_1HZ_V1`。
4. 先发 `STREAM,OFF`，再发 `STATUS`；必须有 `pwm_valid=1`。
5. 发 `FAULT,CLEAR`。正常回复应包含 `fault=NONE fb=PWM no_motion=1`。
6. 暂时不放球，发 `BALANCE,ZERO`。等待 `event=REACHED`，再发 `STATUS`，必须确认 `fault=NONE fb=PWM timeout_armed=0`。

如果这一次回零不正常，不要放球，直接发 `STOP` 并保留日志。

## 视觉与VOFA

如重启后标定丢失，重新执行：

```text
CAL,CENTER
CAL,LEFT
CAL,RIGHT
```

开启VOFA数据：

```text
STREAM,ON
CH0 = ball_pos_mm
CH1 = ball_vel_mm_s
CH2 = motor_cmd_deg
CH3 = motor_real_deg
```

输出周期20 ms，即50 Hz。

## 开始单频测试

1. 把球中心放在中心标记附近，松手。
2. 发送 `SINE,START`。
3. 程序会先确认球在±10 mm内、速度不超过3 mm/s且连续静止500 ms。
4. 看到 `event=STARTED` 后才开始计8秒。
5. 完成或触发安全限制后，程序停止正弦并自动发起回零。

安全条件：

```text
|ball_pos_mm| > 40     -> POSITION_LIMIT
|motor_real_deg| > 2   -> MOTOR_ANGLE_LIMIT
视觉丢失                 -> VISION_LOST
电机反馈或驱动故障         -> MOTOR_FAULT
```

任意时候发送 `STOP` 会立即停止脉冲，但STOP不会自动清除真实故障。

请保存VOFA全部8秒数据，并返回 `[SINE] event=STARTED`、`[SINE_RESULT]`、回零结果和实验曲线。
