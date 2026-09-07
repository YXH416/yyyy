# ROUND-044 键盘累加角度遥控

手动命令使用已保存的平衡位置为0°，协议范围为-15°到+15°，与历史24～40°坐标无关。

机械行程在 `Control/motor_balance_config.h` 配置：

```c
#define MANUAL_MECHANICAL_POS_LIMIT_MDEG  (15000L)
#define MANUAL_MECHANICAL_NEG_LIMIT_MDEG  (15000L)
#define MANUAL_MECHANICAL_LIMIT_CONFIRMED (0U)
```

当前默认为+15°/-15°，但 `CONFIRMED=0`表示尚未实测。这两个数值只是软件上限，不是机构安全证明。请先拿掉球，从±0.5°往返验证，再逐步增加。

例如实测安全范围为+9°/-7°，修改为：

```c
#define MANUAL_MECHANICAL_POS_LIMIT_MDEG  (9000L)
#define MANUAL_MECHANICAL_NEG_LIMIT_MDEG  (7000L)
#define MANUAL_MECHANICAL_LIMIT_CONFIRMED (1U)
```

还有0.5°的实际角越界判定余量：

```c
#define MANUAL_ACTUAL_OVERSHOOT_MDEG (500L)
```

运行时发送 `STATUS` 可看到当前正/负限位、是否已确认、最后手动命令序号、心跳年龄及UART错误/溢出计数。

上位机按键逻辑是每次±1°，长按300 ms后每100 ms继续累加。松开方向键时不发送0°，电机保持已下发的目标角。空格才调用 `BALANCE,ZERO`。
