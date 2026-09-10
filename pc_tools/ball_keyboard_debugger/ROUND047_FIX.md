# ROUND-047 最小遥控版

把键盘调试器砍成最小遥控：只保留相对平衡零点的加角度、减角度、回平衡零点三个动作，去掉手动模式启停、心跳看门狗、机械限位确认、CSV 记录、球位置尺、视觉状态、Fault 复杂提示和停止脉冲按钮。

## PC 侧（pc_tools/ball_keyboard_debugger/app.py）

- `→`/`←` 每次 ±1°，长按 300 ms 后每 100 ms 连续 ±1°，松开保持当前目标角。
- `空格` 发送 `BALANCE,ZERO` 并把目标角归 0。
- 目标范围固定 -15° ~ +15°。
- 界面只显示：串口连接状态、目标角、实际角。
- 实际角来自每 250 ms 一次 `STATUS` 里的 `motor_deg` 字段，不依赖 `STREAM` 或视觉。
- 保留单发送线程 + 队列的串行写；`[ERR] RX_LOST` 只被忽略，绝不触发 STATUS，避免旧版请求风暴。

## 固件侧（mspm0）

- `experiment_protocol.h/.c`：新增 `ANGLE_REL,<float>` 命令，范围 -15.0 ~ +15.0。
- `empty.c`：新增 `RelativeAngle()` 处理器，复用现有相对零点锚定逻辑（`EnterMeasurement` + `CL_SetTargetAngle`），不需要 `MANUAL,START`、心跳或 500 ms 链路看门狗。
- 首次 `ANGLE_REL` 才锚定内环零点；后续直接改目标。当前实际角超出 ±15° 时拒绝并提示先 `BALANCE,ZERO`，避免第一次大跳。
- 反馈失效保护保持不变：`CL_FAULT_NO_ENCODER` / `PWM_LOST` 时立即停止并锁存故障。
- 旧的 `ANGLE,24..40` 绝对坐标命令未使用；新遥控只发 `ANGLE_REL` 和 `BALANCE,ZERO`。

构建标识：`ROUND-047_MINIMAL_REMOTE_V1`。
