# 平衡参考实测记录

来源：用户提交的VOFA串口日志，非示例参数。

- 标定开始：PC 21:15:15.363；MCU 442184 ms。
- 标定完成：PC 21:15:15.678；MCU 442500 ms。
- 绝对PWM参考：176861 mdeg = 176.861°。
- 采样：16次，20 ms间隔，极差0.139°，通过0.20°稳定性判据。
- qei_count=12974，是当时软件参考下的增量计数，不作为断电重用的绝对参考。
- 随后读数motor_relative_deg=0.040表示相对新零点约+0.040°，不是标定失败。
- 小球物理平衡由用户观察确认；编码器采样稳定本身不证明动态抗扰性能。

归一化后的关键日志（仅还原聊天中转义的下划线）：

    [BALANCE] ms=442500 event=SAVED_RAM pwm_mdeg=176861 qei_count=12974 spread_deg=0.139 source_code_update=PENDING
    [BALANCE] valid=1 pwm_mdeg=176861 pwm_abs_deg=176.861 motor_relative_deg=0.040

ROUND-037已经将此记录写入Control/motor_balance_config.h：
MEASURED_BALANCE_VALID=1，MEASURED_BALANCE_PWM_MDEG=176861。
电机测量坐标 = wrap(PWM绝对角度 − 176.861°)。这不改变上电动作，也不设置未经测量的机械限位。

日志随后出现视觉新鲜度变化：

    [VISION] ms=446771 fresh=0
    [VISION] ms=446817 fresh=1
    [VISION] ms=447068 fresh=0
    [VISION] ms=458925 fresh=1

第二段视觉失效状态约11.857秒，说明这段时间没有持续的新鲜位置帧。
不能仅据此区分球被拿走/遮挡、检测丢失或串口/相机中断；需对照当时画面。
这不影响之前已经完成的编码器标定，后续开始视觉闭环前需确认球可见时数据连续。
