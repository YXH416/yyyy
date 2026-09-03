/**
  ******************************************************************************
  * @file    closed_loop.c
  * @brief   单轴分段脉冲位置闭环和反馈故障检测
  ******************************************************************************
  * 控制器根据TIMG8硬件QEI反馈计算位置误差，并通过TIMA1输出短脉冲段逐步逼近目标。
  * 所有处理均为非阻塞方式，当前脉冲段结束后才会计算并发送下一段。
  ******************************************************************************
  */
#include "closed_loop.h"
#include "encoder.h"
#include "demo_config.h"

/* 连续3个周期处于容差内才算到位，防止只是瞬间经过目标位置。 */
#define CL_SETTLE_CYCLES    3U
/* 每段最多8个微步(按16细分等比例缩放)，短脉冲段便于及时重测位置。 */
#define CL_MAX_BURST_STEPS  ((8U * D36A_MICROSTEP) / 16U)
/* 故障阈值：累计64脉冲不动判无反馈；累计64反向脉冲判方向错误。 */
#define CL_CHECK_STEP_LIMIT 64U
#define CL_QEI_MOVE_CONFIRM       2
#define CL_PWM_MOVE_CONFIRM       1
#define CL_PWM_FALLBACK_CONFIRM   2
#define CL_REVERSE_LIMIT    64U
#define CL_REVERSE_COUNTS   3

/* 单轴闭环状态和“发完一段脉冲后核对反馈”所需的统计量。 */
typedef struct {
    volatile int32_t target_count;
    volatile uint8_t active;
    volatile uint8_t reached;
    volatile CL_Fault_t fault;
    uint8_t settle_cycles;
    uint8_t feedback_pending;
    int8_t expected_sign;
    uint32_t check_steps;
    uint32_t reverse_steps;
    int32_t check_start_pos;
    int32_t check_start_pwm_count;
    float pwm_zero_angle_deg;
    uint8_t positive_dir_level;
    uint8_t check_start_pwm_valid;
    uint8_t pwm_zero_valid;
    int8_t pwm_sign;
    CL_FeedbackSource_t feedback_source;
} CL_State_t;

static CL_State_t s_cl;
static uint8_t s_initialized;

/* 角度转编码器计数并按正负方向四舍五入。 */
static int32_t CL_AngleToCount(float angle)
{
    float scaled = angle * ENCODER_COUNTS_PER_REV / 360.0f;
    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) :
                              (int32_t)(scaled - 0.5f);
}

/* 编码器计数转角度，4000计数对应360度。 */
static float CL_CountToAngle(int32_t count)
{
    return (float)count * 360.0f / ENCODER_COUNTS_PER_REV;
}

static float CL_WrapAngleDelta(float delta)
{
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

static void CL_CapturePwmZero(void)
{
    float angle;
    if (Encoder_GetPwmAngle(ENCODER_AXIS_X, &angle) != 0U) {
        s_cl.pwm_zero_angle_deg = angle;
        s_cl.pwm_zero_valid = 1U;
    } else {
        s_cl.pwm_zero_angle_deg = 0.0f;
        s_cl.pwm_zero_valid = 0U;
    }
}

/* Read PWM relative to the current software zero, before applying direction. */
static uint8_t CL_ReadPwmRawCount(int32_t *count)
{
    float angle;
    float delta;
    if (count == 0 || s_cl.pwm_zero_valid == 0U ||
        Encoder_GetPwmAngle(ENCODER_AXIS_X, &angle) == 0U) {
        return 0U;
    }
    delta = CL_WrapAngleDelta(angle - s_cl.pwm_zero_angle_deg);
    *count = CL_AngleToCount(delta);
    return 1U;
}

static uint8_t CL_ReadCurrentCount(int32_t *current)
{
    int32_t raw;
    if (current == 0) return 0U;
    if (s_cl.feedback_source == CL_FEEDBACK_QEI) {
        *current = Encoder_GetCount(ENCODER_AXIS_X);
        return 1U;
    }
    if (CL_ReadPwmRawCount(&raw) == 0U) return 0U;
    *current = (s_cl.pwm_sign >= 0) ? raw : -raw;
    return 1U;
}

/* 分段限速：误差越大频率越高，靠近目标后降速，减少超调和振荡。 */
static uint32_t CL_SelectFrequency(uint32_t error)
{
    if (error > 800U) return 3000U;
    if (error > 200U) return 1800U;
    if (error > 50U) return 900U;
    if (error > 15U) return 450U;
    return 400U;
}

/* 按电机脉冲/编码器计数比例换算本段步数，向上取整且限制最大段长。 */
static uint32_t CL_ErrorToSteps(uint32_t error)
{
    uint64_t numerator = (uint64_t)error * MOTOR_STEPS_PER_REV;
    uint32_t steps = (uint32_t)((numerator + ENCODER_COUNTS_PER_REV - 1U) /
                                ENCODER_COUNTS_PER_REV);
    if (steps == 0U) steps = 1U;
    if (steps > CL_MAX_BURST_STEPS) steps = CL_MAX_BURST_STEPS;
    return steps;
}

/* 进入故障状态时立即停止电机，并清除尚未核对的反馈统计。 */
static void CL_SetFault(CL_Fault_t fault)
{
    s_cl.fault = fault;
    s_cl.active = 0U;
    s_cl.reached = 0U;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    s_cl.check_start_pwm_valid = 0U;
    Motor_Stop(MOTOR_AXIS_X);
}

/**
  * 核对上一脉冲段的A/B反馈：
  * 1. 朝期望方向移动达到阈值：反馈正常，清空累计；
  * 2. 明显反向：累计反向脉冲，超限报方向故障；
  * 3. 几乎不动：累计已发脉冲，超限报无编码器反馈。
  */
static uint8_t CL_CheckFeedback(int32_t current)
{
    int32_t movement;
    int32_t target_error;
    int32_t move_confirm;
    if (s_cl.feedback_pending == 0U) return 0U;
    s_cl.feedback_pending = 0U;

    /*
     * ROUND-004: a one-microstep PWM correction can already put the shaft
     * inside the closed-loop tolerance. That is successful feedback, not a
     * stalled encoder. ROUND-003 checked movement first and could accumulate
     * 64 tiny corrections into a false NO_ENCODER at ee=1..2.
     */
    target_error = s_cl.target_count - current;
    if (target_error < 0) target_error = -target_error;
    if ((uint32_t)target_error <= CL_TOLERANCE_COUNTS) {
        s_cl.check_steps = 0U;
        s_cl.reverse_steps = 0U;
        s_cl.expected_sign = 0;
        return 0U;
    }

    movement = current - s_cl.check_start_pos;
    move_confirm = (s_cl.feedback_source == CL_FEEDBACK_PWM) ?
                   CL_PWM_MOVE_CONFIRM : CL_QEI_MOVE_CONFIRM;
    if ((s_cl.expected_sign > 0 && movement >= move_confirm) ||
        (s_cl.expected_sign < 0 && movement <= -move_confirm)) {
        s_cl.check_steps = 0U;
        s_cl.reverse_steps = 0U;
        return 0U;
    }

    /*
     * ROUND-003 fallback: if A/B QEI did not change but the PB20 absolute
     * PWM angle did, determine its sign from the commanded direction and use
     * PWM feedback for the rest of this run.
     */
    if (s_cl.feedback_source == CL_FEEDBACK_QEI &&
        s_cl.check_start_pwm_valid != 0U) {
        int32_t pwm_now;
        if (CL_ReadPwmRawCount(&pwm_now) != 0U) {
            int32_t pwm_movement = pwm_now - s_cl.check_start_pwm_count;
            if (pwm_movement >= CL_PWM_FALLBACK_CONFIRM ||
                pwm_movement <= -CL_PWM_FALLBACK_CONFIRM) {
                s_cl.pwm_sign =
                    ((s_cl.expected_sign > 0 && pwm_movement > 0) ||
                     (s_cl.expected_sign < 0 && pwm_movement < 0)) ? 1 : -1;
                s_cl.feedback_source = CL_FEEDBACK_PWM;
                s_cl.check_steps = 0U;
                s_cl.reverse_steps = 0U;
                return 1U;
            }
        }
    }
    if ((s_cl.expected_sign > 0 && movement <= -CL_REVERSE_COUNTS) ||
        (s_cl.expected_sign < 0 && movement >= CL_REVERSE_COUNTS)) {
        s_cl.reverse_steps += s_cl.check_steps;
        s_cl.check_steps = 0U;
        s_cl.expected_sign = 0;
        if (s_cl.reverse_steps >= CL_REVERSE_LIMIT) CL_SetFault(CL_FAULT_DIRECTION);
        return 0U;
    }
    if (s_cl.check_steps >= CL_CHECK_STEP_LIMIT) CL_SetFault(CL_FAULT_NO_ENCODER);
    return 0U;
}

/* 上电位置作为零点，闭环初始为空闲且已到位状态。 */
void CL_Init(void)
{
    s_cl.target_count = 0;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.settle_cycles = 0U;
    s_cl.feedback_pending = 0U;
    s_cl.expected_sign = 0;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    s_cl.check_start_pos = 0;
    s_cl.check_start_pwm_count = 0;
    s_cl.positive_dir_level = AXIS_X_POSITIVE_DIR_LEVEL;
    s_cl.check_start_pwm_valid = 0U;
    s_cl.pwm_zero_valid = 0U;
    s_cl.pwm_sign = 1;
    s_cl.feedback_source = CL_FEEDBACK_QEI;
    Encoder_SetZero(ENCODER_AXIS_X);
    CL_CapturePwmZero();
    s_initialized = 1U;
}

/**
  * 闭环核心处理：等待当前脉冲段结束，核对反馈，计算位置误差，
  * 再选择方向、步数和频率发送下一小段脉冲。
  */
void CL_Process(void)
{
    int32_t current, error;
    uint32_t error_abs, steps, frequency;
    uint8_t direction;
    int8_t sign;
    if (s_initialized == 0U) return;
    if (Motor_IsBusy(MOTOR_AXIS_X) != 0U) return;
    if (CL_ReadCurrentCount(&current) == 0U) {
        if (s_cl.active != 0U) CL_SetFault(CL_FAULT_NO_ENCODER);
        return;
    }
    if (CL_CheckFeedback(current) != 0U) {
        if (CL_ReadCurrentCount(&current) == 0U) {
            CL_SetFault(CL_FAULT_NO_ENCODER);
            return;
        }
    }
    if (s_cl.active == 0U || s_cl.fault != CL_FAULT_NONE) return;
    error = s_cl.target_count - current;
    error_abs = (error >= 0) ? (uint32_t)error : (uint32_t)(-error);
    if (error_abs <= CL_TOLERANCE_COUNTS) {
        if (s_cl.settle_cycles < CL_SETTLE_CYCLES) s_cl.settle_cycles++;
        if (s_cl.settle_cycles >= CL_SETTLE_CYCLES) s_cl.reached = 1U;
        return;
    }
    s_cl.settle_cycles = 0U;
    s_cl.reached = 0U;
    steps = CL_ErrorToSteps(error_abs);
    frequency = CL_SelectFrequency(error_abs);
    if (error > 0) {
        direction = s_cl.positive_dir_level;
        sign = 1;
    } else {
        direction = (uint8_t)!s_cl.positive_dir_level;
        sign = -1;
    }
    if (Motor_SetDirection(MOTOR_AXIS_X, direction) != MOTOR_OK ||
        Motor_Start(MOTOR_AXIS_X, steps, frequency) != MOTOR_OK) {
        CL_SetFault(CL_FAULT_DRIVER);
        return;
    }
    if (s_cl.check_steps == 0U || s_cl.expected_sign != sign) {
        s_cl.check_start_pos = current;
        s_cl.check_start_pwm_valid =
            CL_ReadPwmRawCount(&s_cl.check_start_pwm_count);
        s_cl.check_steps = 0U;
        s_cl.expected_sign = sign;
    }
    s_cl.check_steps += steps;
    s_cl.feedback_pending = 1U;
}

/* 新目标以软件零点为基准，转换为QEI目标计数后启动闭环。 */
MotorStatus_t CL_SetTargetAngle(MotorAxis_t axis, float target_deg)
{
    if (axis != MOTOR_AXIS_X || s_initialized == 0U || target_deg != target_deg ||
        target_deg > 100000.0f || target_deg < -100000.0f ||
        s_cl.fault != CL_FAULT_NONE) return MOTOR_ERROR;
    if (s_cl.pwm_zero_valid == 0U) CL_CapturePwmZero();
    s_cl.target_count = CL_AngleToCount(target_deg);
    s_cl.active = 1U;
    s_cl.reached = 0U;
    s_cl.settle_cycles = 0U;
    return MOTOR_OK;
}

/* 将当前位置设为0并取消当前目标，同时清除故障和反馈统计。 */
void CL_SetZero(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X || s_initialized == 0U) return;
    Motor_Stop(axis);
    Encoder_SetZero(ENCODER_AXIS_X);
    s_cl.feedback_source = CL_FEEDBACK_QEI;
    s_cl.pwm_sign = 1;
    CL_CapturePwmZero();
    s_cl.target_count = 0;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    s_cl.check_start_pwm_valid = 0U;
}

void CL_SetZeroAll(void) { CL_SetZero(MOTOR_AXIS_X); }

/* 立即停止当前目标；停止不等同于到位，因此reached置0。 */
void CL_Stop(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X) return;
    Motor_Stop(axis);
    s_cl.active = 0U;
    s_cl.reached = 0U;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    s_cl.check_start_pwm_valid = 0U;
}

void CL_StopAll(void) { CL_Stop(MOTOR_AXIS_X); }

/* 清故障后以当前位置为目标，保持原地等待下一条命令。 */
void CL_ClearFault(MotorAxis_t axis)
{
    int32_t current;
    if (axis != MOTOR_AXIS_X || s_initialized == 0U) return;
    Motor_Stop(axis);
    if (CL_ReadCurrentCount(&current) == 0U) {
        current = Encoder_GetCount(ENCODER_AXIS_X);
    }
    s_cl.target_count = current;
    s_cl.active = 0U;
    s_cl.reached = 1U;
    s_cl.fault = CL_FAULT_NONE;
    s_cl.feedback_pending = 0U;
    s_cl.check_steps = 0U;
    s_cl.reverse_steps = 0U;
    s_cl.check_start_pwm_valid = 0U;
}

/* 停机时翻转逻辑正方向，用于现场修正DIR与编码器方向不一致。 */
MotorStatus_t CL_TogglePositiveDirLevel(MotorAxis_t axis)
{
    if (axis != MOTOR_AXIS_X || Motor_IsBusy(axis) != 0U) return MOTOR_ERROR;
    s_cl.positive_dir_level = (uint8_t)!s_cl.positive_dir_level;
    s_cl.reverse_steps = 0U;
    return MOTOR_OK;
}

uint8_t CL_IsReached(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X && s_initialized != 0U) ? s_cl.reached : 0U;
}

CL_Fault_t CL_GetFault(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X && s_initialized != 0U) ? s_cl.fault : CL_FAULT_DRIVER;
}

float CL_GetCurrentAngle(MotorAxis_t axis)
{
    int32_t current;
    if (axis != MOTOR_AXIS_X || CL_ReadCurrentCount(&current) == 0U) {
        return 0.0f;
    }
    return CL_CountToAngle(current);
}

/* 把内部状态复制到快照，避免串口层直接依赖闭环私有变量。 */
void CL_GetSnapshot(MotorAxis_t axis, CL_Snapshot_t *snapshot)
{
    int32_t current;
    float pwm_angle = 0.0f;
    if (axis != MOTOR_AXIS_X || snapshot == 0 || s_initialized == 0U) return;
    if (CL_ReadCurrentCount(&current) == 0U) {
        current = Encoder_GetCount(ENCODER_AXIS_X);
    }
    snapshot->current_count = current;
    snapshot->target_count = s_cl.target_count;
    snapshot->error_count = snapshot->target_count - snapshot->current_count;
    snapshot->current_angle_deg = CL_CountToAngle(snapshot->current_count);
    snapshot->target_angle_deg = CL_CountToAngle(snapshot->target_count);
    snapshot->active = s_cl.active;
    snapshot->reached = s_cl.reached;
    snapshot->fault = s_cl.fault;
    snapshot->feedback_source = s_cl.feedback_source;
    snapshot->pwm_valid = Encoder_GetPwmAngle(
        ENCODER_AXIS_X, &pwm_angle);
    snapshot->pwm_angle_deg = pwm_angle;
}


