/*
 * ROUND-037 -- save measured 176.861-degree balance reference.
 * No automatic movement on boot; ANGLE commands explicitly move the motor.
 * PA17 remains a sequential calibration shortcut; PB7 stops the motor.
 */
#include "board.h"
#include "buttons.h"
#include "closed_loop.h"
#include "encoder.h"
#include "k230.h"
#include "motor.h"
#include "experiment_console.h"
#include "motor_balance_config.h"

#include <stdio.h>
#define printf Console_Printf

#define BUILD_ID                         "ROUND-037_BALANCE_176861_V1"
#define CONTROL_PERIOD_MS                (5U)
#define K230_FRESH_MS                    (250U)
#define SCALE_CAPTURE_SAMPLES            (12U)
#define SCALE_CAPTURE_SPREAD_LIMIT_PX    (8)
#define TELEMETRY_PERIOD_MS             (20U)
#define CAPTURE_TIMEOUT_MS              (3000U)
#define MOTOR_TIMEOUT_MS                (10000U)
#define FIXED_PWM_REFERENCE_DEG         (49.3f)
#define INVALID_MEASUREMENT             (-9999.0f)
#define BALANCE_SAMPLES                 (16U)
#define BALANCE_SPREAD_DEG              (0.20f)

typedef enum {
    SCALE_WAIT_CENTER = 0,
    SCALE_WAIT_NEG_50,
    SCALE_WAIT_POS_50,
    SCALE_READY
} ScaleStage_t;

typedef struct {
    ScaleStage_t stage;
    uint8_t collecting;
    uint8_t sample_count;
    int32_t sample_sum;
    int32_t sample_min;
    int32_t sample_max;
    int32_t centre_px;
    int32_t neg_50_px;
    int32_t pos_50_px;
    float neg_mm_per_px;
    float pos_mm_per_px;
    uint8_t valid;
    uint32_t capture_start_ms;
} ScaleCalibration_t;

static const char *ScaleStageName(ScaleStage_t stage)
{
    switch (stage) {
    case SCALE_WAIT_CENTER: return "CENTER";
    case SCALE_WAIT_NEG_50: return "NEG_50";
    case SCALE_WAIT_POS_50: return "POS_50";
    default: return "READY";
    }
}

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void ScaleReset(ScaleCalibration_t *scale)
{
    scale->stage = SCALE_WAIT_CENTER;
    scale->collecting = 0U;
    scale->sample_count = 0U;
    scale->sample_sum = 0;
    scale->sample_min = 0;
    scale->sample_max = 0;
    scale->centre_px = 0;
    scale->neg_50_px = 0;
    scale->pos_50_px = 0;
    scale->neg_mm_per_px = 0.0f;
    scale->pos_mm_per_px = 0.0f;
    scale->valid = 0U;
    scale->capture_start_ms = 0U;
}

static void ScaleStartCapture(ScaleCalibration_t *scale, uint32_t now_ms)
{
    scale->collecting = 1U;
    scale->sample_count = 0U;
    scale->sample_sum = 0;
    scale->sample_min = 0;
    scale->sample_max = 0;
    scale->capture_start_ms = now_ms;
    printf("[SCALE_CAPTURE] ms=%lu state=SAMPLING step=%s samples=%u "
           "instruction=KEEP_BALL_STILL\r\n",
           (unsigned long)now_ms, ScaleStageName(scale->stage),
           (unsigned int)SCALE_CAPTURE_SAMPLES);
}

static void ScaleFinishCapture(ScaleCalibration_t *scale, uint32_t now_ms)
{
    int32_t mean_px = scale->sample_sum / (int32_t)SCALE_CAPTURE_SAMPLES;
    int32_t spread_px = scale->sample_max - scale->sample_min;
    const char *captured_step = ScaleStageName(scale->stage);
    const char *next_instruction;

    scale->collecting = 0U;
    if (spread_px > SCALE_CAPTURE_SPREAD_LIMIT_PX) {
        printf("[SCALE_CAPTURE] ms=%lu state=REJECTED step=%s mean_px=%ld "
               "spread_px=%ld limit_px=%d instruction=KEEP_STILL_AND_REPEAT_CAL\r\n",
               (unsigned long)now_ms, captured_step, (long)mean_px,
               (long)spread_px, SCALE_CAPTURE_SPREAD_LIMIT_PX);
        return;
    }

    if (scale->stage == SCALE_WAIT_CENTER) {
        scale->centre_px = mean_px;
        scale->stage = SCALE_WAIT_NEG_50;
        next_instruction = "PUT_BALL_MINUS_50MM_SEND_CAL_LEFT";
    } else if (scale->stage == SCALE_WAIT_NEG_50) {
        scale->neg_50_px = mean_px;
        scale->stage = SCALE_WAIT_POS_50;
        next_instruction = "PUT_BALL_PLUS_50MM_SEND_CAL_RIGHT";
    } else {
        float neg_span = (float)(scale->centre_px - scale->neg_50_px);
        float pos_span;
        scale->pos_50_px = mean_px;
        pos_span = (float)(scale->pos_50_px - scale->centre_px);
        if (AbsFloat(neg_span) <= 5.0f || AbsFloat(pos_span) <= 5.0f ||
            neg_span * pos_span <= 0.0f) {
            printf("[SCALE_RESULT] ms=%lu state=REJECTED centre_px=%ld neg_px=%ld "
                   "pos_px=%ld reason=ORDER_OR_SPAN_INVALID instruction=RESTART_CAL_CENTER\r\n",
                   (unsigned long)now_ms, (long)scale->centre_px,
                   (long)scale->neg_50_px, (long)scale->pos_50_px);
            ScaleReset(scale);
            return;
        }
        scale->neg_mm_per_px = 50.0f / neg_span;
        scale->pos_mm_per_px = 50.0f / pos_span;
        scale->valid = 1U;
        scale->stage = SCALE_READY;
        printf("[SCALE_RESULT] ms=%lu state=READY centre_px=%ld neg50_px=%ld "
               "pos50_px=%ld neg_mm_per_px=%.6f pos_mm_per_px=%.6f "
               "scale_mismatch_pct=%.1f\r\n",
               (unsigned long)now_ms, (long)scale->centre_px,
               (long)scale->neg_50_px, (long)scale->pos_50_px,
               scale->neg_mm_per_px, scale->pos_mm_per_px,
               100.0f * AbsFloat(scale->pos_mm_per_px - scale->neg_mm_per_px) /
                   AbsFloat((scale->pos_mm_per_px + scale->neg_mm_per_px) * 0.5f));
        return;
    }

    printf("[SCALE_CAPTURE] ms=%lu state=ACCEPTED step=%s mean_px=%ld "
           "spread_px=%ld next=%s\r\n",
           (unsigned long)now_ms, captured_step, (long)mean_px,
           (long)spread_px, next_instruction);
}

static void ScaleAddSample(ScaleCalibration_t *scale, int32_t pixel,
                           uint32_t now_ms)
{
    if (scale->collecting == 0U) return;
    if (scale->sample_count == 0U) {
        scale->sample_min = pixel;
        scale->sample_max = pixel;
    } else {
        if (pixel < scale->sample_min) scale->sample_min = pixel;
        if (pixel > scale->sample_max) scale->sample_max = pixel;
    }
    scale->sample_sum += pixel;
    scale->sample_count++;
    if (scale->sample_count >= SCALE_CAPTURE_SAMPLES) {
        ScaleFinishCapture(scale, now_ms);
    }
}

static float ScalePixelToMm(const ScaleCalibration_t *scale, int32_t pixel)
{
    int32_t delta_px;
    if (scale->valid == 0U) return 0.0f;
    delta_px = pixel - scale->centre_px;
    return ((float)delta_px * scale->pos_mm_per_px < 0.0f) ?
           ((float)delta_px * scale->neg_mm_per_px) :
           ((float)delta_px * scale->pos_mm_per_px);
}

/* The clock advances in an interrupt, independently of formatting/UART work. */
static volatile uint32_t s_clock_ms;
static ScaleCalibration_t s_scale;
static uint32_t s_last_frame_ms, s_motor_command_ms;
static uint8_t s_have_frame, s_velocity_ready, s_stream = 1U;
static uint8_t s_motor_reference_valid;
static float s_motor_origin_deg, s_motor_target_deg = INVALID_MEASUREMENT;
static int32_t s_raw_px;
static float s_pos_mm, s_velocity_mm_s, s_previous_pos_mm;
static uint32_t s_last_drop_count;
static uint8_t s_measurement_mode;
static float s_measurement_start_pwm;
static uint8_t s_balance_valid = MEASURED_BALANCE_VALID;
static int32_t s_balance_pwm_mdeg = MEASURED_BALANCE_PWM_MDEG;
static uint8_t s_balance_collecting, s_balance_samples;
static float s_balance_first, s_balance_sum, s_balance_min, s_balance_max;

void SysTick_Handler(void) { s_clock_ms++; }

static uint8_t VisionFresh(uint32_t now)
{
    return s_have_frame && (uint32_t)(now - s_last_frame_ms) <= K230_FRESH_MS;
}

static float WrapDegrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* The measured balance zero and legacy experiment zero are kept separate.
 * These are motor angles, NOT physical pipe slope. */
static float MotorRealAngle(void)
{
    float pwm;
    if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm)) return INVALID_MEASUREMENT;
    if (s_balance_valid)
        return WrapDegrees(pwm - (float)s_balance_pwm_mdeg * 0.001f);
    if (s_measurement_mode)
        return WrapDegrees(pwm - s_measurement_start_pwm);
    return WrapDegrees(pwm - FIXED_PWM_REFERENCE_DEG);
}

static void PrintStatus(uint32_t now)
{
    K230_Stats stats;
    CL_Snapshot_t motor;
    float pwm = INVALID_MEASUREMENT;
    uint8_t pwm_valid = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm);
    K230_GetStats(&stats);
    CL_GetSnapshot(MOTOR_AXIS_X, &motor);
    printf("[STATUS] ms=%lu vision=%u cal=%s sampling=%u raw_px=%ld "
           "pos_mm=%.2f pwm_valid=%u pwm_abs_deg=%.2f motor_deg=%.2f "
           "cmd_deg=%.2f qei=%ld active=%u reached=%u fault=%u "
           "parse=%lu dropped=%lu rx_errors=%lu tx_drops=%lu\r\n",
           (unsigned long)now, (unsigned)VisionFresh(now),
           ScaleStageName(s_scale.stage), (unsigned)s_scale.collecting,
           (long)s_raw_px,
           (s_scale.valid && VisionFresh(now)) ? s_pos_mm : INVALID_MEASUREMENT,
           (unsigned)pwm_valid, pwm_valid ? pwm : INVALID_MEASUREMENT,
           MotorRealAngle(), s_motor_target_deg, (long)motor.current_count,
           (unsigned)motor.active, (unsigned)motor.reached, (unsigned)motor.fault,
           (unsigned long)stats.parse_errors, (unsigned long)stats.dropped_frames,
           (unsigned long)Console_GetRxErrors(), (unsigned long)Console_GetTxDrops());
}

static void StopExperiment(uint32_t now, const char *reason)
{
    CL_StopAll();
    s_scale.collecting = 0;
    s_balance_collecting = 0;
    s_motor_target_deg = MotorRealAngle();
    printf("[STOP] ms=%lu reason=%s pulses=STOPPED driver=ENABLED\r\n",
           (unsigned long)now, reason);
}

static void Capture(ScaleStage_t requested, uint32_t now)
{
    CL_Snapshot_t motor;
    if (s_scale.collecting || s_balance_collecting) {
        printf("[ERR] CAL_BUSY wait_for_result\r\n");
        return;
    }
    if (!VisionFresh(now)) {
        printf("[ERR] VISION_STALE repeat_CAL_when_ball_visible\r\n");
        return;
    }
    CL_GetSnapshot(MOTOR_AXIS_X, &motor);
    if (Motor_IsBusy(MOTOR_AXIS_X) || (motor.active && !motor.reached)) {
        printf("[ERR] MOTOR_MOVING wait_or_send_STOP\r\n");
        return;
    }
    if (requested == SCALE_WAIT_CENTER) ScaleReset(&s_scale);
    if (requested != s_scale.stage) {
        printf("[ERR] CAL_ORDER expected=%s\r\n", ScaleStageName(s_scale.stage));
        return;
    }
    s_velocity_ready = 0;
    s_velocity_mm_s = 0;
    ScaleStartCapture(&s_scale, now);
}

static void CommandAngle(float requested, uint32_t now)
{
    float actual = MotorRealAngle();
    if (s_measurement_mode || s_balance_valid) {
        printf("[ERR] MEASUREMENT_COORDINATES use_JOG_plus1_or_minus1\r\n");
        return;
    }
    if (s_scale.collecting || s_balance_collecting) {
        printf("[ERR] CAL_BUSY angle_not_changed\r\n");
        return;
    }
    if (actual == INVALID_MEASUREMENT) {
        printf("[ERR] PWM_INVALID angle_not_changed\r\n");
        return;
    }
    if (actual < 0.0f || actual > 42.0f) {
        printf("[ERR] MOTOR_OUTSIDE_SETUP_RANGE actual_deg=%.2f expected=0..42\r\n",
               actual);
        return;
    }
    if (CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE) {
        printf("[ERR] MOTOR_FAULT inspect_STATUS_and_hardware\r\n");
        return;
    }
    if (!s_motor_reference_valid) {
        /* Rebase once at the measured physical pose, without moving to zero. */
        CL_SetZero(MOTOR_AXIS_X);
        s_motor_origin_deg = actual;
        s_motor_reference_valid = 1;
    }
    if (CL_SetTargetAngle(MOTOR_AXIS_X, requested - s_motor_origin_deg) != MOTOR_OK) {
        printf("[ERR] ANGLE_REJECTED\r\n");
        return;
    }
    s_motor_command_ms = now;
    s_motor_target_deg = requested;
    printf("[ANGLE] ms=%lu target_deg=%.2f actual_deg=%.2f ref_pwm_deg=49.30\r\n",
           (unsigned long)now, requested, actual);
}

static uint8_t MeasurementReady(float *pwm)
{
    CL_Snapshot_t motor;
    if (s_scale.collecting || s_balance_collecting) {
        printf("[ERR] CAL_BUSY wait_or_send_STOP\r\n");
        return 0;
    }
    if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, pwm)) {
        printf("[ERR] PWM_INVALID measurement_not_started\r\n");
        return 0;
    }
    CL_GetSnapshot(MOTOR_AXIS_X, &motor);
    if (motor.fault != CL_FAULT_NONE) {
        printf("[ERR] MOTOR_FAULT inspect_STATUS_and_hardware\r\n");
        return 0;
    }
    if (Motor_IsBusy(MOTOR_AXIS_X) || (motor.active && !motor.reached)) {
        printf("[ERR] MOTOR_MOVING wait_until_reached_before_next_step\r\n");
        return 0;
    }
    return 1;
}

static void EnterMeasurement(float pwm)
{
    if (!s_measurement_mode) {
        s_measurement_start_pwm = pwm;
        s_measurement_mode = 1;
        s_motor_reference_valid = 0;
        printf("[MEASURE] origin_pwm_deg=%.3f saved_balance=%u boot_pose_unchanged=1\r\n",
               pwm, (unsigned)s_balance_valid);
    }
    if (!s_motor_reference_valid) {
        CL_SetZero(MOTOR_AXIS_X);
        s_motor_origin_deg = s_balance_valid ?
            WrapDegrees(pwm - (float)s_balance_pwm_mdeg * 0.001f) :
            WrapDegrees(pwm - s_measurement_start_pwm);
        s_motor_target_deg = s_motor_origin_deg;
        s_motor_reference_valid = 1;
    }
}

static void JogOneDegree(float delta, uint32_t now)
{
    float pwm, actual, requested, requested_pwm;
    if (!MeasurementReady(&pwm)) return;
    EnterMeasurement(pwm);
    actual = MotorRealAngle();
    if (actual == INVALID_MEASUREMENT) return;
    requested = actual + delta;
    if (requested <= -180.0f || requested >= 180.0f) {
        printf("[ERR] JOG_CROSSES_COORDINATE_WRAP\r\n");
        return;
    }
    if (CL_SetTargetAngle(MOTOR_AXIS_X, requested - s_motor_origin_deg) != MOTOR_OK) {
        printf("[ERR] JOG_REJECTED\r\n");
        return;
    }
    s_motor_target_deg = requested;
    s_motor_command_ms = now;
    requested_pwm = pwm + delta;
    if (requested_pwm < 0.0f) requested_pwm += 360.0f;
    if (requested_pwm >= 360.0f) requested_pwm -= 360.0f;
    printf("[JOG] ms=%lu delta_deg=%.1f from_pwm_deg=%.3f "
           "target_pwm_deg=%.3f target_relative_deg=%.3f\r\n",
           (unsigned long)now, delta, pwm, requested_pwm, requested);
}

static void PrintBalance(void)
{
    if (!s_balance_valid) {
        printf("[BALANCE] valid=0 find_balance_then_send_CAL_BALANCE\r\n");
        return;
    }
    printf("[BALANCE] valid=1 pwm_mdeg=%ld pwm_abs_deg=%.3f "
           "motor_relative_deg=%.3f\r\n",
           (long)s_balance_pwm_mdeg, (float)s_balance_pwm_mdeg * 0.001f,
           MotorRealAngle());
    printf("[COPY_TO_CODE] #define MEASURED_BALANCE_VALID (1U)\r\n");
    printf("[COPY_TO_CODE] #define MEASURED_BALANCE_PWM_MDEG (%ldL)\r\n",
           (long)s_balance_pwm_mdeg);
}

static void StartBalanceCapture(uint32_t now)
{
    float pwm;
    if (!MeasurementReady(&pwm)) return;
    EnterMeasurement(pwm);
    /* Keep driver holding torque; freeze correction pulses during measurement. */
    CL_StopAll();
    s_balance_collecting = 1;
    s_balance_samples = 0;
    s_balance_sum = 0;
    s_balance_min = 0;
    s_balance_max = 0;
    s_motor_target_deg = MotorRealAngle();
    printf("[BALANCE] ms=%lu event=SAMPLING samples=16 period_ms=20 "
           "physical_balance_confirmed_by=USER\r\n", (unsigned long)now);
}

static void BalanceSample20ms(uint32_t now)
{
    float pwm, delta, mean;
    int32_t qei;
    if (!s_balance_collecting) return;
    if (!Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm)) {
        s_balance_collecting = 0;
        printf("[ERR] BALANCE_PWM_LOST repeat_CAL_BALANCE\r\n");
        return;
    }
    if (!s_balance_samples) s_balance_first = pwm;
    /* Average around the first sample, including a possible 359.99/0.01 wrap. */
    delta = WrapDegrees(pwm - s_balance_first);
    s_balance_sum += delta;
    if (delta < s_balance_min) s_balance_min = delta;
    if (delta > s_balance_max) s_balance_max = delta;
    if (++s_balance_samples < BALANCE_SAMPLES) return;
    s_balance_collecting = 0;
    if (s_balance_max - s_balance_min > BALANCE_SPREAD_DEG) {
        printf("[ERR] BALANCE_UNSTABLE spread_deg=%.3f repeat_CAL_BALANCE\r\n",
               s_balance_max - s_balance_min);
        return;
    }
    mean = s_balance_first + s_balance_sum / BALANCE_SAMPLES;
    if (mean < 0.0f) mean += 360.0f;
    if (mean >= 360.0f) mean -= 360.0f;
    s_balance_pwm_mdeg = (int32_t)(mean * 1000.0f + 0.5f);
    if (s_balance_pwm_mdeg >= 360000) s_balance_pwm_mdeg = 0;
    s_balance_valid = 1;
    qei = Encoder_GetCount(ENCODER_AXIS_X);
    /* The display zero changed. Rebase the inner-loop offset on the next jog. */
    CL_StopAll();
    s_motor_reference_valid = 0;
    s_motor_target_deg = MotorRealAngle();
    printf("[BALANCE] ms=%lu event=SAVED_RAM pwm_mdeg=%ld qei_count=%ld "
           "spread_deg=%.3f source_code_update=PENDING\r\n",
           (unsigned long)now, (long)s_balance_pwm_mdeg, (long)qei,
           s_balance_max - s_balance_min);
    PrintBalance();
}

static void PrintHelp(void)
{
    /* Log lines contain no CSV numeric lists; only ball: lines make curves. */
    printf("[HELP] CAL,CENTER | CAL,LEFT | CAL,RIGHT | CAL,RESET\r\n");
    printf("[HELP] ANGLE,31 (24..40 deg) | STOP | STATUS | HELP\r\n");
    printf("[HELP] JOG,+1 | JOG,-1 | CAL,BALANCE | BALANCE,SHOW\r\n");
    printf("[HELP] STREAM,ON | STREAM,OFF ; commands_end_in_LF_or_CRLF\r\n");
    printf("[HELP] CH0=position_mm CH1=velocity_mm_s CH2=motor_cmd_deg "
           "CH3=motor_real_deg period_ms=20 invalid=-9999\r\n");
}

static void HandleCommand(ExperimentCommand command, uint32_t now)
{
    switch (command.type) {
    case EXP_CENTER: Capture(SCALE_WAIT_CENTER, now); break;
    case EXP_LEFT: Capture(SCALE_WAIT_NEG_50, now); break;
    case EXP_RIGHT: Capture(SCALE_WAIT_POS_50, now); break;
    case EXP_RESET:
        ScaleReset(&s_scale);
        s_velocity_ready = 0;
        s_velocity_mm_s = 0;
        printf("[CAL] RESET next=CENTER\r\n");
        break;
    case EXP_ANGLE: CommandAngle(command.angle_deg, now); break;
    case EXP_JOG_PLUS: JogOneDegree(1.0f, now); break;
    case EXP_JOG_MINUS: JogOneDegree(-1.0f, now); break;
    case EXP_BALANCE_CAL: StartBalanceCapture(now); break;
    case EXP_BALANCE_SHOW: PrintBalance(); break;
    case EXP_STOP: StopExperiment(now, "COMMAND"); break;
    case EXP_STATUS: PrintStatus(now); PrintBalance(); break;
    case EXP_STREAM_ON: s_stream = 1; printf("[STREAM] ON\r\n"); break;
    case EXP_STREAM_OFF: s_stream = 0; printf("[STREAM] OFF\r\n"); break;
    case EXP_HELP: PrintHelp(); break;
    default:
        printf("[ERR] BAD_COMMAND_OR_RANGE use_HELP angle_range=24..40\r\n");
        break;
    }
}

static void ReceivePosition(K230_CenterData center, uint32_t now)
{
    uint32_t elapsed = now - s_last_frame_ms;
    uint8_t was_valid = s_scale.valid;
    K230_Stats stats;
    float dt_s;
    K230_GetStats(&stats);
    if (!VisionFresh(now)) s_velocity_ready = 0;
    s_raw_px = center.relative_x;
    /* Protect sums and conversions from malformed position payloads. */
    if (s_raw_px < -32768 || s_raw_px > 32767) {
        s_have_frame = 0;
        return;
    }
    s_have_frame = 1;
    s_last_frame_ms = now;
    ScaleAddSample(&s_scale, s_raw_px, now);
    s_pos_mm = ScalePixelToMm(&s_scale, s_raw_px);
    /* Use camera interval when no received frame was dropped; otherwise use
     * the elapsed interval covering both position samples. */
    dt_s = (float)((center.frame_dt_ms && stats.dropped_frames == s_last_drop_count)
                   ? center.frame_dt_ms : elapsed) * 0.001f;
    if (was_valid && s_scale.valid && s_velocity_ready &&
        dt_s >= 0.005f && dt_s <= 0.25f) {
        float measured = (s_pos_mm - s_previous_pos_mm) / dt_s;
        float alpha = dt_s / (0.10f + dt_s);
        s_velocity_mm_s += alpha * (measured - s_velocity_mm_s);
    } else {
        s_velocity_mm_s = 0;
    }
    s_previous_pos_mm = s_pos_mm;
    s_velocity_ready = s_scale.valid;
    s_last_drop_count = stats.dropped_frames;
}

int main(void)
{
    uint32_t last_service = 0, last_control = 0, last_output = 0;
    uint8_t last_fresh = 0;
    SYSCFG_DL_init();
    /* Legacy remote/demo timer has no role in this serial experiment. */
    DL_TimerG_stopCounter(TIMER_0_INST);
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    Motor_Init();
    Encoder_Init();
    CL_Init();
    CL_StopAll();
    Buttons_Init();
    K230_Init();
    ScaleReset(&s_scale);
    Console_Init();
    SysTick_Config(CPUCLK_FREQ / 1000U);

    printf("[BOOT] BUILD_ID=%s auto_motion=OFF uart0=115200_8N1 "
           "TX=PA10 RX=PA11\r\n", BUILD_ID);
    PrintHelp();
    PrintBalance();

    while (1) {
        K230_CenterData center;
        ExperimentCommand command;
        uint32_t now = s_clock_ms;
        uint8_t fresh;
        CL_Snapshot_t motor;

        if (now != last_service) {
            Encoder_Tick(now - last_service);
            last_service = now;
            Buttons_Update();
            if (Buttons_TakeStartPress()) StopExperiment(now, "PB7");
            else if (Buttons_TakeCalibratePress()) {
                Capture(s_scale.stage == SCALE_READY ? SCALE_WAIT_CENTER :
                        s_scale.stage, now);
            }
        }
        /* Expire a partial capture before allowing a late frame into it. */
        if (s_scale.collecting &&
            (now - s_scale.capture_start_ms >= CAPTURE_TIMEOUT_MS ||
             !VisionFresh(now))) {
            s_scale.collecting = 0;
            printf("[ERR] CAL_TIMEOUT_OR_VISION_LOST repeat_current_CAL\r\n");
        }
        if (K230_PollCenter(&center)) ReceivePosition(center, now);
        fresh = VisionFresh(now);
        if (fresh != last_fresh) {
            last_fresh = fresh;
            printf("[VISION] ms=%lu fresh=%u\r\n",
                   (unsigned long)now, (unsigned)fresh);
        }
        if (!fresh) { s_velocity_ready = 0; s_velocity_mm_s = 0; }

        if (Console_TakeCommand(&command)) HandleCommand(command, now);
        if (now - last_control >= CONTROL_PERIOD_MS) {
            last_control = now;
            CL_GetSnapshot(MOTOR_AXIS_X, &motor);
            if (motor.active && MotorRealAngle() == INVALID_MEASUREMENT) {
                StopExperiment(now, "PWM_LOST");
            } else if (motor.active && !motor.reached &&
                       now - s_motor_command_ms >= MOTOR_TIMEOUT_MS) {
                StopExperiment(now, "MOTOR_TIMEOUT");
            } else {
                CL_Process();
                if (motor.active && CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE)
                    StopExperiment(now, "MOTOR_FAULT");
            }
        }
        if (now - last_output >= TELEMETRY_PERIOD_MS) {
            last_output += TELEMETRY_PERIOD_MS;
            /* Do not burst old samples after a scheduler overrun. */
            if (now - last_output >= TELEMETRY_PERIOD_MS) last_output = now;
            BalanceSample20ms(now);
            if (s_stream) {
                uint8_t valid = s_scale.valid && fresh;
                printf("ball:%.2f,%.2f,%.2f,%.2f\r\n",
                       valid ? s_pos_mm : INVALID_MEASUREMENT,
                       valid ? s_velocity_mm_s : INVALID_MEASUREMENT,
                       s_motor_target_deg, MotorRealAngle());
            }
        }
        Console_DrainTx();
    }
}
