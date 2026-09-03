/*
 * ROUND-033 -- repeatable vision scale-calibration and data-acquisition base.
 *
 * This replaces the automatic trajectory/PID application. It never commands
 * a motor angle. It turns K230 relative pixels into a measured pipe coordinate
 * and publishes clean telemetry for later experiments.
 *
 * PA17: CENTRE -> 50 mm toward motor -> 50 mm away from motor.
 * PB7: print one status record; it never moves the motor.
 */
#include "board.h"
#include "buttons.h"
#include "closed_loop.h"
#include "encoder.h"
#include "k230.h"
#include "motor.h"

#include <stdio.h>

#define BUILD_ID                         "ROUND-033_SCALE_CAL_DAQ_V1"
#define CONTROL_PERIOD_MS                (5U)
#define K230_FRESH_MS                    (250U)
#define SCALE_CAPTURE_SAMPLES            (12U)
#define SCALE_CAPTURE_SPREAD_LIMIT_PX    (8)
#define TELEMETRY_UNCAL_PERIOD_MS        (250U)
#define TELEMETRY_CAL_PERIOD_MS          (100U)

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
}

static void ScaleStartCapture(ScaleCalibration_t *scale, uint32_t now_ms)
{
    scale->collecting = 1U;
    scale->sample_count = 0U;
    scale->sample_sum = 0;
    scale->sample_min = 0;
    scale->sample_max = 0;
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
               "spread_px=%ld limit_px=%d instruction=KEEP_STILL_AND_PRESS_PA17\r\n",
               (unsigned long)now_ms, captured_step, (long)mean_px,
               (long)spread_px, SCALE_CAPTURE_SPREAD_LIMIT_PX);
        return;
    }

    if (scale->stage == SCALE_WAIT_CENTER) {
        scale->centre_px = mean_px;
        scale->stage = SCALE_WAIT_NEG_50;
        next_instruction = "MOVE_BALL_TOWARD_MOTOR_TO_MINUS_50MM_PRESS_PA17";
    } else if (scale->stage == SCALE_WAIT_NEG_50) {
        scale->neg_50_px = mean_px;
        scale->stage = SCALE_WAIT_POS_50;
        next_instruction = "MOVE_BALL_AWAY_FROM_MOTOR_TO_PLUS_50MM_PRESS_PA17";
    } else {
        float neg_span = (float)(scale->centre_px - scale->neg_50_px);
        float pos_span;
        scale->pos_50_px = mean_px;
        pos_span = (float)(scale->pos_50_px - scale->centre_px);
        if (neg_span <= 5.0f || pos_span <= 5.0f) {
            printf("[SCALE_RESULT] ms=%lu state=REJECTED centre_px=%ld neg_px=%ld "
                   "pos_px=%ld reason=ORDER_OR_SPAN_INVALID instruction=RESTART_PA17\r\n",
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
                   ((scale->pos_mm_per_px + scale->neg_mm_per_px) * 0.5f));
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
    return (delta_px < 0) ? ((float)delta_px * scale->neg_mm_per_px) :
                            ((float)delta_px * scale->pos_mm_per_px);
}

static void PrintStatus(uint32_t now_ms, const ScaleCalibration_t *scale,
                        uint8_t vision_valid, int32_t raw_px, float pos_mm,
                        float velocity_mm_s)
{
    CL_Snapshot_t motor;
    K230_Stats stats;
    CL_GetSnapshot(MOTOR_AXIS_X, &motor);
    K230_GetStats(&stats);
    printf("[DAQ] ms=%lu vision=%u raw_px=%ld pos_mm=%.2f vel_mm_s=%.2f "
           "cal=%s motor_target_deg=%.2f motor_actual_deg=%.2f "
           "qei=%ld fb=%s frames=%lu valid=%lu drop=%lu\r\n",
           (unsigned long)now_ms, (unsigned int)vision_valid, (long)raw_px,
           pos_mm, velocity_mm_s, ScaleStageName(scale->stage),
           motor.target_angle_deg, motor.current_angle_deg,
           (long)motor.current_count,
           (motor.feedback_source == CL_FEEDBACK_PWM) ? "PWM" : "QEI",
           (unsigned long)stats.completed_frames,
           (unsigned long)stats.valid_frames,
           (unsigned long)stats.dropped_frames);
}

int main(void)
{
    ScaleCalibration_t scale;
    K230_CenterData center;
    uint32_t now_ms = 0U;
    uint32_t last_frame_ms = 0U;
    uint32_t last_telemetry_ms = 0U;
    uint8_t control_divider = 0U;
    uint8_t have_frame = 0U;
    int32_t raw_px = 0;
    float pos_mm = 0.0f;
    float velocity_mm_s = 0.0f;
    float previous_pos_mm = 0.0f;

    SYSCFG_DL_init();
    Motor_Init();
    Encoder_Init();
    CL_Init();
    CL_StopAll();
    Buttons_Init();
    K230_Init();
    ScaleReset(&scale);

    printf("\r\nBUILD_ID=%s\r\n", BUILD_ID);
    printf("MODE=VISION_SCALE_CALIBRATION_ONLY motor_auto_motion=DISABLED\r\n");
    printf("PA17: CENTER -> TOWARD_MOTOR_-50MM -> AWAY_FROM_MOTOR_+50MM.\r\n");
    printf("PB7: print status only. K230 input=function16 relative_x pixels.\r\n");

    while (1) {
        uint8_t vision_valid;
        uint32_t telemetry_period = (scale.valid != 0U) ?
                                    TELEMETRY_CAL_PERIOD_MS :
                                    TELEMETRY_UNCAL_PERIOD_MS;

        if (K230_PollCenter(&center)) {
            float dt_s;
            raw_px = center.relative_x;
            have_frame = 1U;
            last_frame_ms = now_ms;
            ScaleAddSample(&scale, raw_px, now_ms);
            pos_mm = ScalePixelToMm(&scale, raw_px);
            dt_s = (center.frame_dt_ms > 0U) ?
                   ((float)center.frame_dt_ms / 1000.0f) : 0.0f;
            if (scale.valid != 0U && dt_s > 0.005f && dt_s < 0.25f) {
                float instant_velocity = (pos_mm - previous_pos_mm) / dt_s;
                velocity_mm_s = 0.75f * velocity_mm_s + 0.25f * instant_velocity;
            } else if (scale.valid == 0U) {
                velocity_mm_s = 0.0f;
            }
            previous_pos_mm = pos_mm;
        }

        Buttons_Update();
        if (Buttons_TakeCalibratePress()) {
            if (scale.stage == SCALE_READY) {
                printf("[SCALE] ms=%lu event=RESTART\r\n", (unsigned long)now_ms);
                ScaleReset(&scale);
            }
            if (have_frame == 0U || (now_ms - last_frame_ms) > K230_FRESH_MS) {
                printf("[SCALE_CAPTURE] ms=%lu state=REJECTED reason=VISION_STALE "
                       "instruction=CHECK_K230_AND_PRESS_PA17\r\n",
                       (unsigned long)now_ms);
            } else if (scale.collecting == 0U) {
                ScaleStartCapture(&scale, now_ms);
            }
        }
        if (Buttons_TakeStartPress()) {
            vision_valid = (have_frame != 0U &&
                            (now_ms - last_frame_ms) <= K230_FRESH_MS);
            PrintStatus(now_ms, &scale, vision_valid, raw_px, pos_mm, velocity_mm_s);
        }

        Encoder_Tick(1U);
        control_divider++;
        if (control_divider >= CONTROL_PERIOD_MS) {
            control_divider = 0U;
            /* active stays zero in this build; this only refreshes feedback. */
            CL_Process();
        }

        if ((now_ms - last_telemetry_ms) >= telemetry_period) {
            last_telemetry_ms = now_ms;
            vision_valid = (have_frame != 0U &&
                            (now_ms - last_frame_ms) <= K230_FRESH_MS);
            PrintStatus(now_ms, &scale, vision_valid, raw_px, pos_mm, velocity_mm_s);
        }
        now_ms++;
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}
