/**
 * ROUND-032 keeps the ROUND-031 automatic sequence unchanged and strengthens
 * only the arbitrary-position hold with bounded 22/40 degree frame-driven
 * pulses.
 *
 * Boot: align the fixed physical PWM reference (49.3 deg) when usable;
 * otherwise log a warning and use the current position as temporary QEI zero.
 * Then command +31 deg center/setup angle and wait 10 seconds.
 * After the delay, log any ordinary vision-start rejection and force task one
 * to start: center -> +5 cm -> -5 cm.  The final target must remain inside a
 * 1.2 cm band for one second; larger errors use a short 24/40 degree breakaway
 * pulse followed by visual PD capture.  Fifteen seconds after task one is
 * complete, the latest ball position becomes a persistent visual hold target.
 * PB7/remote START is an emergency stop during any automatic stage. Other
 * button input is ignored so it cannot bypass the automatic safety sequence.
 */
#include "board.h"
#include "buttons.h"
#include "closed_loop.h"
#include "encoder.h"
#include "k230.h"
#include "motor.h"
#include "remote_buttons.h"
#include "visual_trajectory.h"
#include "visual_trajectory_config.h"

#include <stdio.h>

#define APP_CONTROL_PERIOD_MS             (5U)
#define APP_TELEMETRY_PERIOD_MS           (250U)
#define APP_K230_STATS_PERIOD_MS          (1000U)
#define APP_HOLD_TELEMETRY_PERIOD_MS      (500U)
#define APP_HOLD_TELEMETRY_WINDOW_MS      (5000U)
#define HEALTH_MONITOR_PERIOD_MS          (100U)
#define HEALTH_STARTUP_REPORT_MS          (2000U)
#define HEALTH_K230_STALE_MS              (500U)
#define HEALTH_REMOTE_STALE_MS            (2500U)
#define HEALTH_WARNING_HOLD_MS            (1000U)
#define HEALTH_QEI_CONFIRM_COUNTS         (2)
#define HEALTH_PWM_CONFIRM_DEG            (0.20f)
#define FIXED_ZERO_PWM_DEG                (49.300f)
#define MAX_ALIGN_FROM_POSITIVE_DEG       (42.0f)
#define MAX_ALIGN_FROM_NEGATIVE_DEG       (15.0f)
#define ZERO_VERIFY_TOLERANCE_DEG         (1.0f)
#define AUTO_TASK_DELAY_MS                (10000U)
#define TASK1_FINAL_VERIFY_MS             (1000U)
#define ARBITRARY_TARGET_DELAY_MS         (15000U)
#define AUTO_ZERO_ALIGN_TIMEOUT_MS        (10000U)
#define AUTO_SETUP_ALIGN_TIMEOUT_MS       (10000U)

typedef enum {
    APP_ZERO_WAIT = 0,
    APP_ZERO_ALIGNING,
    APP_SETUP_ALIGNING,
    APP_AUTO_COUNTDOWN,
    APP_TASK1_RUN,
    APP_TASK1_VERIFY,
    APP_RETARGET_WAIT,
    APP_ARBITRARY_HOLD,
    APP_ABORTED
} AppStage_t;

typedef enum {
    K230_HEALTH_WAIT = 0,
    K230_HEALTH_OK,
    K230_HEALTH_STALE,
    K230_HEALTH_DATA_ERROR,
    K230_HEALTH_RX_DROPPED,
    K230_HEALTH_COUNT
} K230Health_t;

typedef enum {
    ENCODER_HEALTH_PWM_INVALID = 0,
    ENCODER_HEALTH_QEI_UNTESTED,
    ENCODER_HEALTH_OK,
    ENCODER_HEALTH_QEI_FALLBACK,
    ENCODER_HEALTH_NO_FEEDBACK,
    ENCODER_HEALTH_DIRECTION,
    ENCODER_HEALTH_COUNT
} EncoderHealth_t;

typedef enum {
    MOTOR_HEALTH_UNTESTED = 0,
    MOTOR_HEALTH_MOVING,
    MOTOR_HEALTH_OK,
    MOTOR_HEALTH_FAULT,
    MOTOR_HEALTH_COUNT
} MotorHealth_t;

typedef enum {
    REMOTE_HEALTH_WAIT = 0,
    REMOTE_HEALTH_OK,
    REMOTE_HEALTH_STALE,
    REMOTE_HEALTH_PULSE_ERROR,
    REMOTE_HEALTH_STUCK_LOW,
    REMOTE_HEALTH_COUNT
} RemoteHealth_t;

typedef struct {
    uint8_t startup_reported;
    K230Health_t last_k230;
    EncoderHealth_t last_encoder;
    MotorHealth_t last_motor;
    RemoteHealth_t last_remote;
    K230_Stats previous_k230_stats;
    uint32_t previous_remote_invalid;
    uint32_t previous_remote_overruns;
    uint32_t previous_remote_stale;
    uint32_t previous_remote_stuck;
    uint32_t last_rate_ms;
    uint32_t last_rate_valid_frames;
    uint32_t frame_rate_x10;
    uint32_t k230_error_until_ms;
    uint32_t k230_drop_until_ms;
    uint32_t remote_error_until_ms;
    uint32_t remote_stuck_until_ms;
    uint8_t motion_tracking;
    uint8_t motion_qei_seen;
    uint8_t motion_pwm_seen;
    uint8_t qei_verified;
    uint8_t pwm_motion_verified;
    uint8_t motor_verified;
    int32_t motion_start_qei;
    float motion_start_pwm_deg;
    uint8_t motion_start_pwm_valid;
} HealthContext_t;

static HealthContext_t s_health;

static int32_t ScaleFloat(float value, float scale)
{
    float scaled = value * scale;
    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) :
                              (int32_t)(scaled - 0.5f);
}

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float WrapAngleDelta(float delta)
{
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

static const char *AppStageName(AppStage_t stage)
{
    switch (stage) {
    case APP_ZERO_WAIT: return "ZERO_WAIT";
    case APP_ZERO_ALIGNING: return "ZERO_ALIGNING";
    case APP_SETUP_ALIGNING: return "SETUP_ALIGNING";
    case APP_AUTO_COUNTDOWN: return "AUTO_COUNTDOWN";
    case APP_TASK1_RUN: return "TASK1_RUN";
    case APP_TASK1_VERIFY: return "TASK1_VERIFY";
    case APP_RETARGET_WAIT: return "RETARGET_WAIT";
    case APP_ARBITRARY_HOLD: return "ARBITRARY_HOLD";
    default: return "ABORTED";
    }
}

static uint8_t IsVisualControlStage(AppStage_t stage)
{
    return (stage == APP_TASK1_RUN ||
            stage == APP_TASK1_VERIFY ||
            stage == APP_ARBITRARY_HOLD) ? 1U : 0U;
}

static uint8_t IsTrajectoryMoving(VT_State_t state)
{
    switch (state) {
    case VT_STATE_OUTBOUND_DRIVE:
    case VT_STATE_OUTBOUND_BRAKE:
    case VT_STATE_RETURN_DRIVE:
    case VT_STATE_RETURN_BRAKE:
    case VT_STATE_CAPTURE:
    case VT_STATE_HOLD_BREAKAWAY:
        return 1U;
    default:
        return 0U;
    }
}

static const char *FeedbackName(CL_FeedbackSource_t source)
{
    return (source == CL_FEEDBACK_PWM) ? "PWM" : "QEI";
}

static const char *FaultName(CL_Fault_t fault)
{
    switch (fault) {
    case CL_FAULT_NONE: return "NONE";
    case CL_FAULT_NO_ENCODER: return "NO_ENCODER";
    case CL_FAULT_DIRECTION: return "DIRECTION";
    default: return "DRIVER";
    }
}

static int32_t HealthAbsInt32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static const char *K230HealthName(K230Health_t state)
{
    switch (state) {
    case K230_HEALTH_WAIT: return "WAIT_VISIBLE_BALL";
    case K230_HEALTH_OK: return "OK";
    case K230_HEALTH_STALE: return "STALE_CHECK_BALL_OR_LINK";
    case K230_HEALTH_DATA_ERROR: return "DATA_ERROR";
    case K230_HEALTH_RX_DROPPED: return "RX_DROPPED";
    default: return "UNKNOWN";
    }
}

static const char *EncoderHealthName(EncoderHealth_t state)
{
    switch (state) {
    case ENCODER_HEALTH_PWM_INVALID: return "PWM_INVALID";
    case ENCODER_HEALTH_QEI_UNTESTED: return "PWM_OK_QEI_UNTESTED";
    case ENCODER_HEALTH_OK: return "OK_PWM_QEI";
    case ENCODER_HEALTH_QEI_FALLBACK: return "QEI_FALLBACK_PWM_ONLY";
    case ENCODER_HEALTH_NO_FEEDBACK: return "NO_ENCODER_FEEDBACK";
    case ENCODER_HEALTH_DIRECTION: return "DIRECTION_ERROR";
    default: return "UNKNOWN";
    }
}

static const char *MotorHealthName(MotorHealth_t state)
{
    switch (state) {
    case MOTOR_HEALTH_UNTESTED: return "UNTESTED_WAIT_AUTO_MOTION";
    case MOTOR_HEALTH_MOVING: return "MOVING";
    case MOTOR_HEALTH_OK: return "OK_REACHED";
    case MOTOR_HEALTH_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

static const char *RemoteHealthName(RemoteHealth_t state)
{
    switch (state) {
    case REMOTE_HEALTH_WAIT: return "WAIT_FIRST_PULSE";
    case REMOTE_HEALTH_OK: return "OK";
    case REMOTE_HEALTH_STALE: return "STALE_NO_HEARTBEAT";
    case REMOTE_HEALTH_PULSE_ERROR: return "PULSE_ERROR";
    case REMOTE_HEALTH_STUCK_LOW: return "PB2_STUCK_LOW";
    default: return "UNKNOWN";
    }
}

static void HealthInit(void)
{
    RemoteButtons_Stats remote_stats;

    K230_GetStats(&s_health.previous_k230_stats);
    RemoteButtons_GetStats(&remote_stats);
    s_health.startup_reported = 0U;
    s_health.last_k230 = K230_HEALTH_COUNT;
    s_health.last_encoder = ENCODER_HEALTH_COUNT;
    s_health.last_motor = MOTOR_HEALTH_COUNT;
    s_health.last_remote = REMOTE_HEALTH_COUNT;
    s_health.previous_remote_invalid = remote_stats.invalid_pulses;
    s_health.previous_remote_overruns = remote_stats.event_overruns;
    s_health.previous_remote_stale = remote_stats.stale_events;
    s_health.previous_remote_stuck = remote_stats.stuck_events;
    s_health.last_rate_ms = 0U;
    s_health.last_rate_valid_frames =
        s_health.previous_k230_stats.valid_frames;
    s_health.frame_rate_x10 = 0U;
    s_health.k230_error_until_ms = 0U;
    s_health.k230_drop_until_ms = 0U;
    s_health.remote_error_until_ms = 0U;
    s_health.remote_stuck_until_ms = 0U;
    s_health.motion_tracking = 0U;
    s_health.motion_qei_seen = 0U;
    s_health.motion_pwm_seen = 0U;
    s_health.qei_verified = 0U;
    s_health.pwm_motion_verified = 0U;
    s_health.motor_verified = 0U;
    s_health.motion_start_qei = 0;
    s_health.motion_start_pwm_deg = 0.0f;
    s_health.motion_start_pwm_valid = 0U;
}

static void HealthObserveMotion(const CL_Snapshot_t *cl)
{
    uint8_t moving;
    int32_t qei_count;
    float pwm_angle = 0.0f;
    uint8_t pwm_valid;

    moving = ((cl->active != 0U && cl->reached == 0U) ||
              Motor_IsBusy(MOTOR_AXIS_X) != 0U) ? 1U : 0U;
    qei_count = Encoder_GetCount(ENCODER_AXIS_X);
    pwm_valid = Encoder_GetPwmAngle(ENCODER_AXIS_X, &pwm_angle);

    if (moving != 0U) {
        if (s_health.motion_tracking == 0U) {
            s_health.motion_tracking = 1U;
            s_health.motion_qei_seen = 0U;
            s_health.motion_pwm_seen = 0U;
            s_health.motion_start_qei = qei_count;
            s_health.motion_start_pwm_deg = pwm_angle;
            s_health.motion_start_pwm_valid = pwm_valid;
        } else {
            if (HealthAbsInt32(qei_count - s_health.motion_start_qei) >=
                HEALTH_QEI_CONFIRM_COUNTS) {
                s_health.motion_qei_seen = 1U;
                s_health.qei_verified = 1U;
            }
            if (pwm_valid != 0U &&
                s_health.motion_start_pwm_valid != 0U &&
                AbsFloat(WrapAngleDelta(
                    pwm_angle - s_health.motion_start_pwm_deg)) >=
                    HEALTH_PWM_CONFIRM_DEG) {
                s_health.motion_pwm_seen = 1U;
                s_health.pwm_motion_verified = 1U;
            }
        }
        return;
    }

    if (s_health.motion_tracking != 0U) {
        if (cl->fault == CL_FAULT_NONE &&
            (s_health.motion_qei_seen != 0U ||
             s_health.motion_pwm_seen != 0U)) {
            s_health.motor_verified = 1U;
        }
        s_health.motion_tracking = 0U;
    }
}

static void PrintK230Health(uint32_t now_ms, const char *reason,
                            K230Health_t state, const K230_Stats *stats,
                            const VT_Snapshot_t *vt)
{
    printf("[CHECK_K230] ms=%lu reason=%s status=%s fps_x10=%lu "
           "age=%lu complete=%lu valid=%lu parse=%lu overflow=%lu "
           "dropped=%lu\r\n",
           (unsigned long)now_ms, reason, K230HealthName(state),
           (unsigned long)s_health.frame_rate_x10,
           (unsigned long)vt->vision_age_ms,
           (unsigned long)stats->completed_frames,
           (unsigned long)stats->valid_frames,
           (unsigned long)stats->parse_errors,
           (unsigned long)stats->overflow_errors,
           (unsigned long)stats->dropped_frames);
}

static void PrintEncoderHealth(uint32_t now_ms, const char *reason,
                               EncoderHealth_t state,
                               const CL_Snapshot_t *cl)
{
    printf("[CHECK_ENCODER] ms=%lu reason=%s status=%s pwm_mdeg=%ld "
           "qei=%ld fb=%s qei_tested=%u pwm_motion=%u fault=%s\r\n",
           (unsigned long)now_ms, reason, EncoderHealthName(state),
           (long)(cl->pwm_valid ?
               ScaleFloat(cl->pwm_angle_deg, 1000.0f) : -1),
           (long)Encoder_GetCount(ENCODER_AXIS_X),
           FeedbackName(cl->feedback_source),
           (unsigned int)s_health.qei_verified,
           (unsigned int)s_health.pwm_motion_verified,
           FaultName(cl->fault));
}

static void PrintMotorHealth(uint32_t now_ms, const char *reason,
                             MotorHealth_t state, const CL_Snapshot_t *cl)
{
    printf("[CHECK_MOTOR] ms=%lu reason=%s status=%s active=%u reached=%u "
           "busy=%u enc=%ld target=%ld err=%ld fault=%s\r\n",
           (unsigned long)now_ms, reason, MotorHealthName(state),
           (unsigned int)cl->active, (unsigned int)cl->reached,
           (unsigned int)Motor_IsBusy(MOTOR_AXIS_X),
           (long)cl->current_count, (long)cl->target_count,
           (long)cl->error_count, FaultName(cl->fault));
}

static void PrintRemoteHealth(uint32_t now_ms, const char *reason,
                              RemoteHealth_t state,
                              const RemoteButtons_Stats *stats)
{
    printf("[CHECK_REMOTE] ms=%lu reason=%s status=%s pin=PB2 line=%s "
           "low_ms=%u seen=%u age=%lu valid=%lu hb=%lu cal=%lu "
           "start=%lu invalid=%lu overrun=%lu stale=%lu stuck=%lu\r\n",
           (unsigned long)now_ms, reason, RemoteHealthName(state),
           (stats->line_high != 0U) ? "HIGH" : "LOW",
           (unsigned int)stats->current_low_ms,
           (unsigned int)stats->link_seen,
           (unsigned long)stats->last_valid_age_ms,
           (unsigned long)stats->valid_pulses,
           (unsigned long)stats->heartbeat_pulses,
           (unsigned long)stats->calibrate_pulses,
           (unsigned long)stats->start_pulses,
           (unsigned long)stats->invalid_pulses,
           (unsigned long)stats->event_overruns,
           (unsigned long)stats->stale_events,
           (unsigned long)stats->stuck_events);
}

static void HealthUpdate(uint32_t now_ms, const VT_Snapshot_t *vt)
{
    K230_Stats stats;
    CL_Snapshot_t cl;
    K230Health_t k230_state;
    EncoderHealth_t encoder_state;
    MotorHealth_t motor_state;
    RemoteButtons_Stats remote_stats;
    RemoteHealth_t remote_state;
    uint32_t elapsed_ms;
    uint32_t valid_delta;
    uint8_t moving;

    K230_GetStats(&stats);
    RemoteButtons_GetStats(&remote_stats);
    CL_GetSnapshot(MOTOR_AXIS_X, &cl);
    HealthObserveMotion(&cl);

    elapsed_ms = now_ms - s_health.last_rate_ms;
    if (elapsed_ms >= 1000U) {
        valid_delta = stats.valid_frames -
                      s_health.last_rate_valid_frames;
        s_health.frame_rate_x10 =
            (valid_delta * 10000U) / elapsed_ms;
        s_health.last_rate_ms = now_ms;
        s_health.last_rate_valid_frames = stats.valid_frames;
    }

    if (stats.parse_errors > s_health.previous_k230_stats.parse_errors ||
        stats.overflow_errors >
            s_health.previous_k230_stats.overflow_errors) {
        s_health.k230_error_until_ms = now_ms + HEALTH_WARNING_HOLD_MS;
    }
    if (stats.dropped_frames >
        s_health.previous_k230_stats.dropped_frames) {
        s_health.k230_drop_until_ms = now_ms + HEALTH_WARNING_HOLD_MS;
    }
    s_health.previous_k230_stats = stats;

    if (remote_stats.invalid_pulses > s_health.previous_remote_invalid ||
        remote_stats.event_overruns > s_health.previous_remote_overruns ||
        remote_stats.stale_events > s_health.previous_remote_stale) {
        s_health.remote_error_until_ms = now_ms + HEALTH_WARNING_HOLD_MS;
    }
    if (remote_stats.stuck_events > s_health.previous_remote_stuck) {
        s_health.remote_stuck_until_ms = now_ms + HEALTH_WARNING_HOLD_MS;
    }
    s_health.previous_remote_invalid = remote_stats.invalid_pulses;
    s_health.previous_remote_overruns = remote_stats.event_overruns;
    s_health.previous_remote_stale = remote_stats.stale_events;
    s_health.previous_remote_stuck = remote_stats.stuck_events;

    if (now_ms < s_health.k230_error_until_ms) {
        k230_state = K230_HEALTH_DATA_ERROR;
    } else if (now_ms < s_health.k230_drop_until_ms) {
        k230_state = K230_HEALTH_RX_DROPPED;
    } else if (vt->position_valid != 0U &&
               vt->vision_age_ms <= HEALTH_K230_STALE_MS) {
        k230_state = K230_HEALTH_OK;
    } else if (now_ms < HEALTH_STARTUP_REPORT_MS &&
               stats.valid_frames == 0U) {
        k230_state = K230_HEALTH_WAIT;
    } else {
        k230_state = K230_HEALTH_STALE;
    }

    if (cl.fault == CL_FAULT_DIRECTION) {
        encoder_state = ENCODER_HEALTH_DIRECTION;
    } else if (cl.fault == CL_FAULT_NO_ENCODER) {
        encoder_state = ENCODER_HEALTH_NO_FEEDBACK;
    } else if (cl.pwm_valid == 0U) {
        encoder_state = ENCODER_HEALTH_PWM_INVALID;
    } else if (cl.feedback_source == CL_FEEDBACK_PWM) {
        encoder_state = ENCODER_HEALTH_QEI_FALLBACK;
    } else if (s_health.qei_verified != 0U) {
        encoder_state = ENCODER_HEALTH_OK;
    } else {
        encoder_state = ENCODER_HEALTH_QEI_UNTESTED;
    }

    moving = ((cl.active != 0U && cl.reached == 0U) ||
              Motor_IsBusy(MOTOR_AXIS_X) != 0U) ? 1U : 0U;
    if (cl.fault != CL_FAULT_NONE) {
        motor_state = MOTOR_HEALTH_FAULT;
    } else if (moving != 0U) {
        motor_state = MOTOR_HEALTH_MOVING;
    } else if (s_health.motor_verified != 0U) {
        motor_state = MOTOR_HEALTH_OK;
    } else {
        motor_state = MOTOR_HEALTH_UNTESTED;
    }

    if (remote_stats.stuck_low != 0U ||
        now_ms < s_health.remote_stuck_until_ms) {
        remote_state = REMOTE_HEALTH_STUCK_LOW;
    } else if (now_ms < s_health.remote_error_until_ms) {
        remote_state = REMOTE_HEALTH_PULSE_ERROR;
    } else if (remote_stats.link_seen == 0U) {
        remote_state = REMOTE_HEALTH_WAIT;
    } else if (remote_stats.last_valid_age_ms > HEALTH_REMOTE_STALE_MS) {
        remote_state = REMOTE_HEALTH_STALE;
    } else {
        remote_state = REMOTE_HEALTH_OK;
    }

    if (s_health.startup_reported == 0U) {
        if (now_ms < HEALTH_STARTUP_REPORT_MS) return;
        s_health.startup_reported = 1U;
        printf("[CHECK_BEGIN] ms=%lu visible_ball_required=1 "
               "auto_reset_tests_QEI_and_motor=1\r\n",
               (unsigned long)now_ms);
        PrintK230Health(now_ms, "STARTUP", k230_state, &stats, vt);
        PrintEncoderHealth(now_ms, "STARTUP", encoder_state, &cl);
        PrintMotorHealth(now_ms, "STARTUP", motor_state, &cl);
        PrintRemoteHealth(now_ms, "STARTUP", remote_state, &remote_stats);
        s_health.last_k230 = k230_state;
        s_health.last_encoder = encoder_state;
        s_health.last_motor = motor_state;
        s_health.last_remote = remote_state;
        return;
    }

    if (k230_state != s_health.last_k230) {
        PrintK230Health(now_ms, "CHANGE", k230_state, &stats, vt);
        s_health.last_k230 = k230_state;
    }
    if (encoder_state != s_health.last_encoder) {
        PrintEncoderHealth(now_ms, "CHANGE", encoder_state, &cl);
        s_health.last_encoder = encoder_state;
    }
    if (motor_state != s_health.last_motor) {
        PrintMotorHealth(now_ms, "CHANGE", motor_state, &cl);
        s_health.last_motor = motor_state;
    }
    if (remote_state != s_health.last_remote) {
        PrintRemoteHealth(now_ms, "CHANGE", remote_state, &remote_stats);
        s_health.last_remote = remote_state;
    }
}

static void PrintZero(const char *tag, float current, float error)
{
    printf("[ZERO] %s ref_mdeg=%ld current_mdeg=%ld error_mdeg=%ld\r\n",
           tag,
           (long)ScaleFloat(FIXED_ZERO_PWM_DEG, 1000.0f),
           (long)ScaleFloat(current, 1000.0f),
           (long)ScaleFloat(error, 1000.0f));
}

static void PrintZeroInvalid(const char *tag)
{
    printf("[ZERO] %s ref_mdeg=%ld current_mdeg=NA error_mdeg=NA\r\n",
           tag, (long)ScaleFloat(FIXED_ZERO_PWM_DEG, 1000.0f));
}

static void PrintAutoCountdown(uint32_t now_ms, uint32_t deadline_ms)
{
    printf("[AUTO] ms=%lu event=TASK1_COUNTDOWN delay_ms=%lu "
           "deadline_ms=%lu place_ball_at_physical_center=1\r\n",
           (unsigned long)now_ms,
           (unsigned long)AUTO_TASK_DELAY_MS,
           (unsigned long)deadline_ms);
}

static void PrintTask1Verify(uint32_t now_ms, const VT_Snapshot_t *vt,
                             const char *action)
{
    int32_t error_px = vt->final_target_pixel - vt->raw_pixel;
    printf("[TASK1_VERIFY] ms=%lu error_px=%ld threshold_px=%ld "
           "speed_x10=%ld action=%s\r\n",
           (unsigned long)now_ms,
           (long)error_px,
           (long)ScaleFloat(VT_TASK1_HOLD_EXIT_ERROR_PX, 1.0f),
           (long)ScaleFloat(vt->velocity_px_s, 10.0f),
           action);
}

static void PrintRetarget(uint32_t now_ms, const VT_Snapshot_t *vt,
                          const char *event)
{
    printf("[RETARGET] ms=%lu event=%s fresh=%u age_ms=%lu "
           "raw_px=%ld target_px=%ld deadband_px=%ld\r\n",
           (unsigned long)now_ms,
           event,
           (unsigned int)(vt->position_valid != 0U &&
                          vt->vision_age_ms <= VT_VISION_TIMEOUT_MS),
           (unsigned long)vt->vision_age_ms,
           (long)vt->raw_pixel,
           (long)vt->final_target_pixel,
           (long)ScaleFloat(vt->hold_exit_error_px, 1.0f));
}

static void PrintAppEvent(const char *event, AppStage_t stage,
                          uint32_t now_ms)
{
    printf("[APP_EVENT] ms=%lu event=%s app=%s\r\n",
           (unsigned long)now_ms, event, AppStageName(stage));
}

static void PrintTrajectory(const char *tag, AppStage_t app_stage,
                            uint32_t now_ms)
{
    VT_Snapshot_t vt;
    CL_Snapshot_t cl;
    VisualTrajectory_GetSnapshot(&vt);
    CL_GetSnapshot(MOTOR_AXIS_X, &cl);
    printf("[VT] %s ms=%lu app=%s s=%s ev=%s px=%ld c=%ld p5=%ld n5=%ld "
            "vr=%ld vf=%ld rem=%ld pred=%ld cmd=%ld hold=%ld db=%ld persist=%u "
           "age=%lu sa=%lu ta=%lu "
           "ok=%lu rej=%lu fb=%s pw=%ld enc=%ld et=%ld ee=%ld busy=%u "
           "fault=%s\r\n",
           tag, (unsigned long)now_ms, AppStageName(app_stage),
           VisualTrajectory_StateName(vt.state),
           VisualTrajectory_EventName(vt.last_event),
           (long)vt.raw_pixel, (long)vt.center_pixel,
           (long)vt.first_target_pixel, (long)vt.final_target_pixel,
            (long)ScaleFloat(vt.regression_velocity_px_s, 10.0f),
            (long)ScaleFloat(vt.velocity_px_s, 10.0f),
           (long)ScaleFloat(vt.remaining_px, 10.0f),
           (long)ScaleFloat(vt.predicted_stop_px, 10.0f),
            (long)ScaleFloat(vt.requested_angle_deg, 1000.0f),
            (long)ScaleFloat(vt.hold_angle_deg, 1000.0f),
            (long)ScaleFloat(vt.hold_exit_error_px, 1.0f),
            (unsigned int)vt.persistent_hold_mode,
           (unsigned long)vt.vision_age_ms,
           (unsigned long)vt.state_age_ms,
           (unsigned long)vt.trajectory_age_ms,
           (unsigned long)vt.accepted_frames,
           (unsigned long)vt.rejected_frames,
           FeedbackName(cl.feedback_source),
           (long)(cl.pwm_valid ?
               ScaleFloat(cl.pwm_angle_deg, 1000.0f) : -1),
           (long)cl.current_count, (long)cl.target_count,
           (long)cl.error_count,
           (unsigned int)Motor_IsBusy(MOTOR_AXIS_X),
           FaultName(cl.fault));
}

static void PrintK230Stats(uint32_t now_ms)
{
    K230_Stats stats;
    K230_GetStats(&stats);
    printf("[K230_STATS] ms=%lu complete=%lu valid=%lu parse=%lu "
           "overflow=%lu dropped=%lu\r\n",
           (unsigned long)now_ms,
           (unsigned long)stats.completed_frames,
           (unsigned long)stats.valid_frames,
           (unsigned long)stats.parse_errors,
           (unsigned long)stats.overflow_errors,
           (unsigned long)stats.dropped_frames);
}

/* Keep periodic output short so UART0 printing does not starve UART2/control. */
static void PrintTrajectorySample(uint32_t now_ms)
{
    VT_Snapshot_t vt;
    CL_Snapshot_t cl;
    VisualTrajectory_GetSnapshot(&vt);
    CL_GetSnapshot(MOTOR_AXIS_X, &cl);
    printf("[V] ms=%lu s=%s px=%ld target=%ld err=%ld db=%ld "
           "vr=%ld vf=%ld rem=%ld pred=%ld cmd=%ld age=%lu "
           "ok=%lu rej=%lu pw=%ld ee=%ld\r\n",
           (unsigned long)now_ms,
           VisualTrajectory_StateName(vt.state),
           (long)vt.raw_pixel,
           (long)vt.final_target_pixel,
           (long)(vt.final_target_pixel - vt.raw_pixel),
           (long)ScaleFloat(vt.hold_exit_error_px, 1.0f),
           (long)ScaleFloat(vt.regression_velocity_px_s, 10.0f),
           (long)ScaleFloat(vt.velocity_px_s, 10.0f),
           (long)ScaleFloat(vt.remaining_px, 10.0f),
           (long)ScaleFloat(vt.predicted_stop_px, 10.0f),
           (long)ScaleFloat(vt.requested_angle_deg, 1000.0f),
           (unsigned long)vt.vision_age_ms,
           (unsigned long)vt.accepted_frames,
           (unsigned long)vt.rejected_frames,
           (long)(cl.pwm_valid ?
               ScaleFloat(cl.pwm_angle_deg, 1000.0f) : -1),
           (long)cl.error_count);
}

static uint8_t CommandAngle(float angle, uint32_t now_ms,
                            AppStage_t app_stage)
{
    float minimum_angle = (app_stage == APP_ARBITRARY_HOLD) ?
                          VT_PERSISTENT_MIN_ANGLE_DEG :
                          VT_MIN_ANGLE_DEG;
    float maximum_angle = (app_stage == APP_ARBITRARY_HOLD) ?
                          VT_PERSISTENT_MAX_ANGLE_DEG :
                          VT_MAX_ANGLE_DEG;
    if (angle < minimum_angle || angle > maximum_angle ||
        CL_SetTargetAngle(MOTOR_AXIS_X, angle) != MOTOR_OK) {
        printf("[MOTOR_COMMAND] ms=%lu result=REJECT angle_mdeg=%ld app=%s\r\n",
               (unsigned long)now_ms,
               (long)ScaleFloat(angle, 1000.0f),
               AppStageName(app_stage));
        return 0U;
    }
    return 1U;
}

int main(void)
{
    uint32_t loop_ms = 0U;
    uint32_t control_divider = 0U;
    uint32_t warmup;
    uint8_t zero_fallback_required = 0U;
    uint8_t startup_pwm_valid = 0U;
    float startup_pwm = 0.0f;
    float zero_delta = 0.0f;
    uint32_t stage_deadline_ms = 0U;
    uint32_t auto_start_deadline_ms = 0U;
    uint32_t task1_verify_since_ms = 0U;
    uint32_t retarget_deadline_ms = 0U;
    uint8_t task1_verify_timing = 0U;
    uint8_t task1_correction_used = 0U;
    AppStage_t app_stage = APP_ZERO_WAIT;
    VT_State_t last_vt_state = VT_STATE_READY;

    SYSCFG_DL_init();
    Motor_Init();
    Encoder_Init();
    CL_Init();
    Buttons_Init();
    K230_Init();
    VisualTrajectory_Init();

    for (warmup = 0U; warmup < 1000U; warmup++) {
        Encoder_Tick(1U);
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
    RemoteButtons_Init();
    CL_SetZero(MOTOR_AXIS_X);

    if (Encoder_GetPwmAngle(ENCODER_AXIS_X, &startup_pwm) == 0U) {
        zero_fallback_required = 1U;
    } else {
        startup_pwm_valid = 1U;
        zero_delta = WrapAngleDelta(FIXED_ZERO_PWM_DEG - startup_pwm);
        if (zero_delta < -MAX_ALIGN_FROM_POSITIVE_DEG ||
            zero_delta > MAX_ALIGN_FROM_NEGATIVE_DEG) {
            zero_fallback_required = 1U;
        }
    }
    HealthInit();

    printf("\r\nBUILD_ID=ROUND-032_PERSISTENT_22_40_FRAME_V1\r\n");
    printf("AUTO: absolute zero when valid, else current-position zero; "
           "+31 deg -> wait 10 s -> task1.\r\n");
    printf("AUTO: start checks are logged, then task1 is forced to start.\r\n");
    printf("TASK1: final error <=1.2 cm is not corrected; larger error uses "
           "24/40 deg breakaway plus PD.\r\n");
    printf("AFTER TASK1: wait 15 s without visual correction, then lock the "
           "current ball position and hold it.\r\n");
    printf("FINAL HOLD: every new K230 frame is checked immediately; "
           "two-frame confirmation, 22/40 deg bounded pulses, 32 deg pause.\r\n");
    printf("TIMING: K230 about 30 fps, MCU polls each 1 ms, motor command "
           "service each 5 ms; 10 us cannot contain a new camera position.\r\n");
    printf("WARNING: motor moves automatically after every boot; keep clear.\r\n");
    printf("No three-point calibration: task1 captures current ball position as center.\r\n");
    printf("Coordinates: left=toward motor=negative; right=away=positive.\r\n");
    printf("Angles: task1 remains 24..40 deg; final hold alone uses "
           "22/32/40 deg.\r\n");
    printf("UART: events always; motion 4 Hz; HOLD first 5 s; idle silent.\r\n");
    printf("Health: keep ball visible; first CHECK report after 2 s.\r\n");
    printf("QEI and motor are tested by the automatic reset movement.\r\n");
    printf("Remote RX: PB2 active-low one-wire, sampled by 1 ms ISR.\r\n");
    printf("Remote wiring requires common GND and 3.3 V logic.\r\n");
    printf("Remote pulses: HEARTBEAT=20 ms CAL=100 ms START=400 ms.\r\n");
    if (startup_pwm_valid == 0U) {
        PrintZeroInvalid("AUTO_ZERO_FALLBACK_PWM_INVALID_CURRENT_IS_ZERO");
        if (CommandAngle(VT_SETUP_ANGLE_DEG, loop_ms,
                         APP_SETUP_ALIGNING) != 0U) {
            app_stage = APP_SETUP_ALIGNING;
            stage_deadline_ms = loop_ms + AUTO_SETUP_ALIGN_TIMEOUT_MS;
            PrintAppEvent("AUTO_FALLBACK_COMMAND_31_STARTED",
                          app_stage, loop_ms);
        } else {
            CL_StopAll();
            app_stage = APP_ABORTED;
            PrintAppEvent("AUTO_ABORT_FALLBACK_31_COMMAND_REJECTED",
                          app_stage, loop_ms);
        }
    } else if (zero_fallback_required != 0U) {
        PrintZero("AUTO_ZERO_FALLBACK_UNSAFE_DELTA_CURRENT_IS_ZERO",
                  startup_pwm, zero_delta);
        if (CommandAngle(VT_SETUP_ANGLE_DEG, loop_ms,
                         APP_SETUP_ALIGNING) != 0U) {
            app_stage = APP_SETUP_ALIGNING;
            stage_deadline_ms = loop_ms + AUTO_SETUP_ALIGN_TIMEOUT_MS;
            PrintAppEvent("AUTO_FALLBACK_COMMAND_31_STARTED",
                          app_stage, loop_ms);
        } else {
            CL_StopAll();
            app_stage = APP_ABORTED;
            PrintAppEvent("AUTO_ABORT_FALLBACK_31_COMMAND_REJECTED",
                          app_stage, loop_ms);
        }
    } else if (CL_SetTargetAngle(MOTOR_AXIS_X, zero_delta) == MOTOR_OK) {
        app_stage = APP_ZERO_ALIGNING;
        stage_deadline_ms = loop_ms + AUTO_ZERO_ALIGN_TIMEOUT_MS;
        PrintZero("AUTO_ALIGN_COMMAND", startup_pwm, zero_delta);
        PrintAppEvent("AUTO_ZERO_ALIGN_STARTED", app_stage, loop_ms);
    } else {
        CL_StopAll();
        app_stage = APP_ABORTED;
        PrintAppEvent("AUTO_ABORT_ZERO_COMMAND_REJECTED", app_stage, loop_ms);
    }

    while (1) {
        K230_CenterData center;
        float current_pwm;
        float verify_error;
        VT_Snapshot_t vt_snapshot;
        uint8_t calibrate_pressed;
        uint8_t start_pressed;
        RemoteButtonEvent_t remote_event = REMOTE_BUTTON_EVENT_NONE;

        if (K230_PollCenter(&center)) {
            VisualTrajectory_OnVision(center.relative_x, center.frame_dt_ms);
        }

        Buttons_Update();
        calibrate_pressed = Buttons_TakeCalibratePress() ? 1U : 0U;
        start_pressed = Buttons_TakeStartPress() ? 1U : 0U;
        if (RemoteButtons_TakeEvent(&remote_event) &&
            remote_event == REMOTE_BUTTON_EVENT_CALIBRATE) {
            calibrate_pressed = 1U;
            printf("[REMOTE_BUTTON] ms=%lu pin=PB2 cmd=CAL "
                   "action=IGNORED_AUTO_SEQUENCE\r\n",
                   (unsigned long)loop_ms);
        } else if (remote_event == REMOTE_BUTTON_EVENT_START) {
            start_pressed = 1U;
            printf("[REMOTE_BUTTON] ms=%lu pin=PB2 cmd=START "
                   "action=EMERGENCY_STOP\r\n",
                   (unsigned long)loop_ms);
        }
        if (calibrate_pressed != 0U && start_pressed != 0U) {
            calibrate_pressed = 0U;
            PrintAppEvent("INPUT_CONFLICT_START_EMERGENCY_WINS",
                          app_stage, loop_ms);
        }
        if (calibrate_pressed != 0U) {
            PrintAppEvent("CAL_IGNORED_AUTOMATIC_SEQUENCE",
                          app_stage, loop_ms);
        }

        if (start_pressed != 0U) {
            if (app_stage != APP_ABORTED) {
                if (IsVisualControlStage(app_stage) != 0U) {
                    (void)VisualTrajectory_Stop();
                }
                CL_StopAll();
                app_stage = APP_ABORTED;
                PrintAppEvent("START_EMERGENCY_STOP",
                              app_stage, loop_ms);
            } else {
                PrintAppEvent("START_IGNORED_ALREADY_ABORTED",
                              app_stage, loop_ms);
            }
            /*
             * Do not command another angle here: an emergency stop must not
             * create a second automatic movement after the button is pressed.
             */
        }

        if (app_stage == APP_ZERO_ALIGNING &&
            CL_IsReached(MOTOR_AXIS_X) != 0U &&
            Motor_IsBusy(MOTOR_AXIS_X) == 0U) {
            if (Encoder_GetPwmAngle(ENCODER_AXIS_X, &current_pwm) == 0U) {
                PrintZeroInvalid(
                    "AUTO_ZERO_VERIFY_FALLBACK_PWM_INVALID_CURRENT_IS_ZERO");
                CL_SetZero(MOTOR_AXIS_X);
                if (CommandAngle(VT_SETUP_ANGLE_DEG, loop_ms,
                                 APP_SETUP_ALIGNING) != 0U) {
                    app_stage = APP_SETUP_ALIGNING;
                    stage_deadline_ms =
                        loop_ms + AUTO_SETUP_ALIGN_TIMEOUT_MS;
                    PrintAppEvent("AUTO_FALLBACK_COMMAND_31_STARTED",
                                  app_stage, loop_ms);
                } else {
                    CL_StopAll();
                    app_stage = APP_ABORTED;
                    PrintAppEvent(
                        "AUTO_ABORT_FALLBACK_31_COMMAND_REJECTED",
                        app_stage, loop_ms);
                }
            } else {
                verify_error = WrapAngleDelta(FIXED_ZERO_PWM_DEG - current_pwm);
                if (AbsFloat(verify_error) <= ZERO_VERIFY_TOLERANCE_DEG) {
                    CL_SetZero(MOTOR_AXIS_X);
                    PrintZero("ZERO_ALIGNED", current_pwm, verify_error);
                    if (CommandAngle(VT_SETUP_ANGLE_DEG, loop_ms,
                                     APP_SETUP_ALIGNING) != 0U) {
                        app_stage = APP_SETUP_ALIGNING;
                        stage_deadline_ms =
                            loop_ms + AUTO_SETUP_ALIGN_TIMEOUT_MS;
                        PrintAppEvent("AUTO_COMMAND_31_STARTED",
                                      app_stage, loop_ms);
                    } else {
                        CL_StopAll();
                        app_stage = APP_ABORTED;
                        PrintAppEvent("AUTO_ABORT_31_COMMAND_REJECTED",
                                      app_stage, loop_ms);
                    }
                } else {
                    PrintZero("AUTO_ZERO_VERIFY_FALLBACK_ERROR_CURRENT_IS_ZERO",
                              current_pwm, verify_error);
                    CL_SetZero(MOTOR_AXIS_X);
                    if (CommandAngle(VT_SETUP_ANGLE_DEG, loop_ms,
                                     APP_SETUP_ALIGNING) != 0U) {
                        app_stage = APP_SETUP_ALIGNING;
                        stage_deadline_ms =
                            loop_ms + AUTO_SETUP_ALIGN_TIMEOUT_MS;
                        PrintAppEvent("AUTO_FALLBACK_COMMAND_31_STARTED",
                                      app_stage, loop_ms);
                    } else {
                        CL_StopAll();
                        app_stage = APP_ABORTED;
                        PrintAppEvent(
                            "AUTO_ABORT_FALLBACK_31_COMMAND_REJECTED",
                            app_stage, loop_ms);
                    }
                }
            }
        } else if (app_stage == APP_ZERO_ALIGNING &&
                   loop_ms >= stage_deadline_ms) {
            CL_StopAll();
            app_stage = APP_ABORTED;
            PrintAppEvent("AUTO_ABORT_ZERO_ALIGN_TIMEOUT",
                          app_stage, loop_ms);
        }

        if (app_stage == APP_SETUP_ALIGNING &&
            CL_IsReached(MOTOR_AXIS_X) != 0U &&
            Motor_IsBusy(MOTOR_AXIS_X) == 0U) {
            app_stage = APP_AUTO_COUNTDOWN;
            auto_start_deadline_ms = loop_ms + AUTO_TASK_DELAY_MS;
            PrintAutoCountdown(loop_ms, auto_start_deadline_ms);
            PrintAppEvent("AUTO_31_REACHED_WAIT_10S",
                          app_stage, loop_ms);
        } else if (app_stage == APP_SETUP_ALIGNING &&
                   loop_ms >= stage_deadline_ms) {
            CL_StopAll();
            app_stage = APP_ABORTED;
            PrintAppEvent("AUTO_ABORT_31_ALIGN_TIMEOUT",
                          app_stage, loop_ms);
        }

        if (app_stage == APP_AUTO_COUNTDOWN &&
            loop_ms >= auto_start_deadline_ms) {
            VT_Event_t start_event;
            if (CL_IsReached(MOTOR_AXIS_X) == 0U ||
                Motor_IsBusy(MOTOR_AXIS_X) != 0U) {
                app_stage = APP_ABORTED;
                PrintAppEvent("AUTO_TASK1_REJECT_MOTOR_NOT_READY",
                              app_stage, loop_ms);
            } else {
                start_event = VisualTrajectory_Start();
                if (start_event == VT_EVENT_STARTED) {
                    app_stage = APP_TASK1_RUN;
                    PrintAppEvent("AUTO_TASK1_STARTED",
                                  app_stage, loop_ms);
                } else {
                    PrintAppEvent("AUTO_TASK1_CHECK_REJECTED_FORCE_START",
                                  app_stage, loop_ms);
                    PrintTrajectory("AUTO_10S_CHECK", app_stage, loop_ms);
                    start_event = VisualTrajectory_StartForced(0);
                    if (start_event == VT_EVENT_STARTED) {
                        app_stage = APP_TASK1_RUN;
                        PrintAppEvent("AUTO_TASK1_FORCED_STARTED",
                                      app_stage, loop_ms);
                    } else {
                        app_stage = APP_ABORTED;
                        PrintAppEvent("AUTO_TASK1_FORCE_FAILED",
                                      app_stage, loop_ms);
                    }
                }
                PrintTrajectory("AUTO_10S", app_stage, loop_ms);
            }
        }

        /*
         * Task one is complete only after the final target has stayed inside
         * the 1.2 cm band at low speed for a full second.  If it leaves the
         * band, visual_trajectory applies a bounded 24/40 degree breakaway
         * pulse and PD capture; this timer then starts again from zero.
         */
        VisualTrajectory_GetSnapshot(&vt_snapshot);
        if (app_stage == APP_TASK1_RUN &&
            vt_snapshot.state == VT_STATE_HOLD) {
            app_stage = APP_TASK1_VERIFY;
            task1_verify_since_ms = loop_ms;
            task1_verify_timing = 1U;
            task1_correction_used = 0U;
            PrintTask1Verify(loop_ms, &vt_snapshot, "OBSERVE_1S");
            PrintAppEvent("TASK1_FINAL_HOLD_VERIFY_STARTED",
                          app_stage, loop_ms);
        }

        if (app_stage == APP_TASK1_VERIFY) {
            int32_t final_error_px =
                vt_snapshot.final_target_pixel - vt_snapshot.raw_pixel;
            uint8_t low_speed =
                (vt_snapshot.speed_valid == 0U ||
                 AbsFloat(vt_snapshot.velocity_px_s) <=
                     VT_HOLD_VERIFY_SPEED_PX_S) ? 1U : 0U;

            if (vt_snapshot.state == VT_STATE_HOLD &&
                AbsFloat((float)final_error_px) <=
                    VT_TASK1_HOLD_EXIT_ERROR_PX &&
                low_speed != 0U) {
                if (task1_verify_timing == 0U) {
                    task1_verify_timing = 1U;
                    task1_verify_since_ms = loop_ms;
                }
                if ((loop_ms - task1_verify_since_ms) >=
                    TASK1_FINAL_VERIFY_MS) {
                    PrintTask1Verify(
                        loop_ms, &vt_snapshot,
                        (task1_correction_used != 0U) ?
                        "CORRECTED_AND_COMPLETE" :
                        "NO_INTERVENTION_COMPLETE");
                    (void)VisualTrajectory_Stop();
                    if (CommandAngle(VT_FINAL_NEUTRAL_ANGLE_DEG,
                                     loop_ms,
                                     APP_RETARGET_WAIT) != 0U) {
                        app_stage = APP_RETARGET_WAIT;
                        retarget_deadline_ms =
                            loop_ms + ARBITRARY_TARGET_DELAY_MS;
                        PrintAppEvent(
                            "TASK1_COMPLETE_WAIT_15S_RETARGET",
                            app_stage, loop_ms);
                    } else {
                        CL_StopAll();
                        app_stage = APP_ABORTED;
                        PrintAppEvent(
                            "AUTO_ABORT_RETARGET_NEUTRAL_REJECTED",
                            app_stage, loop_ms);
                    }
                }
            } else {
                task1_verify_timing = 0U;
                if (VisualTrajectory_IsRunning() != 0U &&
                    vt_snapshot.state != VT_STATE_HOLD &&
                    task1_correction_used == 0U) {
                    task1_correction_used = 1U;
                    PrintTask1Verify(loop_ms, &vt_snapshot,
                                     "CORRECTION_24_40_PD_STARTED");
                }
            }
        }

        if (app_stage == APP_RETARGET_WAIT &&
            loop_ms >= retarget_deadline_ms) {
            VT_Event_t lock_event;
            VisualTrajectory_GetSnapshot(&vt_snapshot);
            PrintRetarget(loop_ms, &vt_snapshot, "LOCK_INPUT");
            lock_event = VisualTrajectory_LockCurrentHoldTarget(0);
            app_stage = APP_ARBITRARY_HOLD;
            VisualTrajectory_GetSnapshot(&vt_snapshot);
            PrintAppEvent(
                (lock_event == VT_EVENT_HOLD_TARGET_LOCKED) ?
                "ARBITRARY_TARGET_LOCKED" :
                "ARBITRARY_TARGET_FORCED",
                app_stage, loop_ms);
            PrintRetarget(loop_ms, &vt_snapshot, "TARGET_LOCKED");
            PrintTrajectory("ARBITRARY_HOLD_STARTED",
                            app_stage, loop_ms);
        }

        if (app_stage != APP_ABORTED &&
            CL_GetFault(MOTOR_AXIS_X) != CL_FAULT_NONE) {
            if (IsVisualControlStage(app_stage) != 0U) {
                (void)VisualTrajectory_Stop();
            }
            CL_StopAll();
            app_stage = APP_ABORTED;
            PrintAppEvent("AUTO_ABORT_MOTOR_FAULT_REBOOT",
                          app_stage, loop_ms);
        }

        Encoder_Tick(1U);
        VisualTrajectory_Tick1ms();
        control_divider++;
        if (control_divider >= APP_CONTROL_PERIOD_MS) {
            float target_angle;
            control_divider = 0U;
            if (IsVisualControlStage(app_stage) != 0U &&
                VisualTrajectory_Control5ms(&target_angle) != 0U) {
                if (CommandAngle(target_angle, loop_ms,
                                 app_stage) == 0U) {
                    (void)VisualTrajectory_Stop();
                    CL_StopAll();
                    app_stage = APP_ABORTED;
                }
            }
            CL_Process();
        }

        VisualTrajectory_GetSnapshot(&vt_snapshot);
        if (vt_snapshot.state != last_vt_state) {
            last_vt_state = vt_snapshot.state;
            PrintTrajectory("STATE_CHANGE", app_stage, loop_ms);
        }
        if ((loop_ms % HEALTH_MONITOR_PERIOD_MS) == 0U) {
            HealthUpdate(loop_ms, &vt_snapshot);
        }

        loop_ms++;
        if (IsVisualControlStage(app_stage) != 0U) {
            if (IsTrajectoryMoving(vt_snapshot.state) != 0U) {
                if ((loop_ms % APP_TELEMETRY_PERIOD_MS) == 0U) {
                    PrintTrajectorySample(loop_ms);
                }
                if ((loop_ms % APP_K230_STATS_PERIOD_MS) == 0U) {
                    PrintK230Stats(loop_ms);
                }
            } else if (vt_snapshot.state == VT_STATE_HOLD &&
                       vt_snapshot.state_age_ms <=
                           APP_HOLD_TELEMETRY_WINDOW_MS) {
                if ((loop_ms % APP_HOLD_TELEMETRY_PERIOD_MS) == 0U) {
                    PrintTrajectorySample(loop_ms);
                }
                if ((loop_ms % APP_K230_STATS_PERIOD_MS) == 0U) {
                    PrintK230Stats(loop_ms);
                }
            }
        }
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}


