#include "visual_trajectory.h"
#include "visual_trajectory_config.h"

#include <stddef.h>

typedef struct {
    VT_State_t state;
    VT_Event_t last_event;
    uint8_t position_valid;
    uint8_t speed_valid;
    int32_t raw_pixel;
    int32_t center_pixel;
    int32_t first_target_pixel;
    int32_t final_target_pixel;
    float regression_velocity_px_s;
    float velocity_px_s;
    float remaining_px;
    float predicted_stop_px;
    float desired_angle_deg;
    float last_sent_angle_deg;
    float hold_angle_deg;
    float outbound_peak_pixel;
    uint8_t last_sent_valid;
    uint8_t force_command;
    uint8_t launched;
    uint8_t persistent_hold_mode;
    int8_t hold_breakaway_direction;
    int32_t hold_breakaway_start_pixel;
    float hold_exit_error_px;
    uint32_t vision_age_ms;
    uint32_t state_age_ms;
    uint32_t trajectory_age_ms;
    uint32_t command_age_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    uint32_t stable_ms;
    uint32_t hold_exit_ms;
    uint32_t hold_breakaway_elapsed_ms;
    uint32_t vision_clock_ms;
    uint32_t sample_time_ms[VT_SPEED_SAMPLE_COUNT];
    int32_t sample_pixel[VT_SPEED_SAMPLE_COUNT];
    uint8_t sample_count;
    uint8_t hold_exit_frames;
} VT_Context_t;

static VT_Context_t s_vt;

static float VT_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int32_t VT_AbsInt32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static float VT_Clamp(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint8_t VT_IsActiveState(VT_State_t state)
{
    return (state == VT_STATE_OUTBOUND_DRIVE ||
            state == VT_STATE_OUTBOUND_BRAKE ||
            state == VT_STATE_RETURN_DRIVE ||
            state == VT_STATE_RETURN_BRAKE ||
            state == VT_STATE_CAPTURE ||
            state == VT_STATE_HOLD ||
            state == VT_STATE_HOLD_BREAKAWAY) ? 1U : 0U;
}

static float VT_MinCommandAngle(void)
{
    return (s_vt.persistent_hold_mode != 0U) ?
           VT_PERSISTENT_MIN_ANGLE_DEG : VT_MIN_ANGLE_DEG;
}

static float VT_MaxCommandAngle(void)
{
    return (s_vt.persistent_hold_mode != 0U) ?
           VT_PERSISTENT_MAX_ANGLE_DEG : VT_MAX_ANGLE_DEG;
}

static void VT_SetDesiredAngle(float angle)
{
    angle = VT_Clamp(angle, VT_MinCommandAngle(), VT_MaxCommandAngle());
    s_vt.desired_angle_deg = angle;
}

static void VT_EnterState(VT_State_t state, VT_Event_t event)
{
    if (s_vt.state != state) {
        s_vt.state = state;
        s_vt.state_age_ms = 0U;
        s_vt.stable_ms = 0U;
        s_vt.hold_exit_ms = 0U;
        s_vt.hold_breakaway_elapsed_ms = 0U;
        s_vt.hold_exit_frames = 0U;
    }
    s_vt.last_event = event;
    s_vt.force_command = 1U;
}

static void VT_ResetSamples(void)
{
    s_vt.sample_count = 0U;
    s_vt.speed_valid = 0U;
    s_vt.regression_velocity_px_s = 0.0f;
    s_vt.velocity_px_s = 0.0f;
}

static void VT_RemoveFirstSample(void)
{
    uint8_t index;
    if (s_vt.sample_count == 0U) return;
    for (index = 1U; index < s_vt.sample_count; index++) {
        s_vt.sample_time_ms[index - 1U] = s_vt.sample_time_ms[index];
        s_vt.sample_pixel[index - 1U] = s_vt.sample_pixel[index];
    }
    s_vt.sample_count--;
}

static void VT_UpdateVelocity(void)
{
    uint8_t index;
    float mean_time = 0.0f;
    float mean_pixel = 0.0f;
    float numerator = 0.0f;
    float denominator = 0.0f;
    float velocity;
    int32_t minimum_pixel;
    int32_t maximum_pixel;
    uint32_t base_time;
    uint32_t span_ms;

    if (s_vt.sample_count < VT_SPEED_MIN_SAMPLES) return;
    span_ms = s_vt.sample_time_ms[s_vt.sample_count - 1U] -
              s_vt.sample_time_ms[0U];
    if (span_ms < VT_SPEED_MIN_SPAN_MS) return;

    base_time = s_vt.sample_time_ms[0U];
    minimum_pixel = s_vt.sample_pixel[0U];
    maximum_pixel = minimum_pixel;
    for (index = 0U; index < s_vt.sample_count; index++) {
        mean_time += (float)(s_vt.sample_time_ms[index] - base_time) * 0.001f;
        mean_pixel += (float)s_vt.sample_pixel[index];
        if (s_vt.sample_pixel[index] < minimum_pixel) {
            minimum_pixel = s_vt.sample_pixel[index];
        }
        if (s_vt.sample_pixel[index] > maximum_pixel) {
            maximum_pixel = s_vt.sample_pixel[index];
        }
    }
    mean_time /= (float)s_vt.sample_count;
    mean_pixel /= (float)s_vt.sample_count;
    for (index = 0U; index < s_vt.sample_count; index++) {
        float sample_time =
            (float)(s_vt.sample_time_ms[index] - base_time) * 0.001f;
        float time_delta = sample_time - mean_time;
        numerator += time_delta *
                     ((float)s_vt.sample_pixel[index] - mean_pixel);
        denominator += time_delta * time_delta;
    }
    if (denominator <= 0.000001f) return;
    velocity = numerator / denominator;
    velocity = VT_Clamp(velocity, -VT_MAX_PLAUSIBLE_SPEED_PX_S,
                        VT_MAX_PLAUSIBLE_SPEED_PX_S);
    if ((maximum_pixel - minimum_pixel) <= VT_STILL_JITTER_SPAN_PX) {
        velocity = 0.0f;
    }
    s_vt.regression_velocity_px_s = velocity;
    if (s_vt.speed_valid == 0U) {
        s_vt.velocity_px_s = velocity;
    } else {
        s_vt.velocity_px_s =
            VT_SPEED_FILTER_ALPHA * velocity +
            (1.0f - VT_SPEED_FILTER_ALPHA) * s_vt.velocity_px_s;
    }
    s_vt.speed_valid = 1U;
}

static void VT_PushSample(int32_t pixel, uint32_t dt_ms)
{
    if (s_vt.sample_count >= VT_SPEED_SAMPLE_COUNT) {
        VT_RemoveFirstSample();
    }
    s_vt.vision_clock_ms += dt_ms;
    s_vt.sample_time_ms[s_vt.sample_count] = s_vt.vision_clock_ms;
    s_vt.sample_pixel[s_vt.sample_count] = pixel;
    s_vt.sample_count++;
    while (s_vt.sample_count > VT_SPEED_MIN_SAMPLES &&
           (s_vt.sample_time_ms[s_vt.sample_count - 1U] -
            s_vt.sample_time_ms[0U]) > VT_SPEED_WINDOW_MS) {
        VT_RemoveFirstSample();
    }
    VT_UpdateVelocity();
}

static float VT_StopDistance(float directed_speed)
{
    float distance;
    if (directed_speed <= 0.0f) return 0.0f;
    distance = directed_speed * VT_ACTUATOR_DELAY_S +
               directed_speed * directed_speed /
                   (2.0f * VT_BRAKING_ACCEL_PX_S2) +
               VT_BRAKING_MARGIN_PX;
    return distance;
}

static uint8_t VT_ShouldBrake(int8_t direction, int32_t target_pixel)
{
    float directed_speed;
    float regression_directed_speed;
    float remaining;
    float stopping_distance;
    remaining = (float)direction *
                ((float)target_pixel - (float)s_vt.raw_pixel);
    if (remaining <= VT_FORCE_BRAKE_REMAINING_PX) {
        s_vt.remaining_px = remaining;
        s_vt.predicted_stop_px = (float)s_vt.raw_pixel;
        return 1U;
    }
    if (s_vt.speed_valid == 0U) return 0U;
    directed_speed = (float)direction * s_vt.velocity_px_s;
    regression_directed_speed =
        (float)direction * s_vt.regression_velocity_px_s;
    if (regression_directed_speed > directed_speed) {
        directed_speed = regression_directed_speed;
    }
    stopping_distance = VT_StopDistance(directed_speed);
    s_vt.remaining_px = remaining;
    s_vt.predicted_stop_px = (float)s_vt.raw_pixel +
                             (float)direction * stopping_distance;
    if (remaining <= VT_BRAKING_MARGIN_PX) return 1U;
    return (directed_speed >= VT_MIN_PREDICT_SPEED_PX_S &&
            remaining <= stopping_distance) ? 1U : 0U;
}

static float VT_CaptureAngle(void)
{
    float error = (float)s_vt.final_target_pixel - (float)s_vt.raw_pixel;
    float offset = VT_CAPTURE_KP_DEG_PER_PX * error -
                   VT_CAPTURE_KD_DEG_S_PER_PX * s_vt.velocity_px_s;

    offset = VT_Clamp(offset, -VT_CAPTURE_MAX_OFFSET_DEG,
                      VT_CAPTURE_MAX_OFFSET_DEG);
    return VT_Clamp(VT_FINAL_NEUTRAL_ANGLE_DEG + offset,
                    VT_MIN_ANGLE_DEG, VT_MAX_ANGLE_DEG);
}

static void VT_SetFault(VT_State_t state, VT_Event_t event)
{
    VT_EnterState(state, event);
    VT_SetDesiredAngle(VT_SAFE_ANGLE_DEG);
}

static void VT_UpdateStateFromVision(uint32_t dt_ms)
{
    float error;
    float directed_speed;
    float peak_backoff;

    switch (s_vt.state) {
    case VT_STATE_OUTBOUND_DRIVE:
        if ((float)s_vt.raw_pixel > s_vt.outbound_peak_pixel) {
            s_vt.outbound_peak_pixel = (float)s_vt.raw_pixel;
        }
        if ((float)(s_vt.raw_pixel - s_vt.center_pixel) >=
            VT_LAUNCH_PROGRESS_PX) {
            s_vt.launched = 1U;
        }
        if ((float)(s_vt.center_pixel - s_vt.raw_pixel) >=
            VT_WRONG_DIRECTION_PX) {
            VT_SetFault(VT_STATE_FAULT_DIRECTION,
                        VT_EVENT_WRONG_DIRECTION);
            return;
        }
        if (s_vt.launched != 0U &&
            VT_ShouldBrake(+1, s_vt.first_target_pixel) != 0U) {
            VT_EnterState(VT_STATE_OUTBOUND_BRAKE,
                          VT_EVENT_OUTBOUND_BRAKE);
            VT_SetDesiredAngle(VT_TOWARD_DRIVE_ANGLE_DEG);
        }
        return;

    case VT_STATE_OUTBOUND_BRAKE:
        if ((float)s_vt.raw_pixel > s_vt.outbound_peak_pixel) {
            s_vt.outbound_peak_pixel = (float)s_vt.raw_pixel;
        }
        peak_backoff = s_vt.outbound_peak_pixel - (float)s_vt.raw_pixel;
        if (peak_backoff >= VT_REVERSE_PROGRESS_PX &&
            (s_vt.speed_valid == 0U ||
             s_vt.velocity_px_s <= -VT_REVERSE_SPEED_PX_S)) {
            VT_EnterState(VT_STATE_RETURN_DRIVE,
                          VT_EVENT_RETURN_STARTED);
            VT_SetDesiredAngle(VT_TOWARD_DRIVE_ANGLE_DEG);
        }
        return;

    case VT_STATE_RETURN_DRIVE:
        if (VT_ShouldBrake(-1, s_vt.final_target_pixel) != 0U) {
            VT_EnterState(VT_STATE_RETURN_BRAKE,
                          VT_EVENT_RETURN_BRAKE);
            VT_SetDesiredAngle(VT_AWAY_DRIVE_ANGLE_DEG);
        }
        return;

    case VT_STATE_RETURN_BRAKE:
        error = (float)s_vt.final_target_pixel - (float)s_vt.raw_pixel;
        directed_speed = -s_vt.velocity_px_s;
        if (s_vt.raw_pixel <= s_vt.final_target_pixel ||
            (VT_AbsFloat(error) <= VT_CAPTURE_ENTRY_DISTANCE_PX &&
             (s_vt.speed_valid == 0U ||
              directed_speed <= VT_CAPTURE_ENTRY_SPEED_PX_S))) {
            VT_EnterState(VT_STATE_CAPTURE, VT_EVENT_CAPTURE_STARTED);
            VT_SetDesiredAngle(VT_CaptureAngle());
        }
        return;

    case VT_STATE_CAPTURE:
        error = (float)s_vt.final_target_pixel - (float)s_vt.raw_pixel;
        VT_SetDesiredAngle(VT_CaptureAngle());
        if (VT_AbsFloat(error) <= VT_SETTLE_ERROR_PX &&
            s_vt.speed_valid != 0U &&
            VT_AbsFloat(s_vt.velocity_px_s) <= VT_SETTLE_SPEED_PX_S) {
            if (s_vt.stable_ms <= UINT32_MAX - dt_ms) {
                s_vt.stable_ms += dt_ms;
            }
            if (s_vt.stable_ms >= VT_SETTLE_CONFIRM_MS) {
                s_vt.hold_angle_deg = VT_CaptureAngle();
                VT_EnterState(VT_STATE_HOLD, VT_EVENT_SETTLED);
                VT_SetDesiredAngle(s_vt.hold_angle_deg);
            }
        } else {
            s_vt.stable_ms = 0U;
        }
        return;

    case VT_STATE_HOLD:
        error = (float)s_vt.final_target_pixel - (float)s_vt.raw_pixel;
        if (VT_AbsFloat(error) > s_vt.hold_exit_error_px) {
            uint8_t exit_confirmed = 0U;
            if (s_vt.persistent_hold_mode != 0U) {
                if (s_vt.hold_exit_frames < UINT8_MAX) {
                    s_vt.hold_exit_frames++;
                }
                if (s_vt.hold_exit_frames >=
                    VT_PERSISTENT_HOLD_EXIT_FRAMES) {
                    exit_confirmed = 1U;
                }
            } else {
                if (s_vt.hold_exit_ms <= UINT32_MAX - dt_ms) {
                    s_vt.hold_exit_ms += dt_ms;
                }
                if (s_vt.hold_exit_ms >= VT_HOLD_EXIT_CONFIRM_MS) {
                    exit_confirmed = 1U;
                }
            }
            if (exit_confirmed != 0U) {
                s_vt.hold_breakaway_direction =
                    (error >= 0.0f) ? (int8_t)1 : (int8_t)-1;
                s_vt.hold_breakaway_start_pixel = s_vt.raw_pixel;
                /* A long HOLD must not make the next correction time out. */
                s_vt.trajectory_age_ms = 0U;
                VT_EnterState(VT_STATE_HOLD_BREAKAWAY,
                              VT_EVENT_HOLD_EXIT);
                if (s_vt.persistent_hold_mode != 0U) {
                    VT_SetDesiredAngle(
                        (s_vt.hold_breakaway_direction > 0) ?
                        VT_PERSISTENT_AWAY_ANGLE_DEG :
                        VT_PERSISTENT_TOWARD_ANGLE_DEG);
                } else {
                    VT_SetDesiredAngle(
                        (s_vt.hold_breakaway_direction > 0) ?
                        VT_AWAY_DRIVE_ANGLE_DEG :
                        VT_TOWARD_DRIVE_ANGLE_DEG);
                }
            }
        } else {
            s_vt.hold_exit_ms = 0U;
            s_vt.hold_exit_frames = 0U;
        }
        return;

    case VT_STATE_HOLD_BREAKAWAY: {
        float directed_progress;
        float directed_speed = 0.0f;
        error = (float)s_vt.final_target_pixel - (float)s_vt.raw_pixel;

        /*
         * The arbitrary-position controller deliberately uses bounded strong
         * pulses only: 22 or 40 degrees, then 32 degrees to observe the next
         * direct frame.  It never falls through to the small-angle task-one PD.
         */
        if (s_vt.persistent_hold_mode != 0U) {
            uint8_t crossed_target =
                ((error > 0.0f && s_vt.hold_breakaway_direction < 0) ||
                 (error < 0.0f && s_vt.hold_breakaway_direction > 0)) ?
                1U : 0U;
            if (s_vt.hold_breakaway_elapsed_ms <= UINT32_MAX - dt_ms) {
                s_vt.hold_breakaway_elapsed_ms += dt_ms;
            }
            if (VT_AbsFloat(error) <= s_vt.hold_exit_error_px ||
                crossed_target != 0U ||
                s_vt.hold_breakaway_elapsed_ms >=
                    VT_PERSISTENT_PULSE_MAX_MS) {
                s_vt.hold_angle_deg = VT_FINAL_NEUTRAL_ANGLE_DEG;
                VT_EnterState(VT_STATE_HOLD,
                              VT_EVENT_HOLD_BREAKAWAY_DONE);
                VT_SetDesiredAngle(VT_FINAL_NEUTRAL_ANGLE_DEG);
            }
            return;
        }

        directed_progress = (float)s_vt.hold_breakaway_direction *
            (float)(s_vt.raw_pixel - s_vt.hold_breakaway_start_pixel);
        if (s_vt.speed_valid != 0U) {
            directed_speed = (float)s_vt.hold_breakaway_direction *
                             s_vt.velocity_px_s;
        }
        if (VT_AbsFloat(error) <= VT_SETTLE_ERROR_PX ||
            ((s_vt.state_age_ms >= VT_HOLD_BREAKAWAY_MIN_MS) &&
             (directed_progress >= VT_HOLD_BREAKAWAY_PROGRESS_PX ||
              directed_speed >= VT_HOLD_BREAKAWAY_SPEED_PX_S)) ||
            s_vt.state_age_ms >= VT_HOLD_BREAKAWAY_MAX_MS ||
            ((error > 0.0f && s_vt.hold_breakaway_direction < 0) ||
             (error < 0.0f && s_vt.hold_breakaway_direction > 0))) {
            VT_EnterState(VT_STATE_CAPTURE,
                          VT_EVENT_HOLD_BREAKAWAY_DONE);
            VT_SetDesiredAngle(VT_CaptureAngle());
        }
        return;
    }

    default:
        return;
    }
}

void VisualTrajectory_Init(void)
{
    s_vt.state = VT_STATE_READY;
    s_vt.last_event = VT_EVENT_NONE;
    s_vt.position_valid = 0U;
    s_vt.speed_valid = 0U;
    s_vt.raw_pixel = 0;
    s_vt.center_pixel = 0;
    s_vt.first_target_pixel = 0;
    s_vt.final_target_pixel = 0;
    s_vt.regression_velocity_px_s = 0.0f;
    s_vt.velocity_px_s = 0.0f;
    s_vt.remaining_px = 0.0f;
    s_vt.predicted_stop_px = 0.0f;
    s_vt.desired_angle_deg = VT_SAFE_ANGLE_DEG;
    s_vt.last_sent_angle_deg = VT_SAFE_ANGLE_DEG;
    s_vt.hold_angle_deg = VT_FINAL_NEUTRAL_ANGLE_DEG;
    s_vt.outbound_peak_pixel = 0.0f;
    s_vt.last_sent_valid = 0U;
    s_vt.force_command = 0U;
    s_vt.launched = 0U;
    s_vt.persistent_hold_mode = 0U;
    s_vt.hold_breakaway_direction = 0;
    s_vt.hold_breakaway_start_pixel = 0;
    s_vt.hold_exit_error_px = VT_TASK1_HOLD_EXIT_ERROR_PX;
    s_vt.vision_age_ms = VT_VISION_TIMEOUT_MS + 1U;
    s_vt.state_age_ms = 0U;
    s_vt.trajectory_age_ms = 0U;
    s_vt.command_age_ms = 0U;
    s_vt.accepted_frames = 0U;
    s_vt.rejected_frames = 0U;
    s_vt.stable_ms = 0U;
    s_vt.hold_exit_ms = 0U;
    s_vt.hold_breakaway_elapsed_ms = 0U;
    s_vt.hold_exit_frames = 0U;
    s_vt.vision_clock_ms = 0U;
    VT_ResetSamples();
}

void VisualTrajectory_OnVision(int32_t relative_pixel, uint32_t frame_dt_ms)
{
    uint32_t dt_ms = frame_dt_ms;
    if (dt_ms < VT_MIN_FRAME_DT_MS || dt_ms > VT_MAX_FRAME_DT_MS) {
        dt_ms = VT_DEFAULT_FRAME_DT_MS;
    }
    if (VT_AbsInt32(relative_pixel) > VT_POSITION_LIMIT_PX) {
        s_vt.rejected_frames++;
        return;
    }
    if (s_vt.position_valid != 0U &&
        s_vt.vision_age_ms <= VT_VISION_TIMEOUT_MS &&
        VT_AbsInt32(relative_pixel - s_vt.raw_pixel) >
        VT_MAX_SINGLE_JUMP_PX) {
        s_vt.rejected_frames++;
        return;
    }
    if (s_vt.vision_age_ms > VT_VISION_TIMEOUT_MS) {
        VT_ResetSamples();
    }
    s_vt.raw_pixel = relative_pixel;
    s_vt.position_valid = 1U;
    s_vt.vision_age_ms = 0U;
    s_vt.accepted_frames++;
    VT_PushSample(relative_pixel, dt_ms);
    if (VT_IsActiveState(s_vt.state) != 0U) {
        VT_UpdateStateFromVision(dt_ms);
    }
}

static VT_Event_t VT_StartAtCenter(int32_t center_pixel)
{
    s_vt.center_pixel = center_pixel;
    s_vt.first_target_pixel = s_vt.center_pixel +
                              VT_POSITIVE_5CM_OFFSET_PX;
    s_vt.final_target_pixel = s_vt.center_pixel +
                              VT_NEGATIVE_5CM_OFFSET_PX;
    if (VT_AbsInt32(s_vt.first_target_pixel) > VT_POSITION_LIMIT_PX ||
        VT_AbsInt32(s_vt.final_target_pixel) > VT_POSITION_LIMIT_PX) {
        s_vt.last_event = VT_EVENT_START_NOT_CENTER;
        return s_vt.last_event;
    }
    s_vt.trajectory_age_ms = 0U;
    s_vt.persistent_hold_mode = 0U;
    s_vt.hold_exit_error_px = VT_TASK1_HOLD_EXIT_ERROR_PX;
    s_vt.outbound_peak_pixel = (float)s_vt.raw_pixel;
    s_vt.launched = 0U;
    s_vt.stable_ms = 0U;
    s_vt.hold_exit_ms = 0U;
    s_vt.remaining_px = (float)(s_vt.first_target_pixel - s_vt.raw_pixel);
    s_vt.predicted_stop_px = (float)s_vt.raw_pixel;
    VT_EnterState(VT_STATE_OUTBOUND_DRIVE, VT_EVENT_STARTED);
    VT_SetDesiredAngle(VT_AWAY_DRIVE_ANGLE_DEG);
    return s_vt.last_event;
}

VT_Event_t VisualTrajectory_Start(void)
{
    int32_t minimum_pixel;
    int32_t maximum_pixel;
    uint8_t index;
    if (s_vt.position_valid == 0U ||
        s_vt.vision_age_ms > VT_START_VISION_MAX_AGE_MS) {
        s_vt.last_event = VT_EVENT_START_NO_VISION;
        return s_vt.last_event;
    }
    if (VT_AbsInt32(s_vt.raw_pixel) > VT_START_CENTER_LIMIT_PX) {
        s_vt.last_event = VT_EVENT_START_NOT_CENTER;
        return s_vt.last_event;
    }
    if (s_vt.sample_count < VT_START_STABLE_SAMPLES) {
        s_vt.last_event = VT_EVENT_START_MOVING;
        return s_vt.last_event;
    }
    minimum_pixel = s_vt.sample_pixel[0U];
    maximum_pixel = minimum_pixel;
    for (index = 1U; index < s_vt.sample_count; index++) {
        if (s_vt.sample_pixel[index] < minimum_pixel) {
            minimum_pixel = s_vt.sample_pixel[index];
        }
        if (s_vt.sample_pixel[index] > maximum_pixel) {
            maximum_pixel = s_vt.sample_pixel[index];
        }
    }
    if ((maximum_pixel - minimum_pixel) > VT_START_STABLE_SPAN_PX) {
        s_vt.last_event = VT_EVENT_START_MOVING;
        return s_vt.last_event;
    }
    if (s_vt.speed_valid != 0U &&
        VT_AbsFloat(s_vt.velocity_px_s) > VT_START_SPEED_LIMIT_PX_S) {
        s_vt.last_event = VT_EVENT_START_MOVING;
        return s_vt.last_event;
    }
    return VT_StartAtCenter(s_vt.raw_pixel);
}

VT_Event_t VisualTrajectory_StartForced(int32_t fallback_center_pixel)
{
    int32_t center_pixel = (s_vt.position_valid != 0U) ?
                           s_vt.raw_pixel : fallback_center_pixel;

    if (VT_AbsInt32(center_pixel) > VT_POSITION_LIMIT_PX ||
        VT_AbsInt32(center_pixel + VT_POSITIVE_5CM_OFFSET_PX) >
            VT_POSITION_LIMIT_PX ||
        VT_AbsInt32(center_pixel + VT_NEGATIVE_5CM_OFFSET_PX) >
            VT_POSITION_LIMIT_PX) {
        center_pixel = fallback_center_pixel;
    }
    if (VT_AbsInt32(center_pixel) > VT_POSITION_LIMIT_PX ||
        VT_AbsInt32(center_pixel + VT_POSITIVE_5CM_OFFSET_PX) >
            VT_POSITION_LIMIT_PX ||
        VT_AbsInt32(center_pixel + VT_NEGATIVE_5CM_OFFSET_PX) >
            VT_POSITION_LIMIT_PX) {
        center_pixel = 0;
    }

    s_vt.raw_pixel = center_pixel;
    s_vt.position_valid = 1U;
    s_vt.vision_age_ms = 0U;
    s_vt.vision_clock_ms = 0U;
    VT_ResetSamples();
    return VT_StartAtCenter(center_pixel);
}

VT_Event_t VisualTrajectory_LockCurrentHoldTarget(
    int32_t fallback_target_pixel)
{
    int32_t target_pixel = (s_vt.position_valid != 0U) ?
                           s_vt.raw_pixel : fallback_target_pixel;

    if (VT_AbsInt32(target_pixel) > VT_POSITION_LIMIT_PX) {
        target_pixel = fallback_target_pixel;
    }
    if (VT_AbsInt32(target_pixel) > VT_POSITION_LIMIT_PX) {
        target_pixel = 0;
    }

    s_vt.raw_pixel = target_pixel;
    s_vt.center_pixel = target_pixel;
    s_vt.first_target_pixel = target_pixel;
    s_vt.final_target_pixel = target_pixel;
    s_vt.position_valid = 1U;
    /* Forced start: allow one watchdog window for the next direct frame. */
    s_vt.vision_age_ms = 0U;
    s_vt.vision_clock_ms = 0U;
    s_vt.trajectory_age_ms = 0U;
    s_vt.persistent_hold_mode = 1U;
    s_vt.hold_exit_error_px = VT_PERSISTENT_HOLD_EXIT_ERROR_PX;
    s_vt.hold_angle_deg = VT_FINAL_NEUTRAL_ANGLE_DEG;
    s_vt.hold_breakaway_direction = 0;
    s_vt.hold_breakaway_start_pixel = target_pixel;
    VT_ResetSamples();
    VT_EnterState(VT_STATE_HOLD, VT_EVENT_HOLD_TARGET_LOCKED);
    VT_SetDesiredAngle(VT_FINAL_NEUTRAL_ANGLE_DEG);
    return s_vt.last_event;
}

VT_Event_t VisualTrajectory_Stop(void)
{
    s_vt.persistent_hold_mode = 0U;
    VT_EnterState(VT_STATE_STOPPED, VT_EVENT_STOPPED);
    VT_SetDesiredAngle(VT_SAFE_ANGLE_DEG);
    return s_vt.last_event;
}

void VisualTrajectory_Tick1ms(void)
{
    if (s_vt.vision_age_ms < UINT32_MAX) s_vt.vision_age_ms++;
    if (s_vt.state_age_ms < UINT32_MAX) s_vt.state_age_ms++;
    if (s_vt.command_age_ms < UINT32_MAX) s_vt.command_age_ms++;
    if (VT_IsActiveState(s_vt.state) == 0U) return;
    if (s_vt.trajectory_age_ms < UINT32_MAX) s_vt.trajectory_age_ms++;

    if (s_vt.vision_age_ms > VT_VISION_TIMEOUT_MS) {
        VT_SetFault(VT_STATE_FAULT_VISION, VT_EVENT_VISION_LOST);
        return;
    }
    if (s_vt.persistent_hold_mode == 0U &&
        s_vt.state != VT_STATE_HOLD &&
        s_vt.trajectory_age_ms > VT_TOTAL_TRAJECTORY_TIMEOUT_MS) {
        VT_SetFault(VT_STATE_FAULT_TIMEOUT, VT_EVENT_TIMEOUT);
        return;
    }
    if (s_vt.state == VT_STATE_OUTBOUND_DRIVE &&
        s_vt.launched == 0U &&
        s_vt.state_age_ms > VT_NO_MOTION_TIMEOUT_MS) {
        VT_SetFault(VT_STATE_FAULT_TIMEOUT, VT_EVENT_TIMEOUT);
        return;
    }
    if (s_vt.state == VT_STATE_OUTBOUND_BRAKE &&
        s_vt.state_age_ms > VT_OUTBOUND_BRAKE_TIMEOUT_MS) {
        VT_SetFault(VT_STATE_FAULT_TIMEOUT, VT_EVENT_TIMEOUT);
    }
}

uint8_t VisualTrajectory_Control5ms(float *target_angle_deg)
{
    float desired;
    uint32_t command_period_ms;
    if (target_angle_deg == NULL) return 0U;
    if (s_vt.state == VT_STATE_READY) return 0U;
    if (s_vt.state == VT_STATE_CAPTURE && s_vt.position_valid != 0U) {
        VT_SetDesiredAngle(VT_CaptureAngle());
    }
    desired = VT_Clamp(s_vt.desired_angle_deg,
                       VT_MinCommandAngle(), VT_MaxCommandAngle());
    command_period_ms = (s_vt.persistent_hold_mode != 0U) ?
                        VT_PERSISTENT_COMMAND_PERIOD_MS :
                        VT_COMMAND_PERIOD_MS;
    if (s_vt.force_command == 0U &&
        s_vt.command_age_ms < command_period_ms) {
        return 0U;
    }
    if (s_vt.last_sent_valid != 0U &&
        s_vt.force_command == 0U &&
        VT_AbsFloat(desired - s_vt.last_sent_angle_deg) <
        VT_MIN_COMMAND_CHANGE_DEG) {
        return 0U;
    }
    *target_angle_deg = desired;
    s_vt.last_sent_angle_deg = desired;
    s_vt.last_sent_valid = 1U;
    s_vt.force_command = 0U;
    s_vt.command_age_ms = 0U;
    return 1U;
}

uint8_t VisualTrajectory_IsRunning(void)
{
    return VT_IsActiveState(s_vt.state);
}

void VisualTrajectory_GetSnapshot(VT_Snapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    snapshot->state = s_vt.state;
    snapshot->last_event = s_vt.last_event;
    snapshot->position_valid = s_vt.position_valid;
    snapshot->speed_valid = s_vt.speed_valid;
    snapshot->persistent_hold_mode = s_vt.persistent_hold_mode;
    snapshot->raw_pixel = s_vt.raw_pixel;
    snapshot->center_pixel = s_vt.center_pixel;
    snapshot->first_target_pixel = s_vt.first_target_pixel;
    snapshot->final_target_pixel = s_vt.final_target_pixel;
    snapshot->regression_velocity_px_s =
        s_vt.regression_velocity_px_s;
    snapshot->velocity_px_s = s_vt.velocity_px_s;
    snapshot->remaining_px = s_vt.remaining_px;
    snapshot->predicted_stop_px = s_vt.predicted_stop_px;
    snapshot->requested_angle_deg = s_vt.desired_angle_deg;
    snapshot->hold_angle_deg = s_vt.hold_angle_deg;
    snapshot->hold_exit_error_px = s_vt.hold_exit_error_px;
    snapshot->vision_age_ms = s_vt.vision_age_ms;
    snapshot->state_age_ms = s_vt.state_age_ms;
    snapshot->trajectory_age_ms = s_vt.trajectory_age_ms;
    snapshot->accepted_frames = s_vt.accepted_frames;
    snapshot->rejected_frames = s_vt.rejected_frames;
    snapshot->stable_ms = s_vt.stable_ms;
}

const char *VisualTrajectory_StateName(VT_State_t state)
{
    switch (state) {
    case VT_STATE_READY: return "READY";
    case VT_STATE_OUTBOUND_DRIVE: return "OUT_DRIVE";
    case VT_STATE_OUTBOUND_BRAKE: return "OUT_BRAKE";
    case VT_STATE_RETURN_DRIVE: return "RET_DRIVE";
    case VT_STATE_RETURN_BRAKE: return "RET_BRAKE";
    case VT_STATE_CAPTURE: return "CAPTURE";
    case VT_STATE_HOLD: return "HOLD";
    case VT_STATE_HOLD_BREAKAWAY: return "HOLD_BREAKAWAY";
    case VT_STATE_FAULT_VISION: return "FAULT_VISION";
    case VT_STATE_FAULT_TIMEOUT: return "FAULT_TIMEOUT";
    case VT_STATE_FAULT_DIRECTION: return "FAULT_DIRECTION";
    default: return "STOPPED";
    }
}

const char *VisualTrajectory_EventName(VT_Event_t event)
{
    switch (event) {
    case VT_EVENT_NONE: return "NONE";
    case VT_EVENT_STARTED: return "STARTED";
    case VT_EVENT_START_NO_VISION: return "START_NO_VISION";
    case VT_EVENT_START_NOT_CENTER: return "START_NOT_CENTER";
    case VT_EVENT_START_MOVING: return "START_MOVING";
    case VT_EVENT_OUTBOUND_BRAKE: return "OUTBOUND_BRAKE";
    case VT_EVENT_RETURN_STARTED: return "RETURN_STARTED";
    case VT_EVENT_RETURN_BRAKE: return "RETURN_BRAKE";
    case VT_EVENT_CAPTURE_STARTED: return "CAPTURE_STARTED";
    case VT_EVENT_SETTLED: return "SETTLED";
    case VT_EVENT_HOLD_EXIT: return "HOLD_EXIT";
    case VT_EVENT_HOLD_BREAKAWAY_DONE: return "HOLD_BREAKAWAY_DONE";
    case VT_EVENT_HOLD_TARGET_LOCKED: return "HOLD_TARGET_LOCKED";
    case VT_EVENT_VISION_LOST: return "VISION_LOST";
    case VT_EVENT_TIMEOUT: return "TIMEOUT";
    case VT_EVENT_WRONG_DIRECTION: return "WRONG_DIRECTION";
    default: return "STOPPED";
    }
}


