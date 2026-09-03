#ifndef VISUAL_TRAJECTORY_H
#define VISUAL_TRAJECTORY_H

#include <stdint.h>

typedef enum {
    VT_STATE_READY = 0,
    VT_STATE_OUTBOUND_DRIVE,
    VT_STATE_OUTBOUND_BRAKE,
    VT_STATE_RETURN_DRIVE,
    VT_STATE_RETURN_BRAKE,
    VT_STATE_CAPTURE,
    VT_STATE_HOLD,
    VT_STATE_HOLD_BREAKAWAY,
    VT_STATE_FAULT_VISION,
    VT_STATE_FAULT_TIMEOUT,
    VT_STATE_FAULT_DIRECTION,
    VT_STATE_STOPPED
} VT_State_t;

typedef enum {
    VT_EVENT_NONE = 0,
    VT_EVENT_STARTED,
    VT_EVENT_START_NO_VISION,
    VT_EVENT_START_NOT_CENTER,
    VT_EVENT_START_MOVING,
    VT_EVENT_OUTBOUND_BRAKE,
    VT_EVENT_RETURN_STARTED,
    VT_EVENT_RETURN_BRAKE,
    VT_EVENT_CAPTURE_STARTED,
    VT_EVENT_SETTLED,
    VT_EVENT_HOLD_EXIT,
    VT_EVENT_HOLD_BREAKAWAY_DONE,
    VT_EVENT_HOLD_TARGET_LOCKED,
    VT_EVENT_VISION_LOST,
    VT_EVENT_TIMEOUT,
    VT_EVENT_WRONG_DIRECTION,
    VT_EVENT_STOPPED
} VT_Event_t;

typedef struct {
    VT_State_t state;
    VT_Event_t last_event;
    uint8_t position_valid;
    uint8_t speed_valid;
    uint8_t persistent_hold_mode;
    int32_t raw_pixel;
    int32_t center_pixel;
    int32_t first_target_pixel;
    int32_t final_target_pixel;
    float regression_velocity_px_s;
    float velocity_px_s;
    float remaining_px;
    float predicted_stop_px;
    float requested_angle_deg;
    float hold_angle_deg;
    float hold_exit_error_px;
    uint32_t vision_age_ms;
    uint32_t state_age_ms;
    uint32_t trajectory_age_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    uint32_t stable_ms;
} VT_Snapshot_t;

void VisualTrajectory_Init(void);
void VisualTrajectory_OnVision(int32_t relative_pixel, uint32_t frame_dt_ms);
VT_Event_t VisualTrajectory_Start(void);

/*
 * Start even when the ordinary fresh/stable/center checks reject. The caller
 * must log the original rejection first. The existing 150 ms vision watchdog
 * remains active after this forced start.
 */
VT_Event_t VisualTrajectory_StartForced(int32_t fallback_center_pixel);

/*
 * Lock the most recently accepted visual position as a persistent target.
 * If no position has ever been accepted, fallback_target_pixel is used.  This
 * call deliberately starts instead of blocking; the 150 ms runtime vision
 * watchdog remains active and reports a missing/stale stream afterwards.
 */
VT_Event_t VisualTrajectory_LockCurrentHoldTarget(
    int32_t fallback_target_pixel);
VT_Event_t VisualTrajectory_Stop(void);
void VisualTrajectory_Tick1ms(void);

/* Call every 5 ms. Returns 1 only when a new motor target should be sent. */
uint8_t VisualTrajectory_Control5ms(float *target_angle_deg);

uint8_t VisualTrajectory_IsRunning(void);
void VisualTrajectory_GetSnapshot(VT_Snapshot_t *snapshot);
const char *VisualTrajectory_StateName(VT_State_t state);
const char *VisualTrajectory_EventName(VT_Event_t event);

#endif /* VISUAL_TRAJECTORY_H */


