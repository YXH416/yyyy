#ifndef VISUAL_TRAJECTORY_CONFIG_H
#define VISUAL_TRAJECTORY_CONFIG_H

/* ROUND-023: measured coordinate is positive away from the motor. */
/* Latest reliable three-point run: center 18, +5 cm 136, -5 cm -104. */
#define VT_POSITIVE_5CM_OFFSET_PX            (118)
#define VT_NEGATIVE_5CM_OFFSET_PX            (-122)

/* Angles already exercised on the real mechanism. */
#define VT_SETUP_ANGLE_DEG                   (31.0f)
#define VT_SAFE_ANGLE_DEG                    (32.0f)
#define VT_AWAY_DRIVE_ANGLE_DEG              (40.0f)
#define VT_TOWARD_DRIVE_ANGLE_DEG            (24.0f)
#define VT_FINAL_NEUTRAL_ANGLE_DEG           (32.0f)
#define VT_MIN_ANGLE_DEG                     (24.0f)
#define VT_MAX_ANGLE_DEG                     (40.0f)

/*
 * ROUND-032: only the arbitrary-position hold uses the stronger measured
 * correction pair.  Task one keeps its original 24..40 degree limits.
 */
#define VT_PERSISTENT_TOWARD_ANGLE_DEG       (22.0f)
#define VT_PERSISTENT_AWAY_ANGLE_DEG         (40.0f)
#define VT_PERSISTENT_MIN_ANGLE_DEG          (22.0f)
#define VT_PERSISTENT_MAX_ANGLE_DEG          (40.0f)

/* Direct-vision timing and validation. */
#define VT_DEFAULT_FRAME_DT_MS               (20U)
#define VT_MIN_FRAME_DT_MS                   (8U)
#define VT_MAX_FRAME_DT_MS                   (120U)
#define VT_START_VISION_MAX_AGE_MS           (150U)
#define VT_VISION_TIMEOUT_MS                 (150U)
#define VT_START_CENTER_LIMIT_PX             (50)
#define VT_POSITION_LIMIT_PX                 (340)
#define VT_MAX_SINGLE_JUMP_PX                (60)
#define VT_MAX_PLAUSIBLE_SPEED_PX_S          (800.0f)
#define VT_START_SPEED_LIMIT_PX_S            (25.0f)
#define VT_START_STABLE_SAMPLES              (5U)
#define VT_START_STABLE_SPAN_PX              (4)

/* Regression velocity: 4-8 direct samples over roughly 0.15 s. */
#define VT_SPEED_SAMPLE_COUNT                (8U)
#define VT_SPEED_MIN_SAMPLES                 (3U)
#define VT_SPEED_WINDOW_MS                   (160U)
#define VT_SPEED_MIN_SPAN_MS                 (80U)
#define VT_STILL_JITTER_SPAN_PX              (2)
#define VT_SPEED_FILTER_ALPHA                (0.40f)

/* Predictive braking: v*delay + v^2/(2*a) + margin. */
#define VT_ACTUATOR_DELAY_S                  (0.12f)
#define VT_BRAKING_ACCEL_PX_S2               (260.0f)
#define VT_BRAKING_MARGIN_PX                 (8.0f)
#define VT_FORCE_BRAKE_REMAINING_PX          (12.0f)
#define VT_MIN_PREDICT_SPEED_PX_S            (20.0f)
#define VT_LAUNCH_PROGRESS_PX                (3.0f)
#define VT_REVERSE_PROGRESS_PX               (2.0f)
#define VT_REVERSE_SPEED_PX_S                (6.0f)
#define VT_WRONG_DIRECTION_PX                (8.0f)

/* Near-target capture and quiet final hold. */
#define VT_CAPTURE_ENTRY_DISTANCE_PX         (35.0f)
#define VT_CAPTURE_ENTRY_SPEED_PX_S          (100.0f)
#define VT_CAPTURE_KP_DEG_PER_PX             (0.080f)
#define VT_CAPTURE_KD_DEG_S_PER_PX           (0.025f)
#define VT_CAPTURE_MAX_OFFSET_DEG            (8.0f)
#define VT_SETTLE_ERROR_PX                   (7.0f)
#define VT_SETTLE_SPEED_PX_S                 (10.0f)
#define VT_SETTLE_CONFIRM_MS                 (300U)
#define VT_TASK1_HOLD_EXIT_ERROR_PX          (29.0f)
#define VT_PERSISTENT_HOLD_EXIT_ERROR_PX     (12.0f)
#define VT_HOLD_VERIFY_SPEED_PX_S            (18.0f)
#define VT_HOLD_EXIT_CONFIRM_MS              (150U)
#define VT_PERSISTENT_HOLD_EXIT_FRAMES       (2U)
#define VT_PERSISTENT_PULSE_MAX_MS           (120U)

/*
 * A short full-angle pulse overcomes the measured static friction before the
 * near-target PD takes over.  Never keep either 24 or 40 degrees applied for
 * longer than VT_HOLD_BREAKAWAY_MAX_MS.
 */
#define VT_HOLD_BREAKAWAY_MIN_MS             (100U)
#define VT_HOLD_BREAKAWAY_MAX_MS             (250U)
#define VT_HOLD_BREAKAWAY_PROGRESS_PX        (3.0f)
#define VT_HOLD_BREAKAWAY_SPEED_PX_S         (15.0f)

/* Commands are rate limited independently of the motor position loop. */
#define VT_COMMAND_PERIOD_MS                 (50U)
#define VT_PERSISTENT_COMMAND_PERIOD_MS      (5U)
#define VT_MIN_COMMAND_CHANGE_DEG            (0.10f)

/* Safety watchdogs. */
#define VT_NO_MOTION_TIMEOUT_MS              (1500U)
#define VT_OUTBOUND_BRAKE_TIMEOUT_MS         (1800U)
#define VT_TOTAL_TRAJECTORY_TIMEOUT_MS       (8000U)
#define VT_OVERSHOOT_LIMIT_PX                (65.0f)

#endif /* VISUAL_TRAJECTORY_CONFIG_H */


