#ifndef MOTOR_BALANCE_CONFIG_H
#define MOTOR_BALANCE_CONFIG_H

/* ROUND-037: user CAL,BALANCE result at MCU ms=442500.
 * Absolute PWM = 176.861 deg; 16-sample spread = 0.139 deg.
 * This changes the measurement zero only; it never commands boot motion. */
#define MEASURED_BALANCE_VALID       (1U)
#define MEASURED_BALANCE_PWM_MDEG    (176861L)

/* ROUND-043 manual control uses the measured balance pose as 0 deg.
 * Protocol range and mechanical travel are deliberately separate settings.
 * There is no measured travel limit yet, so the mechanical setting defaults
 * to the full protocol range and is explicitly marked UNCONFIRMED. This keeps
 * the debugger usable without claiming that +/-15 deg is mechanically safe.
 * After a no-ball travel check, set the POS/NEG limits independently (for
 * example 9000 means 9 deg in that direction) and change CONFIRMED to 1. */
#define MANUAL_COMMAND_LIMIT_MDEG     (15000L)
#define MANUAL_MECHANICAL_POS_LIMIT_MDEG (15000L)
#define MANUAL_MECHANICAL_NEG_LIMIT_MDEG (15000L)
#define MANUAL_MECHANICAL_LIMIT_CONFIRMED (0U)
#define MANUAL_ACTUAL_OVERSHOOT_MDEG  (500L)

#if MEASURED_BALANCE_VALID && \
    (MEASURED_BALANCE_PWM_MDEG < 0 || MEASURED_BALANCE_PWM_MDEG >= 360000)
#error "Measured PWM reference must be in [0, 360000) millidegrees"
#endif

#if MANUAL_COMMAND_LIMIT_MDEG <= 0 || MANUAL_COMMAND_LIMIT_MDEG > 15000
#error "Manual command limit must be in (0, 15000] millidegrees"
#endif
#if MANUAL_MECHANICAL_POS_LIMIT_MDEG <= 0 || \
    MANUAL_MECHANICAL_POS_LIMIT_MDEG > MANUAL_COMMAND_LIMIT_MDEG || \
    MANUAL_MECHANICAL_NEG_LIMIT_MDEG <= 0 || \
    MANUAL_MECHANICAL_NEG_LIMIT_MDEG > MANUAL_COMMAND_LIMIT_MDEG
#error "Manual mechanical limit must not exceed the command limit"
#endif
#if MANUAL_MECHANICAL_LIMIT_CONFIRMED > 1U
#error "Manual mechanical limit confirmation must be 0 or 1"
#endif
#endif
