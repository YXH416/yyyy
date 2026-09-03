#ifndef MOTOR_BALANCE_CONFIG_H
#define MOTOR_BALANCE_CONFIG_H

/* ROUND-036: awaiting the real CAL,BALANCE record. Values are millidegrees.
 * Once measured, change both defines to the exact [COPY_TO_CODE] values.
 * This changes the measurement zero only; it never commands boot motion. */
#define MEASURED_BALANCE_VALID       (0U)
#define MEASURED_BALANCE_PWM_MDEG    (0L)

#if MEASURED_BALANCE_VALID && \
    (MEASURED_BALANCE_PWM_MDEG < 0 || MEASURED_BALANCE_PWM_MDEG >= 360000)
#error "Measured PWM reference must be in [0, 360000) millidegrees"
#endif
#endif
