#ifndef MOTOR_BALANCE_CONFIG_H
#define MOTOR_BALANCE_CONFIG_H

/* ROUND-037: user CAL,BALANCE result at MCU ms=442500.
 * Absolute PWM = 176.861 deg; 16-sample spread = 0.139 deg.
 * This changes the measurement zero only; it never commands boot motion. */
#define MEASURED_BALANCE_VALID       (1U)
#define MEASURED_BALANCE_PWM_MDEG    (176861L)

#if MEASURED_BALANCE_VALID && \
    (MEASURED_BALANCE_PWM_MDEG < 0 || MEASURED_BALANCE_PWM_MDEG >= 360000)
#error "Measured PWM reference must be in [0, 360000) millidegrees"
#endif
#endif
