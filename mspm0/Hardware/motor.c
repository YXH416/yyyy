/**
  ******************************************************************************
  * @file    motor.c
  * @brief   ����TIMA1Ӳ��PWM��D36A STEP���巢����
  ******************************************************************************
  * Ӳ��PWM��֤���������ȶ�������ģʽ��ÿ��PWM�����ж���ݼ�ʣ�ಽ����
  * ����������رն�ʱ����ǿ��STEPΪ�ͣ�����ֹͣλ�ö��ë�����塣
  ******************************************************************************
  */
#include "motor.h"

/* �жϺ���ѭ�������ĵ�������״̬������ʹ��volatile�� */
static volatile uint32_t s_remaining_steps;
static volatile uint8_t s_busy;
static volatile uint8_t s_continuous;
static volatile uint8_t s_direction;

/* ��ֹPWM���ʱǿ��STEP���ֵ͵�ƽ��������������ʶ����ء� */
static void Motor_ForceLow(void)
{
    DL_TimerA_setCCPOutputDisabled(MOTOR_STEP_TIMER,
        DL_TIMER_CCP_DIS_OUT_LOW, DL_TIMER_CCP_DIS_OUT_LOW);
}

/* �ָ�CCPͨ���ɶ�ʱ��������ơ� */
static void Motor_UsePWM(void)
{
    DL_TimerA_setCCPOutputDisabled(MOTOR_STEP_TIMER,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL, DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

/*
 * ֹͣ�ײ㶯������������Ҫ��֤ԭ���ԣ��ж��ڲ���ֱ�ӵ��ã�
 * ��ѭ������ʱ������ٽ�����ֹ�ͼƲ��ж�ͬʱ�޸�״̬��
 */
static void Motor_StopUnsafe(void)
{
    DL_TimerA_disableInterrupt(MOTOR_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_stopCounter(MOTOR_STEP_TIMER);
    DL_TimerA_setCaptureCompareValue(MOTOR_STEP_TIMER, 0U, MOTOR_STEP_CC_INDEX);
    Motor_ForceLow();
    s_remaining_steps = 0U;
    s_busy = 0U;
    s_continuous = 0U;
}

/* ��STEPƵ�ʻ���Ϊ1MHz��ʱ�����ڣ���������16λ��ʱ����Ч��Χ�� */
static uint16_t Motor_FrequencyToPeriod(uint32_t frequency_hz)
{
    uint32_t period = (MOTOR_STEP_TIMER_CLK_HZ + frequency_hz / 2U) / frequency_hz;
    if (period < 100U) period = 100U;
    if (period > 65535U) period = 65535U;
    return (uint16_t)period;
}

/* ����50%ռ�ձ�STEP���Σ���������LOAD���¿�ʼ����֤�׸����������� */
static void Motor_ConfigurePWM(uint32_t frequency_hz)
{
    uint32_t period = Motor_FrequencyToPeriod(frequency_hz);
    uint32_t load = period - 1U;
    DL_TimerA_stopCounter(MOTOR_STEP_TIMER);
    Motor_UsePWM();
    DL_TimerA_setLoadValue(MOTOR_STEP_TIMER, load);
    DL_TimerA_setTimerCount(MOTOR_STEP_TIMER, load);
    DL_TimerA_setCaptureCompareValue(MOTOR_STEP_TIMER, period / 2U,
                                     MOTOR_STEP_CC_INDEX);
}

/* �ϵ�Ĭ�ϣ�EN�ø�ʹ����������DIR�õͣ�STEPֹͣ�ұ��ֵ͵�ƽ�� */
void Motor_Init(void)
{
    DL_GPIO_initDigitalOutputFeatures(IOMUX_PINCM34,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_DRIVE_STRENGTH_LOW,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_enableOutput(MOTOR_EN_PORT, MOTOR_EN_PIN);
    DL_GPIO_setPins(MOTOR_EN_PORT, MOTOR_EN_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    s_direction = 0U;
    Motor_StopUnsafe();
    NVIC_ClearPendingIRQ(PWM_0_INST_INT_IRQN);
    NVIC_EnableIRQ(PWM_0_INST_INT_IRQN);
}

/* ֻ����ͣ��ʱ�ı�DIR�������˶�������ͻȻ������ɶ����� */
MotorStatus_t Motor_SetDirection(MotorAxis_t axis, uint8_t high_level)
{
    if (axis != MOTOR_AXIS_X || high_level > 1U) return MOTOR_ERROR;
    if (s_busy != 0U) return MOTOR_BUSY;
    if (high_level != 0U) DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    else DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_PIN);
    s_direction = high_level;
    return MOTOR_OK;
}

uint8_t Motor_GetDirection(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X) ? s_direction : 0U;
}

/* ������������Σ����жϽ����ٽ�������֤busy��ʣ�ಽ��ͬ�����¡� */
MotorStatus_t Motor_Start(MotorAxis_t axis, uint32_t steps,
                          uint32_t frequency_hz)
{
    uint32_t primask;
    if (axis != MOTOR_AXIS_X || frequency_hz < MOTOR_MIN_FREQ_HZ ||
        frequency_hz > MOTOR_MAX_FREQ_HZ) return MOTOR_ERROR;
    if (steps == 0U) { Motor_Stop(axis); return MOTOR_OK; }
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_busy != 0U) {
        if (primask == 0U) __enable_irq();
        return MOTOR_BUSY;
    }
    Motor_ConfigurePWM(frequency_hz);
    s_remaining_steps = steps;
    s_continuous = 0U;
    s_busy = 1U;
    DL_TimerA_clearInterruptStatus(MOTOR_STEP_TIMER,
                                   DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(MOTOR_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(MOTOR_STEP_TIMER);
    if (primask == 0U) __enable_irq();
    return MOTOR_OK;
}

/* ʵ��2������ת���ر�ZERO�Ʋ��жϣ���Ӳ��PWM�����������С� */
void Motor_StartContinuous(uint32_t frequency_hz, uint8_t high_level)
{
    uint32_t primask;
    if (frequency_hz < MOTOR_MIN_FREQ_HZ || frequency_hz > MOTOR_MAX_FREQ_HZ) return;
    Motor_Stop(MOTOR_AXIS_X);
    (void)Motor_SetDirection(MOTOR_AXIS_X, high_level);
    primask = __get_PRIMASK();
    __disable_irq();
    Motor_ConfigurePWM(frequency_hz);
    s_remaining_steps = UINT32_MAX;
    s_continuous = 1U;
    s_busy = 1U;
    DL_TimerA_disableInterrupt(MOTOR_STEP_TIMER, DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_startCounter(MOTOR_STEP_TIMER);
    if (primask == 0U) __enable_irq();
}

/* �ɴ���ѭ����ȫ���õ�ֹͣ�ӿڡ� */
void Motor_Stop(MotorAxis_t axis)
{
    uint32_t primask;
    if (axis != MOTOR_AXIS_X) return;
    primask = __get_PRIMASK();
    __disable_irq();
    Motor_StopUnsafe();
    if (primask == 0U) __enable_irq();
}

void Motor_StopAll(void) { Motor_Stop(MOTOR_AXIS_X); }
uint8_t Motor_IsBusy(MotorAxis_t axis) { return (axis == MOTOR_AXIS_X) ? s_busy : 0U; }
uint32_t Motor_GetRemainingSteps(MotorAxis_t axis)
{
    return (axis == MOTOR_AXIS_X) ? s_remaining_steps : 0U;
}

/**
  * STEP���ڼ����жϣ�һ��ZERO�¼���Ӧ���һ������STEP���ڡ�
  * ����ģʽ�����뱾�����߼������޲�������0������ֹͣ�����
  */
void TIMA1_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(MOTOR_STEP_TIMER) != DL_TIMERA_IIDX_ZERO) return;
    if (s_busy == 0U || s_continuous != 0U) return;
    if (s_remaining_steps > 0U) s_remaining_steps--;
    if (s_remaining_steps == 0U) Motor_StopUnsafe();
}


