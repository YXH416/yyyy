/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   TIMG8Ӳ��QEI��TIMG12 PWM��ϲ�������
  ******************************************************************************
  * QEI������Ϊ16λ����ģ��ÿ5ms��ȡһ�β���int16_t��ֵ��չ��int32_t��Ȧ������
  * PWM�жϼ�¼�������������ں��������½��ĸߵ�ƽʱ�䣬�ٰ�MT6816��ʽ����Ƕȡ�
  ******************************************************************************
  */
#include "encoder.h"
#include "demo_config.h"
#include "ti_msp_dl_config.h"

/* ����ʹ��SysConfig���ɵ�ʵ������������֧����Ϊȱ�����ɺ�ʱ�ļ��ݺ󱸡� */
#ifdef QEI_0_INST
#define ENCODER_QEI_INST QEI_0_INST
#else
#define ENCODER_QEI_INST TIMG8
#endif

#ifdef ENCODER_PWM_INST
#define ENCODER_PWM_TIMER      ENCODER_PWM_INST
#define ENCODER_PWM_LOAD       ENCODER_PWM_INST_LOAD_VALUE
#define ENCODER_PWM_IRQN       ENCODER_PWM_INST_INT_IRQN
#else
#define ENCODER_PWM_TIMER      TIMG12
#define ENCODER_PWM_LOAD       3999999U /* 50 ms at 80 MHz */
#define ENCODER_PWM_IRQN       TIMG12_INT_IRQn
#endif

/* ����100msû����ЧPWM���أ����ж�������PWM�źŶ�ʧ�� */
#define ENCODER_PWM_TIMEOUT_MS 100U

/* A/B����λ��״̬��Ӳ��16λ���������ڲ�����չΪ����32λ��Ȧ������ */
typedef struct {
    volatile int32_t accumulated;
    int32_t zero;
    int32_t speed_last;
    uint16_t last_hw_count;
    volatile int32_t z_count;
    volatile int32_t count_at_z;
} EncoderState_t;

static EncoderState_t s_encoder;
/* PWM����״̬��TIMG12�ж�д�롢��ѭ����ȡ���������Ϊvolatile�� */
static volatile uint32_t s_pwm_last_rise;
static volatile uint32_t s_pwm_period;
static volatile uint32_t s_pwm_high;
static volatile uint32_t s_pwm_age_ms;
static volatile uint8_t s_pwm_have_rise;
static volatile uint8_t s_pwm_valid;

/* ��QEI��ʼ��������Keil������empty.syscfg����QEI_0���ã���������ᱻ�õ��� */
static void Encoder_InitQEIWithoutGeneratedConfig(void)
{
#ifndef QEI_0_INST
    static DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };

    DL_TimerG_enablePower(ENCODER_QEI_INST);
    DL_TimerG_reset(ENCODER_QEI_INST);
    DL_Common_delayCycles(POWER_STARTUP_DELAY);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM2,
        IOMUX_PINCM2_PF_TIMG8_CCP0); /* PA1 = encoder A */
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM1,
        IOMUX_PINCM1_PF_TIMG8_CCP1); /* PA0 = encoder B */
    DL_TimerG_setClockConfig(ENCODER_QEI_INST, &clock_config);
    DL_TimerG_configQEI(ENCODER_QEI_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(ENCODER_QEI_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(ENCODER_QEI_INST, 0xFFFFU);
    DL_TimerG_setTimerCount(ENCODER_QEI_INST, 0U);
    DL_TimerG_enableClock(ENCODER_QEI_INST);
    DL_TimerG_startCounter(ENCODER_QEI_INST);
#endif
}

/* ��PWM�����ʼ��������������empty.syscfg����ENCODER_PWM���á� */
static void Encoder_InitPwmWithoutGeneratedConfig(void)
{
#ifndef ENCODER_PWM_INST
    static DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };
    static DL_TimerG_CaptureCombinedConfig capture_config = {
        .captureMode = DL_TIMER_CAPTURE_COMBINED_MODE_PULSE_WIDTH_AND_PERIOD,
        .period = ENCODER_PWM_LOAD,
        .startTimer = DL_TIMER_START,
        .inputChan = DL_TIMER_INPUT_CHAN_0,
        .inputInvMode = DL_TIMER_CC_INPUT_INV_NOINVERT
    };

    DL_TimerG_enablePower(ENCODER_PWM_TIMER);
    DL_TimerG_reset(ENCODER_PWM_TIMER);
    DL_Common_delayCycles(POWER_STARTUP_DELAY);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM48,
        IOMUX_PINCM48_PF_TIMG12_CCP0); /* PB20 = encoder PWM */
    DL_TimerG_setClockConfig(ENCODER_PWM_TIMER, &clock_config);
    DL_TimerG_initCaptureCombinedMode(ENCODER_PWM_TIMER, &capture_config);
    DL_TimerG_enableInterrupt(ENCODER_PWM_TIMER,
        DL_TIMERG_INTERRUPT_CC0_DN_EVENT | DL_TIMERG_INTERRUPT_CC1_DN_EVENT);
    DL_TimerG_enableClock(ENCODER_PWM_TIMER);
#endif
    NVIC_ClearPendingIRQ(ENCODER_PWM_IRQN);
    NVIC_EnableIRQ(ENCODER_PWM_IRQN);
}

/* TIMG���¼�����������������ʱ�̵ļ��ʱͬʱ�������������ơ� */
static uint32_t Encoder_PwmElapsed(uint32_t older, uint32_t newer)
{
    if (older >= newer) return older - newer;
    return older + (ENCODER_PWM_LOAD + 1U - newer);
}

/* ��ȡQEI��ǰ�߼����򣬲�Ӧ��ENCODER_AXIS_X_SIGN���ֶ�����������һ�¡� */
static int8_t Encoder_GetDirectionSign(void)
{
    uint8_t counting_down =
        (DL_TimerG_getQEIDirection(ENCODER_QEI_INST) == DL_TIMER_QEI_DIR_DOWN) ? 1U : 0U;
#if (ENCODER_AXIS_X_SIGN < 0)
    counting_down = (uint8_t)!counting_down;
#endif
    return (counting_down != 0U) ? -1 : 1;
}

/* ��Z�ж�ʱ��ȡ���ӽ�����ʱ�̵�A/Bλ�ã����ȴ���һ��5ms���ڲ����� */
static int32_t Encoder_GetCountAtCurrentEdge(void)
{
    uint16_t now = (uint16_t)DL_TimerG_getTimerCount(ENCODER_QEI_INST);
    int16_t delta = (int16_t)(now - s_encoder.last_hw_count);
#if (ENCODER_AXIS_X_SIGN < 0)
    delta = (int16_t)-delta;
#endif
    return s_encoder.accumulated + (int32_t)delta - s_encoder.zero;
}
/* �������״̬���ϵ統ǰλ����ΪA/B�ۼƼ�������㡣 */
void Encoder_Init(void)
{
    s_pwm_last_rise = 0U;
    s_pwm_period = 0U;
    s_pwm_high = 0U;
    s_pwm_age_ms = ENCODER_PWM_TIMEOUT_MS + 1U;
    s_pwm_have_rise = 0U;
    s_pwm_valid = 0U;
    Encoder_InitQEIWithoutGeneratedConfig();
    Encoder_InitPwmWithoutGeneratedConfig();
    DL_TimerG_setTimerCount(ENCODER_QEI_INST, 0U);
    DL_TimerG_startCounter(ENCODER_QEI_INST);
    s_encoder.accumulated = 0;
    s_encoder.zero = 0;
    s_encoder.speed_last = 0;
    s_encoder.last_hw_count = 0U;
    s_encoder.z_count = 0;
    s_encoder.count_at_z = 0;
    NVIC_ClearPendingIRQ(ENCODER_Z_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_Z_INT_IRQN);
}

/**
  * 5ms���ڲ���QEI��
  * ��16λ�޷��Ų�ֵǿ��ת��Ϊint16_t�����ڵ�����λ��С��32768����ʱ
  * �Զ�����0��65535֮�����������ƣ����ۼӳ�32λ��Ȧλ�á�
  */
void Encoder_Tick(uint32_t elapsed_ms)
{
    uint16_t now;
    int16_t delta;
    now = (uint16_t)DL_TimerG_getTimerCount(ENCODER_QEI_INST);
    delta = (int16_t)(now - s_encoder.last_hw_count);
    s_encoder.last_hw_count = now;
#if (ENCODER_AXIS_X_SIGN < 0)
    delta = (int16_t)-delta;
#endif
    s_encoder.accumulated += (int32_t)delta;
    if (s_pwm_age_ms <= ENCODER_PWM_TIMEOUT_MS) s_pwm_age_ms += elapsed_ms;
    if (s_pwm_age_ms > ENCODER_PWM_TIMEOUT_MS) s_pwm_valid = 0U;
}

int32_t Encoder_GetCount(EncoderAxis_t axis)
{
    if (axis != ENCODER_AXIS_X) return 0;
    return s_encoder.accumulated - s_encoder.zero;
}

float Encoder_GetAngle(EncoderAxis_t axis)
{
    return (float)Encoder_GetCount(axis) * 360.0f / ENCODER_COUNTS_PER_REV;
}

/* �ٶ�=��������/ÿȦ����*360��/����ʱ�䡣 */
float Encoder_CalcSpeedDps(EncoderAxis_t axis, uint32_t period_ms)
{
    int32_t now, delta;
    if (axis != ENCODER_AXIS_X || period_ms == 0U) return 0.0f;
    now = Encoder_GetCount(axis);
    delta = now - s_encoder.speed_last;
    s_encoder.speed_last = now;
    return (float)delta * 360000.0f /
           ((float)ENCODER_COUNTS_PER_REV * (float)period_ms);
}

/* ��������ֻ�޸����ƫ�ƣ���ǿ�Ƹ�дӲ��QEI�������� */
void Encoder_SetZero(EncoderAxis_t axis)
{
    if (axis != ENCODER_AXIS_X) return;
    s_encoder.zero = s_encoder.accumulated;
    s_encoder.speed_last = 0;
    s_encoder.z_count = 0;
    s_encoder.count_at_z = 0;
}

/* ��������ֻ�޸����ƫ�ƣ���ǿ�Ƹ�дӲ��QEI�������� */
void Encoder_SetZeroAll(void) { Encoder_SetZero(ENCODER_AXIS_X); }

/* ��ȡ����Z�������ۼƵ��з��ž�Ȧ���� */
int32_t Encoder_GetZCount(EncoderAxis_t axis)
{
    return (axis == ENCODER_AXIS_X) ? s_encoder.z_count : 0;
}

int32_t Encoder_GetCountAtLastZ(EncoderAxis_t axis)
{
    return (axis == ENCODER_AXIS_X) ? s_encoder.count_at_z : 0;
}

/**
  * PWMռ�ձ�ת���ԽǶȡ�
  * �ο�MS42CG/MT6816��ʽ��λ����=duty*4115-1����ӳ�䵽0~360�ȡ�
  */
uint8_t Encoder_GetPwmAngle(EncoderAxis_t axis, float *angle_deg)
{
    float duty;
    float angle;
    uint32_t period;
    uint32_t high;
    if (axis != ENCODER_AXIS_X || angle_deg == 0 || s_pwm_valid == 0U) return 0U;
    period = s_pwm_period;
    high = s_pwm_high;
    if (period == 0U || high == 0U || high >= period) return 0U;
    duty = (float)high / (float)period;
    angle = ((duty * 4115.0f) - 1.0f) * 360.0f / 4115.0f;
    if (angle < 0.0f) angle = 0.0f;
    if (angle >= 360.0f) angle -= 360.0f;
    *angle_deg = angle;
    return 1U;
}

/**
  * PWM���벶���жϣ�CC1��¼�����ز��������ڣ�CC0��¼�½��ز�����ߵ�ƽ��
  * ֻ��0<�ߵ�ƽ<����ʱ�ŷ�����Ч���ݲ�ˢ�³�ʱ��ʱ����
  */
void TIMG12_IRQHandler(void)
{
    DL_TIMER_IIDX pending;
    do {
        pending = DL_TimerG_getPendingInterrupt(ENCODER_PWM_TIMER);
        if (pending == DL_TIMERG_IIDX_CC1_DN) {
            uint32_t rise = DL_TimerG_getCaptureCompareValue(
                ENCODER_PWM_TIMER, DL_TIMER_CC_1_INDEX);
            if (s_pwm_have_rise != 0U) {
                s_pwm_period = Encoder_PwmElapsed(s_pwm_last_rise, rise);
            }
            s_pwm_last_rise = rise;
            s_pwm_have_rise = 1U;
        } else if (pending == DL_TIMERG_IIDX_CC0_DN) {
            uint32_t fall = DL_TimerG_getCaptureCompareValue(
                ENCODER_PWM_TIMER, DL_TIMER_CC_0_INDEX);
            if (s_pwm_have_rise != 0U) {
                uint32_t high = Encoder_PwmElapsed(s_pwm_last_rise, fall);
                s_pwm_high = high;
                if (s_pwm_period > high && high != 0U) {
                    s_pwm_age_ms = 0U;
                    s_pwm_valid = 1U;
                }
            }
        }
    } while ((uint32_t)pending != 0U);
}
/**
  * ������Z�������жϡ�
  * Z�źű����������򣬱���ͬʱ��ȡӲ��QEI���򣬲��ܵõ��з��ž�Ȧ����
  */
void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(ENCODER_Z_PORT)) {
        case ENCODER_Z_Z_IIDX:
            s_encoder.z_count += Encoder_GetDirectionSign();
            s_encoder.count_at_z = Encoder_GetCountAtCurrentEdge();
            break;
        default:
            break;
    }
}

