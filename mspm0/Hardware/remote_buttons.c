#include "remote_buttons.h"

#include "remote_button_decoder.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

/* SysConfig TIMER_0 is TIMG0 at 1.25 MHz; 1250 counts gives exactly 1 ms. */
#define REMOTE_TIMER_LOAD_VALUE             (1249U)
#define REMOTE_PENDING_MAX_AGE_MS           (1000U)

static RemotePulseDecoder_t s_decoder;
static volatile RemoteButtons_Stats s_stats;
static volatile RemoteButtonEvent_t s_pending_event;
static volatile uint32_t s_pending_age_ms;

static uint8_t ReadLineHigh(void)
{
    return (DL_GPIO_readPins(REMOTE_BUTTON_RX_PORT,
                             REMOTE_BUTTON_RX_PIN) != 0U) ? 1U : 0U;
}

static uint32_t EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void ExitCritical(uint32_t primask)
{
    if (primask == 0U) __enable_irq();
}

static void RegisterEvent(RemoteButtonEvent_t event)
{
    if (s_pending_event != REMOTE_BUTTON_EVENT_NONE) {
        s_stats.event_overruns++;
        return;
    }
    s_pending_event = event;
    s_pending_age_ms = 0U;
}

static void SamplePB2Every1ms(void)
{
    uint16_t pulse_ms = 0U;
    RemotePulseEvent_t event;

    if (s_stats.link_seen != 0U &&
        s_stats.last_valid_age_ms < UINT32_MAX) {
        s_stats.last_valid_age_ms++;
    }
    if (s_pending_event != REMOTE_BUTTON_EVENT_NONE) {
        if (s_pending_age_ms < UINT32_MAX) s_pending_age_ms++;
        if (s_pending_age_ms > REMOTE_PENDING_MAX_AGE_MS) {
            s_pending_event = REMOTE_BUTTON_EVENT_NONE;
            s_pending_age_ms = 0U;
            s_stats.stale_events++;
        }
    }

    s_stats.line_high = ReadLineHigh();
    event = RemotePulseDecoder_Tick1ms(
        &s_decoder, s_stats.line_high, &pulse_ms);
    s_stats.current_low_ms = s_decoder.low_ms;
    s_stats.stuck_low = s_decoder.stuck_low;

    switch (event) {
    case REMOTE_PULSE_HEARTBEAT:
        s_stats.valid_pulses++;
        s_stats.heartbeat_pulses++;
        s_stats.link_seen = 1U;
        s_stats.last_valid_age_ms = 0U;
        break;
    case REMOTE_PULSE_CALIBRATE:
        s_stats.valid_pulses++;
        s_stats.calibrate_pulses++;
        s_stats.link_seen = 1U;
        s_stats.last_valid_age_ms = 0U;
        RegisterEvent(REMOTE_BUTTON_EVENT_CALIBRATE);
        break;
    case REMOTE_PULSE_START:
        s_stats.valid_pulses++;
        s_stats.start_pulses++;
        s_stats.link_seen = 1U;
        s_stats.last_valid_age_ms = 0U;
        RegisterEvent(REMOTE_BUTTON_EVENT_START);
        break;
    case REMOTE_PULSE_INVALID:
        s_stats.invalid_pulses++;
        break;
    case REMOTE_PULSE_STUCK:
        s_stats.stuck_events++;
        break;
    default:
        break;
    }
}

void RemoteButtons_Init(void)
{
    uint8_t initial_high;

    /* PB2 is GPIO input with pull-up. It is not a UART RX pin on MSPM0G3507. */
    DL_GPIO_initDigitalInputFeatures(
        REMOTE_BUTTON_RX_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);

    initial_high = ReadLineHigh();
    RemotePulseDecoder_Init(&s_decoder, initial_high);

    s_stats.valid_pulses = 0U;
    s_stats.heartbeat_pulses = 0U;
    s_stats.calibrate_pulses = 0U;
    s_stats.start_pulses = 0U;
    s_stats.invalid_pulses = 0U;
    s_stats.event_overruns = 0U;
    s_stats.stale_events = 0U;
    s_stats.stuck_events = 0U;
    s_stats.last_valid_age_ms = UINT32_MAX;
    s_stats.current_low_ms = 0U;
    s_stats.link_seen = 0U;
    s_stats.line_high = initial_high;
    s_stats.stuck_low = 0U;
    s_pending_event = REMOTE_BUTTON_EVENT_NONE;
    s_pending_age_ms = 0U;

    /* TIMER_0 is otherwise unused by ROUND-023. Re-time it from 5 ms to 1 ms. */
    NVIC_DisableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_stopCounter(TIMER_0_INST);
    DL_TimerG_disableInterrupt(TIMER_0_INST,
                               DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_setLoadValue(TIMER_0_INST, REMOTE_TIMER_LOAD_VALUE);
    DL_TimerG_setTimerCount(TIMER_0_INST, REMOTE_TIMER_LOAD_VALUE);
    DL_TimerG_clearInterruptStatus(TIMER_0_INST,
                                  DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableInterrupt(TIMER_0_INST,
                              DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);
}

bool RemoteButtons_TakeEvent(RemoteButtonEvent_t *event)
{
    uint32_t primask;
    RemoteButtonEvent_t pending;

    if (event == NULL) return false;
    primask = EnterCritical();
    pending = s_pending_event;
    if (pending != REMOTE_BUTTON_EVENT_NONE) {
        s_pending_event = REMOTE_BUTTON_EVENT_NONE;
        s_pending_age_ms = 0U;
    }
    ExitCritical(primask);

    *event = pending;
    return (pending != REMOTE_BUTTON_EVENT_NONE);
}

void RemoteButtons_GetStats(RemoteButtons_Stats *stats)
{
    uint32_t primask;
    if (stats == NULL) return;

    primask = EnterCritical();
    stats->valid_pulses = s_stats.valid_pulses;
    stats->heartbeat_pulses = s_stats.heartbeat_pulses;
    stats->calibrate_pulses = s_stats.calibrate_pulses;
    stats->start_pulses = s_stats.start_pulses;
    stats->invalid_pulses = s_stats.invalid_pulses;
    stats->event_overruns = s_stats.event_overruns;
    stats->stale_events = s_stats.stale_events;
    stats->stuck_events = s_stats.stuck_events;
    stats->last_valid_age_ms = s_stats.last_valid_age_ms;
    stats->current_low_ms = s_stats.current_low_ms;
    stats->link_seen = s_stats.link_seen;
    stats->line_high = s_stats.line_high;
    stats->stuck_low = s_stats.stuck_low;
    ExitCritical(primask);
}

void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) !=
        DL_TIMERG_IIDX_ZERO) {
        return;
    }
    DL_TimerG_clearInterruptStatus(TIMER_0_INST,
                                  DL_TIMERG_INTERRUPT_ZERO_EVENT);
    SamplePB2Every1ms();
}


