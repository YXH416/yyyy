#include "remote_button_decoder.h"

#include <stddef.h>

static uint16_t SaturatingIncrement(uint16_t value)
{
    return (value < UINT16_MAX) ? (uint16_t)(value + 1U) : value;
}

static RemotePulseEvent_t ClassifyPulse(uint16_t pulse_ms)
{
    if (pulse_ms >= REMOTE_PULSE_HEARTBEAT_MIN_MS &&
        pulse_ms <= REMOTE_PULSE_HEARTBEAT_MAX_MS) {
        return REMOTE_PULSE_HEARTBEAT;
    }
    if (pulse_ms >= REMOTE_PULSE_CAL_MIN_MS &&
        pulse_ms <= REMOTE_PULSE_CAL_MAX_MS) {
        return REMOTE_PULSE_CALIBRATE;
    }
    if (pulse_ms >= REMOTE_PULSE_START_MIN_MS &&
        pulse_ms <= REMOTE_PULSE_START_MAX_MS) {
        return REMOTE_PULSE_START;
    }
    return REMOTE_PULSE_INVALID;
}

void RemotePulseDecoder_Init(RemotePulseDecoder_t *decoder,
                             uint8_t initial_high)
{
    if (decoder == NULL) return;

    decoder->low_ms = 0U;
    decoder->high_ms = (initial_high != 0U) ? 1U : 0U;
    decoder->low_active = (initial_high != 0U) ? 0U : 1U;
    decoder->pulse_eligible = 0U;
    decoder->armed = 0U;
    decoder->stuck_low = 0U;
    decoder->stuck_reported = 0U;
    decoder->stable_high = (initial_high != 0U) ? 1U : 0U;
    decoder->candidate_high = decoder->stable_high;
    decoder->candidate_count = REMOTE_PULSE_DEBOUNCE_MS;
}

RemotePulseEvent_t RemotePulseDecoder_Tick1ms(
    RemotePulseDecoder_t *decoder, uint8_t line_high, uint16_t *pulse_ms)
{
    RemotePulseEvent_t event = REMOTE_PULSE_NONE;
    uint8_t sampled_high = (line_high != 0U) ? 1U : 0U;

    if (pulse_ms != NULL) *pulse_ms = 0U;
    if (decoder == NULL) return REMOTE_PULSE_INVALID;

    if (sampled_high != decoder->candidate_high) {
        decoder->candidate_high = sampled_high;
        decoder->candidate_count = 1U;
    } else if (decoder->candidate_count < REMOTE_PULSE_DEBOUNCE_MS) {
        decoder->candidate_count++;
    }
    if (decoder->candidate_count >= REMOTE_PULSE_DEBOUNCE_MS) {
        decoder->stable_high = decoder->candidate_high;
    }

    if (decoder->stable_high != 0U) {
        if (decoder->low_active != 0U) {
            if (pulse_ms != NULL) *pulse_ms = decoder->low_ms;
            if (decoder->pulse_eligible != 0U) {
                event = ClassifyPulse(decoder->low_ms);
            }
            decoder->low_active = 0U;
            decoder->pulse_eligible = 0U;
            decoder->low_ms = 0U;
            decoder->high_ms = 1U;
            decoder->armed = 0U;
            decoder->stuck_low = 0U;
            decoder->stuck_reported = 0U;
            return event;
        }

        decoder->high_ms = SaturatingIncrement(decoder->high_ms);
        if (decoder->high_ms >= REMOTE_PULSE_REARM_HIGH_MS) {
            decoder->armed = 1U;
        }
        return REMOTE_PULSE_NONE;
    }

    decoder->high_ms = 0U;
    if (decoder->low_active == 0U) {
        decoder->low_active = 1U;
        decoder->pulse_eligible = decoder->armed;
        decoder->low_ms = 1U;
        decoder->armed = 0U;
    } else {
        decoder->low_ms = SaturatingIncrement(decoder->low_ms);
    }

    if (decoder->low_ms >= REMOTE_PULSE_STUCK_LOW_MS) {
        decoder->stuck_low = 1U;
        decoder->pulse_eligible = 0U;
        if (decoder->stuck_reported == 0U) {
            decoder->stuck_reported = 1U;
            return REMOTE_PULSE_STUCK;
        }
    }
    return REMOTE_PULSE_NONE;
}


