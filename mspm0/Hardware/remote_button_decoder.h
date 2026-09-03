#ifndef REMOTE_BUTTON_DECODER_H
#define REMOTE_BUTTON_DECODER_H

#include <stdint.h>

/*
 * One-wire active-low pulse protocol. Samples come from an independent
 * hardware-timer ISR, so blocking UART0 printf calls cannot distort widths.
 *
 * Sender targets:
 *   heartbeat:  20 ms LOW (optional, recommended once per second)
 *   calibrate: 100 ms LOW
 *   start:     400 ms LOW
 */
#define REMOTE_PULSE_HEARTBEAT_TARGET_MS    (20U)
#define REMOTE_PULSE_CAL_TARGET_MS          (100U)
#define REMOTE_PULSE_START_TARGET_MS        (400U)

#define REMOTE_PULSE_HEARTBEAT_MIN_MS       (12U)
#define REMOTE_PULSE_HEARTBEAT_MAX_MS       (35U)
#define REMOTE_PULSE_CAL_MIN_MS             (70U)
#define REMOTE_PULSE_CAL_MAX_MS             (150U)
#define REMOTE_PULSE_START_MIN_MS           (300U)
#define REMOTE_PULSE_START_MAX_MS           (550U)
#define REMOTE_PULSE_DEBOUNCE_MS             (3U)
#define REMOTE_PULSE_REARM_HIGH_MS          (100U)
#define REMOTE_PULSE_STUCK_LOW_MS           (700U)

typedef enum {
    REMOTE_PULSE_NONE = 0,
    REMOTE_PULSE_HEARTBEAT,
    REMOTE_PULSE_CALIBRATE,
    REMOTE_PULSE_START,
    REMOTE_PULSE_INVALID,
    REMOTE_PULSE_STUCK
} RemotePulseEvent_t;

typedef struct {
    uint16_t low_ms;
    uint16_t high_ms;
    uint8_t low_active;
    uint8_t pulse_eligible;
    uint8_t armed;
    uint8_t stuck_low;
    uint8_t stuck_reported;
    uint8_t stable_high;
    uint8_t candidate_high;
    uint8_t candidate_count;
} RemotePulseDecoder_t;

void RemotePulseDecoder_Init(RemotePulseDecoder_t *decoder,
                             uint8_t initial_high);
RemotePulseEvent_t RemotePulseDecoder_Tick1ms(
    RemotePulseDecoder_t *decoder, uint8_t line_high, uint16_t *pulse_ms);

#endif /* REMOTE_BUTTON_DECODER_H */


