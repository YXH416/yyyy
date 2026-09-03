#ifndef REMOTE_BUTTONS_H
#define REMOTE_BUTTONS_H

#include <stdbool.h>
#include <stdint.h>

/* Receiver wire: secondary-board GPIO output -> main-board PB2. */
#define REMOTE_BUTTON_RX_PORT              GPIOB
#define REMOTE_BUTTON_RX_PIN               DL_GPIO_PIN_2
#define REMOTE_BUTTON_RX_IOMUX              IOMUX_PINCM15

typedef struct {
    uint32_t valid_pulses;
    uint32_t heartbeat_pulses;
    uint32_t calibrate_pulses;
    uint32_t start_pulses;
    uint32_t invalid_pulses;
    uint32_t event_overruns;
    uint32_t stale_events;
    uint32_t stuck_events;
    uint32_t last_valid_age_ms;
    uint16_t current_low_ms;
    uint8_t link_seen;
    uint8_t line_high;
    uint8_t stuck_low;
} RemoteButtons_Stats;

typedef enum {
    REMOTE_BUTTON_EVENT_NONE = 0,
    REMOTE_BUTTON_EVENT_CALIBRATE,
    REMOTE_BUTTON_EVENT_START
} RemoteButtonEvent_t;

void RemoteButtons_Init(void);
bool RemoteButtons_TakeEvent(RemoteButtonEvent_t *event);
void RemoteButtons_GetStats(RemoteButtons_Stats *stats);

#endif /* REMOTE_BUTTONS_H */


