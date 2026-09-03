#ifndef K230_H
#define K230_H

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * K230 UART wiring:
 *   K230 TXD (IO9)  -> MSPM0 PA22 (UART2 RX)
 *   K230 RXD (IO10) -> MSPM0 PA21 (UART2 TX)
 *   K230 GND        -> MSPM0 GND
 *   K230 5V         -> board 5V/VCC
 *
 * Target tracking frame (function ID 15):
 *   $<length>,15,<x>,<y>,<width>,<height>,#
 * x and y are the top-left corner of the target box.
 *
 * Target center / relative-X frame (function ID 16):
 *   $<length>,16,<relative_x>,<frame_dt_ms>,#
 * relative_x is the signed pixel offset of the target centre from the
 * horizontal image midpoint.
 * frame_dt_ms is the K230 measurement interval; zero selects the receiver's
 * backward-compatible default interval.
 */

/*
 * Local hardware defines — independent of SysConfig generated ti_msp_dl_config.
 * These are the same definitions that the moter project's SysConfig generated,
 * placed here so this module is self-contained and portable.
 */
#define K230_UART_INST                      UART2
#define K230_UART_INST_INT_IRQN             UART2_INT_IRQn
#define K230_UART_INST_IRQHandler           UART2_IRQHandler

#define GPIO_K230_UART_RX_PORT              GPIOA
#define GPIO_K230_UART_TX_PORT              GPIOA
#define GPIO_K230_UART_RX_PIN               DL_GPIO_PIN_22
#define GPIO_K230_UART_TX_PIN               DL_GPIO_PIN_21

#define K230_TARGET_TRACKING_FUNCTION_ID    (15U)
#define K230_TARGET_CENTER_FUNCTION_ID      (16U)
#define K230_FRAME_MAX_LENGTH               (64U)
#define K230_CENTER_FRAME_DT_MAX_MS         (60000U)

typedef struct {
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
    int32_t center_x;
    int32_t center_y;
} K230_TargetData;

/*
 * Data carried by a function-ID-16 frame.
 * relative_x  – signed horizontal offset from image centre (pixels).
 * frame_dt_ms – time since the previous transmitted position (milliseconds).
 */
typedef struct {
    int32_t relative_x;
    uint32_t frame_dt_ms;
} K230_CenterData;

typedef struct {
    uint32_t completed_frames;
    uint32_t valid_frames;
    uint32_t parse_errors;
    uint32_t overflow_errors;
    uint32_t dropped_frames;
} K230_Stats;

/* Call once after SYSCFG_DL_init(). */
void K230_Init(void);

/*
 * Call repeatedly in the main loop. Returns true when one complete and valid
 * target frame is available. The received frame is consumed by this call.
 */
bool K230_PollTarget(K230_TargetData *target);

/*
 * Call repeatedly in the main loop. Returns true when one complete and valid
 * function-ID-16 centre frame is available. The frame is consumed by this call.
 */
bool K230_PollCenter(K230_CenterData *center);

/* Public parser for direct frame parsing and protocol-level tests. */
bool K230_ParseFrame(
    const uint8_t *frame, uint8_t length, K230_TargetData *target);

bool K230_ParseCenterFrame(
    const uint8_t *frame, uint8_t length, K230_CenterData *center);

void K230_GetStats(K230_Stats *stats);

#endif /* K230_H */


