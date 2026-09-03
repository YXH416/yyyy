#include "k230.h"

#include <limits.h>
#include <stddef.h>

#define K230_FRAME_HEAD                    ((uint8_t) '$')
#define K230_FRAME_TAIL                    ((uint8_t) '#')
#define K230_PROTOCOL_FIELD_COUNT          (6U)

static volatile uint8_t k230_build_buffer[K230_FRAME_MAX_LENGTH];
static volatile uint8_t k230_build_length;
static volatile bool k230_receiving;

static volatile uint8_t k230_frame_buffer[K230_FRAME_MAX_LENGTH];
static volatile uint8_t k230_frame_length;
static volatile bool k230_frame_ready;
static volatile K230_Stats k230_stats;

static void K230_ReceiveByte(uint8_t byte)
{
    uint8_t i;

    /* A new frame head always resynchronizes the receiver. */
    if (byte == K230_FRAME_HEAD) {
        k230_build_length = 0U;
        k230_receiving = true;
    }

    if (!k230_receiving) {
        return;
    }

    if (k230_build_length >= K230_FRAME_MAX_LENGTH) {
        k230_build_length = 0U;
        k230_receiving = false;
        k230_stats.overflow_errors++;
        return;
    }

    k230_build_buffer[k230_build_length] = byte;
    k230_build_length++;

    if (byte != K230_FRAME_TAIL) {
        return;
    }

    /* Keep the pending frame stable until the main loop consumes it. */
    if (!k230_frame_ready) {
        for (i = 0U; i < k230_build_length; i++) {
            k230_frame_buffer[i] = k230_build_buffer[i];
        }
        k230_frame_length = k230_build_length;
        k230_frame_ready = true;
        k230_stats.completed_frames++;
    } else {
        k230_stats.dropped_frames++;
    }

    k230_build_length = 0U;
    k230_receiving = false;
}

static bool K230_ParseInteger(
    const uint8_t *text, uint8_t length, int32_t *value)
{
    uint32_t magnitude = 0U;
    uint32_t limit = (uint32_t) INT32_MAX;
    uint8_t index = 0U;
    bool negative = false;

    if ((text == NULL) || (value == NULL) || (length == 0U)) {
        return false;
    }

    if (text[0] == (uint8_t) '-') {
        negative = true;
        limit = (uint32_t) INT32_MAX + 1U;
        index = 1U;
        if (length == 1U) {
            return false;
        }
    }

    for (; index < length; index++) {
        uint8_t digit;

        if ((text[index] < (uint8_t) '0') ||
            (text[index] > (uint8_t) '9')) {
            return false;
        }

        digit = (uint8_t) (text[index] - (uint8_t) '0');
        if (magnitude > ((limit - digit) / 10U)) {
            return false;
        }
        magnitude = (magnitude * 10U) + digit;
    }

    if (negative) {
        if (magnitude == ((uint32_t) INT32_MAX + 1U)) {
            *value = INT32_MIN;
        } else {
            *value = -(int32_t) magnitude;
        }
    } else {
        *value = (int32_t) magnitude;
    }

    return true;
}

/*
 * Self-contained hardware initialization for UART2 (K230 communication).
 * This module does NOT depend on SysConfig-generated K230_UART defines —
 * all pin, peripheral, and baud-rate configuration is local.
 */
void K230_Init(void)
{
    k230_build_length = 0U;
    k230_receiving = false;
    k230_frame_length = 0U;
    k230_frame_ready = false;
    k230_stats.completed_frames = 0U;
    k230_stats.valid_frames = 0U;
    k230_stats.parse_errors = 0U;
    k230_stats.overflow_errors = 0U;
    k230_stats.dropped_frames = 0U;

    /* --- Hardware init for UART2 (manually, no SysConfig dependency) --- */
    DL_UART_Main_reset(K230_UART_INST);
    DL_UART_Main_enablePower(K230_UART_INST);

    /* GPIO: PA22 = RX (input), PA21 = TX (output) */
    DL_GPIO_initPeripheralInputFunction(
        IOMUX_PINCM47, IOMUX_PINCM47_PF_UART2_RX);
    DL_GPIO_initPeripheralOutputFunction(
        IOMUX_PINCM46, IOMUX_PINCM46_PF_UART2_TX);

    /* UART config: 115200-8-N-1 */
    {
        static const DL_UART_Main_ClockConfig clockCfg = {
            .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
            .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
        };
        static const DL_UART_Main_Config uartCfg = {
            .mode        = DL_UART_MAIN_MODE_NORMAL,
            .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
            .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
            .parity      = DL_UART_MAIN_PARITY_NONE,
            .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
            .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
        };

        DL_UART_Main_setClockConfig(K230_UART_INST,
            (DL_UART_Main_ClockConfig *)&clockCfg);
        DL_UART_Main_init(K230_UART_INST,
            (DL_UART_Main_Config *)&uartCfg);
    }
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78  (40 MHz / (16 * 21 + 45/64))
     */
    DL_UART_Main_setOversampling(K230_UART_INST,
        DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(K230_UART_INST, 21, 45);

    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(K230_UART_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(K230_UART_INST);

    NVIC_ClearPendingIRQ(K230_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_UART_INST_INT_IRQN);
}

bool K230_ParseFrame(
    const uint8_t *frame, uint8_t length, K230_TargetData *target)
{
    int32_t fields[K230_PROTOCOL_FIELD_COUNT];
    uint8_t field_count = 0U;
    uint8_t field_start = 1U;
    uint8_t index;
    int32_t half_width;
    int32_t half_height;

    if ((frame == NULL) || (target == NULL) ||
        (length < 3U) || (length > K230_FRAME_MAX_LENGTH) ||
        (frame[0] != K230_FRAME_HEAD) ||
        (frame[length - 1U] != K230_FRAME_TAIL)) {
        return false;
    }

    for (index = 1U; index < length; index++) {
        if ((frame[index] != (uint8_t) ',') &&
            (frame[index] != K230_FRAME_TAIL)) {
            continue;
        }

        if (index > field_start) {
            if ((field_count >= K230_PROTOCOL_FIELD_COUNT) ||
                !K230_ParseInteger(&frame[field_start],
                    (uint8_t) (index - field_start),
                    &fields[field_count])) {
                return false;
            }
            field_count++;
        } else if (!((frame[index] == K230_FRAME_TAIL) &&
                     (field_count == K230_PROTOCOL_FIELD_COUNT))) {
            return false;
        }

        field_start = (uint8_t) (index + 1U);
    }

    if ((field_count != K230_PROTOCOL_FIELD_COUNT) ||
        (fields[0] != length) ||
        (fields[1] != (int32_t) K230_TARGET_TRACKING_FUNCTION_ID) ||
        (fields[2] < 0) || (fields[3] < 0) ||
        (fields[4] <= 0) || (fields[5] <= 0)) {
        return false;
    }

    half_width = fields[4] / 2;
    half_height = fields[5] / 2;
    if ((fields[2] > (INT32_MAX - half_width)) ||
        (fields[3] > (INT32_MAX - half_height))) {
        return false;
    }

    target->x = fields[2];
    target->y = fields[3];
    target->width = fields[4];
    target->height = fields[5];
    target->center_x = fields[2] + half_width;
    target->center_y = fields[3] + half_height;

    return true;
}

bool K230_PollTarget(K230_TargetData *target)
{
    uint8_t frame[K230_FRAME_MAX_LENGTH];
    uint8_t length;
    uint8_t index;

    if ((target == NULL) || !k230_frame_ready) {
        return false;
    }

    /* The ISR does not modify this buffer while frame_ready is true. */
    length = k230_frame_length;
    for (index = 0U; index < length; index++) {
        frame[index] = k230_frame_buffer[index];
    }
    k230_frame_ready = false;

    if (K230_ParseFrame(frame, length, target)) {
        k230_stats.valid_frames++;
        return true;
    }

    k230_stats.parse_errors++;
    return false;
}

bool K230_ParseCenterFrame(
    const uint8_t *frame, uint8_t length, K230_CenterData *center)
{
    /* Frame format: $<len>,16,<relative_x>,<frame_dt_ms>,# -> 4 fields */
    #define K230_CENTER_FIELD_COUNT (4U)
    int32_t fields[K230_CENTER_FIELD_COUNT];
    uint8_t field_count = 0U;
    uint8_t field_start = 1U;
    uint8_t index;

    if ((frame == NULL) || (center == NULL) ||
        (length < 3U) || (length > K230_FRAME_MAX_LENGTH) ||
        (frame[0] != K230_FRAME_HEAD) ||
        (frame[length - 1U] != K230_FRAME_TAIL)) {
        return false;
    }

    for (index = 1U; index < length; index++) {
        if ((frame[index] != (uint8_t) ',') &&
            (frame[index] != K230_FRAME_TAIL)) {
            continue;
        }

        if (index > field_start) {
            if ((field_count >= K230_CENTER_FIELD_COUNT) ||
                !K230_ParseInteger(&frame[field_start],
                    (uint8_t) (index - field_start),
                    &fields[field_count])) {
                return false;
            }
            field_count++;
        } else if (!((frame[index] == K230_FRAME_TAIL) &&
                     (field_count == K230_CENTER_FIELD_COUNT))) {
            return false;
        }

        field_start = (uint8_t) (index + 1U);
    }

    if ((field_count != K230_CENTER_FIELD_COUNT) ||
        (fields[0] != length) ||
        (fields[1] != (int32_t) K230_TARGET_CENTER_FUNCTION_ID) ||
        (fields[3] < 0) ||
        (fields[3] > (int32_t) K230_CENTER_FRAME_DT_MAX_MS)) {
        return false;
    }

    center->relative_x = fields[2];
    center->frame_dt_ms = (uint32_t) fields[3];

    return true;
    #undef K230_CENTER_FIELD_COUNT
}

bool K230_PollCenter(K230_CenterData *center)
{
    uint8_t frame[K230_FRAME_MAX_LENGTH];
    uint8_t length;
    uint8_t index;

    if ((center == NULL) || !k230_frame_ready) {
        return false;
    }

    /* The ISR does not modify this buffer while frame_ready is true. */
    length = k230_frame_length;
    for (index = 0U; index < length; index++) {
        frame[index] = k230_frame_buffer[index];
    }
    k230_frame_ready = false;

    if (K230_ParseCenterFrame(frame, length, center)) {
        k230_stats.valid_frames++;
        return true;
    }

    k230_stats.parse_errors++;
    return false;
}

void K230_GetStats(K230_Stats *stats)
{
    if (stats == NULL) {
        return;
    }

    stats->completed_frames = k230_stats.completed_frames;
    stats->valid_frames = k230_stats.valid_frames;
    stats->parse_errors = k230_stats.parse_errors;
    stats->overflow_errors = k230_stats.overflow_errors;
    stats->dropped_frames = k230_stats.dropped_frames;
}

void K230_UART_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(K230_UART_INST))
    {
    case DL_UART_IIDX_RX:
        while (!DL_UART_isRXFIFOEmpty(K230_UART_INST)) {
            K230_ReceiveByte(DL_UART_receiveData(K230_UART_INST));
        }
        break;
    default:
        break;
    }
}


