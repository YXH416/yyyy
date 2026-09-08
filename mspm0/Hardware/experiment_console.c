/* UART0 full duplex console. ISRs receive bytes only, never execute motion. */
#include "ti_msp_dl_config.h"
#include "experiment_console.h"
#include <stdio.h>
#include <stdarg.h>

#define RX_SIZE 512U
#define TX_SIZE 4096U
#define RX_ERRORS (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | \
                   DL_UART_MAIN_INTERRUPT_FRAMING_ERROR | \
                   DL_UART_MAIN_INTERRUPT_PARITY_ERROR | \
                   DL_UART_MAIN_INTERRUPT_BREAK_ERROR)
static volatile uint8_t s_rx[RX_SIZE];
static volatile uint16_t s_read, s_write;
static volatile uint8_t s_bad_rx;
static volatile uint32_t s_rx_errors;
static volatile uint32_t s_rx_overflows;
static volatile uint32_t s_rx_bytes;
static volatile uint32_t s_rx_error_bits;
static uint32_t s_last_loss_report_ms;
static uint8_t s_loss_report_pending;
static uint32_t s_rx_lines;
static char s_tx[TX_SIZE];
static uint16_t s_tx_read, s_tx_write;
static uint32_t s_tx_drops;
static ExperimentParser s_parser;

void Console_Init(void)
{
    ExperimentParser_Reset(&s_parser);
    /* Do not depend on a stale CCS-generated SysConfig enabling the FIFO. */
    DL_UART_Main_enableFIFOs(UART_0_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_0_INST,
                                    DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_clearInterruptStatus(UART_0_INST, RX_ERRORS |
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    DL_UART_Main_enableInterrupt(UART_0_INST, RX_ERRORS |
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void UART_0_INST_IRQHandler(void)
{
    uint32_t errors = DL_UART_Main_getRawInterruptStatus(UART_0_INST, RX_ERRORS);
    if (errors) { s_bad_rx = 1; s_rx_errors++; s_rx_error_bits |= errors; }
    DL_UART_Main_clearInterruptStatus(UART_0_INST, RX_ERRORS |
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    while (!DL_UART_Main_isRXFIFOEmpty(UART_0_INST)) {
        uint8_t byte = DL_UART_Main_receiveData(UART_0_INST);
        uint16_t next = (uint16_t)((s_write + 1U) % RX_SIZE);
        s_rx_bytes++;
        if (next == s_read) { s_bad_rx = 1; s_rx_overflows++; }
        else { s_rx[s_write] = byte; s_write = next; }
    }
}

int Console_TakeCommand(ExperimentCommand *command, uint32_t now_ms)
{
    unsigned budget = RX_SIZE;
    if (s_loss_report_pending &&
        (uint32_t)(now_ms - s_last_loss_report_ms) >= 500U) {
        s_loss_report_pending = 0U;
        s_last_loss_report_ms = now_ms;
        Console_Printf("[ERR] RX_LOST hw_errors=%lu overflow=%lu bits=0x%lx "
                       "framing=%u break=%u overrun=%u resend_after_newline\r\n",
                       (unsigned long)s_rx_errors,
                       (unsigned long)s_rx_overflows,
                       (unsigned long)s_rx_error_bits,
                       !!(s_rx_error_bits & DL_UART_MAIN_INTERRUPT_FRAMING_ERROR),
                       !!(s_rx_error_bits & DL_UART_MAIN_INTERRUPT_BREAK_ERROR),
                       !!(s_rx_error_bits & DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR));
    }
    while (budget--) {
        uint32_t mask = __get_PRIMASK();
        uint8_t byte;
        __disable_irq();
        if (s_bad_rx) {
            /* Never run a command assembled across a lost byte. */
            s_read = s_write;
            s_bad_rx = 0;
            if (!mask) __enable_irq();
            ExperimentParser_Discard(&s_parser);
            s_loss_report_pending = 1U;
            return 0;
        }
        if (s_read == s_write) {
            if (!mask) __enable_irq();
            return 0;
        }
        byte = s_rx[s_read];
        s_read = (uint16_t)((s_read + 1U) % RX_SIZE);
        if (!mask) __enable_irq();
        if (ExperimentParser_Feed(&s_parser, (char)byte, command)) {
            s_rx_lines++;
            return 1;
        }
    }
    return 0;
}

int Console_Printf(const char *format, ...)
{
    char line[768];
    int length, i;
    unsigned free_bytes = (s_tx_read + TX_SIZE - s_tx_write - 1U) % TX_SIZE;
    va_list args;
    va_start(args, format);
    length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length < 0 || length >= (int)sizeof(line) ||
        (unsigned)length > free_bytes) {
        s_tx_drops++;
        return -1;
    }
    /* Reserve/write a whole frame so saturation cannot corrupt CSV framing. */
    for (i = 0; i < length; i++) {
        s_tx[s_tx_write] = line[i];
        s_tx_write = (uint16_t)((s_tx_write + 1U) % TX_SIZE);
    }
    return length;
}

void Console_DrainTx(void)
{
    unsigned budget = 16U;
    while (budget-- && s_tx_read != s_tx_write &&
           !DL_UART_Main_isTXFIFOFull(UART_0_INST)) {
        DL_UART_Main_transmitData(UART_0_INST, (uint8_t)s_tx[s_tx_read]);
        s_tx_read = (uint16_t)((s_tx_read + 1U) % TX_SIZE);
    }
}
uint32_t Console_GetRxErrors(void) { return s_rx_errors; }
uint32_t Console_GetRxOverflows(void) { return s_rx_overflows; }
uint32_t Console_GetRxBytes(void) { return s_rx_bytes; }
uint32_t Console_GetRxLines(void) { return s_rx_lines; }
uint32_t Console_GetTxDrops(void) { return s_tx_drops; }
