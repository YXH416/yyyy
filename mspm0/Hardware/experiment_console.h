#ifndef EXPERIMENT_CONSOLE_H
#define EXPERIMENT_CONSOLE_H
#include <stdint.h>
#include "experiment_protocol.h"
void Console_Init(void);
int Console_Printf(const char *format, ...);
void Console_DrainTx(void);
int Console_TakeCommand(ExperimentCommand *command);
uint32_t Console_GetRxErrors(void);
uint32_t Console_GetTxDrops(void);
#endif
