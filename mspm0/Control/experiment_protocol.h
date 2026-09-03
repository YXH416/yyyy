#ifndef EXPERIMENT_PROTOCOL_H
#define EXPERIMENT_PROTOCOL_H
#include <stdint.h>

typedef enum {
    EXP_NONE = 0, EXP_CENTER, EXP_LEFT, EXP_RIGHT, EXP_RESET,
    EXP_ANGLE, EXP_STOP, EXP_STATUS, EXP_STREAM_ON, EXP_STREAM_OFF,
    EXP_HELP, EXP_INVALID
} ExperimentCommandType;

typedef struct {
    ExperimentCommandType type;
    float angle_deg;
} ExperimentCommand;

/* A malformed/oversized line is discarded in full through its terminator. */
typedef struct {
    char line[64];
    uint8_t length;
    uint8_t discard;
} ExperimentParser;

void ExperimentParser_Reset(ExperimentParser *parser);
void ExperimentParser_Discard(ExperimentParser *parser);
/* Returns 1 on a complete nonempty line, accepting LF, CR or CRLF. */
int ExperimentParser_Feed(ExperimentParser *parser, char byte,
                          ExperimentCommand *command);
#endif
