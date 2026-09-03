#include "experiment_protocol.h"
#include <stdlib.h>
#include <string.h>

void ExperimentParser_Reset(ExperimentParser *parser)
{
    parser->length = 0;
    parser->discard = 0;
}

void ExperimentParser_Discard(ExperimentParser *parser)
{
    parser->length = 0;
    parser->discard = 1;
}

static ExperimentCommand Parse(char *line)
{
    ExperimentCommand command = { EXP_INVALID, 0.0f };
    char *end;
    float angle;
    if (strcmp(line, "CAL,CENTER") == 0) command.type = EXP_CENTER;
    else if (strcmp(line, "CAL,LEFT") == 0) command.type = EXP_LEFT;
    else if (strcmp(line, "CAL,RIGHT") == 0) command.type = EXP_RIGHT;
    else if (strcmp(line, "CAL,RESET") == 0) command.type = EXP_RESET;
    else if (strcmp(line, "STOP") == 0) command.type = EXP_STOP;
    else if (strcmp(line, "STATUS") == 0) command.type = EXP_STATUS;
    else if (strcmp(line, "STREAM,ON") == 0) command.type = EXP_STREAM_ON;
    else if (strcmp(line, "STREAM,OFF") == 0) command.type = EXP_STREAM_OFF;
    else if (strcmp(line, "HELP") == 0) command.type = EXP_HELP;
    else if (strncmp(line, "ANGLE,", 6) == 0) {
        /* Consume the entire numeric token. NaN/Inf/overflow cannot pass. */
        angle = strtof(line + 6, &end);
        if (end != line + 6 && *end == '\0' &&
            angle >= 24.0f && angle <= 40.0f) {
            command.type = EXP_ANGLE;
            command.angle_deg = angle;
        }
    }
    return command;
}

int ExperimentParser_Feed(ExperimentParser *parser, char byte,
                          ExperimentCommand *command)
{
    if (byte == '\r' || byte == '\n') {
        if (parser->discard) {
            ExperimentParser_Reset(parser);
            command->type = EXP_INVALID;
            command->angle_deg = 0;
            return 1;
        }
        if (parser->length == 0) return 0;
        parser->line[parser->length] = '\0';
        *command = Parse(parser->line);
        parser->length = 0;
        return 1;
    }
    if (parser->discard) return 0;
    if (byte < 32 || byte > 126 ||
        parser->length >= sizeof(parser->line) - 1U) {
        ExperimentParser_Discard(parser);
        return 0;
    }
    if (byte >= 'a' && byte <= 'z') byte = (char)(byte - 'a' + 'A');
    parser->line[parser->length++] = byte;
    return 0;
}
