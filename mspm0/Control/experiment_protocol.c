#include "experiment_protocol.h"
#include <stdlib.h>
#include <string.h>

static uint8_t ParseSequence(const char *text, uint32_t *sequence)
{
    char *end;
    unsigned long value;
    if (text == 0 || *text < '0' || *text > '9') return 0U;
    value = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || value > 0xFFFFFFFFUL) return 0U;
    *sequence = (uint32_t)value;
    return 1U;
}

static uint8_t MatchSequenced(const char *line, const char *name,
                              uint32_t *sequence)
{
    size_t length = strlen(name);
    if (strcmp(line, name) == 0) {
        *sequence = 0U;
        return 1U;
    }
    return strncmp(line, name, length) == 0 && line[length] == ',' &&
           ParseSequence(line + length + 1U, sequence);
}

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
    ExperimentCommand command = { EXP_INVALID, 0.0f, 0U };
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
    else if (strcmp(line, "JOG,+1") == 0) command.type = EXP_JOG_PLUS;
    else if (strcmp(line, "JOG,-1") == 0) command.type = EXP_JOG_MINUS;
    else if (strcmp(line, "CAL,BALANCE") == 0) command.type = EXP_BALANCE_CAL;
    else if (strcmp(line, "BALANCE,SHOW") == 0) command.type = EXP_BALANCE_SHOW;
    else if (strcmp(line, "BALANCE,ZERO") == 0) command.type = EXP_BALANCE_ZERO;
    else if (strncmp(line, "ANGLE_REL,", 10) == 0) {
        /* Minimal remote: target relative to balance zero, +/-15 deg only. */
        angle = strtof(line + 10, &end);
        if (end != line + 10 && *end == '\0' &&
            angle >= -15.0f && angle <= 15.0f) {
            command.type = EXP_ANGLE_REL;
            command.angle_deg = angle;
        }
    }
    else if (strcmp(line, "BREAKAWAY,POS") == 0) command.type = EXP_BREAKAWAY_POS;
    else if (strcmp(line, "BREAKAWAY,NEG") == 0) command.type = EXP_BREAKAWAY_NEG;
    else if (strcmp(line, "BREAKAWAY,STATUS") == 0) command.type = EXP_BREAKAWAY_STATUS;
    else if (strcmp(line, "FAULT,CLEAR") == 0) command.type = EXP_FAULT_CLEAR;
    else if (strcmp(line, "SINE,START") == 0) command.type = EXP_SINE_START;
    else if (strcmp(line, "SINE,STATUS") == 0) command.type = EXP_SINE_STATUS;
    else if (MatchSequenced(line, "MANUAL,START", &command.sequence))
        command.type = EXP_MANUAL_START;
    else if (MatchSequenced(line, "MANUAL,HEARTBEAT", &command.sequence))
        command.type = EXP_MANUAL_HEARTBEAT;
    else if (MatchSequenced(line, "MANUAL,STOP", &command.sequence))
        command.type = EXP_MANUAL_STOP;
    else if (strncmp(line, "MANUAL,ANGLE,", 13) == 0) {
        angle = strtof(line + 13, &end);
        if (end != line + 13 &&
            (*end == '\0' || (*end == ',' && ParseSequence(
                end + 1, &command.sequence))) &&
            angle >= -15.0f && angle <= 15.0f) {
            command.type = EXP_MANUAL_ANGLE;
            command.angle_deg = angle;
        }
    }
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
            command->sequence = 0U;
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
