#include "../mspm0/Control/experiment_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static ExperimentCommand Feed(ExperimentParser *p, const char *text, int count)
{
    ExperimentCommand result = { EXP_NONE, 0 }, next;
    int seen = 0;
    while (*text) {
        if (ExperimentParser_Feed(p, *text++, &next)) {
            result = next;
            seen++;
        }
    }
    assert(seen == count);
    return result;
}

int main(void)
{
    ExperimentParser p;
    ExperimentCommand c;
    char long_line[160];
    ExperimentParser_Reset(&p);
    Feed(&p, "CAL,CEN", 0);
    assert(Feed(&p, "TER\r\n", 1).type == EXP_CENTER);
    assert(Feed(&p, "cal,left\n", 1).type == EXP_LEFT);
    assert(Feed(&p, "CAL,RIGHT\r", 1).type == EXP_RIGHT);
    assert(Feed(&p, "\nCAL,RESET\n", 1).type == EXP_RESET);
    assert(Feed(&p, "\r\n\n", 0).type == EXP_NONE);
    assert(Feed(&p, "JOG,+1\r\n", 1).type == EXP_JOG_PLUS);
    assert(Feed(&p, "jog,-1\n", 1).type == EXP_JOG_MINUS);
    assert(Feed(&p, "CAL,BALANCE\n", 1).type == EXP_BALANCE_CAL);
    assert(Feed(&p, "BALANCE,SHOW\n", 1).type == EXP_BALANCE_SHOW);
    Feed(&p, "balance,ze", 0);
    assert(Feed(&p, "ro\r\n", 1).type == EXP_BALANCE_ZERO);
    assert(Feed(&p, "BALANCE,ZEROjunk\n", 1).type == EXP_INVALID);
    assert(Feed(&p, "JOG,+10\n", 1).type == EXP_INVALID);
    assert(Feed(&p, "JOG,-1junk\n", 1).type == EXP_INVALID);
    c = Feed(&p, "ANGLE,31.25\n", 1);
    assert(c.type == EXP_ANGLE && c.angle_deg == 31.25f);
    assert(Feed(&p, "ANGLE,24\n", 1).type == EXP_ANGLE);
    assert(Feed(&p, "ANGLE,40\n", 1).type == EXP_ANGLE);
    const char *bad[] = {"ANGLE,23.9\n", "ANGLE,40.1\n", "ANGLE,76.3\n",
                        "ANGLE,nan\n", "ANGLE,inf\n", "ANGLE,1e999\n",
                        "ANGLE,\n", "ANGLE,31junk\n", "ANGLE,31,32\n"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i)
        assert(Feed(&p, bad[i], 1).type == EXP_INVALID);
    memset(long_line, 'a', sizeof(long_line));
    long_line[sizeof(long_line)-2] = '\n';
    long_line[sizeof(long_line)-1] = 0;
    assert(Feed(&p, long_line, 1).type == EXP_INVALID);
    assert(Feed(&p, "STOP\n", 1).type == EXP_STOP);
    Feed(&p, "ANGLE,3", 0);
    ExperimentParser_Discard(&p);
    assert(Feed(&p, "1\n", 1).type == EXP_INVALID);
    assert(Feed(&p, "STATUS\nSTREAM,ON\nSTREAM,OFF\nHELP\nSTOP\n", 5).type == EXP_STOP);
    Feed(&p, "ANGLE,", 0);
    assert(!ExperimentParser_Feed(&p, '\0', &c));
    assert(Feed(&p, "31\n", 1).type == EXP_INVALID);
    puts("PASS: split lines, CRLF, calibration, bounds, malformed input, overflow, resync");
    return 0;
}
