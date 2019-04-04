#ifndef MYBLINK_H
#define MYBLINK_H

#include <stdbool.h>
#include <stdint.h>

typedef enum BlinkProfiles
{
    PRF_RED_STOP,
    PRF_GREEN_STOP,
    PRF_VIBRO_STOP,

    PRF_SIMPLEBLINK,

    PRF_AKK_FULL,
    PRF_AKK_MEDIUM,
    PRF_AKK_LOW,

    PRF_START_STATION,
    PRF_NORMAL_STATION,
    PRF_FINISH_STATION,

    PRF_CHANGE_MODE,
    PRF_MODE_RUN,
    PRF_MODE_CONNECT,

    PRF_POWER_ON,
    PRF_POWER_OFF
}BlinkProfiles;

void PerformBlink(void);

void InitBlink(void);

void SendToBlink(BlinkProfiles pr);

#endif
