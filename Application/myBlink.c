#include "myBlink.h"
#include "board_key.h"


typedef struct BLINK_PROFILE
{
    bool bContinue;     //повторять циклически (1) или однократно (0)
    uint8_t (*dim)[2];
    uint8_t len;        //количество строк в массиве
}BLINK_PROFILE;


//при изменении размерности массивов исправить .len в SendToBlink!
uint8_t simpleblink[6][2] ={{10, 1},    //red
                            {20, 0},
                            {30, 1},
                            {40, 0},
                            {50, 1},
                            {52, 0}
                        };
uint8_t akk_full[6][2] ={   {10, 1},    //red
                            {15, 0},
                            {25, 1},
                            {30, 0},
                            {40, 1},
                            {42, 0}
                        };
uint8_t akk_medium[6][2] ={ {10, 1},    //red
                            {15, 0},
                            {25, 1},
                            {30, 0},
                            {40, 0},
                            {42, 0}
                        };
uint8_t akk_low[6][2] ={    {10, 1},    //red
                            {15, 0},
                            {25, 0},
                            {30, 0},
                            {40, 0},
                            {42, 0}
                        };
uint8_t start_st[6][2] ={   {2, 1},     //red
                            {4, 0},
                            {6, 1},
                            {8, 0},
                            {10, 1},
                            {12, 0}
                        };
uint8_t normal_st[6][2] ={  {30, 1},    //red
                            {35, 0},
                            {55, 1},
                            {60, 0},
                            {70, 1},
                            {72, 0}
                        };
uint8_t finish_st[6][2] ={  {30, 1},    //red
                            {35, 0},
                            {55, 1},
                            {60, 0},
                            {70, 1},
                            {72, 0}
                        };
uint8_t change_md[4][2] ={  {5, 1},    //vibro
                            {15, 0},
                            {20, 1},
                            {30, 0}
                        };
uint8_t run_md[4][2] =  {   {5, 1},    //green
                            {10, 0},
                            {15, 1},
                            {20, 0}
                        };
uint8_t conn_md[4][2] =  {  {50, 1},    //green
                            {52, 0},
                            {102, 1},
                            {152, 0}
                        };
uint8_t on_pw[4][2] =  {    {5, 1},    //vibro
                            {10, 0},
                            {15, 0},
                            {20, 0}
                        };
uint8_t off_pw[4][2] =  {   {10, 1},    //vibro
                            {15, 0},
                            {25, 1},
                            {30, 0}
                        };

BLINK_PROFILE redBP;
BLINK_PROFILE greenBP;
BLINK_PROFILE vibroBP;


uint8_t maxR = 0, outR;
uint8_t maxG = 0, outG;
uint8_t maxV = 0, outV;

uint8_t iR, valR;
uint8_t iG, valG;
uint8_t iV, valV;

uint8_t cntR, cntG, cntV;

bool ignoreR = true;
bool ignoreG = true;
bool ignoreV = true;

void SendToBlink(BlinkProfiles pr)
{
    switch(pr)
    {
    case PRF_SIMPLEBLINK:
        redBP.bContinue = false;
        redBP.dim = simpleblink;
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(*(redBP.dim + redBP.len - 1));
        ignoreR = false;
        break;
    case PRF_AKK_FULL:
        redBP.bContinue = false;
        redBP.dim = akk_full;
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(*(redBP.dim + redBP.len - 1));
        ignoreR = false;
        break;
    case PRF_AKK_MEDIUM:
        redBP.bContinue = false;
        redBP.dim = akk_medium;
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(*(redBP.dim + redBP.len - 1));
        ignoreR = false;
        break;
    case PRF_AKK_LOW:
        redBP.bContinue = false;
        redBP.dim = akk_low;
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(*(redBP.dim + redBP.len - 1));
        ignoreR = false;
        break;
    case PRF_START_STATION:
        redBP.bContinue = true;
        redBP.dim = start_st;
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(*(redBP.dim + redBP.len - 1));
        ignoreR = false;
        break;
    case PRF_NORMAL_STATION:
        redBP.bContinue = false;
        redBP.dim = normal_st;
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(*(redBP.dim + redBP.len - 1));
        ignoreR = false;
        break;
    case PRF_FINISH_STATION:
        redBP.bContinue = false;
        redBP.dim = finish_st;
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(*(redBP.dim + redBP.len - 1));
        ignoreR = false;
        break;
    case PRF_CHANGE_MODE:
        vibroBP.bContinue = false;
        vibroBP.dim = change_md;
        vibroBP.len = 4;

        iV = 0;
        cntV = 0;
        maxV = *(*(vibroBP.dim + vibroBP.len - 1));
        ignoreV = false;
        break;
    case PRF_MODE_RUN:
        greenBP.bContinue = true;
        greenBP.dim = run_md;
        greenBP.len = 4;

        iG = 0;
        cntG = 0;
        maxG = *(*(greenBP.dim + greenBP.len - 1));
        ignoreG = false;
        break;
    case PRF_MODE_CONNECT:
        greenBP.bContinue = true;
        greenBP.dim = conn_md;
        greenBP.len = 4;

        iG = 0;
        cntG = 0;
        maxG = *(*(greenBP.dim + greenBP.len - 1));
        ignoreG = false;
        break;
    case PRF_POWER_ON:
        vibroBP.bContinue = false;
        vibroBP.dim = on_pw;
        vibroBP.len = 4;

        iV = 0;
        cntV = 0;
        maxV = *(*(vibroBP.dim + vibroBP.len -1));
        ignoreV = false;
        break;
    case PRF_POWER_OFF:
        vibroBP.bContinue = false;
        vibroBP.dim = off_pw;
        vibroBP.len = 4;

        iV = 0;
        cntV = 0;
        maxV = *(*(vibroBP.dim + vibroBP.len -1));
        ignoreV = false;
        break;
    case PRF_RED_STOP:
        ignoreR = true;
        outR = 0;
        break;
    case PRF_GREEN_STOP:
        ignoreG = true;
        outG = 0;
        break;
    case PRF_VIBRO_STOP:
        ignoreV = true;
        outV = 0;
        break;
    default:
        break;
    }
}

void InitBlink(void)
{
    SendToBlink(PRF_RED_STOP);
    SendToBlink(PRF_GREEN_STOP);
    SendToBlink(PRF_VIBRO_STOP);
}


//*(*(arr + i) + j)

void PerformBlink(void)
{
    if (!ignoreR)
    {
        valR = *(*(redBP.dim + iR));
        if (cntR < valR)
        {
            outR = *(*(redBP.dim + iR) + 1);
        }
        else
        {
            if (cntR == maxR)
            {
                if (redBP.bContinue)
                {
                    cntR = 0;
                    iR = 0;
                }
                else ignoreR = true;
            }
            else iR++;
        }
        cntR++;
        Board_setLed0_my(outR);
    }
}
