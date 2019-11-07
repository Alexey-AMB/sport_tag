#include <stdlib.h>

#include "myBlink.h"
#include "board_key.h"

#ifndef NULL
#define NULL      0
#endif

typedef struct BLINK_PROFILE
{
    bool bContinue;     //��������� ���������� (1) ��� ���������� (0)
    uint8_t * dim;
    uint8_t len;        //���������� ����� � �������
}BLINK_PROFILE;


//��� ��������� ����������� �������� ��������� .len � SendToBlink!

uint8_t * simpleblink(void)
{
    uint8_t simpleblink[6][2] ={{10, 1},    //red
                                {20, 0},
                                {30, 1},
                                {40, 0},
                                {50, 1},
                                {52, 0}
                            };
    uint8_t * buf = malloc(sizeof(simpleblink));
    if(buf != NULL)
    {
        memcpy(buf, simpleblink, sizeof(simpleblink));
        return buf;
    }
    return NULL;
}

uint8_t *  simpleblink_v(void)
{
    uint8_t simpleblink_v[6][2] ={{10, 25},    //sound
                                {20, 0},
                                {30, 25},
                                {40, 0},
                                {50, 0},
                                {52, 0}
                            };
    uint8_t * buf = malloc(sizeof(simpleblink_v));
    if (buf != NULL)
    {
        memcpy(buf, simpleblink_v, sizeof(simpleblink_v));
        return buf;
    }
    return NULL;
}

uint8_t * akk_full(void)
{
    uint8_t akk_full[6][2] ={   {10, 1},    //red
                                {15, 0},
                                {25, 1},
                                {30, 0},
                                {40, 1},
                                {42, 0}
                            };
    uint8_t * buf = malloc(sizeof(akk_full));
    if (buf != NULL)
    {
        memcpy(buf, akk_full, sizeof(akk_full));
        return buf;
    }
    return NULL;
}

uint8_t * akk_medium(void)
{
    uint8_t akk_medium[6][2] ={ {10, 1},    //red
                                {15, 0},
                                {25, 1},
                                {30, 0},
                                {40, 0},
                                {42, 0}
                            };
    uint8_t * buf = malloc(sizeof(akk_medium));
    if (buf != NULL)
    {
        memcpy(buf, akk_medium, sizeof(akk_medium));
        return buf;
    }
    return NULL;
}

uint8_t * akk_low(void)
{
    uint8_t akk_low[6][2] ={    {10, 1},    //red
                                {15, 0},
                                {25, 0},
                                {30, 0},
                                {40, 0},
                                {42, 0}
                            };
    uint8_t * buf = malloc(sizeof(akk_low));
    if (buf != NULL)
    {
        memcpy(buf, akk_low, sizeof(akk_low));
        return buf;
    }
    return NULL;
}

uint8_t * start_st(void)
{
    uint8_t start_st[6][2] ={   {2, 1},     //red
                                {4, 0},
                                {6, 1},
                                {8, 0},
                                {10, 1},
                                {12, 0}
                            };
    uint8_t * buf = malloc(sizeof(start_st));
    if (buf != NULL)
    {
        memcpy(buf, start_st, sizeof(start_st));
        return buf;
    }
    return NULL;
}

uint8_t * normal_st(void)
{
    uint8_t normal_st[10][2] ={ {30, 1},    //red
                                {35, 0},
                                {55, 1},
                                {60, 0},
                                {70, 1},
                                {75, 0},
                                {80, 1},
                                {85, 0},
                                {90, 1},
                                {95, 0}
                            };
    uint8_t * buf = malloc(sizeof(normal_st));
    if (buf != NULL)
    {
        memcpy(buf, normal_st, sizeof(normal_st));
        return buf;
    }
    return NULL;
}

uint8_t * normal_st_v(void)
{
    uint8_t normal_st_v[6][2] ={{30, 25},    //vibro
                                {34, 0},
                                {38, 15},
                                {48, 0},
                                {50, 0},
                                {55, 0}
                            };
    uint8_t * buf = malloc(sizeof(normal_st_v));
    if (buf != NULL)
    {
        memcpy(buf, normal_st_v, sizeof(normal_st_v));
        return buf;
    }
    return NULL;
}

uint8_t * finish_st(void)
{
    uint8_t finish_st[6][2] ={  {30, 1},    //red
                                {35, 0},
                                {55, 1},
                                {60, 0},
                                {70, 1},
                                {72, 0}
                            };
    uint8_t * buf = malloc(sizeof(finish_st));
    if (buf != NULL)
    {
        memcpy(buf, finish_st, sizeof(finish_st));
        return buf;
    }
    return NULL;
}

uint8_t * change_md(void)
{
    uint8_t change_md[4][2] ={  {2, 20},    //vibro
                                {4, 25},
                                {6, 20},
                                {8, 0}
                            };
    uint8_t * buf = malloc(sizeof(change_md));
    if (buf != NULL)
    {
        memcpy(buf, change_md, sizeof(change_md));
        return buf;
    }
    return NULL;
}

uint8_t * run_md_n(void)
{
    uint8_t run_md_n[4][2] =  { {2, 1},    //green
                                {18, 0},
                                {20, 1},
                                {36, 0}
                            };
    uint8_t * buf = malloc(sizeof(run_md_n));
    if (buf != NULL)
    {
        memcpy(buf, run_md_n, sizeof(run_md_n));
        return buf;
    }
    return NULL;
}

uint8_t * run_md_t(void)
{
    uint8_t run_md_t[4][2] =  { {2, 1},    //green
                                {4, 0},
                                {6, 1},
                                {8, 0}
                            };
    uint8_t * buf = malloc(sizeof(run_md_t));
    if (buf != NULL)
    {
        memcpy(buf, run_md_t, sizeof(run_md_t));
        return buf;
    }
    return NULL;
}

uint8_t * conn_md(void)
{
    uint8_t conn_md[4][2] =  {  {50, 1},    //green
                                {52, 1},
                                {102, 1},
                                {104, 1}
                            };
    uint8_t * buf = malloc(sizeof(conn_md));
    if (buf != NULL)
    {
        memcpy(buf, conn_md, sizeof(conn_md));
        return buf;
    }
    return NULL;
}

uint8_t * on_pw(void)
{
    uint8_t on_pw[4][2] =  {    {2, 20},    //vibro
                                {4, 25},
                                {6, 30},
                                {8, 0}
                            };
    uint8_t * buf = malloc(sizeof(on_pw));
    if (buf != NULL)
    {
        memcpy(buf, on_pw, sizeof(on_pw));
        return buf;
    }
    return NULL;
}

uint8_t * off_pw(void)
{
    uint8_t off_pw[4][2] =  {   {2, 35},    //vibro
                                {4, 30},
                                {6, 25},
                                {8, 0}
                            };
    uint8_t * buf = malloc(sizeof(off_pw));
    if (buf != NULL)
    {
        memcpy(buf, off_pw, sizeof(off_pw));
        return buf;
    }
    return NULL;
}

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

void ClearBuff(uint8_t * pb)
{
    if (pb != NULL)
    {
        free(pb);
        pb = NULL;
    }
}

void SendToBlink(BlinkProfiles pr)
{
    switch(pr)
    {
    case PRF_SIMPLEBLINK:
        redBP.bContinue = false;
        ClearBuff(redBP.dim);
        redBP.dim = simpleblink();
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(redBP.dim + (redBP.len - 1) * 2);
        ignoreR = false;

        vibroBP.bContinue = false;
        ClearBuff(vibroBP.dim);
        vibroBP.dim = simpleblink_v();
        vibroBP.len = 6;

        iV = 0;
        cntV = 0;
        maxV = *(vibroBP.dim + (vibroBP.len - 1) * 2);
        ignoreV = false;
        break;
    case PRF_AKK_FULL:
        redBP.bContinue = false;
        ClearBuff(redBP.dim);
        redBP.dim = akk_full();
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(redBP.dim + (redBP.len - 1) * 2);
        ignoreR = false;
        break;
    case PRF_AKK_MEDIUM:
        redBP.bContinue = false;
        ClearBuff(redBP.dim);
        redBP.dim = akk_medium();
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(redBP.dim + (redBP.len - 1) * 2);
        ignoreR = false;
        break;
    case PRF_AKK_LOW:
        redBP.bContinue = false;
        ClearBuff(redBP.dim);
        redBP.dim = akk_low();
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(redBP.dim + (redBP.len - 1) * 2);
        ignoreR = false;
        break;
    case PRF_START_STATION:
        redBP.bContinue = false;
        ClearBuff(redBP.dim);
        redBP.dim = start_st();
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(redBP.dim + (redBP.len - 1) * 2);
        ignoreR = false;
        break;
    case PRF_NORMAL_STATION:
        redBP.bContinue = false;
        ClearBuff(redBP.dim);
        redBP.dim = normal_st();
        redBP.len = 10;

        iR = 0;
        cntR = 0;
        maxR = *(redBP.dim + (redBP.len - 1) * 2);
        ignoreR = false;

        vibroBP.bContinue = false;
        ClearBuff(vibroBP.dim);
        vibroBP.dim = normal_st_v();
        vibroBP.len = 6;

        iV = 0;
        cntV = 0;
        maxV = *(vibroBP.dim + (vibroBP.len - 1) * 2);
        ignoreV = false;
        break;
    case PRF_FINISH_STATION:
        redBP.bContinue = false;
        ClearBuff(redBP.dim);
        redBP.dim = finish_st();
        redBP.len = 6;

        iR = 0;
        cntR = 0;
        maxR = *(redBP.dim + (redBP.len - 1) * 2);
        ignoreR = false;

        vibroBP.bContinue = false;
        ClearBuff(vibroBP.dim);
        vibroBP.dim = change_md();
        vibroBP.len = 4;

        iV = 0;
        cntV = 0;
        maxV = *(vibroBP.dim + (vibroBP.len - 1) * 2);
        ignoreV = false;
        break;
    case PRF_CHANGE_MODE:
        vibroBP.bContinue = false;
        ClearBuff(vibroBP.dim);
        vibroBP.dim = change_md();
        vibroBP.len = 4;

        iV = 0;
        cntV = 0;
        maxV = *(vibroBP.dim + (vibroBP.len - 1) * 2);
        ignoreV = false;
        break;
    case PRF_MODE_RUN_NORM:
        greenBP.bContinue = true;
        ClearBuff(greenBP.dim);
        greenBP.dim = run_md_n();
        greenBP.len = 4;

        iG = 0;
        cntG = 0;
        maxG = *(greenBP.dim + (greenBP.len - 1) * 2);
        ignoreG = false;
        break;
    case PRF_MODE_RUN_TURB:
        greenBP.bContinue = true;
        ClearBuff(greenBP.dim);
        greenBP.dim = run_md_t();
        greenBP.len = 4;

        iG = 0;
        cntG = 0;
        maxG = *(greenBP.dim + (greenBP.len - 1) * 2);
        ignoreG = false;
        break;
    case PRF_MODE_CONNECT:
        greenBP.bContinue = true;
        ClearBuff(greenBP.dim);
        greenBP.dim = conn_md();
        greenBP.len = 4;

        iG = 0;
        cntG = 0;
        maxG = *(greenBP.dim + (greenBP.len - 1) * 2);
        ignoreG = false;
        break;
    case PRF_POWER_ON:
        vibroBP.bContinue = false;
        ClearBuff(vibroBP.dim);
        vibroBP.dim = on_pw();
        vibroBP.len = 4;

        iV = 0;
        cntV = 0;
        maxV = *(vibroBP.dim + (vibroBP.len - 1) * 2);
        ignoreV = false;
        break;
    case PRF_POWER_OFF:
        vibroBP.bContinue = false;
        ClearBuff(vibroBP.dim);
        vibroBP.dim = off_pw();
        vibroBP.len = 4;

        iV = 0;
        cntV = 0;
        maxV = *(vibroBP.dim + (vibroBP.len - 1) * 2);
        ignoreV = false;
        break;
    case PRF_RED_STOP:
        ignoreR = true;
        outR = 0;
        ClearBuff(redBP.dim);
        break;
    case PRF_GREEN_STOP:
        ignoreG = true;
        outG = 0;
        ClearBuff(greenBP.dim);
        break;
    case PRF_VIBRO_STOP:
        ignoreV = true;
        outV = 0;
        ClearBuff(vibroBP.dim);
        break;
    default:
        break;
    }
}

void InitBlink(void)
{
    redBP.dim = NULL;
    SendToBlink(PRF_RED_STOP);
    greenBP.dim = NULL;
    SendToBlink(PRF_GREEN_STOP);
    vibroBP.dim = NULL;
    SendToBlink(PRF_VIBRO_STOP);
}

//*(*(arr + i) + j)

void PerformBlink(void)
{
    if (!ignoreR)
    {
        valR = *(redBP.dim + iR * 2);
        if (cntR <= valR)
        {
            outR = *(redBP.dim + iR * 2 + 1);
        }
        else
        {
            if (cntR >= maxR)
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

    if (!ignoreG)
    {
        valG = *(greenBP.dim + iG * 2);
        if (cntG <= valG)
        {
            outG = *(greenBP.dim + iG * 2 + 1);
        }
        else
        {
            if (cntG >= maxG)
            {
                if (greenBP.bContinue)
                {
                    cntG = 0;
                    iG = 0;
                }
                else ignoreG = true;
            }
            else iG++;
        }
        cntG++;
        Board_setLed1_my(outG);
    }

    if (!ignoreV)
    {
        valV = *(vibroBP.dim + iV * 2);
        if (cntV <= valV)
        {
            outV = *(vibroBP.dim + iV * 2 + 1);
        }
        else
        {
            if (cntV >= maxV)
            {
                if (vibroBP.bContinue)
                {
                    cntV = 0;
                    iV = 0;
                }
                else ignoreV = true;
            }
            else iV++;
        }
        cntV++;
        //Board_setVibro(outV);
        Board_setSound(outV);
    }
}
