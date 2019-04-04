/******************************************************************************

 @file       board_key.c

 @brief This file contains the interface to the SRF06EB Key Service.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************

 Copyright (c) 2014-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_02_30_00_28
 Release Date: 2018-10-15 15:51:38
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdbool.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
//#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/Power.h>

#include <ti/drivers/pin/PINCC26XX.h>

#ifdef USE_ICALL
#include <icall.h>
#endif

#include <inc/hw_ints.h>

#include "util.h"
#include "board_key.h"
#include "board.h"

/*********************************************************************
 * TYPEDEFS
 */
#define VIBRO_N         Board_SPI_MASTER_READY
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Board_keyChangeHandler_short(UArg a0);
static void Board_keyChangeHandler_long(UArg a0);
static void Board_keyChangeHandler_verylong(UArg a0);
static void Board_keyCallback(PIN_Handle hPin, PIN_Id pinId);

/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Value of keys Pressed
static uint8_t keysPressed;

// Key debounce clock
static Clock_Struct keyChangeClock_short;
static Clock_Struct keyChangeClock_long;
static Clock_Struct keyChangeClock_verylong;

// Pointer to application callback
keysPressedCB_t appKeyChangeHandler = NULL;

// Memory for the GPIO module to construct a Hwi
Hwi_Struct callbackHwiKeys;

// PIN configuration structure to set all KEY pins as inputs with pullups enabled
PIN_Config keyPinsCfg[] =
{
#if defined(CC26X2R1_LAUNCHXL) || defined(CC13X2R1_LAUNCHXL) || defined(CC13X2P1_LAUNCHXL)
 Board_PIN_BUTTON0   | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
 Board_PIN_BUTTON1   | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
#elif defined(CC2650_LAUNCHXL) || defined(CC2640R2_LAUNCHXL) || defined(CC1350_LAUNCHXL)
 Board_BTN1          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
 Board_BTN2          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
#elif defined(CC2650DK_7ID)  || defined(CC1350DK_7XD)
 Board_KEY_SELECT    | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
 Board_KEY_UP        | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
 Board_KEY_DOWN      | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
 Board_KEY_LEFT      | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
 Board_KEY_RIGHT     | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
#endif
 PIN_TERMINATE
};

PIN_State  keyPins;
PIN_Handle hKeyPins;


//my_test
PIN_Handle ledPinHandle;
PIN_State ledPinState;

PIN_Config ledPinTable[] = {
                            Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                            Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                            Board_DIO21    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                            Board_SPI_MASTER_READY | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                            PIN_TERMINATE
};

/* Wake-up Button pin table */
PIN_Config ButtonTableWakeUp[] = {
                                  Board_PIN_BUTTON0     | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE, PIN_TERMINATE                                 /* Terminate list */
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      Board_initKeys
 *
 * @brief   Enable interrupts for keys on GPIOs.
 *
 * @param   appKeyCB - application key pressed callback
 *
 * @return  none
 */
void Board_initKeys(keysPressedCB_t appKeyCB)
{
    // Initialize KEY pins. Enable int after callback registered
    hKeyPins = PIN_open(&keyPins, keyPinsCfg);
    PIN_registerIntCb(hKeyPins, Board_keyCallback);

#if defined(CC26X2R1_LAUNCHXL) || defined(CC13X2R1_LAUNCHXL) || defined(CC13X2P1_LAUNCHXL)
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_PIN_BUTTON0 | PIN_IRQ_NEGEDGE);
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_PIN_BUTTON1 | PIN_IRQ_NEGEDGE);
#elif defined(CC2650_LAUNCHXL) || defined(CC2640R2_LAUNCHXL) || defined(CC1350_LAUNCHXL)
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_BTN1        | PIN_IRQ_NEGEDGE);
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_BTN2        | PIN_IRQ_NEGEDGE);
#elif defined(CC2650DK_7ID)  || defined(CC1350DK_7XD)
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_KEY_SELECT  | PIN_IRQ_NEGEDGE);
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_KEY_UP      | PIN_IRQ_NEGEDGE);
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_KEY_DOWN    | PIN_IRQ_NEGEDGE);
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_KEY_LEFT    | PIN_IRQ_NEGEDGE);
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, Board_KEY_RIGHT   | PIN_IRQ_NEGEDGE);
#endif

#ifdef POWER_SAVING
    //Enable wakeup
#if defined(CC26X2R1_LAUNCHXL) || defined(CC13X2R1_LAUNCHXL) || defined(CC13X2P1_LAUNCHXL)
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_PIN_BUTTON0 | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_PIN_BUTTON1 | PINCC26XX_WAKEUP_NEGEDGE);
#elif defined(CC2650_LAUNCHXL) || defined(CC2640R2_LAUNCHXL) || defined(CC1350_LAUNCHXL)
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_BTN1        | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_BTN2        | PINCC26XX_WAKEUP_NEGEDGE);
#elif defined(CC2650DK_7ID)  || defined(CC1350DK_7XD)
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_KEY_SELECT | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_KEY_UP | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_KEY_DOWN | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_KEY_LEFT | PINCC26XX_WAKEUP_NEGEDGE);
    PIN_setConfig(hKeyPins, PINCC26XX_BM_WAKEUP, Board_KEY_RIGHT | PINCC26XX_WAKEUP_NEGEDGE);
#endif
#endif //POWER_SAVING

    // Setup keycallback for keys
    Util_constructClock(&keyChangeClock_short, Board_keyChangeHandler_short, KEY_DEBOUNCE_TIMEOUT_SHORT, 0, false, 0);
    Util_constructClock(&keyChangeClock_long, Board_keyChangeHandler_long, KEY_DEBOUNCE_TIMEOUT_LONG, 0, false, 0);
    Util_constructClock(&keyChangeClock_verylong, Board_keyChangeHandler_verylong, KEY_DEBOUNCE_TIMEOUT_VERYLONG, 0, false, 0);

    // Set the application callback
    appKeyChangeHandler = appKeyCB;
}
void Board_initLeds(void)
{
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
}
void Board_setLed0_my(uint8_t iVal)
{
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, iVal);
}
void Board_setLed1_my(uint8_t iVal)
{
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, iVal);
}
//void Board_setCS(uint8_t iVal)
//{
//    PIN_setOutputValue(ledPinHandle, Board_DIO21, iVal);
//}
void Board_setVibro(uint8_t iVal)   //Вывод инверсный. Инвертирование здесь!
{
    uint8_t nval = iVal ^ 1;
    PIN_setOutputValue(ledPinHandle, Board_SPI_MASTER_READY, nval);
}

void Board_Power_down(void)
{
    /* Configure DIO for wake up from shutdown */
    PINCC26XX_setWakeup(ButtonTableWakeUp);

    /* Go to shutdown */
    Power_shutdown(0, 0);

    /* Should never get here, since shutdown will reset. */
    while (1);
}

uint8_t Board_getLed_my(void)
{
    return PIN_getOutputValue(Board_PIN_LED0);
}

void Task_sleepMS(uint32_t period)
{
    Task_sleep(period * 1000 / Clock_tickPeriod);
}

/*********************************************************************
 * @fn      Board_keyCallback
 *
 * @brief   Interrupt handler for Keys
 *
 * @param   none
 *
 * @return  none
 */
static void Board_keyCallback(PIN_Handle hPin, PIN_Id pinId)
{
    Util_startClock(&keyChangeClock_short);
    Util_startClock(&keyChangeClock_long);
    Util_startClock(&keyChangeClock_verylong);
}

/*********************************************************************
 * @fn      Board_keyChangeHandler
 *
 * @brief   Handler for key change
 *
 * @param   UArg a0 - ignored
 *
 * @return  none
 */
static void Board_keyChangeHandler_short(UArg a0)
{
    keysPressed = 0;
    if (PIN_getInputValue(Board_BTN1) == 0 )
    {
        keysPressed |= KEY_LEFT;
    }

    if (PIN_getInputValue(Board_BTN2) == 0 )
    {
        keysPressed |= KEY_RIGHT;
    }

    if(keysPressed != 0)
    {
        if (appKeyChangeHandler != NULL)
        {
            // Notify the application
            (*appKeyChangeHandler)(keysPressed);
        }
    }
    else
    {
        Util_stopClock(&keyChangeClock_long);
        Util_stopClock(&keyChangeClock_verylong);
    }
}

static void Board_keyChangeHandler_long(UArg a0)
{
    keysPressed = 0;
    if (PIN_getInputValue(Board_BTN1) == 0 )
    {
        keysPressed |= KEY_LEFT_LONG;
    }

    if (PIN_getInputValue(Board_BTN2) == 0 )
    {
        keysPressed |= KEY_RIGHT_LONG;
    }

    if(keysPressed != 0)
    {
        if (appKeyChangeHandler != NULL)
        {
            // Notify the application
            (*appKeyChangeHandler)(keysPressed);
        }
    }
    else
    {
        Util_stopClock(&keyChangeClock_verylong);
    }
}

static void Board_keyChangeHandler_verylong(UArg a0)
{
    keysPressed = 0;
    if (PIN_getInputValue(Board_BTN1) == 0 )
    {
        keysPressed |= KEY_LEFT_VERYLONG;
    }

    if (PIN_getInputValue(Board_BTN2) == 0 )
    {
        keysPressed |= KEY_RIGHT_VERYLONG;
    }

    if(keysPressed != 0)
    {
        if (appKeyChangeHandler != NULL)
        {
            // Notify the application
            (*appKeyChangeHandler)(keysPressed);
        }
    }
}
/*********************************************************************
 *********************************************************************/
