/******************************************************************************

 @file       board_key.h

 @brief This file contains the SRF06EB Key Service definitions and prototypes
        prototypes.

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

#ifndef BOARD_KEY_H
#define BOARD_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES *
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
#define KEY_LEFT                 0x0001
#define KEY_RIGHT                0x0002
#define KEY_LEFT_LONG            0x0004
#define KEY_RIGHT_LONG           0x0008
#define KEY_LEFT_VERYLONG        0x0010
#define KEY_RIGHT_VERYLONG       0x0020

// Debounce timeout in milliseconds
#define KEY_DEBOUNCE_TIMEOUT_SHORT      100
#define KEY_DEBOUNCE_TIMEOUT_LONG       2000
#define KEY_DEBOUNCE_TIMEOUT_VERYLONG   5000

/*********************************************************************
 * TYPEDEFS
 */
typedef void (*keysPressedCB_t)(uint8_t keysPressed);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
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
void Board_initKeys(keysPressedCB_t appKeyCB);
//
//my_test
void Board_initLeds(void);

bool Board_initSound(void);

void Board_setLed0_my(uint8_t iVal);
void Board_setLed1_my(uint8_t iVal);
void Board_setCS(uint8_t iVal);
//void Board_setVibro(uint8_t iVal);
void Board_setSound(uint8_t iVal);
uint8_t Board_getLed_my(void);
void Task_sleepMS(uint32_t period); //task sleep in miliseconds
void Board_Power_down(void);
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BOARD_KEY_H */
