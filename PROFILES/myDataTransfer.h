/**********************************************************************************************
 * Filename:       myDataTransfer.h
 *
 * Description:    This file contains the myDataTransfer service definitions and
 *                 prototypes.
 *
 * Copyright (c) 2015-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


#ifndef _MYDATATRANSFER_H_
#define _MYDATATRANSFER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "simple_gatt_profile.h"
/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define MYDATATRANSFER_SERV_UUID 0xBA43

//  Characteristic defines
#define MYDATATRANSFER_MYBUFIN1_ID   6
#define MYDATATRANSFER_MYBUFIN1_UUID 0xBA44
#define MYDATATRANSFER_MYBUFIN1_LEN  23

//  Characteristic defines
#define MYDATATRANSFER_MYBUFOUT1_ID   7
#define MYDATATRANSFER_MYBUFOUT1_UUID 0xBA45
#define MYDATATRANSFER_MYBUFOUT1_LEN  23

//  Characteristic defines
#define MYDATATRANSFER_MYBUFIN2_ID   8
#define MYDATATRANSFER_MYBUFIN2_UUID 0xBA46
#define MYDATATRANSFER_MYBUFIN2_LEN  23

//  Characteristic defines
#define MYDATATRANSFER_MYBUFOUT2_ID   9
#define MYDATATRANSFER_MYBUFOUT2_UUID 0xBA47
#define MYDATATRANSFER_MYBUFOUT2_LEN  23

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*myDataTransferChange_t)(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

typedef struct
{
  myDataTransferChange_t        pfnChangeCb;  // Called when characteristic value changes
  myDataTransferChange_t        pfnCfgChangeCb;
} myDataTransferCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * MyDataTransfer_AddService- Initializes the MyDataTransfer service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t MyDataTransfer_AddService( uint8_t rspTaskId);

/*
 * MyDataTransfer_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t MyDataTransfer_RegisterAppCBs( simpleProfileCBs_t *appCallbacks );

/*
 * MyDataTransfer_SetParameter - Set a MyDataTransfer parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t MyDataTransfer_SetParameter(uint8_t param, uint16_t len, void *value);

/*
 * MyDataTransfer_GetParameter - Get a MyDataTransfer parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t MyDataTransfer_GetParameter(uint8_t param, uint16_t *len, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _MYDATATRANSFER_H_ */
