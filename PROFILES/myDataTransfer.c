/**********************************************************************************************
 * Filename:       myDataTransfer.c
 *
 * Description:    This file contains the implementation of the service.
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


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "myDataTransfer.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// myDataTransfer Service UUID
CONST uint8_t myDataTransferUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MYDATATRANSFER_SERV_UUID)
};

// myBufIn1 UUID
CONST uint8_t myDataTransfer_MyBufIn1UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MYDATATRANSFER_MYBUFIN1_UUID)
};
// myBufOut1 UUID
CONST uint8_t myDataTransfer_MyBufOut1UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MYDATATRANSFER_MYBUFOUT1_UUID)
};
// myBufIn2 UUID
CONST uint8_t myDataTransfer_MyBufIn2UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MYDATATRANSFER_MYBUFIN2_UUID)
};
// myBufOut2 UUID
CONST uint8_t myDataTransfer_MyBufOut2UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(MYDATATRANSFER_MYBUFOUT2_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static simpleProfileCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t myDataTransferDecl = { ATT_UUID_SIZE, myDataTransferUUID };

// Characteristic "MyBufIn1" Properties (for declaration)
static uint8_t myDataTransfer_MyBufIn1Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "MyBufIn1" Value variable
static uint8_t myDataTransfer_MyBufIn1Val[MYDATATRANSFER_MYBUFIN1_LEN] = {0};

// Characteristic "MyBufIn1" CCCD
static gattCharCfg_t *myDataTransfer_MyBufIn1Config;
// Characteristic "MyBufOut1" Properties (for declaration)
static uint8_t myDataTransfer_MyBufOut1Props = GATT_PROP_WRITE;

// Characteristic "MyBufOut1" Value variable
static uint8_t myDataTransfer_MyBufOut1Val[MYDATATRANSFER_MYBUFOUT1_LEN] = {0};
// Characteristic "MyBufIn2" Properties (for declaration)
static uint8_t myDataTransfer_MyBufIn2Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "MyBufIn2" Value variable
static uint8_t myDataTransfer_MyBufIn2Val[MYDATATRANSFER_MYBUFIN2_LEN] = {0};

// Characteristic "MyBufIn2" CCCD
static gattCharCfg_t *myDataTransfer_MyBufIn2Config;
// Characteristic "MyBufOut2" Properties (for declaration)
static uint8_t myDataTransfer_MyBufOut2Props = GATT_PROP_WRITE;

// Characteristic "MyBufOut2" Value variable
static uint8_t myDataTransfer_MyBufOut2Val[MYDATATRANSFER_MYBUFOUT2_LEN] = {0};

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t myDataTransferAttrTbl[] =
{
  // myDataTransfer Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&myDataTransferDecl
  },
    // MyBufIn1 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &myDataTransfer_MyBufIn1Props
    },
      // MyBufIn1 Characteristic Value
      {
        { ATT_UUID_SIZE, myDataTransfer_MyBufIn1UUID },
        GATT_PERMIT_READ,
        0,
        myDataTransfer_MyBufIn1Val
      },
      // MyBufIn1 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&myDataTransfer_MyBufIn1Config
      },
    // MyBufOut1 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &myDataTransfer_MyBufOut1Props
    },
      // MyBufOut1 Characteristic Value
      {
        { ATT_UUID_SIZE, myDataTransfer_MyBufOut1UUID },
        GATT_PERMIT_WRITE,
        0,
        myDataTransfer_MyBufOut1Val
      },
    // MyBufIn2 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &myDataTransfer_MyBufIn2Props
    },
      // MyBufIn2 Characteristic Value
      {
        { ATT_UUID_SIZE, myDataTransfer_MyBufIn2UUID },
        GATT_PERMIT_READ,
        0,
        myDataTransfer_MyBufIn2Val
      },
      // MyBufIn2 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&myDataTransfer_MyBufIn2Config
      },
    // MyBufOut2 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &myDataTransfer_MyBufOut2Props
    },
      // MyBufOut2 Characteristic Value
      {
        { ATT_UUID_SIZE, myDataTransfer_MyBufOut2UUID },
        GATT_PERMIT_WRITE,
        0,
        myDataTransfer_MyBufOut2Val
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t myDataTransfer_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t myDataTransfer_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t myDataTransferCBs =
{
  myDataTransfer_ReadAttrCB,  // Read callback function pointer
  myDataTransfer_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * MyDataTransfer_AddService- Initializes the MyDataTransfer service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t MyDataTransfer_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  myDataTransfer_MyBufIn1Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( myDataTransfer_MyBufIn1Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, myDataTransfer_MyBufIn1Config );
  // Allocate Client Characteristic Configuration table
  myDataTransfer_MyBufIn2Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( myDataTransfer_MyBufIn2Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, myDataTransfer_MyBufIn2Config );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( myDataTransferAttrTbl,
                                        GATT_NUM_ATTRS( myDataTransferAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &myDataTransferCBs );

  return ( status );
}

/*
 * MyDataTransfer_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t MyDataTransfer_RegisterAppCBs( simpleProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

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
bStatus_t MyDataTransfer_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case MYDATATRANSFER_MYBUFIN1_ID:
      if ( len <= MYDATATRANSFER_MYBUFIN1_LEN ) // было ==
      {
        memcpy(myDataTransfer_MyBufIn1Val, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( myDataTransfer_MyBufIn1Config, (uint8_t *)&myDataTransfer_MyBufIn1Val, FALSE,
                                    myDataTransferAttrTbl, GATT_NUM_ATTRS( myDataTransferAttrTbl ),
                                    INVALID_TASK_ID,  myDataTransfer_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case MYDATATRANSFER_MYBUFIN2_ID:
      if ( len <= MYDATATRANSFER_MYBUFIN2_LEN )
      {
        memcpy(myDataTransfer_MyBufIn2Val, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( myDataTransfer_MyBufIn2Config, (uint8_t *)&myDataTransfer_MyBufIn2Val, FALSE,
                                    myDataTransferAttrTbl, GATT_NUM_ATTRS( myDataTransferAttrTbl ),
                                    INVALID_TASK_ID,  myDataTransfer_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * MyDataTransfer_GetParameter - Get a MyDataTransfer parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t MyDataTransfer_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case MYDATATRANSFER_MYBUFOUT1_ID:
      memcpy(value, myDataTransfer_MyBufOut1Val, MYDATATRANSFER_MYBUFOUT1_LEN);
      *len = sizeof(myDataTransfer_MyBufOut1Val);   // test
      break;

    case MYDATATRANSFER_MYBUFOUT2_ID:
      memcpy(value, myDataTransfer_MyBufOut2Val, MYDATATRANSFER_MYBUFOUT2_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          myDataTransfer_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t myDataTransfer_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the MyBufIn1 Characteristic Value
if ( ! memcmp(pAttr->type.uuid, myDataTransfer_MyBufIn1UUID, pAttr->type.len) )
  {
    if ( offset > MYDATATRANSFER_MYBUFIN1_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, MYDATATRANSFER_MYBUFIN1_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the MyBufIn2 Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, myDataTransfer_MyBufIn2UUID, pAttr->type.len) )
  {
    if ( offset > MYDATATRANSFER_MYBUFIN2_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, MYDATATRANSFER_MYBUFIN2_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      myDataTransfer_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t myDataTransfer_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the MyBufOut1 Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, myDataTransfer_MyBufOut1UUID, pAttr->type.len) )
  {
    if ( offset + len > MYDATATRANSFER_MYBUFOUT1_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      //if ( offset + len == MYDATATRANSFER_MYBUFOUT1_LEN)
        paramID = MYDATATRANSFER_MYBUFOUT1_ID;
    }
  }
  // See if request is regarding the MyBufOut2 Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, myDataTransfer_MyBufOut2UUID, pAttr->type.len) )
  {
    if ( offset + len > MYDATATRANSFER_MYBUFOUT2_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      //if ( offset + len == MYDATATRANSFER_MYBUFOUT2_LEN)
        paramID = MYDATATRANSFER_MYBUFOUT2_ID;
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
//  if (paramID != 0xFF)
//    if ( pAppCBs && pAppCBs->pfnChangeCb )
//      pAppCBs->pfnChangeCb(connHandle, paramID, len, pValue); // Call app function from stack task context.

  if ( (paramID != 0xFF ) && pAppCBs && pAppCBs->pfnSimpleProfileChange )
    {
      pAppCBs->pfnSimpleProfileChange( paramID );
    }


  return status;
}
