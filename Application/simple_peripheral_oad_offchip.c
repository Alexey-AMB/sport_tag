 /******************************************************************************

 @file  simple_peripheral_oad_offchip.c

 @brief This file contains the Oad User sample application  based on
        simple_peripheral for use with the CC2650 Bluetooth Low Energy
        Protocol Stack.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2017-2019, Texas Instruments Incorporated
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
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
//#include <ti/drivers/NVS.h>
#include <ti/drivers/ADC.h>

#include <ti/sysbios/hal/Seconds.h>

#include "ExtFlash.h"

#ifdef LED_DEBUG
#include <ti/drivers/PIN.h>
#endif //LED_DEBUG

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
#include "myDataTransfer.h"
#include "ll_common.h"

// Used for imgHdr_t structure
#include "oad_image_header.h"
// Needed for HAL_SYSTEM_RESET()
#include "hal_mcu.h"

#include "oad.h"

#include "peripheral_observer.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "board.h"
#include "board_key.h"

#include "simple_peripheral_oad_offchip.h"

#include "ble_user_config.h"

#include "incommand.h"

#include "myBlink.h"


/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               100


#ifdef PLUS_OBSERVER
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  15

// Scan parameters
#define DEFAULT_SCAN_DURATION                 298   //4000
#define DEFAULT_SCAN_WIND                     177   //80
#define DEFAULT_SCAN_INT                      177   //(240 * 0.625) = 150 ms //80

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE
#endif //#ifdef PLUS_OBSERVER



// Offset into the scanRspData string the software version info is stored
#define OAD_SOFT_VER_OFFSET                   15


// Use UART display
#define SBP_DISPLAY_TYPE Display_Type_UART

// Task configuration
#define SBP_TASK_PRIORITY                     1

// Warning! To optimize RAM, task stack size must be a multiple of 8 bytes
#ifndef SBP_TASK_STACK_SIZE
  #define SBP_TASK_STACK_SIZE                   800
#endif

// Row numbers
#define TBM_ROW_APP           1
#define SBP_ROW_DEV_ADDR      (TBM_ROW_APP)
#define SBP_ROW_CONN_STATUS   (TBM_ROW_APP + 1)
#define SBP_ROW_SECURITY      (TBM_ROW_APP + 2)
#define SBP_ROW_STATUS1       (TBM_ROW_APP + 3)
#define SBP_ROW_STATUS2       (TBM_ROW_APP + 4)

/*
// Application events used with app queue (appEvtHdr_t)
// These are not related to RTOS evts, but instead enqueued via state change CBs
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_KEY_CHANGE_EVT                    0x0003
#define SBP_PASSCODE_NEEDED_EVT               0x0004
// Application specific event ID for Connection Event End Events
#define SBP_CONN_EVT                          0x0005
*/

// Application events
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PAIRING_STATE_EVT                 0x0004
#define SBP_PASSCODE_NEEDED_EVT               0x0008
#ifdef PLUS_OBSERVER
#define SBP_KEY_CHANGE_EVT                    0x0010
#define SBP_OBSERVER_STATE_EVT                0x0020
#endif
#define SBP_CONN_EVT                          0x0040


#define SBP_OAD_QUEUE_EVT                     OAD_QUEUE_EVT       // Event_Id_01
#define SBP_OAD_COMPLETE_EVT                  OAD_DL_COMPLETE_EVT // Event_Id_02
#define SBP_OAD_NO_MEM_EVT                    OAD_OUT_OF_MEM_EVT  // Event_Id_03

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_PERIODIC_EVT                      Event_Id_00
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_OAD_NO_MEM_EVT   | \
                                               SBP_OAD_QUEUE_EVT    | \
                                               SBP_OAD_COMPLETE_EVT | \
                                               SBP_PERIODIC_EVT)// Set the register cause to the registration bit-mask
// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )


//Значение делителя напряжения АЦП
#define ADC_DEVIDE_VALUE                    2
//сигнатура структуры в EPROM
#define SIGNATURE_EPROM_SETTINGS            223

#define LOW                                 0
#define HIGH                                1
#define LEN_AR_BASE_TABLE                   252 //вот ТАК! а 256 не работают!
#define BASE_SAVE_TIMEOUT                   60 //в секундах
#define BASE_SAVE_SERVICE_TIMEOUT           10 //в секундах

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;   // Event payload
} sbpEvt_t;


/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

extern const imgHdr_t _imgHdr;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbpTask;

#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(sbpTaskStack, 8)
#elif __IAR_SYSTEMS_ICC__
#pragma data_alignment=8
#endif    //__TI_COMPILER_VERSION__
uint8_t sbpTaskStack[SBP_TASK_STACK_SIZE];


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[31] =
{
 // complete name
   0x15,   // length of this data
   GAP_ADTYPE_LOCAL_NAME_COMPLETE,
   'S',
   'B',
   'P',
   ' ',
   'O',
   'A',
   'D',
   ' ',
   'A',
   'P',
   'P',
   ' ',
   'v',
   ' ', // These 4 octets are placeholders for the SOFTVER field
   ' ', // which will be updated at init time
   ' ',
   ' ',
   ' ',
   ' ',
   ' ',
   // connection interval range
   0x05,   // length of this data
   GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
   LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 10ms
   HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
   LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 10ms
   HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

   // Tx power level
   0x02,   // length of this data
   GAP_ADTYPE_POWER_LEVEL,
   0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x05,  // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
  LO_UINT16(MYDATATRANSFER_SERV_UUID),
  HI_UINT16(MYDATATRANSFER_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "SPORT_TAG";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

#ifdef PLUS_OBSERVER
// Scanning started flag
static bool scanningStarted = FALSE;

const char *AdvTypeStrings[] = {
  "Connectable undirected",
  "Connectable directed",
  "Scannable undirected",
  "Non-connectable undirected",
  "Scan response"
};
#endif

#if defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED)
// Flag to be stored in NV that tracks whether service changed
// indications needs to be sent out
static uint32_t  sendSvcChngdOnNextBoot = FALSE;
#endif // ( defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED) )

// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0;

// User variable =============================
uint8 bdAddress[B_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; //адрес БТ данного устройства

SPORT_TAG_SETTINGS cur_tag_settings;

WORKMODE previonsMode = MODE_CONNECT;

uint16_t iRecivedLen;
uint16_t iSendedLen;
uint16_t iExpectedLen;
uint8_t * pBuffIn = NULL;
uint16_t  iBuffInLen = 0;
uint8_t * pBuffOut = NULL;
uint16_t  iBuffOutLen = 0;
uint8_t * pBuffCm = NULL;
uint16_t  iBuffCmLen = 0;
uint8_t  iExpectedCrc;
uint8_t  iCurrCmd;

uint8_t NumLastBase = 0;
uint32_t TimeLastBase = 0;

uint8_t arBaseTable[LEN_AR_BASE_TABLE];
uint8_t currPosBaseTable = 0;

uint32_t timeShutdown = 0;

uint32_t SoftVersion = 3;    //версия ПО

//================================================
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init( void );
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimplePeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimplePeripheral_performPeriodicTask(void);
static void SimplePeripheral_clockHandler(UArg arg);

#ifdef PLUS_OBSERVER
static void SimpleBLEPeripheralObserver_processRoleEvent(gapPeriObsRoleEvent_t *pEvent);
static void SimpleBLEPeripheralObserver_handleKeys(uint8_t keys);
static void SimpleBLEPeripheralObserver_StateChangeCB(gapPeriObsRoleEvent_t *pEvent);
#endif

static void SimplePeripheral_sendAttRsp(void);
static void SimplePeripheral_freeAttRsp(uint8_t status);
static void SimplePeripheral_stateChangeCB(gaprole_States_t newState);
static void SimplePeripheral_charValueChangeCB(uint8_t paramID);
static uint8_t SimplePeripheral_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData);

static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimplePeripheral_processOadWriteCB(uint8_t event, uint16_t arg);
static void SimplePeripheral_keyChangeHandler(uint8_t keys);
//static void SimplePeripheral_handleKeys(uint8_t keys);
static uint8_t SimplePeripheral_processL2CAPMsg(l2capSignalEvent_t *pMsg);
static void SimplePeripheral_processPasscode(gapPasskeyNeededEvent_t *pData);
static void SimplePeripheral_passcodeCB(uint8_t *deviceAddr,
                                            uint16_t connHandle,
                                            uint8_t uiInputs, uint8_t uiOutputs,
                                            uint32_t numComparison);

// User functions ================================
static void TimeFunction(void);
void  itoa(uint32_t value, char *string, int radix);
void beep(uint16_t ms, uint8_t n);
static void ReadStartParam(void);

static void ApplyParam(void);
static void CheckAkkumVoltage(void);

static bool ReadEprom_inter_osal(void * buff, uint32_t lenbuff, uint32_t offset);
static bool WriteEprom_inter_osal(void * buff, uint32_t lenbuff, uint32_t offset);
static uint32_t GetADCmicrovolt(void);
static uint32_t GetAkkumVoltage(void);
static void WorkWithInputBuffer(uint8_t * buf, uint16_t lenbuf);
static void SendAsk(OutAsk ask, bool bHaveBuf);
static bool ExecuteCommand(bool bHaveBuf);
static uint8_t GetCRC8(uint8_t * buf, int len);
static uint8_t min(uint8_t a, uint8_t b);

static bool ChangeAdvertisingData(void);

static void WorkWithDiscoBase(uint8_t * pBuf, uint8_t len);
static void AddBaseToList(uint8_t numBase, uint32_t timeBase);
static void SaveBaseTable(void);
static bool MoveEpromBlock(uint8_t strtBlk);

static void ChangeWorkMode(WORKMODE mode);
void MyPowerDown(void);
//================================================

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimplePeripheral_gapRoleCBs =
{
  SimplePeripheral_stateChangeCB     // Profile State Change Callbacks
  
#ifdef PLUS_OBSERVER
  ,SimpleBLEPeripheralObserver_StateChangeCB
#endif
  
};

// GAP Bond Manager Callbacks
static gapBondCBs_t SimplePeripheral_BondMgrCBs =
{
  (pfnPasscodeCB_t)SimplePeripheral_passcodeCB, // Passcode callback,
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimplePeripheral_simpleProfileCBs =
{
  SimplePeripheral_charValueChangeCB // Characteristic value change callback
};

static oadTargetCBs_t SimplePeripheral_oadCBs =
{
  SimplePeripheral_processOadWriteCB // Write Callback.
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
   NOT_REGISTER       = 0,
   FOR_AOA_SCAN       = 1,
   FOR_ATT_RSP        = 2,
   FOR_AOA_SEND       = 4,
   FOR_TOF_SEND       = 8,
   FOR_OAD_SEND       = 0x10,
}connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t       connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

/*********************************************************************
 * @fn      SimplePeripheral_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimplePeripheral_RegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // in case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  if(status == SUCCESS)
  {
    //add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*********************************************************************
 * @fn      SimplePeripheral_UnRegistertToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimplePeripheral_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  // in case  there is no more registration for the connection event than unregister
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

//my key hanlder ====================================

//чтение параметров из ПЗУ из блока №1 в блоке №0 - sendSvcChngdOnNextBoot см. стр 1375
static void ReadStartParam(void)
{
    if(ReadEprom_inter_osal((void *)&cur_tag_settings, sizeof(SPORT_TAG_SETTINGS), 1))
    {
        if(cur_tag_settings.signature == SIGNATURE_EPROM_SETTINGS)
        {
            Display_printf(dispHandle, 0, 0, "Settings read from EPROM - OK.");
            return;
        }
    }

    Display_printf(dispHandle, 0, 0, "Settings read from EPROM - ERROR.");


    cur_tag_settings.mode_tag = MODE_CONNECT;
    cur_tag_settings.powerble_tag = 5;
    memset((void*)cur_tag_settings.name_tag, 0, sizeof(cur_tag_settings.name_tag));
    strcpy(cur_tag_settings.name_tag, "METKA 1");
    memset((void*)cur_tag_settings.password_tag, 0, sizeof(cur_tag_settings.password_tag));
    strcpy(cur_tag_settings.password_tag, "111111");
    cur_tag_settings.timeut_conn = 600;
    //cur_tag_settings.timeut_run = 600;
    cur_tag_settings.treshold_tag = -60;


    memset((void*)cur_tag_settings.fam, 0, sizeof(cur_tag_settings.fam));
    cur_tag_settings.signature = SIGNATURE_EPROM_SETTINGS;

    //crc = GetCRC8((void *)&cur_base_settings, sizeof(SPORT_BASE_SETTINGS));

    Display_printf(dispHandle, 0, 0, "Create default settings.");

    WriteEprom_inter_osal((void *)&cur_tag_settings, sizeof(SPORT_TAG_SETTINGS), 1);
}

//Применеие параметров метки
static void ApplyParam(void)
{
    //применить мощность передатчика BLE
    HCI_EXT_SetTxPowerCmd(cur_tag_settings.powerble_tag);   //значения от 0 до 12 см. ll.h

    ChangeAdvertisingData();
    ChangeWorkMode(cur_tag_settings.mode_tag);

    Task_sleepMS(10);
}

//проверка напряжения батареи перед работой
static void CheckAkkumVoltage(void)
{
    if(GetAkkumVoltage() > 2700000)   // 2700000 - порог батареи 2.7V
    {
        SendToBlink(PRF_AKK_FULL);
        return;
    }
    if(GetAkkumVoltage() > 2200000)
    {
        SendToBlink(PRF_AKK_MEDIUM);
        return;
    }

    SendToBlink(PRF_AKK_LOW);
    return;
}
//чтение буфера из внутреннего ПЗУ .максимальное смещение 16 максимальный размер буфера 252 (по документам 256 но не работает)
static bool ReadEprom_inter_osal(void * buff, uint32_t lenbuff, uint32_t offset)
{
    uint8_t iRet = 0;

    iRet = osal_snv_read(BLE_NVID_CUST_START + offset, lenbuff, buff);

    if(iRet == SUCCESS)
    {
        Display_printf(dispHandle, 0, 0, "Read from EPROM - OK.");
        return true;
    }
    else
    {
        Display_printf(dispHandle, 0, 0, "Read from EPROM - ERROR.");
        return FALSE;
    }
}
//запись буфера во внутреннее ПЗУ .максимальное смещение 16 максимальный размер буфера 252 (по документам 256 но не работает)
static bool WriteEprom_inter_osal(void * buff, uint32_t lenbuff, uint32_t offset)
{
    uint8_t iRet = 0;

    iRet = osal_snv_write(BLE_NVID_CUST_START + offset, lenbuff, buff);

    if(iRet == SUCCESS)
    {
        Display_printf(dispHandle, 0, 0, "Write to EPROM - OK.");
        return true;
    }
    else
    {
        Display_printf(dispHandle, 0, 0, "Write to EPROM - ERROR.");
        return FALSE;
    }
}
//получение значения с ADC0 (напряжение батареи)
static uint32_t GetADCmicrovolt(void)
{
    uint32_t adcValue0MicroVolt = 0;
    uint16_t adcValue0 = 0;
    ADC_Handle   adc;
    ADC_Params   params;
    int_fast16_t res = 0;

    ADC_Params_init(&params);
    adc = ADC_open(Board_ADC0, &params);

    if (adc == NULL) {
        Display_printf(dispHandle, 0, 0, "Error initializing ADC channel 0\n");
        while (1);
    }

    /* Blocking mode conversion */
    res = ADC_convert(adc, &adcValue0);

    if (res == ADC_STATUS_SUCCESS)
    {
        adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0);
        //Display_printf(dispHandle, 0, 0, "%d \n", (char *)adcValue0MicroVolt);
    }
    else
    {
        Display_printf(dispHandle, 0, 0, "ADC channel 0 convert failed\n");
    }

    ADC_close(adc);

    return adcValue0MicroVolt;
}
//померять текущее значение напряжения аккумулятора в микровольтах (занимает 100 ms)
static uint32_t GetAkkumVoltage(void)
{
    uint8_t i = 0;
    uint32_t val = 0;

    for(i=0; i<10; i++)
    {
        val += GetADCmicrovolt();
        Task_sleepMS(10);
    }
    val = val / 10;

    return (val * ADC_DEVIDE_VALUE);
}
//считаем CRC блока
static uint8_t GetCRC8(uint8_t *buf, int lenbuf)
{
    int i;
    uint8_t crc = 0;

    for (i=0; i < lenbuf; i++) crc +=  (uint8_t)*(buf + i);

    return crc;
}


stCommand * curCmd = NULL;
//работа с принятым буфером
static void WorkWithInputBuffer(uint8_t * buf, uint16_t lenbuf)
{
    timeShutdown = Seconds_get() + cur_tag_settings.timeut_conn;

    if((iRecivedLen == 0)&&(iExpectedLen == 0))
       {//не принят еще не один блок
           curCmd = (stCommand*)buf;
           if(curCmd->signature == SIGNATURE_COMMAND)
           {
               iExpectedLen = curCmd->lenbuf;
               iCurrCmd = curCmd->cmd;
               iExpectedCrc = curCmd->crc;
               if(iExpectedLen == 0)
               {
                   ExecuteCommand(false);
                   return;
               }
               else
               {
                   pBuffIn = malloc(iExpectedLen);
                   if(!pBuffIn)
                   {//сообщить об ошибке
                       SendAsk(ASK_MALLOC_ERROR, false);
                       iExpectedLen = 0;
                       iCurrCmd = 0;
                       iExpectedCrc = 0;
                       return;
                   }
                   iBuffInLen = iExpectedLen;
                   if(iExpectedLen <= (lenbuf - sizeof(stCommand)))
                   {
                       memcpy(pBuffIn, buf + sizeof(stCommand), iExpectedLen);
                       if(GetCRC8(pBuffIn, iExpectedLen) == iExpectedCrc)
                       {
                           ExecuteCommand(true);
                           return;
                       }
                       else
                       { //сообщить об ошибке
                           SendAsk(ASK_ERROR_CRC, false);
                           iExpectedLen = 0;
                           iCurrCmd = 0;
                           iExpectedCrc = 0;
                           return;
                       }
                   }
                   else
                   {//если данные не влезли в один буфер
                       memcpy(pBuffIn, buf + sizeof(stCommand), lenbuf - sizeof(stCommand));
                       iRecivedLen = lenbuf - sizeof(stCommand);
                       SendAsk(ASK_NEXT, false);
                       return;
                   }
               }
           }
           else
           { //не сошлась сигнатура
               SendAsk(ASK_ERROR, false);
               iExpectedLen = 0;
               iCurrCmd = 0;
               iExpectedCrc = 0;
               return;
           }
       }

       if(iExpectedLen > iRecivedLen)
       {
           if(iExpectedLen - iRecivedLen > lenbuf)
           {
               memcpy(pBuffIn + iRecivedLen, buf, lenbuf);
               iRecivedLen += lenbuf;
               SendAsk(ASK_NEXT, false);
               return;
           }
           else
           {
               memcpy(pBuffIn + iRecivedLen, buf, iExpectedLen - iRecivedLen);
               if(GetCRC8(pBuffIn, iExpectedLen) == iExpectedCrc)
               {
                   ExecuteCommand(true);
                   return;
               }
               else
               { //сообщить об ошибке
                   SendAsk(ASK_ERROR_CRC, false);
                   iExpectedLen = 0;
                   iCurrCmd = 0;
                   iExpectedCrc = 0;
                   return;
               }
           }
       }
       else
       {   //странная ситуация, но такое было
           iRecivedLen = 0;
           iExpectedLen = 0;
           iSendedLen = 0;
           if(pBuffIn != NULL) free(pBuffIn);
           pBuffIn = NULL;
       }
}

//отправить сообщение на ПК
static void SendAsk(OutAsk ask, bool bHaveBuf)
{
    stCommand cmdOut;
        uint8_t * buf = NULL;
        uint8_t   len = 0;

        if(iSendedLen == 0)
        {
            cmdOut.cmd = ask;
            cmdOut.signature = SIGNATURE_COMMAND;
            cmdOut.lenbuf = 0;
            cmdOut.crc = 0;
            if((bHaveBuf)&&(pBuffOut))
            {
                cmdOut.lenbuf = iBuffOutLen;
                cmdOut.crc = GetCRC8(pBuffOut, iBuffOutLen);
            }

            if((cmdOut.lenbuf + sizeof(stCommand)) > MYDATATRANSFER_MYBUFIN1_LEN)
            {   // все не влезет в один буфер
                len = MYDATATRANSFER_MYBUFIN1_LEN;
                iSendedLen = len - sizeof(stCommand);
            }
            else
            {   // влезет в один буфер
                len = cmdOut.lenbuf + sizeof(stCommand);
                iSendedLen = 0;
            }

            buf = ICall_malloc(len);
            if(!buf)
            {
                Display_print0(dispHandle, 0, 0, "SendAsk: malloc error ");
                return;
            }
            memset(buf, 0, len);
            memcpy(buf, &cmdOut, sizeof(stCommand));
            if((bHaveBuf)&&(pBuffOut)) memcpy(buf + sizeof(stCommand), pBuffOut, len - sizeof(stCommand));
            MyDataTransfer_SetParameter(MYDATATRANSFER_MYBUFIN1_ID, len, buf);
            ICall_free(buf);
            if((iSendedLen == 0)&&(bHaveBuf)) //передача закончена
            {
                free(pBuffOut);
                pBuffOut = NULL;
            }
        }
        else
        {
            if((iBuffOutLen - iSendedLen) > MYDATATRANSFER_MYBUFIN1_LEN)
            {   // все не влезет в один буфер
                len = MYDATATRANSFER_MYBUFIN1_LEN;
                //iSendedLen += len;
            }
            else
            {   // влезет в один буфер
                len = iBuffOutLen - iSendedLen;
                //iSendedLen = 0;
            }

            buf = ICall_malloc(len);
            if(!buf)
            {
                Display_print0(dispHandle, 0, 0, "SendAsk: malloc error ");
                return;
            }
            memset(buf, 0, len);
            if((bHaveBuf)&&(pBuffOut)) memcpy(buf, pBuffOut + iSendedLen, len);
            MyDataTransfer_SetParameter(MYDATATRANSFER_MYBUFIN1_ID, len, buf);
            ICall_free(buf);
            iSendedLen += len;
            if(iSendedLen  == iBuffOutLen) //передача закончена
            {
                if(pBuffOut != NULL) free(pBuffOut);
                pBuffOut = NULL;
                iSendedLen = 0;
            }
        }
}

//выполнить присланную команду
// в параметрах - есть буфер или нет
static bool ExecuteCommand(bool bHaveBuf)
{
    bool bRet = false;
    uint8_t numBlock = 1;

    switch (iCurrCmd)
    {
    case CMD_NONE:
        SendAsk(ASK_OK, false);
        bRet = true;
        break;

    case CMD_SET_BLINK:
        SendToBlink(PRF_SIMPLEBLINK);
        SendAsk(ASK_OK, false);
        bRet = true;
        break;

    case CMD_SET_MODE_RUN:
        ChangeWorkMode(MODE_RUN);
        //SendAsk(ASK_OK, false);
        bRet = true;
        break;

    case CMD_SET_MODE_CONN:
        ChangeWorkMode(MODE_CONNECT);
        //SendAsk(ASK_OK, false);
        bRet = true;
        break;

    case CMD_SET_MODE_SLEEP:
        ChangeWorkMode(MODE_SLEEP);
        //SendAsk(ASK_OK, false);
        bRet = true;
        break;

    case CMD_SET_SETTINGS:
        memcpy((void*)&cur_tag_settings, pBuffIn, sizeof(SPORT_TAG_SETTINGS));
		if(cur_tag_settings.signature == SIGNATURE_EPROM_SETTINGS)
		{
			WriteEprom_inter_osal((void *)&cur_tag_settings, sizeof(SPORT_TAG_SETTINGS), 1);
			ApplyParam();
			SendAsk(ASK_OK, false);
			bRet = true;
		}
		else
		{
			ReadStartParam();
			SendAsk(ASK_ERROR, false);
			bRet = false;
		}
        break;

    case CMD_SET_TIME:
    {
        uint32_t timeNow = 0;
        memcpy((void*)&timeNow, pBuffIn, sizeof(uint32_t));
        Seconds_set(timeNow);
        SendAsk(ASK_OK, false);
        bRet = true;
        break;
    }

    case CMD_NEXT:
        SendAsk(ASK_OK, true);
        bRet = true;
        break;

    case CMD_GET_SETTINGS:
        pBuffOut = malloc(sizeof(SPORT_TAG_SETTINGS));
        if (!pBuffOut)
        {
            SendAsk(ASK_ERROR, false);
            bRet = false;
            break;
        }
        iBuffOutLen = sizeof(SPORT_TAG_SETTINGS);
        memset(pBuffOut, 0, sizeof(SPORT_TAG_SETTINGS));
        memcpy(pBuffOut, (void*)&cur_tag_settings, sizeof(SPORT_TAG_SETTINGS));
        SendAsk(ASK_OK, true);
        bRet = true;
        break;

    case CMD_GET_AKKVOLTAGE:
    {
        pBuffOut = malloc(sizeof(uint32_t));
        if (!pBuffOut)
        {
            SendAsk(ASK_ERROR, false);
            bRet = false;
            break;
        }
        iBuffOutLen = sizeof(uint32_t);
        memset(pBuffOut, 0, sizeof(uint32_t));
        uint32_t av = GetAkkumVoltage();
        memcpy(pBuffOut, &av, sizeof(uint32_t));
        SendAsk(ASK_OK, true);
        bRet = true;
        break;
    }

    case CMD_READ_DATA:
        pBuffOut = malloc(LEN_AR_BASE_TABLE);
        if (!pBuffOut)
        {
            SendAsk(ASK_MALLOC_ERROR, false);
            bRet = false;
            break;
        }
        iBuffOutLen = LEN_AR_BASE_TABLE;
        memset(pBuffOut, 0, LEN_AR_BASE_TABLE);

        numBlock = *pBuffIn;
        if(numBlock < 15)
        {
            ReadEprom_inter_osal(pBuffOut, LEN_AR_BASE_TABLE, numBlock);
            SendAsk(ASK_OK, true);
            bRet = true;
        }
        else
        {
            SendAsk(ASK_ERROR, false);
            bRet = false;
        }
        break;

    case CMD_GET_VERSION:
    {
        pBuffOut = malloc(sizeof(uint32_t));
        if (!pBuffOut)
        {
            SendAsk(ASK_ERROR, false);
            bRet = false;
            break;
        }
        iBuffOutLen = sizeof(uint32_t);
        memset(pBuffOut, 0, sizeof(uint32_t));
        memcpy(pBuffOut, &SoftVersion, sizeof(uint32_t));
        SendAsk(ASK_OK, true);
        bRet = true;
        break;
    }

    default:
        SendAsk(ASK_ERROR, false);
        bRet = false;
        break;
    }

    //--------------
    iExpectedLen = 0;
    iCurrCmd = 0;
    iExpectedCrc = 0;
    if(pBuffIn) free(pBuffIn);
    pBuffIn = NULL;
    iBuffInLen = 0;
    return bRet;
}


//===================================================


/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the OAD User App.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimplePeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimplePeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, FALSE, SBP_PERIODIC_EVT);

  dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);

  // Read in the OAD Software version
  uint8_t swVer[OAD_SW_VER_LEN];
  OAD_getSWVersion(swVer, OAD_SW_VER_LEN);

  ADC_init();
  Board_initKeys(SimplePeripheral_keyChangeHandler);
  Board_initLeds();
  InitBlink();
  
  #ifdef PLUS_OBSERVER
  //Board_initKeys(SimpleBLEPeripheralObserver_keyChangeHandler);

  //Setup GAP Observer params
  {
    // Set the max amount of scan responses
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t), &scanRes);

    // Set scan duration
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);

    // Scan interval and window the same for all scenarios
    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);
  }
#endif

  /**************Init Menu*****************************/

//  tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_0 | TBM_ITEM_5,
//                    TBM_ITEM_1 | TBM_ITEM_2 | TBM_ITEM_3 | TBM_ITEM_4 );
//
//  // Init two button menu
//  tbm_initTwoBtnMenu(dispHandle, &sbpMenuMain, 1, NULL);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    // Setup the dyanmic portion of the scanRspData
    scanRspData[OAD_SOFT_VER_OFFSET] = swVer[0];
    scanRspData[OAD_SOFT_VER_OFFSET + 1] = swVer[1];
    scanRspData[OAD_SOFT_VER_OFFSET + 2] = swVer[2];
    scanRspData[OAD_SOFT_VER_OFFSET + 3] = swVer[3];

    // Set scanRspData
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t), &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),  &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),  &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t), &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t), &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;
    uint8_t scMode = GAPBOND_SECURE_CONNECTION_ALLOW;
    uint8_t replaceBonds = FALSE;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
    GAPBondMgr_SetParameter(GAPBOND_LRU_BOND_REPLACEMENT, sizeof(uint8_t), &replaceBonds);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  //SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile


  // Open the OAD module and add the OAD service to the application
  if(OAD_SUCCESS != OAD_open(OAD_DEFAULT_INACTIVITY_TIME))
  {
    Display_print0(dispHandle, 0, 0, "OAD failed to open");
  }
  else
  {
    // Resiter the OAD callback with the application
    OAD_register(&SimplePeripheral_oadCBs);
  }

//  // Setup the SimpleProfile Characteristic Values
//  {
//    uint8_t charValue1 = 1;
//    uint8_t charValue2 = 2;
//    uint8_t charValue3 = 3;
//    uint8_t charValue4 = 4;
//    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
//
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
//                               &charValue1);
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
//                               &charValue2);
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
//                               &charValue3);
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
//                               &charValue4);
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
//                               charValue5);
//  }
//
//  // Register callback with SimpleGATTprofile
//  SimpleProfile_RegisterAppCBs(&SimplePeripheral_simpleProfileCBs);

  MyDataTransfer_AddService(selfEntity);
  MyDataTransfer_RegisterAppCBs(&SimplePeripheral_simpleProfileCBs);

  // Initalization of characteristics in myDataTransfer that are readable.
  uint8_t myDataTransfer_myBufIn1_initVal[MYDATATRANSFER_MYBUFIN1_LEN] = {0};
  MyDataTransfer_SetParameter(MYDATATRANSFER_MYBUFIN1_ID, MYDATATRANSFER_MYBUFIN1_LEN, myDataTransfer_myBufIn1_initVal);
  uint8_t myDataTransfer_myBufIn2_initVal[MYDATATRANSFER_MYBUFIN2_LEN] = {0};
  MyDataTransfer_SetParameter(MYDATATRANSFER_MYBUFIN2_ID, MYDATATRANSFER_MYBUFIN2_LEN, myDataTransfer_myBufIn2_initVal);


  // Start the Device
  VOID GAPRole_StartDevice(&SimplePeripheral_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);


  //Set default values for Data Length Extension
    {
      //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
      #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
      #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

      //This API is documented in hci.h
      //See the LE Data Length Extension section in the BLE-Stack User's Guide for information on using this command:
      //http://software-dl.ti.com/lprf/sdg-latest/html/cc2640/index.html
      //HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
    }

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  // Get the currently set local supported LE features
  // The HCI will generate an HCI event that will get received in the main
  // loop
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

  uint8_t versionStr[OAD_SW_VER_LEN + 1];

  memcpy(versionStr, swVer, OAD_SW_VER_LEN);

  // Add in Null terminator
  versionStr[OAD_SW_VER_LEN] = NULL;

  // Display Image version
  Display_print1(dispHandle, 0, 0, "SBP Off-chip OAD v%s", versionStr);

//#ifdef LED_DEBUG
//  // Open the LED debug pins
//  if (!PIN_open(&sbpLedState, sbpLedPins))
//  {
//    Display_print0(dispHandle, 0, 0, "Debug PINs failed to open");
//  }
//  else
//  {
//      PIN_Id activeLed;
//      uint8_t numBlinks = 19;
//      if (numBlinks < 15)
//      {
//        activeLed = Board_LED0;
//      }
//      else
//      {
//       activeLed = Board_LED1;
//    }
//    for(uint8_t cnt = 0; cnt < numBlinks; ++cnt)
//    {
//
//      PIN_setOutputValue(&sbpLedState, activeLed, !PIN_getOutputValue(activeLed));
//
//      // Sleep for 100ms, sys-tick for BLE-Stack is 10us,
//      // Task sleep is in # of ticks
//      Task_sleep(10000);
//
//    }
//
//    // Close the pins after using
//    PIN_close(&sbpLedState);
//  }
//#endif //LED_DEBUG

#if defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED)
  /*
   * When switching from persistent app back to the user application for the
   * for the first time after an OAD the device must send a service changed
   * indication. This will cause any peers to rediscover services.
   *
   * To prevent sending a service changed IND on every boot, a flag is stored
   * in NV to determine whether or not the service changed IND needs to be
   * sent
   */
  uint8_t status = osal_snv_read(BLE_NVID_CUST_START,
                                  sizeof(sendSvcChngdOnNextBoot),
                                  (uint8 *)&sendSvcChngdOnNextBoot);
  if(status != SUCCESS)
  {
    /*
     * On first boot the NV item will not have yet been initialzed, and the read
     * will fail. Do a write to set the initial value of the flash in NV
     */
     osal_snv_write(BLE_NVID_CUST_START, sizeof(sendSvcChngdOnNextBoot),
                    (uint8 *)&sendSvcChngdOnNextBoot);
  }
#endif // ( defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED) )

  Util_startClock(&periodicClock);
}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the OAD User App.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1)
{
    Board_wakeUpExtFlash();
    // Initialize application
    SimplePeripheral_init();
    //++++++++++++++++++++++++++++

    ReadStartParam();
    Board_setLed0_my(0);
    Board_setLed1_my(0);
    Board_setVibro(0);
    CheckAkkumVoltage();
    TimeFunction();

    ApplyParam();
    timeShutdown = Seconds_get() + cur_tag_settings.timeut_conn;
    Display_printf(dispHandle, 8, 0, "Version software = %x.", SoftVersion);
    //================================

    //HCI_ReadBDADDRCmd();  //делаем запрос на получение UID (мак BT) ответ получаем в HCI_READ_BDADDR

    //Display_printf(dispHandle, 0, 1, "Goto main loop");
    //Display_printf(dispHandle, 6, 0, "evenCM = %x \n", (char *)eventCM);

    // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SBP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {

          // Get the first message from the Queue
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);

          if (pMsg)
          {
            // Process message.
            SimplePeripheral_processAppMsg(pMsg);

            if (pMsg->pData != NULL)
            {
              // Free the Queue payload if there is one
              ICall_free(pMsg->pData);
            }

            // Free the space from the message.
            ICall_free(pMsg);
          }

        }
      }

      if(events & SBP_OAD_NO_MEM_EVT)
      {
        // The OAD module is unable to allocate memory, print failure, cancel OAD
        Display_print0(dispHandle, SBP_ROW_STATUS1, 0, "OAD malloc fail, cancelling OAD");
        OAD_cancel();

//#ifdef LED_DEBUG
//        // Diplay is not enabled in persist app so use LED
//        if(PIN_open(&sbpLedState, sbpLedPins))
//        {
//          PIN_setOutputValue(&sbpLedState, Board_RLED, Board_LED_ON);
//        }
//#endif //LED_DEBUG
      }

      // OAD Queue processing
      if(events & SBP_OAD_QUEUE_EVT)
      {
        // Process the OAD Message Queue
        uint8_t status = OAD_processQueue();

        // If the OAD state machine encountered an error, print it
        // Return codes can be found in oad_constants.h
        if(status == OAD_DL_COMPLETE)
        {
          Display_print0(dispHandle, SBP_ROW_STATUS1, 0, "OAD DL Complete, wait for Enable");
        }
        else if(status == OAD_IMG_ID_TIMEOUT)
        {
          Display_print0(dispHandle, SBP_ROW_STATUS1, 0, "ImgID Timeout, disconnecting");

          // This may be an attack, terminate the link
          GAPRole_TerminateConnection();
        }
        else if(status != OAD_SUCCESS)
        {
          Display_print1(dispHandle, SBP_ROW_STATUS1, 0, "OAD Error: %d", status);
        }

      }
      if(events & SBP_OAD_COMPLETE_EVT)
      {
        Display_print0(dispHandle, SBP_ROW_STATUS1, 0, "OAD_COMPLETE_EVT: Resetteing");
        // Register for L2CAP Flow Control Events
        L2CAP_RegisterFlowCtrlTask(selfEntity);
      }

      if (events & SBP_PERIODIC_EVT)
      {
        Util_startClock(&periodicClock);

        // Perform periodic application task
        SimplePeripheral_performPeriodicTask();
      }
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_performPeriodicTask(void)
{
    PerformBlink();

    if (scanningStarted == FALSE)   //Да, это лоховство, но по другому получаем DeadLock.
    {
        if(cur_tag_settings.mode_tag == MODE_RUN)
        {
            uint8 status;

            status = GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                            DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                            DEFAULT_DISCOVERY_WHITE_LIST);

            if(status == SUCCESS)
            {
                scanningStarted = TRUE;
                //Display_print0(dispHandle, 4, 0, "Scanning On");
            }
            else
            {
                Display_print1(dispHandle, 4, 0, "Scanning failed: %d", status);
            }
        }
    }

    if(Seconds_get() > timeShutdown) MyPowerDown();
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
	  
#ifdef PLUS_OBSERVER
    case GAP_MSG_EVENT:
      // Process GAP message
      SimpleBLEPeripheralObserver_processRoleEvent((gapPeriObsRoleEvent_t *)pMsg);
      break;
#endif
	  
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // get current feature set from received event (bits 1-9 of
                      // the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          default:
            break;
        }
      }
      break;

    case L2CAP_SIGNAL_EVENT:
      // Process L2CAP signal
      safeToDealloc = SimplePeripheral_processL2CAPMsg((l2capSignalEvent_t *)pMsg);
      break;

      default:
        // do nothing
        break;

    }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processL2CAPMsg
 *
 * @brief   Process L2CAP messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  static bool firstRun = TRUE;

  switch (pMsg->opcode)
  {
    case L2CAP_NUM_CTRL_DATA_PKT_EVT:
    {
      /*
      * We cannot reboot the device immediately after receiving
      * the enable command, we must allow the stack enough time
      * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
      * command. This command will determine the number of
      * packets currently queued up by the LE controller.
      * BIM var is already set via OadPersistApp_processOadWriteCB
      */
      if(firstRun)
      {
        firstRun = false;

        // We only want to set the numPendingMsgs once
        numPendingMsgs = MAX_NUM_PDU - pMsg->cmd.numCtrlDataPktEvt.numDataPkt;
        
        if(numPendingMsgs)
        {
          // Wait the number of connection events
          SimplePeripheral_RegistertToAllConnectionEvent (FOR_OAD_SEND);
        }
      }

      break;
    }
    default:
      break;
  }

  // It's safe to free the incoming message
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if( SimplePeripheral_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
    {
      // First free any pending response
      SimplePeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, SBP_ROW_STATUS2, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    OAD_setBlockSize(pMsg->msg.mtuEvt.MTU);
    Display_print1(dispHandle, SBP_ROW_STATUS2, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport)
{

  if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
  {
    // The GATT server might have returned a blePending as it was trying
    // to process an ATT Response. Now that we finished with this
    // connection event, let's try sending any remaining ATT Responses
    // on the next connection event.
    // Try to retransmit pending ATT Response (if any)
    SimplePeripheral_sendAttRsp();
  }
  if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_OAD_SEND))
  {
    // Wait until all pending messages are sent
    if(numPendingMsgs == 0)
    {

#if defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED)
      // Store the flag to indicate that a service changed IND will
      // be sent at the next boot
      sendSvcChngdOnNextBoot = TRUE;

      uint8_t status = osal_snv_write(BLE_NVID_CUST_START,
                                      sizeof(sendSvcChngdOnNextBoot),
                                      (uint8 *)&sendSvcChngdOnNextBoot);
      if(status != SUCCESS)
      {
        Display_print1(dispHandle, 5, 0, "SNV WRITE FAIL: %d", status);
      }
#endif // ( defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED) )

      // Reset the system
      HAL_SYSTEM_RESET();
    }
    numPendingMsgs--;

  }

}

/*********************************************************************
 * @fn      SimplePeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimplePeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      SimplePeripheral_UnRegistertToAllConnectionEvent (FOR_ATT_RSP);
      // We're done with the response message
      SimplePeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, SBP_ROW_STATUS2, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimplePeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, SBP_ROW_STATUS2, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, SBP_ROW_STATUS2, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
    {
      SimplePeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;
    }

    case SBP_CHAR_CHANGE_EVT:
    {
      SimplePeripheral_processCharValueChangeEvt(pMsg->hdr.state);

      break;
    }

    case SBP_KEY_CHANGE_EVT:
    {
      //SimplePeripheral_handleKeys(pMsg->hdr.state);

      SimpleBLEPeripheralObserver_handleKeys(pMsg->hdr.state);

      break;
    }

    case SBP_PASSCODE_NEEDED_EVT:
    {
        SimplePeripheral_processPasscode((gapPasskeyNeededEvent_t*)pMsg->pData);
        break;
    }

    case SBP_CONN_EVT:
    {
        SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
        break;
    }

#ifdef PLUS_OBSERVER
    //    case SBP_KEY_CHANGE_EVT:
    //      SimpleBLEPeripheralObserver_handleKeys(pMsg->hdr.state);
    //      break;

    case SBP_OBSERVER_STATE_EVT:
        SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg->pData);
        break;
#endif

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimplePeripheral_processStateChangeEvt(gaprole_States_t newState)
{	
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        //bdAddress

        // Display device address
        Display_print1(dispHandle, SBP_ROW_DEV_ADDR, 0, "BD Addr: %s", Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, SBP_ROW_STATUS1, 0, "GAPRole Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, SBP_ROW_CONN_STATUS, 0, "Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;
        uint16_t connHandle = 0;

        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);

        //Util_startClock(&periodicClock);
        timeShutdown = Seconds_get() + cur_tag_settings.timeut_conn;

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          Display_print1(dispHandle, SBP_ROW_CONN_STATUS, 0,
                          "Connected to: %s", Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Display_print1(dispHandle, SBP_ROW_CONN_STATUS, 0,
                          "Connected to : %s", Util_convertBdAddr2Str(peerAddress));
        }

#if defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED)
        if(sendSvcChngdOnNextBoot == TRUE)
        {
          GAPBondMgr_ServiceChangeInd( connHandle, TRUE);

          sendSvcChngdOnNextBoot = FALSE;
        }
#endif // ( defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED) )


        iRecivedLen = 0;
        iExpectedLen = 0;
        iSendedLen = 0;
        if(pBuffIn != NULL) free(pBuffIn);
        pBuffIn = NULL;
        if(pBuffOut != NULL) free (pBuffOut);
        pBuffOut = NULL;

      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, SBP_ROW_STATUS1, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      //Util_stopClock(&periodicClock);

        iRecivedLen = 0;
        iExpectedLen = 0;
        iSendedLen = 0;
        if(pBuffIn != NULL) free(pBuffIn);
        pBuffIn = NULL;
        if(pBuffOut != NULL) free (pBuffOut);
        pBuffOut = NULL;

        timeShutdown = Seconds_get() + cur_tag_settings.timeut_conn;

      SimplePeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, SBP_ROW_CONN_STATUS, 0, "Disconnected");

      // Cancel the OAD if one is going on
      // A disconnect forces the peer to re-identify
      OAD_cancel();

      // Clear remaining lines
      Display_clearLines(dispHandle, SBP_ROW_CONN_STATUS, 5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimplePeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, SBP_ROW_STATUS1, 0, "Timed Out");

      // Clear remaining lines
      Display_clearLines(dispHandle, SBP_ROW_CONN_STATUS, 5);

      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, SBP_ROW_STATUS1, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, SBP_ROW_STATUS1);
      break;
  }
}

//#ifdef PLUS_OBSERVER
/*********************************************************************
 * @fn      Util_convertBytes2Str
 *
 * @brief   Convert bytes to string. Used to print advertising data.
 *
 * @param   pData - data
 * @param   length - data length
 *
 * @return  Adv/Scan data as a string
 */
char *Util_convertBytes2Str(uint8_t *pData, uint8_t length)
{
  uint8_t     charCnt;
  char        hex[] = "0123456789ABCDEF";
  static char str[(3*31)+1];
  char        *pStr = str;

  for (charCnt = 0; charCnt < length && charCnt < 31; charCnt++)
  {
    *pStr++ = hex[*pData >> 4];
    *pStr++ = hex[*pData++ & 0x0F];
    *pStr++ = ':';
  }
  pStr = NULL;

  return str;
}

/*********************************************************************
 * @fn      SimpleBLEPeripheralObserver_processRoleEvent
 *
 * @brief   Peripheral Observer role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLEPeripheralObserver_processRoleEvent(gapPeriObsRoleEvent_t *pEvent)
{
    switch (pEvent->gap.opcode)
    {
    case GAP_DEVICE_INFO_EVENT:

        if(pEvent->deviceInfo.rssi > cur_tag_settings.treshold_tag) //сигнал сильнее порога
        {
            //memcpy(&CurrBaseBdAddr, pEvent->deviceInfo.addr, 6);
            //Print scan response data otherwise advertising data
            if(pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP)
            {
                //Display_print1(dispHandle, 4, 0, "Scan Response Addr: %s", Util_convertBdAddr2Str(pEvent->deviceInfo.addr));
                //Display_print1(dispHandle, 5, 0, "Scan Response Data: %s", Util_convertBytes2Str(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen));
            }
            else
            {
                Display_print2(dispHandle, 6, 0, "Advertising Addr: %s Advertising Type: %s", Util_convertBdAddr2Str(pEvent->deviceInfo.addr), AdvTypeStrings[pEvent->deviceInfo.eventType]);
                Display_print1(dispHandle, 7, 0, "Advertising Data: %s", Util_convertBytes2Str(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen));

                WorkWithDiscoBase(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen);
            }
        }

        ICall_free(pEvent->deviceInfo.pEvtData);
        ICall_free(pEvent);

        break;

    case GAP_DEVICE_DISCOVERY_EVENT:
        // discovery complete
        scanningStarted = FALSE;

        //Display_print0(dispHandle, 7, 0, "GAP_DEVICE_DISC_EVENT");
        //Display_print1(dispHandle, 5, 0, "Devices discovered: %d", pEvent->discCmpl.numDevs);
        //Display_print0(dispHandle, 4, 0, "Scanning Off");

        ICall_free(pEvent->discCmpl.pDevList);
        ICall_free(pEvent);

        break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheralObserver_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_RIGHT
 *                 KEY_LEFT
 *
 * @return  none
 */
static void SimpleBLEPeripheralObserver_handleKeys(uint8_t keys)
{
    timeShutdown = Seconds_get() + cur_tag_settings.timeut_conn;

    if (keys & KEY_RIGHT)
    {

        SendToBlink(PRF_START_STATION);

        return;
    }

    if (keys & KEY_LEFT)
    {
        //uint8 status;

        if(cur_tag_settings.mode_tag == MODE_SLEEP)
        {
            uint8_t adv_enabled;
            adv_enabled = TRUE;
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof (uint8_t), & adv_enabled);
            Util_startClock(&periodicClock);
            ChangeWorkMode(previonsMode);
        }
        return;
    }

    if (keys & KEY_LEFT_LONG)
    {
        //Display_print0(dispHandle, 4, 0, "Button left long");

        if(cur_tag_settings.mode_tag == MODE_CONNECT)
        {
            ChangeWorkMode(MODE_RUN);
            return;
        }
        if(cur_tag_settings.mode_tag == MODE_RUN)
        {
            ChangeWorkMode(MODE_CONNECT);
            return;
        }
    }

    if (keys & KEY_LEFT_VERYLONG)
    {
        //Display_print0(dispHandle, 4, 0, "Button left very long");
        ChangeWorkMode(MODE_SLEEP);
        return;
    }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheralObserver_StateChangeCB
 *
 * @brief   Peripheral observer event callback function.
 *
 * @param   pEvent - pointer to event structure
 */
static void SimpleBLEPeripheralObserver_StateChangeCB(gapPeriObsRoleEvent_t *pEvent)
{
  switch(pEvent->gap.opcode)
  {
    case GAP_DEVICE_INFO_EVENT:
    {
      gapDeviceInfoEvent_t *pDevInfoMsg;

      if (pDevInfoMsg = ICall_malloc(sizeof(gapDeviceInfoEvent_t)))
      {
        memcpy(pDevInfoMsg, pEvent, sizeof(gapDeviceInfoEvent_t));

        if (pDevInfoMsg->pEvtData = ICall_malloc(pEvent->deviceInfo.dataLen))
        {
          memcpy(pDevInfoMsg->pEvtData, pEvent->deviceInfo.pEvtData,
                pEvent->deviceInfo.dataLen);
          SimplePeripheral_enqueueMsg(SBP_OBSERVER_STATE_EVT, SUCCESS,
                                        (uint8 *)pDevInfoMsg);
        }
        else
        {
          ICall_freeMsg(pDevInfoMsg);
        }
      }
      break;
    }

    case GAP_DEVICE_DISCOVERY_EVENT:
    {
      gapDevDiscEvent_t *pDevDiscMsg;

      if (pDevDiscMsg = ICall_malloc(sizeof(gapDevDiscEvent_t)))
      {
        memcpy(pDevDiscMsg, pEvent, sizeof(gapDevDiscEvent_t));

        if (pDevDiscMsg->pDevList = ICall_malloc((pEvent->discCmpl.numDevs)*
                                                  sizeof(gapDevRec_t)))
        {
          memcpy(pDevDiscMsg->pDevList, pEvent->discCmpl.pDevList,
                (pEvent->discCmpl.numDevs)*sizeof(gapDevRec_t));
          SimplePeripheral_enqueueMsg(SBP_OBSERVER_STATE_EVT, SUCCESS,
                                        (uint8 *)pDevDiscMsg);
        }
        else
        {
          ICall_freeMsg(pDevDiscMsg);
        }
      }
      break;
    }

    default:
      break;
  }

  // Free the stack message
  ICall_freeMsg(pEvent);
}
//#endif

/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramID)
{
  //uint8_t newValue;
    uint8_t buff[MYDATATRANSFER_MYBUFOUT1_LEN] = {0};
    uint16_t len = 0; //пока не используется

    switch(paramID)
    {
//    case SIMPLEPROFILE_CHAR1:
//        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);
//
//        Display_print1(dispHandle, 4, 0, "Char 1: %d", (uint16_t)newValue);
//        break;
//
//    case SIMPLEPROFILE_CHAR3:
//        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
//
//        Display_print1(dispHandle, 4, 0, "Char 3: %d", (uint16_t)newValue);
//        break;

    case MYDATATRANSFER_MYBUFOUT1_ID:
        if(MyDataTransfer_GetParameter(MYDATATRANSFER_MYBUFOUT1_ID, &len, buff) == SUCCESS)
        {
            Display_print1(dispHandle, 4, 0, "bufout1: %x", (char *)*buff);
            WorkWithInputBuffer(buff, MYDATATRANSFER_MYBUFOUT1_LEN);
        }
        break;

    case MYDATATRANSFER_MYBUFOUT2_ID:
        if(MyDataTransfer_GetParameter(MYDATATRANSFER_MYBUFOUT2_ID, &len, buff) == SUCCESS)
            Display_print1(dispHandle, 4, 0, "bufout2: %x", (char *)*buff);
        break;

    default:
        // should not reach here!
        break;
    }
}

/*********************************************************************
* @fn      SimplePeripheral_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void SimplePeripheral_processPasscode(gapPasskeyNeededEvent_t *pData)
{
  // Use static passcode
  uint32_t passcode = 111111;
  Display_print1(dispHandle, SBP_ROW_SECURITY, 0, "Passcode: %d", passcode);
  // Send passcode to GAPBondMgr
  GAPBondMgr_PasscodeRsp(pData->connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
* @fn      SimplePeripheral_handleKeys
*
* @brief   Handles all key events for this device.
*
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
//static void SimplePeripheral_handleKeys(uint8_t keys)
//{
//  if (keys & KEY_LEFT)
//  {
//    // Check if the key is still pressed
//    if (PIN_getInputValue(Board_BUTTON0) == 0)
//    {
//      //tbm_buttonLeft();
//    }
//  }
//  else if (keys & KEY_RIGHT)
//  {
//    // Check if the key is still pressed
//    if (PIN_getInputValue(Board_BUTTON1) == 0)
//    {
//      //tbm_buttonRight();
//    }
//  }
//}

/*********************************************************************
 * Callback Functions - These run in the calling thread's context
 *********************************************************************/

 /*********************************************************************
* @fn      SimplePeripheral_passcodeCB
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void SimplePeripheral_passcodeCB(uint8_t *deviceAddr,
                                            uint16_t connHandle,
                                            uint8_t uiInputs, uint8_t uiOutputs,
                                            uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    SimplePeripheral_enqueueMsg(SBP_PASSCODE_NEEDED_EVT, NULL, (uint8_t *) pData);
  }
}


/*********************************************************************
* @fn      SimplePeripheral_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   keys - bitmask of keys pressed
*
* @return  none
*/
static void SimplePeripheral_keyChangeHandler(uint8_t keys)
{
  SimplePeripheral_enqueueMsg(SBP_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimplePeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimplePeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_charValueChangeCB(uint8_t paramID)
{
  SimplePeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD reset service
 *
 * @param   connHandle - the connection Handle this request is from.
 * @param   bim_var    - bim_var to set before resetting.
 *
 * @return  None.
 */
void SimplePeripheral_processOadWriteCB(uint8_t event, uint16_t arg)
{
  Event_post(syncEvent, event);
}

/*********************************************************************
 * SWI Functions - These functions run at higher priority than any task
 *********************************************************************/

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if( SimplePeripheral_enqueueMsg(SBP_CONN_EVT, 0 ,(uint8_t *) pReport) == FALSE)
  {
    ICall_free(pReport);
  }

}

/*********************************************************************
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimplePeripheral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbpEvt_t *pMsg = ICall_malloc(sizeof(sbpEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}
/*********************************************************************/

static void TimeFunction(void)
{
    //uint32_t sec;

    //time_t t;

    uint32_t timeNow = 1550062258;

    Seconds_set(timeNow);

    //while( (sec = Seconds_get()) != timeNow + 5) {} // <- This actually waits for 5 seconds!

    //t   = time(NULL);



//    uint8 ownAddress[B_ADDR_LEN];
//    GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
//    // Display device address
//    Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));

//      HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_MINUS_21_DBM);
//      Вы можете найти HCI_EXT_SetTxPowerCmd в hci.h. Вы должны использовать эту функцию в SimpleBLEPeripheral_init ().
}

void beep(uint16_t ms, uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++)
    {

        Board_setLed0_my(HIGH);
        Board_setLed1_my(HIGH);

        Task_sleepMS(ms);
        //Watchdog.reset();

        Board_setLed0_my(LOW);
        Board_setLed1_my(LOW);

        if (i < n - 1)
        {
            Task_sleepMS(ms);
        }
    }
}

static uint8_t min(uint8_t a, uint8_t b)
{
    if(a < b) return a;
    return b;
}

static bool ChangeAdvertisingData(void)
{
    char* buf = NULL;
    uint8_t lenbuf = 20;
    bStatus_t status = 0;

    buf = malloc(lenbuf);
    if(buf == NULL) return false;
    memset(buf, ' ', lenbuf);

    //if(strlen(cur_tag_settings.fam) == 0)   //в рекламе не должно быть завершающих нулей
    //{
        memcpy(buf, cur_tag_settings.name_tag, min(strlen(cur_tag_settings.name_tag), lenbuf));
    //}
    //else
    //{
        //memcpy(buf, cur_tag_settings.fam, min(strlen(cur_tag_settings.fam), lenbuf));
    //}

    memcpy(scanRspData + 2, buf, lenbuf);

    memset(attDeviceName, 0, GAP_DEVICE_NAME_LEN);  //а здесь строка завершается 0
    memcpy(attDeviceName, buf, lenbuf);

    free(buf);


    status = GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);

    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    if(status == SUCCESS)
    {
        //Board_setLed0_my(Board_getLed_my()^1);
        return true;
    }
    return false;
}

static void ChangeWorkMode(WORKMODE mode)
{
    uint8 status;
    uint8_t adv_enabled;

    switch(mode)
    {
    case MODE_CONNECT:
        Display_print0(dispHandle, 8, 0, "MODE_CONNECT");
        previonsMode = cur_tag_settings.mode_tag;
        if(scanningStarted == TRUE) status = GAPRole_CancelDiscovery();
        if(status == SUCCESS)
        {
            Display_print0(dispHandle, 4, 0, "CancelDiscovery");
        }
        cur_tag_settings.mode_tag = MODE_CONNECT;

        adv_enabled = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof (uint8_t), & adv_enabled);

        SendToBlink(PRF_MODE_CONNECT);
        break;

    case MODE_RUN:
        Display_print0(dispHandle, 8, 0, "MODE_RUN");
        previonsMode = cur_tag_settings.mode_tag;
        // Отключаем соединение
        GAPRole_TerminateConnection();
        // Остановить рекламу
        adv_enabled = FALSE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof (uint8_t), & adv_enabled);
        cur_tag_settings.mode_tag = MODE_RUN;
        //Start scanning if not already scanning
        if (scanningStarted == FALSE)
        {
            status = GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE, DEFAULT_DISCOVERY_ACTIVE_SCAN, DEFAULT_DISCOVERY_WHITE_LIST);

            if(status == SUCCESS)
            {
                scanningStarted = TRUE;
                Display_print0(dispHandle, 4, 0, "Scanning On");
            }
            else
            {
                Display_print1(dispHandle, 4, 0, "Scanning failed: %d", status);
            }
        }
        SendToBlink(PRF_MODE_RUN);
        break;

    case MODE_SLEEP:
        Display_print0(dispHandle, 8, 0, "MODE_SLEEP");
        cur_tag_settings.mode_tag = MODE_SLEEP;
        MyPowerDown();
        break;

    default:
        break;
    }

    SendToBlink(PRF_CHANGE_MODE);
}

//извлекаем из принятой рекламы номер станции и время на станции
// логика работы в номерами станций
static void WorkWithDiscoBase(uint8_t * pBuf, uint8_t len)
{
    uint8_t tmp[5] = {'B', 'A', 'S', 'E', '_' };
    uint8_t i = 0;
    uint8_t numCurrBase = 0;
    uint32_t timeCurrBase = 0;

    if(len < 11) return;

    for(i=0; i < (len - 5); i++)
    {
        if(memcmp(pBuf + i, &tmp, 5) == 0)
        {
            numCurrBase = *(pBuf + i + 5);
            memcpy((void *)&timeCurrBase, (void *)(pBuf + i + 7), sizeof(uint32_t));
            break;
        }
    }

    if(numCurrBase == 0) return;

    timeShutdown = Seconds_get() + cur_tag_settings.timeut_conn;

    switch (numCurrBase)
    {
    case START_STATION_NUM:     //стартовую базу фиксируем при ВЫХОДЕ из зоны приема, запись происходит при обнаружении следующей базы
        NumLastBase = numCurrBase;
        TimeLastBase = timeCurrBase;
        SendToBlink(PRF_START_STATION);
        break;

    case FINISH_STATION_NUM:
        if(NumLastBase == START_STATION_NUM) AddBaseToList(NumLastBase, TimeLastBase);
        if(NumLastBase != 0)    //если были базы до этой
        {
            AddBaseToList(numCurrBase, timeCurrBase);
            if(currPosBaseTable > 3)
            {
                SaveBaseTable();
                NumLastBase = 0;
                TimeLastBase = 0;
                memset(arBaseTable, 0, LEN_AR_BASE_TABLE);
                currPosBaseTable = 0;
                SendToBlink(PRF_FINISH_STATION);

                Task_sleepMS(5000);
                ChangeWorkMode(MODE_CONNECT);
                break;
            }
            SendToBlink(PRF_FINISH_STATION);
        }
        break;

    case CHECK_STATION_NUM:
        if((timeCurrBase - TimeLastBase > BASE_SAVE_SERVICE_TIMEOUT)||(numCurrBase != NumLastBase))
        {
            NumLastBase = numCurrBase;
            TimeLastBase = timeCurrBase;
            //AddBaseToList(numCurrBase, timeCurrBase);
            SendToBlink(PRF_NORMAL_STATION);
        }
        break;

    case CLEAR_STATION_NUM:
        if((timeCurrBase - TimeLastBase > BASE_SAVE_SERVICE_TIMEOUT)||(numCurrBase != NumLastBase))
        {
            NumLastBase = numCurrBase;
            TimeLastBase = timeCurrBase;
            memset(arBaseTable, 0, LEN_AR_BASE_TABLE);
            currPosBaseTable = 0;
            SendToBlink(PRF_SIMPLEBLINK);
        }
        break;

    default:
        if((timeCurrBase - TimeLastBase > BASE_SAVE_TIMEOUT)||(numCurrBase != NumLastBase))
        {
            if(NumLastBase == START_STATION_NUM) AddBaseToList(NumLastBase, TimeLastBase);

            NumLastBase = numCurrBase;
            TimeLastBase = timeCurrBase;
            AddBaseToList(numCurrBase, timeCurrBase);
            SendToBlink(PRF_NORMAL_STATION);
        }
        break;
    }
}

//записываем номер и время базы в таблицу ОЗУ
static void AddBaseToList(uint8_t numBase, uint32_t timeBase)
{
    uint8_t pageData[4];

    pageData[0] = numBase;
    pageData[1] = (timeBase & 0x00FF0000)>>16;  //порядок старших - младших???
    pageData[2] = (timeBase & 0x0000FF00)>>8;
    pageData[3] = (timeBase & 0x000000FF);

    if(currPosBaseTable >= LEN_AR_BASE_TABLE - sizeof(pageData))
    {
        //Display_printf(dispHandle, 0, 0, "Save table. End of block."); //test
        SaveBaseTable();        //дошли до конца блока
        memset(arBaseTable, 0, LEN_AR_BASE_TABLE);
        currPosBaseTable = 0;
    }

    if (currPosBaseTable == 0)          //Начало блока - время старта 4 байта. Далее по 4 байта: номер станции и три байта времени.
    {
        //Display_printf(dispHandle, 0, 0, "Init table."); //test
        memcpy(arBaseTable, (void *)&timeBase, sizeof(timeBase));
        currPosBaseTable += sizeof(timeBase);
    }

    memcpy(arBaseTable + currPosBaseTable, &pageData, sizeof(pageData));
    currPosBaseTable += sizeof(pageData);
}

//Первый блок EPROM для настроек. Начиная со второго для результатов.
//Начало блока - время старта 4 байта. Далее по 4 байта: номер станции и три байта времени.
//Сначала пишем в озу, как блок готов или завершён забег сдвигаем блоки 2->3 и тд, переписываем в пзу.
//При уходе в сон если начат блок надо сохранить.
static void SaveBaseTable(void)
{

    MoveEpromBlock(2);
    WriteEprom_inter_osal((uint8_t *)arBaseTable, LEN_AR_BASE_TABLE, 2);

//    uint8_t * tmpBuf = malloc(LEN_AR_BASE_TABLE); test only!
//
//    ReadEprom_inter_osal(tmpBuf, LEN_AR_BASE_TABLE, 2);
//
//    free(tmpBuf);
}

//сдвигаем блоки ПЗУ (256 байт) начиная с strtBlk вправо. Последние исчезают.
static bool MoveEpromBlock(uint8_t strtBlk)
{
    uint8_t MAXBLOKSEPROM = 15;
    uint8_t iNumCurrBlock = 0;

    if(strtBlk > MAXBLOKSEPROM) return false;

    uint8_t * tmpBuf = malloc(LEN_AR_BASE_TABLE);
    if(tmpBuf == NULL) return false;

    for(iNumCurrBlock = MAXBLOKSEPROM - 1; iNumCurrBlock >= strtBlk; iNumCurrBlock -= 1)
    {
        ReadEprom_inter_osal(tmpBuf, LEN_AR_BASE_TABLE, iNumCurrBlock);
        WriteEprom_inter_osal(tmpBuf, LEN_AR_BASE_TABLE, iNumCurrBlock + 1);
        Display_printf(dispHandle, 0, 0, "Shift blok."); //test
    }

    free(tmpBuf);
    return true;
}

//POWER_DOWN
void MyPowerDown(void)
{
    uint8_t adv_enabled;

    //Останавливаем таймер
    Util_stopClock(&periodicClock);

    Board_setLed0_my(0);
    Board_setLed1_my(0);
    Board_setVibro(0);

    // Отключаем соединение
    GAPRole_TerminateConnection();

    // Остановить рекламу
    adv_enabled = FALSE;
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof (uint8_t), & adv_enabled);

    if(scanningStarted == TRUE) GAPRole_CancelDiscovery();

    Display_printf(dispHandle, 0, 0, "Stop task BLE.");

    if(currPosBaseTable != 0) SaveBaseTable();

    Task_sleepMS(100);

    Board_shutDownExtFlash();

    ExtFlash_open();
    ExtFlash_close();

    Display_printf(dispHandle, 0, 0, "Shutdown.");
    Display_close(dispHandle);
    Task_sleepMS(5000);
    Board_Power_down();
}



