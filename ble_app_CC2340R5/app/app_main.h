/******************************************************************************

@file  app_main.h

Group: WCS, BTS
$Target Device: DEVICES $

******************************************************************************
$License: BSD3 2022 $
******************************************************************************
$Release Name: PACKAGE NAME $
$Release Date: PACKAGE RELEASE DATE $
*****************************************************************************/

#ifndef APP_MAIN_H_
#define APP_MAIN_H_

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

//*****************************************************************************
//! Defines
//*****************************************************************************
Display_Handle handle;

//*****************************************************************************
//! Typedefs
//*****************************************************************************
PACKED_ALIGNED_TYPEDEF_STRUCT
{
  /// Type of TargetA address in the directed advertising PDU
  uint8_t  addressType;
  /// TargetA address
  BLEAppUtil_BDaddr  address;
}App_scanResults;

// Connected device information
PACKED_ALIGNED_TYPEDEF_STRUCT
{
  uint16_t  connHandle;             // Connection Handle
  BLEAppUtil_BDaddr peerAddress;    // The address of the peer device
} App_connInfo;

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Peripheral_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the peripheral
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Peripheral_start(void);

/*********************************************************************
 * @fn      Broadcaster_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the broadcaster
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Broadcaster_start(void);

/*********************************************************************
 * @fn      Central_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the central
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Central_start(void);

/*********************************************************************
 * @fn      Observer_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the observer
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Observer_start(void);

/*********************************************************************
 * @fn      Connection_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the connection
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Connection_start(void);

/*********************************************************************
 * @fn      Pairing_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the pairing
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Pairing_start(void);

/*********************************************************************
 * @fn      Data_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the data
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Data_start(void);

/*********************************************************************
 * @fn      DevInfo_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Device Info service.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t DevInfo_start(void);

/*********************************************************************
 * @fn      SimpleGatt_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Simple GATT profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t Proximity_start(void);

/*********************************************************************
 * @fn      OAD_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the OAD profile and service.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t OAD_start(void);

/*********************************************************************
 * @fn      DataStream_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Data Stream profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t DataStream_start(void);

/*********************************************************************
 * @fn      CGM_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the CGM profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t CGM_start(void);

/*********************************************************************
 * @fn      GL_start

 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Glucose profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t GL_start(void);

/*********************************************************************
 * @fn      Scan_getScanResList
 *
 * @brief   Get the scan result list
 *
 * @param   scanRes - a scan list pointer
 *
 * @return  The number of results in the list
 */
uint8 Scan_getScanResList(App_scanResults **scanRes);

/*********************************************************************
 * @fn      Connection_getConnList
 *
 * @brief   Get the connection list
 *
 * @return  connection list
 */
App_connInfo *Connection_getConnList(void);

/*********************************************************************
 * @fn      Connection_getConnhandle
 *
 * @brief   Find connHandle in the connected device list by index
 *
 * @return  the connHandle found. If there is no match,
 *          MAX_NUM_BLE_CONNS will be returned.
 */
uint16_t Connection_getConnhandle(uint8_t index);

/*********************************************************************
 * @fn      Connection_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, LL_INACTIVE_CONNECTIONS will be returned.
 */
uint16_t Connection_getConnIndex(uint16_t connHandle);

#endif /* APP_MAIN_H_ */
