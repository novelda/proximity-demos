/**
 * @file proximity_service.c
 * @brief BLE profile interacting with higher level APIs for the Novelda X4F103 sensor.
 *
 * This file contains the implementation of a BLE profile responsible for configuring
 * the Novelda X4F103 sensor and relaying information about presence detection.
 */

#include <string.h>
#include <icall.h>
#include "icall_ble_api.h"
#include "proximity.h"
#include "proximity_service.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>

void ProximityProfile_callback(uint8_t paramID);
void ProximityProfile_invokeFromFWContext(char *pData);

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Proximity Profile Service UUID: 0xFFF0
GATT_BT_UUID(proximityProfile_ServUUID, PROXIMITYPROFILE_SERV_UUID);

// Characteristic UUIDs
GATT_BT_UUID(proximityProfile_DetectionUUID, PROXIMITYPROFILE_DETECTION_UUID);
GATT_BT_UUID(proximityProfile_RangeUUID, PROXIMITYPROFILE_RANGE_UUID);
GATT_BT_UUID(proximityProfile_SensitivityUUID, PROXIMITYPROFILE_SENSITIVITY_UUID);
GATT_BT_UUID(proximityProfile_TimeoutUUID, PROXIMITYPROFILE_TIMEOUT_UUID);


/*********************************************************************
 * LOCAL VARIABLES
 */
static ProximityProfile_CBs_t *proximityProfile_appCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Proximity Profile Service attribute
static CONST gattAttrType_t proximityProfile_Service = { ATT_BT_UUID_SIZE, proximityProfile_ServUUID };

// Characteristic Properties
static uint8_t proximityProfile_DetectionProps = GATT_PROP_NOTIFY | GATT_PROP_READ;
static uint8_t proximityProfile_RangeProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8_t proximityProfile_SensitivityProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8_t proximityProfile_TimeoutProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

static gattCharCfg_t *proximityProfile_DetectionConfig;

// Characteristic Values
static uint8_t proximityProfile_Detection = 0;
static uint16_t proximityProfile_Range = 0;
static uint8_t proximityProfile_Sensitivity = 0;
static uint16_t proximityProfile_Timeout = 0;

// Characteristic User Descriptions
static uint8_t proximityProfile_DetectionUserDesp[] = "Detection";
static uint8_t proximityProfile_RangeUserDesp[] = "Range";
static uint8_t proximityProfile_SensitivityUserDesp[] = "Sensitivity";
static uint8_t proximityProfile_TimeoutUserDesp[] = "Timeout";

/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t proximityProfile_attrTbl[] =
{
    // Proximity Profile Service
    GATT_BT_ATT(primaryServiceUUID, GATT_PERMIT_READ, (uint8_t *)&proximityProfile_Service),

    // Proximity Characteristic Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &proximityProfile_DetectionProps),
    // Proximity Characteristic Value
    GATT_BT_ATT(proximityProfile_DetectionUUID, GATT_PERMIT_READ, &proximityProfile_Detection),
    GATT_BT_ATT( clientCharCfgUUID,            GATT_PERMIT_READ | GATT_PERMIT_WRITE,  (uint8_t *) &proximityProfile_DetectionConfig ),
    // Proximity Characteristic User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ, proximityProfile_DetectionUserDesp),

    // Range Characteristic Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &proximityProfile_RangeProps),
    // Range Characteristic Value
    GATT_BT_ATT(proximityProfile_RangeUUID, GATT_PERMIT_READ | GATT_PERMIT_WRITE , (uint8_t *)&proximityProfile_Range),
    // Range Characteristic User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ, proximityProfile_RangeUserDesp),

    // Sensitivity Characteristic Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &proximityProfile_SensitivityProps),
    // Sensitivity Characteristic Value
    GATT_BT_ATT(proximityProfile_SensitivityUUID, GATT_PERMIT_READ | GATT_PERMIT_WRITE , &proximityProfile_Sensitivity),
    // Sensitivity Characteristic User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ, proximityProfile_SensitivityUserDesp),

    // Timeout Characteristic Declaration
    GATT_BT_ATT(characterUUID, GATT_PERMIT_READ, &proximityProfile_TimeoutProps),
    // Timeout Characteristic Value
    GATT_BT_ATT(proximityProfile_TimeoutUUID, GATT_PERMIT_READ | GATT_PERMIT_WRITE , (uint8_t *)&proximityProfile_Timeout),
    // Timeout Characteristic User Description
    GATT_BT_ATT(charUserDescUUID, GATT_PERMIT_READ, proximityProfile_TimeoutUserDesp),
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
bStatus_t ProximityProfile_readAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr,
                                      uint8_t *pValue, uint16_t *pLen,
                                      uint16_t offset, uint16_t maxLen,
                                      uint8_t method);
bStatus_t ProximityProfile_writeAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t len,
                                       uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Proximity Profile Service Callbacks
CONST gattServiceCBs_t proximityProfile_CBs =
{
    ProximityProfile_readAttrCB,  // Read callback function pointer
    ProximityProfile_writeAttrCB, // Write callback function pointer
    NULL                          // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @brief Adds the Proximity Profile service.
 *
 * This function adds the Proximity Profile service to the GATT server.
 *
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t ProximityProfile_addService(void)
{
    uint8_t status = SUCCESS;

    proximityProfile_DetectionConfig = (gattCharCfg_t *)ICall_malloc( sizeof( gattCharCfg_t ) *
                                                                   MAX_NUM_BLE_CONNS );
    // Initialize Client Characteristic Configuration attributes
      GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, proximityProfile_DetectionConfig );

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(proximityProfile_attrTbl,
                                        GATT_NUM_ATTRS(proximityProfile_attrTbl),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &proximityProfile_CBs);

    // Return status value
    return (status);
}

/**
 * @brief Registers application callbacks for the Proximity Profile service.
 *
 * @param appCallbacks Callback functions provided by the application.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t ProximityProfile_registerAppCBs(ProximityProfile_CBs_t *appCallbacks)
{
    if (appCallbacks)
    {
        proximityProfile_appCBs = appCallbacks;
        return (SUCCESS);
    }
    else
    {
        return (bleAlreadyInRequestedMode);
    }
}


/**
 * @brief Sets a parameter for the Proximity Profile service.
 *
 * @param param Parameter to set.
 * @param len Length of the value.
 * @param value Pointer to the value to set.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t ProximityProfile_setParameter(uint8_t param, uint8_t len, void *value)
{
    bStatus_t status = SUCCESS;

    switch (param)
    {
        case PROXIMITYPROFILE_DETECTION:
            if (len == sizeof(uint8_t))
            {
                proximityProfile_Detection = *((uint8_t *)value);
                // See if Notification has been enabled
                GATTServApp_ProcessCharCfg( proximityProfile_DetectionConfig, &proximityProfile_Detection, FALSE,
                                            proximityProfile_attrTbl, GATT_NUM_ATTRS( proximityProfile_attrTbl ),
                                            INVALID_TASK_ID, ProximityProfile_readAttrCB );
            }
            else
            {
                status = bleInvalidRange;
            }
            break;

        case PROXIMITYPROFILE_RANGE:
            if (len == sizeof(uint16_t))
            {
                proximityProfile_Range = *((uint16_t *)value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;

        case PROXIMITYPROFILE_SENSITIVITY:
            if (len == sizeof(uint8_t))
            {
                proximityProfile_Sensitivity = *((uint8_t *)value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;

        case PROXIMITYPROFILE_TIMEOUT:
            if (len == sizeof(uint16_t))
            {
                proximityProfile_Timeout = *((uint16_t *)value);
            }
            else
            {
                status = bleInvalidRange;
            }
            break;

        default:
            status = INVALIDPARAMETER;
            break;
    }

    // Return status value
    return (status);
}

/**
 * @brief Gets a parameter from the Proximity Profile service.
 *
 * @param param Parameter to get.
 * @param value Pointer to store the retrieved value.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t ProximityProfile_getParameter(uint8_t param, void *value)
{
    bStatus_t status = SUCCESS;

    switch (param)
    {
        case PROXIMITYPROFILE_DETECTION:
            *((uint8_t *)value) = proximityProfile_Detection;
            break;

        case PROXIMITYPROFILE_RANGE:
            *((uint16_t *)value) = proximityProfile_Range;
            break;

        case PROXIMITYPROFILE_SENSITIVITY:
            *((uint8_t *)value) = proximityProfile_Sensitivity;
            break;

        case PROXIMITYPROFILE_TIMEOUT:
            *((uint16_t *)value) = proximityProfile_Timeout;
            break;

        default:
            status = INVALIDPARAMETER;
            break;
    }

    // Return status value
    return (status);
}

/**
 * @brief Callback for reading attributes of the Proximity Profile service.
 *
 * This function is a callback for reading attributes of the Proximity Profile service.
 *
 * @param connHandle Connection handle.
 * @param pAttr Pointer to the attribute.
 * @param pValue Pointer to store the attribute value.
 * @param pLen Pointer to store the attribute length.
 * @param offset Offset for attribute reading.
 * @param maxLen Maximum length of the attribute.
 * @param method Method of reading the attribute.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t ProximityProfile_readAttrCB(uint16_t connHandle,
                                      gattAttribute_t *pAttr,
                                      uint8_t *pValue, uint16_t *pLen,
                                      uint16_t offset, uint16_t maxLen,
                                      uint8_t method)
{
    bStatus_t status = SUCCESS;

    // Make sure it's not a blob operation (no attributes in the profile are long)
    if (offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }

    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch (uuid)
        {
            case PROXIMITYPROFILE_DETECTION_UUID:
            case PROXIMITYPROFILE_SENSITIVITY_UUID:
                *pLen = 1;
                pValue[0] = *pAttr->pValue;
                break;

            case PROXIMITYPROFILE_RANGE_UUID:
            case PROXIMITYPROFILE_TIMEOUT_UUID:
                *pLen = sizeof(uint16_t);
                VOID memcpy(pValue, pAttr->pValue, sizeof(uint16_t));
                break;

            default:
                *pLen = 0;
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else
    {
        // 128-bit UUID
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
    }

    // Return status value
    return (status);
}

/**
 * @brief Callback for writing attributes of the Proximity Profile service.
 *
 * This function is a callback for writing attributes of the Proximity Profile service.
 *
 * @param connHandle Connection handle.
 * @param pAttr Pointer to the attribute.
 * @param pValue Pointer to the value to be written.
 * @param len Length of the value.
 * @param offset Offset for attribute writing.
 * @param method Method of writing the attribute.
 * @return Status of the operation (SUCCESS or an error code).
 */
bStatus_t ProximityProfile_writeAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t len,
                                       uint16_t offset, uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t notifyApp = 0xFF;

    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        switch (uuid)
        {
            case PROXIMITYPROFILE_RANGE_UUID:
            case PROXIMITYPROFILE_SENSITIVITY_UUID:
            case PROXIMITYPROFILE_TIMEOUT_UUID:
                // Validate the value
                // Make sure it's not a blob oper
                if (offset == 0)
                {
                    if (len != 1 && len != sizeof(uint16_t))
                    {
                        status = ATT_ERR_INVALID_VALUE_SIZE;
                    }
                }
                else
                {
                    status = ATT_ERR_ATTR_NOT_LONG;
                }
                //Reset back to 0 the previous value of the attr
                // for the case where the previous value was len=2 and the new value is length 1
                if(uuid != PROXIMITYPROFILE_SENSITIVITY_UUID)
                {
                    pAttr->pValue[0] = 0;
                    pAttr->pValue[1] = 0;
                }
                // this takes care of the case where a longer than len == 1 value tries to be written into sensitivity attr
                else if(len == 2)
                {
                    status = ATT_ERR_INVALID_VALUE_SIZE;
                }
                // Write the value
                if (status == SUCCESS)
                {
                    if (len == 1)
                    {
                        uint8_t *pCurValue = (uint8_t *)pAttr->pValue;
                        *pCurValue = pValue[0];
                    }
                    else
                    {
                        uint16_t *pCurValue = (uint16_t *)pAttr->pValue;
                        *pCurValue = BUILD_UINT16(pValue[0], pValue[1]);
                    }
                }
                break;

            case GATT_CLIENT_CHAR_CFG_UUID:
                status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                         offset, GATT_CLIENT_CFG_NOTIFY );
                break;

            default:
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
        switch (uuid)
                {
                    case PROXIMITYPROFILE_RANGE_UUID:
                        notifyApp = PROXIMITYPROFILE_RANGE;
                        break;
                    case PROXIMITYPROFILE_SENSITIVITY_UUID:
                        notifyApp = PROXIMITYPROFILE_SENSITIVITY;
                        break;
                    case PROXIMITYPROFILE_TIMEOUT_UUID:
                        notifyApp = PROXIMITYPROFILE_TIMEOUT;
                        break;
                    case GATT_CLIENT_CHAR_CFG_UUID:
                        notifyApp = PROXIMITYPROFILE_DETECTION;
                        break;

                        break;

                }
    }
    else
    {
        // 128-bit UUID
        status = ATT_ERR_INVALID_HANDLE;
    }

    // If a characteristic value changed then callback function to notify application of change
    if ((notifyApp != 0xFF ) && proximityProfile_appCBs && proximityProfile_appCBs->pfnProximityProfile_Change)
    {
        ProximityProfile_callback(notifyApp);
    }

    // Return status value
    return (status);
}

/**
 * @brief Handler for the Connect event.
 *
 * This function handles the Connect event by starting the sensor.
 */
void ProximityProfile_on_connect(void)
{
    startSensor();
}

/**
 * @brief Handler for the Disconnect event.
 *
 * This function handles the Disconnect event by stopping the sensor if no active connections are present.
 */
void ProximityProfile_on_disconnect(void)
{
    if(linkDB_NumActive() > 0)
    {
        stopSensor();
    }
}

/**
 * @brief Callback function for the Proximity Profile service.
 *
 * This function is a callback for the Proximity Profile service.
 *
 * @param paramID Parameter ID for the callback.
 */
void ProximityProfile_callback(uint8_t paramID)
{
    char *pData = ICall_malloc(sizeof(char));

    if (pData == NULL)
    {
        return;
    }

    pData[0] = paramID;

    BLEAppUtil_invokeFunction(ProximityProfile_invokeFromFWContext, pData);
}

/**
 * @brief Invokes the Proximity Profile from the firmware context.
 *
 * This function invokes the Proximity Profile from the firmware context.
 *
 * @param pData Data to be passed to the invoked function.
 */
void ProximityProfile_invokeFromFWContext(char *pData)
{
    proximityProfile_appCBs->pfnProximityProfile_Change(pData[0]);
}
