/**
 * @file proximity_service.c
 * @brief Implementation of a custom Bluetooth Low Energy (BLE) service for proximity Sensor.
 *
 * This file contains the implementation of a custom BLE service tailored for an proximity sensor.
 * It defines the service characteristics and handles various BLE events such as connections,
 * disconnections, and attribute writes. This service allows interaction with the proximity sensor
 * and its related settings.
 */


#include "proximity.h"
#include "proximity_service.h"
#include <string.h>
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "nrf_log.h"



static uint8_t proximityProfile_DetectionUserDesc[] = "Detection";
static uint8_t proximityProfile_RangeUserDesc[] = "Range";
static uint8_t proximityProfile_SensitivityUserDesc[] = "Sensitivity";
static uint8_t proximityProfile_TimeoutUserDesc[] = "Timeout";


/**
 * @brief Function for adding the Detection characteristic.
 *
 * This function initializes the characteristics for the detection.
 *
 * @param[in] p_occu Pointer to the proximity Service structure.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
static ret_code_t proximity_char_add(ble_proximity_service_t* p_occu)
{
    static uint8_t g_occu_val = 0;
    ble_add_char_params_t char_params;
    memset(&char_params, 0, sizeof(ble_add_char_params_t));

    // Set the required parameters
    char_params.uuid_type = BLE_UUID_TYPE_BLE;
    char_params.uuid = BLE_UUID_DETECTION_CHAR;
    char_params.char_props.read = 1;
    char_params.char_props.notify = 1;
    char_params.read_access = SEC_OPEN;
    char_params.write_access = SEC_OPEN;
    char_params.cccd_write_access = SEC_OPEN;
    char_params.max_len = sizeof(uint8_t);
    char_params.init_len = sizeof(uint8_t);
    char_params.p_init_value = &g_occu_val;

    ble_add_char_user_desc_t user_desc;
    memset(&user_desc, 0, sizeof(ble_add_char_user_desc_t));
    user_desc.max_size = sizeof(proximityProfile_DetectionUserDesc);
    user_desc.size = sizeof(proximityProfile_DetectionUserDesc);
    user_desc.p_char_user_desc = proximityProfile_DetectionUserDesc;
    user_desc.char_props.read = 1;
    user_desc.read_access = SEC_OPEN;

    char_params.p_user_descr = &user_desc;


    return characteristic_add(p_occu->service_handle,
                              &char_params,
                              &p_occu->detection_value_handles);

}

/**
 * @brief Function for adding the Range characteristic.
 *
 * This function initializes the characteristics for the range value.
 *
 * @param[in] p_occu Pointer to the proximity Service structure.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
static ret_code_t range_char_add(ble_proximity_service_t* p_occu)
{
    uint16_t init_range = getRange();
    ble_add_char_params_t char_params;
    memset(&char_params, 0, sizeof(ble_add_char_params_t));

    // Set the required parameters
    char_params.uuid_type = BLE_UUID_TYPE_BLE;
    char_params.uuid = BLE_UUID_RANGE_CHAR;
    char_params.char_props.read = 1;
    char_params.char_props.write = 1;
    char_params.char_props.write_wo_resp = 1;
    char_params.read_access = SEC_OPEN;
    char_params.write_access = SEC_OPEN;
    char_params.cccd_write_access = SEC_OPEN;
    char_params.max_len = sizeof(uint16_t);
    char_params.init_len = sizeof(uint16_t);
    char_params.p_init_value = (uint8_t*)&init_range;

    ble_add_char_user_desc_t user_desc;
    memset(&user_desc, 0, sizeof(ble_add_char_user_desc_t));
    user_desc.max_size = sizeof(proximityProfile_RangeUserDesc);
    user_desc.size = sizeof(proximityProfile_RangeUserDesc);
    user_desc.p_char_user_desc = proximityProfile_RangeUserDesc;
    user_desc.char_props.read = 1;
    user_desc.read_access = SEC_OPEN;

    char_params.p_user_descr = &user_desc;

    return characteristic_add(p_occu->service_handle,
                              &char_params,
                              &p_occu->range_handles);
}

/**
 * @brief Function for adding the Sensitivity characteristic.
 *
 * This function initializes the characteristics for sensitivity.
 *
 * @param[in] p_occu Pointer to the proximity Service structure.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
static ret_code_t sensitivity_char_add(ble_proximity_service_t* p_occu)
{
    uint8_t init_sens = getSensitivity();
    ble_add_char_params_t char_params;
    memset(&char_params, 0, sizeof(ble_add_char_params_t));

    // Set the required parameters
    char_params.uuid_type = BLE_UUID_TYPE_BLE;
    char_params.uuid = BLE_UUID_SENSITIVITY_CHAR;
    char_params.char_props.read = 1;
    char_params.char_props.write = 1;
    char_params.char_props.write_wo_resp = 1;
    char_params.read_access = SEC_OPEN;
    char_params.write_access = SEC_OPEN;
    char_params.cccd_write_access = SEC_OPEN;
    char_params.max_len = sizeof(uint8_t);
    char_params.init_len = sizeof(uint8_t);
    char_params.p_init_value = &init_sens;

    ble_add_char_user_desc_t user_desc;
    memset(&user_desc, 0, sizeof(ble_add_char_user_desc_t));
    user_desc.max_size = sizeof(proximityProfile_SensitivityUserDesc);
    user_desc.size = sizeof(proximityProfile_SensitivityUserDesc);
    user_desc.p_char_user_desc = proximityProfile_SensitivityUserDesc;
    user_desc.char_props.read = 1;
    user_desc.read_access = SEC_OPEN;

    char_params.p_user_descr = &user_desc;

    return characteristic_add(p_occu->service_handle,
                              &char_params,
                              &p_occu->sensitivity_handles);
}

/**
 * @brief Function for adding the Timeout characteristic.
 *
 * This function initializes the characteristics for the timeout.
 *
 * @param[in] p_occu Pointer to the proximity Service structure.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
static ret_code_t timeout_char_add(ble_proximity_service_t* p_occu)
{
    uint16_t init_timeout = getTimeout();
    ble_add_char_params_t char_params;
    memset(&char_params, 0, sizeof(ble_add_char_params_t));

    // Set the required parameters
    char_params.uuid_type = BLE_UUID_TYPE_BLE;
    char_params.uuid = BLE_UUID_TIMEOUT_CHAR;
    char_params.char_props.read = 1;
    char_params.char_props.write = 1;
    char_params.char_props.write_wo_resp = 1;
    char_params.read_access = SEC_OPEN;
    char_params.write_access = SEC_OPEN;
    char_params.cccd_write_access = SEC_OPEN;
    char_params.max_len = sizeof(uint16_t);
    char_params.init_len = sizeof(uint16_t);
    char_params.p_init_value = (uint8_t*)&init_timeout;

    ble_add_char_user_desc_t user_desc;
    memset(&user_desc, 0, sizeof(ble_add_char_user_desc_t));
    user_desc.max_size = sizeof(proximityProfile_TimeoutUserDesc);
    user_desc.size = sizeof(proximityProfile_TimeoutUserDesc);
    user_desc.p_char_user_desc = proximityProfile_TimeoutUserDesc;
    user_desc.char_props.read = 1;
    user_desc.read_access = SEC_OPEN;

    char_params.p_user_descr = &user_desc;

    return characteristic_add(p_occu->service_handle,
                              &char_params,
                              &p_occu->timeout_handles);
}

/**
 * @brief Function for initializing the custom proximity service.
 *
 * This function initializes the custom BLE service for the proximity sensor.
 *
 * @param[in] p_occu        Pointer to the proximity Service structure.
 * @param[in] ble_app_sem   Pointer to the BLE application semaphore.
 * @param[in] updateSensCb  Callback function to update the sensor value.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
ret_code_t ble_proximity_service_init(ble_proximity_service_t* p_occu, void* ble_app_sem, updateSensorValueCb_t updateSensCb)
{
    ret_code_t err_code;
    ble_uuid_t service_uuid;
    if (p_occu == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Initialize service structure
    p_occu->conn_handle = BLE_CONN_HANDLE_INVALID;


    BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_PROXIMITY_SERVICE);

    // Add the service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                                   &service_uuid,
                                                   &p_occu->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add proximity Value Characteristic
    err_code = proximity_char_add(p_occu);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Range Characteristic
    err_code = range_char_add(p_occu);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Sensitivity Characteristic
    err_code = sensitivity_char_add(p_occu);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Timeout Characteristic
    err_code = timeout_char_add(p_occu);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    proximityInit(ble_app_sem, updateSensCb);

    return NRF_SUCCESS;
}

/**
 * @brief Function for updating the Detection characteristic.
 *
 * This function updates the detection characteristic and sends a notification.
 *
 * @param[in] p_occu           Pointer to the proximity Service structure.
 * @param[in] detection_value  New detection to update.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
ret_code_t ble_proximity_service_detection_update(ble_proximity_service_t* p_occu, uint8_t detection_value)
{
    ret_code_t err_code;
    ble_gatts_value_t gatts_value;
    if (p_occu == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &detection_value;


    // Update the value of the detection characteristic and send a notification
    err_code = sd_ble_gatts_value_set(p_occu->conn_handle,
                                      p_occu->detection_value_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    uint16_t len = sizeof(detection_value);
    ble_gatts_hvx_params_t hvx_params;
    if (p_occu->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = p_occu->detection_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.p_data = &detection_value;
        hvx_params.p_len  = &len;
    }

    return sd_ble_gatts_hvx(p_occu->conn_handle, &hvx_params);
}

/**
 * @brief Function for updating the Range characteristic value.
 *
 * This function updates the range characteristic value.
 *
 * @param[in] p_occu     Pointer to the proximity Service structure.
 * @param[in] new_value  New range value to update.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
ret_code_t ble_proximity_service_range_update(ble_proximity_service_t* p_occu, uint16_t new_value)
{
    ret_code_t err_code;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&new_value;

    if(new_value >= MIN_RANGE_VALUE && new_value <= MAX_RANGE_VALUE)
    {
        stopSensor();
        setRange(new_value);
        startSensor();

        NRF_LOG_INFO("Range value Updated to %d\nSensor restarted successfully.", new_value);
    }
    else
    {

        NRF_LOG_INFO("Range value %d is out of bounds.\nShould be within [%d, %d]", new_value, MIN_RANGE_VALUE, MAX_RANGE_VALUE);
        new_value = getRange();

    }
    // Update the characteristic value
    err_code = sd_ble_gatts_value_set(p_occu->conn_handle,
                                    p_occu->range_handles.value_handle,
                                    &gatts_value);

    return err_code;
}

/**
 * @brief Function for updating the Timeout characteristic value.
 *
 * This function updates the timeout characteristic value.
 *
 * @param[in] p_occu     Pointer to the proximity Service structure.
 * @param[in] new_value  New timeout value to update.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
ret_code_t ble_proximity_service_timeout_update(ble_proximity_service_t* p_occu, uint16_t new_value)
{
    ret_code_t err_code;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)&new_value;

    // Update the characteristic value
    err_code = sd_ble_gatts_value_set(p_occu->conn_handle,
                                      p_occu->timeout_handles.value_handle,
                                      &gatts_value);

    setPresenceTimeout(new_value);

    NRF_LOG_INFO("Timeout value Updated to %dms", new_value);

    return err_code;
}

/**
 * @brief Function for updating the Sensitivity characteristic value.
 *
 * This function updates the sensitivity characteristic value.
 *
 * @param[in] p_occu     Pointer to the proximity Service structure.
 * @param[in] new_value  New sensitivity value to update.
 * @return NRF_SUCCESS if successful, otherwise an error code.
 */
ret_code_t ble_proximity_service_sensitivity_update(ble_proximity_service_t* p_occu, uint8_t new_value)
{
    ret_code_t err_code;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &new_value;

    if(new_value >= MIN_SENSITIVITY_VALUE && new_value <= MAX_SENSITIVITY_VALUE)
    {
        stopSensor();
        setSensitivity(new_value);
        startSensor();
        NRF_LOG_INFO("Sensitivity value Updated to %d\nSensor restarted successfully.", new_value);
    }
    else
    {
        NRF_LOG_INFO("Sensitivity value %d is out of bounds.\nShould be within [%d, %d]", new_value, MIN_SENSITIVITY_VALUE, MAX_SENSITIVITY_VALUE);
        new_value = getSensitivity();
    }
    // Update the characteristic value
    err_code = sd_ble_gatts_value_set(p_occu->conn_handle,
                                    p_occu->sensitivity_handles.value_handle,
                                    &gatts_value);
    return err_code;
}


/**
 * @brief Function for handling the Connect event.
 *
 * @param[in] p_occu     Pointer to the proximity Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_proximity_service_t* p_occu, ble_evt_t const * p_ble_evt)
{
    p_occu->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    startSensor();
}



/**
 * @brief Function for handling the Disconnect event.
 *
 * @param[in] p_occu     Pointer to the proximity Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_proximity_service_t* p_occu, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_occu->conn_handle = BLE_CONN_HANDLE_INVALID;
    stopSensor();
}


/**
 * @brief Function for handling the Write event.
 *
 * @param[in] p_occu     Pointer to the proximity Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_proximity_service_t * p_occu, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_occu->timeout_handles.value_handle)
    {
        uint16_t timout_value;
        if(p_evt_write->len == 1)
        {
            timout_value = (uint16_t)*(uint8_t*)p_evt_write->data;
        }
        else
        {
            timout_value = *(uint16_t*)p_evt_write->data;
        }
        ble_proximity_service_timeout_update(p_occu, timout_value);
    }
    if (p_evt_write->handle == p_occu->range_handles.value_handle)
    {
        uint16_t range_value;
        if(p_evt_write->len == 1)
        {
            range_value = (uint16_t)*(uint8_t*)p_evt_write->data;
        }
        else
        {
            range_value = *(uint16_t*)p_evt_write->data;
        }
        ble_proximity_service_range_update(p_occu, range_value);
    }
    if (p_evt_write->handle == p_occu->sensitivity_handles.value_handle)
    {
        uint8_t sens_value = *(uint8_t*)p_evt_write->data;
        ble_proximity_service_sensitivity_update(p_occu, sens_value);
    }
}

/**
 * @brief Function for handling GATT events related to the custom service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  Pointer to the proximity Service structure.
 */
void ble_proximity_service_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context)
{
    ble_proximity_service_t* p_occu = (ble_proximity_service_t*)p_context;

    if (p_ble_evt == NULL || p_context == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_occu, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_occu, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_occu, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


