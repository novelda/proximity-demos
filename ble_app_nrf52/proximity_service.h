/**
 * @file proximity_service.h
 * @brief Custom Bluetooth Low Energy (BLE) service interface for proximity Sensor.
 *
 * This header file defines the interface for a custom BLE service tailored for an proximity sensor.
 * It includes the declaration of the service structure, function prototypes, and data types needed
 * to interact with the proximity sensor and configure its related settings.
 */


#ifndef BLE_PROXIMITY_S_H__
#define BLE_PROXIMITY_S_H__

#include "proximity.h"
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "ble_srv_common.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief   Macro for defining a ble_proximity instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_PROXIMITY_DEF(_name)                                                                          \
static ble_proximity_service_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_proximity_service_on_ble_evt, &_name)

// UUID for the custom service
#define BLE_UUID_PROXIMITY_SERVICE  0x20F1

// UUIDs for the characteristics
#define BLE_UUID_DETECTION_CHAR      0x2BAD
#define BLE_UUID_RANGE_CHAR          0x2BB1
#define BLE_UUID_SENSITIVITY_CHAR    0x2BB2
#define BLE_UUID_TIMEOUT_CHAR        0x2BB3



typedef struct
{
    uint16_t                    service_handle;         // Handle of proximity and Presence Service
    uint8_t                     uuid_type;              // UUID type for the service
    ble_gatts_char_handles_t    detection_value_handles; // Handles for Detection characteristic
    ble_gatts_char_handles_t    range_handles;           // Handles for Range characteristic
    ble_gatts_char_handles_t    sensitivity_handles;     // Handles for Sensitivity characteristic
    ble_gatts_char_handles_t    timeout_handles;         // Handles for Timeout characteristic
    uint16_t                    conn_handle;            // Connection handle to identify the connected peer
} ble_proximity_service_t;


// Function for initializing the custom GATT service
extern ret_code_t ble_proximity_service_init(ble_proximity_service_t* p_occu, void* ble_app_sem, updateSensorValueCb_t updateSensCb);

// Function for updating the detection characteristic
extern ret_code_t ble_proximity_service_detection_update(ble_proximity_service_t* p_proximity_service, uint8_t detection_value);

// Function for handling GATT events related to the custom service
extern void ble_proximity_service_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context);


#ifdef __cplusplus
}
#endif

#endif // BLE_proximity_S_H__