/**
 * @file app_proximity.c
 * @brief Application code for the Proximity Profile service.
 *
 * This file contains the implementation of the application code using the Proximity Profile service.
 */

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <app_main.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "proximity.h"
#include "proximity_service.h"
#include <ti/display/Display.h>


//*****************************************************************************
//! Globals
//*****************************************************************************

static void Proximity_changeCB( uint8_t paramId );
extern void sensor_run_thread(void * pvParameter);
void Proximity_on_proximity_evt(uint8_t proximity);
static void detection_task(void * pvParameter);
static TaskHandle_t m_proximity_task;
static TaskHandle_t m_sensor_task;
SemaphoreHandle_t appSemHandle;

// Simple GATT Profile Callbacks
static ProximityProfile_CBs_t proximity_profileCBs =
{
  Proximity_changeCB // Simple GATT Characteristic value change callback
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      Proximity_ChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void Proximity_changeCB( uint8_t paramId )
{
  uint16_t newValue = 0;
  ProximityProfile_getParameter(paramId, &newValue);

  switch( paramId )
  {
    case PROXIMITYPROFILE_RANGE:
      {
        if(newValue >= MIN_RANGE_VALUE && newValue <= MAX_RANGE_VALUE)
        {
            Display_printf(handle, 0, 0, "Range value = %d", newValue);
            stopSensor();
            setRange(newValue);
            startSensor();
        }
        else
        {
            uint16_t charRange = getRange();
            Display_printf(handle, 0, 0, "New Range value %d is out of rage.Allowed value range[%d, %d]", newValue, MIN_RANGE_VALUE, MAX_RANGE_VALUE);
            ProximityProfile_setParameter( PROXIMITYPROFILE_RANGE, sizeof(uint16_t),
                                                &charRange );
        }
      }
      break;

    case PROXIMITYPROFILE_SENSITIVITY:
      {
          if(newValue >= MIN_SENSITIVITY_VALUE && newValue <= MAX_SENSITIVITY_VALUE)
          {
              Display_printf(handle, 0, 0, "Sensitivity value = %d", newValue);
            stopSensor();
            setSensitivity(newValue);
            startSensor();
          }
          else
          {
              uint8_t charSensitivity = getSensitivity();
              Display_printf(handle, 0, 0, "New Sensitivity value %d is out of rage.Allowed value range[%d, %d]", newValue, MIN_SENSITIVITY_VALUE, MAX_SENSITIVITY_VALUE);
              ProximityProfile_setParameter( PROXIMITYPROFILE_SENSITIVITY, sizeof(uint8_t),
                                                  &charSensitivity );
          }
      }
      break;
    case PROXIMITYPROFILE_TIMEOUT:
      {
          Display_printf(handle, 0, 0, "Timeout value = %d", newValue);
        setPresenceTimeout(newValue);
          break;
      }
    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      Proximity_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the Simple GATT profile.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t Proximity_start( void )
{
  bStatus_t status = SUCCESS;

  //Initialize I2C being used by The sensor
  I2C_init();

  // Add Simple GATT service
  status = ProximityProfile_addService();
  if(status != SUCCESS)
  {
	// Return status value
    return(status);
  }

    uint8_t charProximity = 0;
    uint16_t charRange = getRange();
    uint8_t charSensitivity = getSensitivity();
    uint16_t charTimeout = getTimeout();

    ProximityProfile_setParameter( PROXIMITYPROFILE_DETECTION, sizeof(uint8_t),
                                    &charProximity );
    ProximityProfile_setParameter( PROXIMITYPROFILE_RANGE, sizeof(uint16_t),
                                    &charRange );
    ProximityProfile_setParameter( PROXIMITYPROFILE_SENSITIVITY, sizeof(uint8_t),
                                    &charSensitivity );
    ProximityProfile_setParameter( PROXIMITYPROFILE_TIMEOUT, sizeof(uint16_t),
                                    &charTimeout );
  // Register callback with SimpleGATTprofile
  status = ProximityProfile_registerAppCBs( &proximity_profileCBs );
  appSemHandle = xSemaphoreCreateBinary();
  proximityInit(appSemHandle, Proximity_on_proximity_evt);

  if (pdPASS != xTaskCreate(detection_task, "DET", 256, NULL, 1, &m_proximity_task))
  {
      status = FAILURE;
  }
  if (pdPASS != xTaskCreate(sensor_run_thread, "SEN", 356, NULL, 2, &m_sensor_task))
  {
      status = FAILURE;
  }

  GPIO_setConfig(CONFIG_GPIO_LEDR, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
  GPIO_setConfig(CONFIG_GPIO_LEDG, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);



  // Return status value
  return(status);
}

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

/**
 * @brief Task handling detection events.
 *
 * This function is a task handling detection events in the background.
 *
 * @param pvParameter Task parameters (not used).
 */
static void detection_task(void * pvParameter)
{
    ICall_registerApp(&selfEntity, &syncEvent);
    for(;;)
    {
        if(pdTRUE == xSemaphoreTake(appSemHandle, portMAX_DELAY))
        {
            processSensorEvent();
        }

    }
}

/**
 * @brief Event handler for BLE proximity service on proximity event.
 *
 * This function is an event handler for the BLE proximity service on proximity event.
 *
 * @param detection Detection value.
 */
void Proximity_on_proximity_evt(uint8_t detection)
{
    static uint8_t last_detection;
    if(last_detection != detection)
    {
        ProximityProfile_setParameter(PROXIMITYPROFILE_DETECTION, sizeof(uint8_t), &detection);
        Display_printf(handle, 0, 0, "Detection value = %d", detection);
        last_detection = detection;
        GPIO_write(CONFIG_GPIO_LEDG, detection);

    }
}

