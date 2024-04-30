/**
 * @file proximity.c
 * @brief Implementation of an proximity sensor module.
 *
 * This module implements an proximity sensor using the Novelda chip interface and FreeRTOS.
 * It includes functions to start and stop the sensor, configure sensitivity, range, and timeout,
 * and process sensor events.
 */

#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include "app_timer.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <task.h>
/* Driver configuration */
#include <novelda_chipinterface.h>
#include "novelda_sensor.h"
#include "proximity.h"




volatile uint32_t              gEvents;
volatile bool                  gpresence;
volatile bool                  gSensorRunning = false;
static uint8_t                 gSensitivity = DEFAULT_SENSITIVITY;
static uint16_t                gRange = DEFAULT_RANGE;
static uint16_t                gTimeout = PRESENCE_TIME_OUT_MS;
TimerHandle_t                  presenceTimer;
SemaphoreHandle_t              appSem;

updateSensorValueCb_t updateSensorValCb;

static void sensor_event_callback(void);


/**
 * @brief Timer callback for presence detection.
 *
 * This callback is executed when the presence timer expires and updates the presence status.
 *
 * @param[in] arg0 Unused timer argument.
 */
void clkPresenceCallback(TimerHandle_t arg0)
{
    (void)arg0;
    chipinterface_interrupt_state_t state;
    chipinterface_get_interrupt_state(&state);
    gpresence = (state == chipinterface_interrupt_asserted);
    gEvents |= EVENT_SENSOR_PRESENCE;

    // post the application semaphore so the sensor data can be processed
    xSemaphoreGive(appSem);
}

/**
 * @brief Configure the clock and timers for sensor operations.
 *
 * This function initializes the timer module, creates the presence timer, and configures it.
 *
 * @return 0 on success.
 */
int configure_clock()
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    presenceTimer = xTimerCreate("PRSENCE",
                                   pdMS_TO_TICKS(gTimeout),
                                   pdFALSE,
                                   NULL,
                                   clkPresenceCallback);

    return 0;
}

/**
 * @brief Start the proximity sensor.
 *
 * This function starts the proximity sensor if it is not already running.
 */
void startSensor()
{
    if(!gSensorRunning)
    {
        gSensorRunning = true;
        sensor_run_remote(gSensitivity, gRange, sensor_event_callback);
    }
}

/**
 * @brief Stop the proximity sensor.
 *
 * This function stops the proximity sensor and halts the presence timer.
 */
void stopSensor()
{
    if(gSensorRunning)
    {
        gSensorRunning = false;
        sensor_stop_remote();

        if (xTimerIsTimerActive(presenceTimer) == pdTRUE)
        {
             xTimerStop(presenceTimer, 0);
        }
    }
}

/**
 * @brief Set the sensitivity of the proximity sensor.
 *
 * @param[in] sens Sensitivity value to set.
 */
void setSensitivity(uint8_t sens)
{
    gSensitivity = sens;
}

/**
 * @brief Set the range of the proximity sensor.
 *
 * @param[in] range Range value to set.
 */
void setRange(uint16_t range)
{
    gRange = range;
}

/**
 * @brief Get the sensitivity of the proximity sensor.
 *
 * @return Current sensitivity value.
 */
uint8_t  getSensitivity(void)
{
    return gSensitivity;
}

/**
 * @brief Get the range of the proximity sensor.
 *
 * @return Current range value.
 */
uint16_t getRange(void)
{
    return gRange;
}

/**
 * @brief Get the timeout value for presence detection.
 *
 * @return Current timeout value.
 */
uint16_t getTimeout(void)
{
    return gTimeout;
}

/**
 * @brief Change the range of the proximity sensor.
 *
 * This function increments the range value within a specified limit.
 *
 * @return New range value.
 */
uint16_t changeRange(void)
{
    gRange = (gRange + 100) % 500; // limit max range to 400cm
    gRange = gRange == 0 ? 100 : gRange;
    return gRange;
}

/**
 * @brief Set the presence timeout value for the proximity sensor.
 *
 * @param[in] tmout New timeout value.
 */
void setPresenceTimeout(uint16_t tmout)
{
    gTimeout = tmout;
    xTimerChangePeriod(presenceTimer, pdMS_TO_TICKS(tmout), 0);
}

/**
 * @brief Sensor event callback function.
 *
 * This callback is triggered when a sensor event occurs, and it sets the presence flag and
 * posts the application semaphore for further processing.
 */
void sensor_event_callback(void)
{
    gpresence = true;
    gEvents |= EVENT_SENSOR_PRESENCE;
    // post the application semaphore so the sensor event can be processed
    xSemaphoreGive(appSem);

}

/**
 * @brief Get the sensor value (presence status).
 *
 * @return 1 if the sensor is active (presence detected), 0 otherwise.
 */
uint8_t getSensorValue()
{
    return (uint8_t)(xTimerIsTimerActive(presenceTimer) == pdTRUE);
}

/**
 * @brief Initialize the proximity sensor module.
 *
 * This function initializes the proximity sensor module, configures the clock and timers,
 * and sets the callback function for updating the sensor value.
 *
 * @param[in] taskSem       Semaphore for task synchronization.
 * @param[in] updateSensCb  Callback function for updating the sensor value.
 */
void proximityInit(void* taskSem, updateSensorValueCb_t updateSensCb)
{
    appSem = (SemaphoreHandle_t)taskSem;
    configure_clock();
    updateSensorValCb = updateSensCb;
    sensor_init();
}


/**
 * @brief Process sensor events.
 *
 * This function processes sensor events and updates the sensor value using the callback function.
 */
void processSensorEvent()
{
    if(gEvents & EVENT_SENSOR_PRESENCE)
    {
        if(gpresence)
        {
            // this will start or reset the timeout back to precense timeout when called
            xTimerStart(presenceTimer, 0);
        }
        updateSensorValCb(gpresence);

        gEvents ^= EVENT_SENSOR_PRESENCE;
    }

}
