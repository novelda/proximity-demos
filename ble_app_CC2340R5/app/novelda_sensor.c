/**
 * @file novelda_sensor.c
 * @brief Implementation of the Novelda X4 proximity sensor module.
 *
 * This module provides the implementation of the Novelda X4 proximity sensor interface, including
 * initialization, sensor control, and event processing. It interacts with the Novelda chip
 * interface and FreeRTOS-based applications.
 */


#include <ti/drivers/GPIO.h>
#include <ti_drivers_config.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <task.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <novelda_chipinterface.h>
#include <x4sensor_configuration_blob.h>
#include "novelda_sensor.h"
#include <ti/display/Display.h>
#include <app_main.h>



volatile static SemaphoreHandle_t sensorSemHandle;
volatile uint32_t gSensor_Events;
volatile presence_callback gPresence_cb;
uint8_t gSensitivity;
uint16_t gRange;
volatile bool gRunning = false;


/**
 * @brief Initialize the proximity sensor module.
 *
 * This function initializes the proximity sensor module, creating semaphores for messages and events.
 */
void sensor_init(void)
{

    /* create semaphores for messages / events */
   sensorSemHandle = xSemaphoreCreateBinary();
    gSensor_Events |= EVENT_SENSOR_INIT;
    xSemaphoreGive(sensorSemHandle);
}

/**
 * @brief Start the proximity sensor remotely.
 *
 * This function configures the sensor with sensitivity, range, and a callback for presence detection.
 *
 * @param[in] sensitivity Sensitivity level to set.
 * @param[in] range Range value to set.
 * @param[in] callback Callback function for presence detection.
 */
void sensor_run_remote(uint8_t sensitivity, uint16_t range, presence_callback callback)
{
    gSensitivity = sensitivity;
    gRange = range;
    gPresence_cb = callback;
    gSensor_Events |= EVENT_SENSOR_START;
    xSemaphoreGive(sensorSemHandle);
}

/**
 * @brief Stop the proximity sensor remotely.
 *
 * This function stops the proximity sensor, switches the interrupt back to rising edge, and uninitializes
 * the GPIO input.
 */
void sensor_stop_remote(void)
{
    gRunning = false;
    // Switch interrupt back to rising edge to be able to restart the sensor
    GPIO_disableInt(CONFIG_GPIO_X4_IRQ_0);
    GPIO_setConfig(CONFIG_GPIO_X4_IRQ_0, GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING);
    GPIO_enableInt(CONFIG_GPIO_X4_IRQ_0);
    x4sensor_stop();
}


/**
 * @brief Sensor running thread.
 *
 * This thread processes sensor events and presence detection while the sensor is active.
 *
 * @param[in] pvParameter Unused thread parameter.
 */
void sensor_run_thread(void * pvParameter)
{
    const x4sensor_info_t *sensor_info;
    while (1)
    {
        if(gRunning)
        {
            //pend on irq semaphore
            if(chipinterface_wait_for_interrupt(portMAX_DELAY) == CHIPINTERFACE_SUCCESS)
            {
                //irq happened report back to application that there is presence
                if(gPresence_cb)
                {
                    gPresence_cb();
                }
            }
        }
        else
        {
            xSemaphoreTake(sensorSemHandle, portMAX_DELAY);
            if(gSensor_Events & EVENT_SENSOR_INIT)
            {

                MAIN_ASSERT(x4sensor_initialize_i2c(x4sensor_configuration_blob, x4sensor_configuration_blob_size),
                                                               X4SENSOR_SUCCESS);
                sensor_info = x4sensor_get_info();
                Display_printf(handle, 0, 0, "*** Novelda Sensor ID: 0x%X Chip Version: %d ***", sensor_info->sample_id, sensor_info->chip_revision);
                gSensor_Events ^= EVENT_SENSOR_INIT;

            }
            if(gSensor_Events & EVENT_SENSOR_START)
            {
                x4sensor_set_range_cm(gRange);
                x4sensor_set_sensitivity_level(gSensitivity);
                x4sensor_start_normal_mode();

                // Switch interrupt to both edges so we don't have to poll the irq line
                // this will trigger an interrupt once the irq line goes down which will restart the proximity timer
                GPIO_disableInt(CONFIG_GPIO_X4_IRQ_0);
                GPIO_setConfig(CONFIG_GPIO_X4_IRQ_0, GPIO_CFG_IN_PU| GPIO_CFG_IN_INT_BOTH_EDGES);
                GPIO_enableInt(CONFIG_GPIO_X4_IRQ_0);

                gRunning = true;
                gSensor_Events ^= EVENT_SENSOR_START;

            }

        }
    }
}
