/**
 * @file chipinterface_ti_freertos.c
 * @brief Implementation of the Novelda Chip Interface for the Texas Instruments platform usint FreeRtos.
 *
 * This module provides an implementation of the Novelda Chip Interface for the TI platform.
 * It includes functions for interfacing with the Novelda chip via I2C or SPI, managing GPIO pins,
 * and handling interrupts and semaphores.
 *
 * The implementation supports the TI platform and provides hardware abstraction for the Novelda X4 radar chip.
 */

/**
 * @note
 * Copyright Novelda AS 2022.
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <novelda_chipinterface.h>

#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <task.h>
/* Driver configuration */
#include "ti_drivers_config.h"

#define CI_EVENTS_IRQ  0x01
#define CI_EVENTS_DISSABLE 0x02

#define X4_IRQ_0 CONFIG_GPIO_X4_IRQ_0
#define X4_IRQ_1 0

#define SENSORS_SUPPORTED 1

struct chipinterface_context_t {
    const void *parent_context;
    uint8_t list_index;
    const void *iface_handle;
    uint_least8_t gpio_cs;
    uint_least8_t gpio_radar_en;
    uint_least8_t gpio_irq;
    uint8_t i2c_slave_address;
};
struct chipinterface_context_t *context_list[SENSORS_SUPPORTED];

SemaphoreHandle_t irqSem = NULL;
static uint8_t gEvents;
bool gStarted = false;

/**
 * @brief GPIO callback for sensor IRQ.
 *
 * This function handles the GPIO callback when an interrupt is triggered on the sensor IRQ pin.
 * It sets the corresponding event and gives the semaphore to notify the application.
 *
 * @param[in] index   GPIO pin index.
 * @param[in] action  GPIO action (polarity).
 */
static void gpio_irq_callback(uint_least8_t index)
{
    /*
     * Callbacks on the GPIO driver do not provide any other
     * info other than the GPIO pin where the IRQ was triggered
     * Therefore we need to keep track of the context locally
     * to retrieve the correct parent context
       */

    switch(index)
    {
    case X4_IRQ_0:
        gEvents |= CI_EVENTS_IRQ;
        xSemaphoreGive(irqSem);
       break;
       /* Insert other case statements here to support
        * multiple sensors i.e
        * case X4_IRQ_1:
        * chipinterface_on_interrupt(context_list[1]->parent_context);
        * break;
        * */
    default:
        break;
    }
}

/**
 * @brief Initialize the radar enable GPIO pin.
 *
 * This function configures and initializes the GPIO pin used to enable the radar sensor.
 *
 * @param[in] gpio_index  GPIO pin index.
 */
static void gpio_init_radar_en(uint_least8_t gpio_index)
{
    GPIO_setConfig(gpio_index, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
}

/**
 * @brief Initialize the chip select (CS) GPIO pin.
 *
 * This function configures and initializes the GPIO pin used as chip select (CS) for SPI communication.
 *
 * @param[in] gpio_index  GPIO pin index.
 */
static void gpio_init_cs(uint_least8_t gpio_index)
{
    GPIO_setConfig(gpio_index, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
}

/**
 * @brief Initialize the sensor IRQ GPIO pin.
 *
 * This function configures and initializes the GPIO pin used to trigger an interrupt on sensor events.
 *
 * @param[in] gpio_index  GPIO pin index.
 */
static void gpio_init_irq(uint_least8_t gpio_index)
{
    GPIO_setConfig(gpio_index, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_setCallback(CONFIG_GPIO_X4_IRQ_0, gpio_irq_callback);
    GPIO_enableInt(CONFIG_GPIO_X4_IRQ_0);
}



/**
 * @brief Create an I2C interface for the chip.
 *
 * This function initializes and configures an I2C interface for communication with the radar chip.
 *
 * @param[in] frequencyHz     I2C communication frequency (Hz).
 * @param[in] slave_address   Slave address of the radar chip.
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_create_i2c(uint32_t frequencyHz, uint8_t slave_address)
{
    uint8_t context_idx = 0;
    if(!irqSem)
    {
        irqSem = xSemaphoreCreateBinary();
    }

    //modify code with hardcoded index 0 if there is a need to support more than one sensor
    if(context_list[context_idx] == NULL)
    {
        context_list[context_idx] = malloc(sizeof(struct chipinterface_context_t));

        // initialize optional I2C bus parameters
        I2C_Params params;
        I2C_Handle *i2cHandle = malloc(sizeof(I2C_Handle));
        I2C_Params_init(&params);
        params.bitRate = I2C_400kHz;

        // Open I2C bus for usage
        *i2cHandle = I2C_open(0, &params);

        context_list[context_idx]->iface_handle = (void*)i2cHandle;
        context_list[context_idx]->i2c_slave_address = slave_address;
        context_list[context_idx]->list_index = 0;
        context_list[context_idx]->i2c_slave_address = slave_address;
        context_list[context_idx]->gpio_radar_en = CONFIG_GPIO_X4_EN_0;
        context_list[context_idx]->gpio_irq = CONFIG_GPIO_X4_IRQ_0;

        gpio_init_radar_en(context_list[context_idx]->gpio_radar_en);
        gpio_init_irq(context_list[context_idx]->gpio_irq);

        return CHIPINTERFACE_SUCCESS;
    }

    return CHIPINTERFACE_FAILURE;
}

/**
 * @brief Create an SPI interface for the chip.
 *
 * This function initializes and configures an SPI interface for communication with the radar chip.
 *
 * @param[in] frequencyHz     SPI communication frequency (Hz).
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_create_spi(uint32_t frequencyHz, const chipinterface_spi_config_t *configuration)
{
    uint8_t context_idx = 0;
    if(!irqSem)
    {
        irqSem = xSemaphoreCreateBinary();
    }
    //modify code with hardcoded index 0 if there is a need to support more than one sensor
    if(context_list[context_idx] != NULL)
    {
        context_list[context_idx] = malloc(sizeof(struct chipinterface_context_t));

        // initialize optional I2C bus parameters
        SPI_Params params;
        SPI_Handle *spi_handle = malloc(sizeof(SPI_Handle));
        SPI_Params_init(&params);
        params.bitRate = frequencyHz;

        // Open SPI bus for usage
        *spi_handle = SPI_open(0, &params);

        context_list[context_idx]->iface_handle = (void*)spi_handle;
        context_list[context_idx]->list_index = context_idx;
        context_list[context_idx]->gpio_radar_en = CONFIG_GPIO_X4_EN_0;
        context_list[context_idx]->gpio_irq = CONFIG_GPIO_X4_IRQ_0;
        context_list[context_idx]->gpio_cs = CONFIG_GPIO_X4_CS_0;

        gpio_init_radar_en(context_list[context_idx]->gpio_radar_en);
        gpio_init_irq(context_list[context_idx]->gpio_irq);
        gpio_init_cs(context_list[context_idx]->gpio_cs);

        return CHIPINTERFACE_SUCCESS;
    }

    return CHIPINTERFACE_FAILURE;
}

/**
 * @brief Delete the I2C interface.
 *
 * This function deinitializes and deletes the I2C interface used for communication with X4 sensor
 *
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_delete_i2c(void)
{
    uint8_t context_index = 0;
    I2C_Handle *i2c_handle = (I2C_Handle*)context_list[context_index]->iface_handle;
    // Close I2C
    I2C_close(*i2c_handle);
    free(i2c_handle);
    free(context_list[context_index]);
    context_list[context_index] = NULL;

    return CHIPINTERFACE_SUCCESS;
}

/**
 * @brief Delete the SPI interface.
 *
 * This function deinitializes and deletes the SPI interface used for radar communication.
 *
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_delete_spi()
{
    uint8_t context_index = 0;
    SPI_Handle *spi_handle = (SPI_Handle*)context_list[context_index]->iface_handle;
    // Close SPI
    SPI_close(*spi_handle);
    free(spi_handle);
    free(context_list[context_index]);
    context_list[context_index] = NULL;

    return CHIPINTERFACE_SUCCESS;
}

/**
 * @brief Enable or disable the X4 chip.
 *
 * This function enables or disables the sensor by controlling the radar enable (EN) GPIO pin.
 *
 * @param[in] enabled   Set to true to enable the chip, false to disable it.
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_set_chip_enabled(bool enabled)
{
    uint8_t state = enabled ? 1 : 0;
    GPIO_write( context_list[0]->gpio_radar_en, state);
    if(enabled == false && gStarted)
    {
        gEvents |= CI_EVENTS_DISSABLE;
        gStarted = false;
        xSemaphoreGive(irqSem);
    }

    return CHIPINTERFACE_SUCCESS;
}

/**
 * @brief Wait for a specified time in microseconds.
 *
 * This function waits for the specified time in microseconds.
 *
 * @param[in] microseconds Time to wait in microseconds.
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_wait_us(uint32_t microseconds)
{
    uint32_t millisecs;
    uint32_t ticks;
    millisecs = (uint32_t)(microseconds/1000);
    if(!millisecs)
    {
        millisecs = 1;
    }
    ticks  = pdMS_TO_TICKS(millisecs);
    vTaskDelay(ticks);
    return CHIPINTERFACE_SUCCESS;
}

/**
 * @brief Get the current time in microseconds.
 *
 * This function retrieves the current time in microseconds since system startup.
 *
 * @param[out] microseconds Pointer to the variable to store the time in microseconds.
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_get_time_microseconds(uint32_t *microseconds)
{
    TickType_t tickCount = xTaskGetTickCount();
    TickType_t tickRate = configTICK_RATE_HZ;

    *microseconds = ((uint32_t)tickCount * 1000000) / (uint32_t)tickRate;
	return CHIPINTERFACE_SUCCESS;
}

/**
 * @brief Read data from the I2C interface.
 *
 * This function reads data from the I2C interface.
 *
 * @param[out] data   Pointer to the data buffer to store the read data.
 * @param[in] size    Number of bytes to read.
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_read_i2c(uint8_t *data, size_t size)
{
    I2C_Handle *i2c_handle = (I2C_Handle*)context_list[0]->iface_handle;
    I2C_Transaction transaction;

    if(size == 0)
    {
        return CHIPINTERFACE_SUCCESS;
    }
    // Read from I2C slave device
    transaction.readBuf = data;
    transaction.readCount = size;
    transaction.writeCount = 0;
    transaction.targetAddress = context_list[0]->i2c_slave_address;
    if(I2C_transfer(*i2c_handle, &transaction))
    {
        return CHIPINTERFACE_SUCCESS;
    }
    else
    {
        return CHIPINTERFACE_FAILURE;
    }
}

/**
 * @brief Write data to the I2C interface.
 *
 * This function writes data to the I2C interface.
 *
 * @param[in] data     Pointer to the data buffer to write.
 * @param[in] size     Number of bytes to write.
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_write_i2c(const uint8_t* data, size_t size)
{
    I2C_Handle *i2c_handle = (I2C_Handle*)context_list[0]->iface_handle;
    I2C_Transaction transaction;
    // Write to I2C slave device
    transaction.writeBuf = (uint8_t*) data;
    transaction.writeCount = size;
    transaction.readCount = 0;
    transaction.targetAddress = context_list[0]->i2c_slave_address;
    if(I2C_transfer(*i2c_handle, &transaction))
    {
        return CHIPINTERFACE_SUCCESS;
    }
    else
    {
        return CHIPINTERFACE_FAILURE;
    }
}

/**
 * @brief Perform a SPI data transfer.
 *
 * This function performs a SPI data transfer, sending and receiving data simultaneously.
 *
 * @param[in] wdata       Pointer to the data buffer to write.
 * @param[in] wlength     Number of bytes to write.
 * @param[out] rdata      Pointer to the data buffer to store the received data.
 * @param[in] rlength     Number of bytes to read.
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_transfer_spi(const uint8_t *wdata, size_t wlength, uint8_t *rdata, size_t rlength)
{
    SPI_Handle *spi_handle = (SPI_Handle*)context_list[0]->iface_handle;
    SPI_Transaction transaction;
    bool transfer_ok = false;


    /* Set CS before transfer */
    GPIO_write(context_list[0]->gpio_cs, 1);

    if(wlength)
    {
        transaction.count = wlength;
        transaction.txBuf = (void *) wdata;
        transaction.rxBuf = NULL;
        /* Perform SPI transfer */
        transfer_ok = SPI_transfer(*spi_handle, &transaction);
    }
    if(rlength)
    {
        transaction.count = rlength;
        transaction.txBuf = NULL;
        transaction.rxBuf = (void *) rdata;
        /* Perform SPI transfer */
        transfer_ok = SPI_transfer(*spi_handle, &transaction);
    }


    return transfer_ok ? CHIPINTERFACE_SUCCESS : CHIPINTERFACE_FAILURE;
}

/**
 * @brief Get the interrupt state of the sensor.
 *
 * This function retrieves the interrupt state of the sensor, whether it is asserted or deasserted.
 *
 * @param[out] state Pointer to the variable to store the interrupt state.
 * @return CHIPINTERFACE_SUCCESS if successful, or CHIPINTERFACE_FAILURE if an error occurs.
 */
chipinterface_error_t chipinterface_get_interrupt_state(chipinterface_interrupt_state_t *state)
{
    *state = GPIO_read(CONFIG_GPIO_X4_IRQ_0) ?
            chipinterface_interrupt_asserted : chipinterface_interrupt_deasserted;
    return CHIPINTERFACE_SUCCESS;
}

/**
 * @brief Wait for a sensor interrupt.
 *
 * This function waits for a sensor interrupt to occur, given a specified time limit in microseconds.
 *
 * @param[in] microseconds Time limit to wait for an interrupt in microseconds.
 * @return CHIPINTERFACE_SUCCESS if an interrupt occurs, or CHIPINTERFACE_FAILURE if a timeout occurs.
 */
chipinterface_error_t chipinterface_wait_for_interrupt(uint32_t microseconds)
{
    uint32_t wait;
    if(microseconds != portMAX_DELAY)
    {
        uint32_t millisecs = (uint32_t)(microseconds/1000);
        if(!millisecs)
        {
            millisecs = 1;
        }
        wait  = pdMS_TO_TICKS(millisecs);
    }
    else
    {
        wait = microseconds;
        gStarted = true;

    }
    if(xSemaphoreTake(irqSem, wait))
    {
        if(gEvents & CI_EVENTS_IRQ)
        {
            gEvents = 0;
            return CHIPINTERFACE_SUCCESS;
        }
        if(gEvents & CI_EVENTS_DISSABLE)
        {
            return CHIPINTERFACE_FAILURE;
        }
    }
    return CHIPINTERFACE_FAILURE;
}
