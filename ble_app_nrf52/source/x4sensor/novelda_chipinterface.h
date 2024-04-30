/*
* Copyright Novelda AS 2024.
*/
#ifndef NOVELDA_CHIPINTERFACE_H
#define NOVELDA_CHIPINTERFACE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * :file: novelda_chipinterface.h
 *
 * A common communication interface API for interfacing signal processing
 * libraries provided by Novelda to the X4 and X7 hardware sensors.
 *
 * The chipinterface API is meant to simplify the integration of Novelda
 * products into custom environments and is limited to general read/write
 * transactions as well as 2 IO pins - one output for enabling the sensor and
 * one input to detect interrupt events. Although it supports both, I2C and
 * SPI, only one variant is usually used per project. Therefore, integrators
 * may focus on one interface for one specific sensor.
 *
 * The proposed workflow is as follows:
 *
 * 1. Add the provided high-level sensor library provided by Novelda into your
 *    project. You should see linker errors due to missing chipinterface functions.
 *
 * 2. Find the missing functions below and implement them based upon the
 *    available documentation. Demo chipinterface implementations are available
 *    for the Zephyr OS and may serve as a blueprint.
 *
 * 3. Always hook up a logic analyzer to the bus when testing your code.
 */

/**
 * Status code returned by most of the chipinterface functions.
 */
typedef int8_t chipinterface_error_t;

/**
 * The function call succeeded.
 */
static const chipinterface_error_t CHIPINTERFACE_SUCCESS = 0;

/**
 * The function call failed due to an error.
 */
static const chipinterface_error_t CHIPINTERFACE_FAILURE = -1;

/**
 * The function timed out.
 *
 * :See: :c:func:`chipinterface_wait_us`,
 *       :c:func:`chipinterface_wait_for_interrupt`
 */
static const chipinterface_error_t CHIPINTERFACE_TIMEOUT = -2;

/**
 * Represents an infinite wait time.
 *
 * :See: :c:func:`chipinterface_wait_us`,
 *       :c:func:`chipinterface_wait_for_interrupt`
 */
static const uint32_t CHIPINTERFACE_WAIT_FOREVER = UINT32_MAX;

/**
 * :brief: The current level of the interrupt line
 *
 * The interrupt line can either be asserted or deasserted.
 * Asserted usually means "high" while deasserted means "low".
 *
 * :See: chipinterface_get_interrupt_state
 */
typedef enum chipinterface_interrupt_state_t {
    /** The interrupt line is asserted */
    chipinterface_interrupt_asserted,
    /** The interrupt line is deasserted */
    chipinterface_interrupt_deasserted
} chipinterface_interrupt_state_t;

/**
 * Clock polarity (CPOL) for SPI communication.
 */
typedef enum chipinterface_polarity_t {
    /**
     * The clock signal idles at the logical low voltage (CPOL = 0).
     */
    chipinterface_polarity_low,
    /**
     * The clock signal idles at the logical high voltage (CPOL = 1).
     */
    chipinterface_polarity_high
} chipinterface_polarity_t;

/**
 * Clock phase (CPHA) for SPI communication.
 */
typedef enum chipinterface_phase_t {
    /**
     * The first data bit is outputted immediately when CS activates.
     * Subsequent bits are outputted when SCLK transitions to its idle voltage
     * level. Sampling occurs when SCLK transitions from its idle voltage level
     * (CPHA = 0).
     */
    chipinterface_phase_leading,
    /**
     * The first data bit is outputted on SCLK's first clock edge after CS
     * activates. Subsequent bits are outputted when SCLK transitions from its
     * idle voltage level. Sampling occurs when SCLK transitions to its idle
     * voltage level (CPHA = 1).
     */
    chipinterface_phase_trailing
} chipinterface_phase_t;

/**
 * Bit order for SPI communication.
 */
typedef enum chipinterface_bitorder_t {
    /**
     * The most significant bit (MSB) is outputted first on the bus.
     */
    chipinterface_bitorder_msb,
    /**
     * The least significant bit (LSB) is first outputted to the bus.
     */
    chipinterface_bitorder_lsb
} chipinterface_bitorder_t;

/**
 * SPI clock configuration to be used for the connection. Novelda X4 and X7
 * sensors require a different SPI clock configuration. Novelda provides
 * default values for their sensors.
 *
 * :See: :c:var:`chipinterface_default_x4_spi_config`,
 *       :c:var:`chipinterface_default_x7_spi_config`
 *
 */
typedef struct chipinterface_spi_config_t {
    /** Clock polarity */
    chipinterface_polarity_t clock_polarity;
    /** Clock phase */
    chipinterface_phase_t clock_phase;
    /** Bit order */
    chipinterface_bitorder_t bit_order;
} chipinterface_spi_config_t;

/**
 * SPI configuration to be used for communication to X4 sensors.
 */
static const chipinterface_spi_config_t chipinterface_default_x4_spi_config = {
    chipinterface_polarity_low,
    chipinterface_phase_leading,
    chipinterface_bitorder_msb
};

/**
 * SPI configuration to be used for communication to X7 sensors.
 */
static const chipinterface_spi_config_t chipinterface_default_x7_spi_config = {
    chipinterface_polarity_low,
    chipinterface_phase_trailing,
    chipinterface_bitorder_lsb
};

/**
 * :brief: Enables and disables the sensor.
 *
 * The X4 and X7 sensors feature a hardware signal to switch the sensor on and
 * off. The pin is active high.
 *
 * :param enabled: true if the sensor should be enabled, otherwise false
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_set_chip_enabled(bool enabled);

/**
 * :brief: Waits for a :c:var:`microseconds` period.
 *
 * This function is called whenever the X4/7 drivers need to wait for the
 * sensor. The underlying implementation is free to do whatever while waiting.
 * It may yield the current thread or bring the whole system into a sleep mode
 * or just busy-wait.
 *
 * The actual time may not exactly match the desired period, but it must be at
 * least as long as specified. Longer wait times will prolong the firmware
 * upload, but the sensor will still work. Most of the time, this function is
 * called with rather small values below 1000 microseconds. For wait times
 * below the available tick resolution, consider to implement a busy-wait loop.
 *
 * :param microseconds: the timespan to wait in microseconds
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_wait_us(uint32_t microseconds);

/**
 * :brief: Determines the elapsed time in microseconds
 *
 * In order to achieve a correct frame rate and when no interrupt is used, the
 * elapsed time must be monitored. This function writes a monotonic increasing
 * counter value into the output parameter :c:var:`microseconds`. The
 * timestamps may wrap around.
 *
 * It is enough if the underlying implementation to provide a resolution of 1 ms,
 * but it should not be less.
 *
 * :param microseconds: the current time stamp in microseconds
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_get_time_microseconds(uint32_t *microseconds);

/**
 * :brief: Initializes the interface for I2C communication
 *
 * This function initializes the hardware interface and gets it ready for
 * communication.
 *
 * :param frequencyHz: the I2C clock frequency
 * :param slave_address: the address of the X4/X7 chip
 *
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_create_i2c(uint32_t frequencyHz, uint8_t slave_address);

/**
 * :brief: Deinitializes the chipinterface
 *
 * This function should free up any resources acquired by
 * :c:func:`chipinterface_create_i2c`.
 *
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_delete_i2c();

/**
 * :brief: Reads data from the sensor via I2C
 *
 * This function reads data from the sensor hardware via I2C and stores to the
 * buffer pointed to by :c:var:`data`. That buffer is guaranteed to be
 * :c:var:`size` bytes large. The IO configuration has been set in
 * :c:func:`chipinterface_create_i2c`.
 *
 * The implementation may read from the sensor in chunks.
 *
 * :param data: pointer to the internal buffer
 * :param size: number of bytes to read
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_read_i2c(uint8_t *data, size_t size);

/**
 * :brief: Writes data to the sensor via I2C
 *
 * This function writes :c:var:`size` bytes provided in the :c:var:`data`
 * buffer to the sensor hardware via I2C. The actual write may happen in
 * chunks. The IO configuration has been set in
 * :c:func:`chipinterface_create_i2c`.
 *
 * :param data: pointer to the buffer containing the data to write
 * :param size: number of bytes to write
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_write_i2c(const uint8_t* data, size_t size);

/**
 * :brief: Initializes the interface for SPI communication
 *
 * This function initializes the hardware interface and gets it ready for
 * communication.
 *
 * :param frequencyHz: the SPI clock frequency
 * :param configuration: a pointer to a valid clock configuration object
 *
 * The configuration object is a temporary argument. When needed by the chip
 * interface implementation beyond this function call, the implementation must
 * store an internal copy of the object.
 *
 * The chip interface implementation may be tailored towards either X4 or X7
 * and is thus free to ignore frequencyHz and configuration entirely.
 *
 * :See: :c:var:`chipinterface_default_x4_spi_config`,
 *       :c:var:`chipinterface_default_x7_spi_config`
 *
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_create_spi(uint32_t frequencyHz, const chipinterface_spi_config_t *configuration);

/**
 * :brief: Deinitializes the chipinterface
 *
 * This function should free up any resources acquired by
 * :c:func:`chipinterface_create_spi`.
 *
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_delete_spi();

/**
 * :brief: Sets the SPI/I2C clock frequency
 *
 * This function sets the SPI/I2C clock frequency at run-time to the specified
 * value.
 *
 * This function is only needed for X7 sensors.
 *
 * :param frequencyHz: the desired SPI/I2C clock frequency
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_set_clock_frequency(uint32_t frequencyHz);

/**
 * :brief: Transfers data from and to the sensor via SPI.
 *
 * This function writes and reads data to and from the sensor using the SPI
 * bus. The hardware must first send :c:var:`wlength` bytes and then read in
 * :c:var:`rlength` bytes. So the resulting transaction on the bus is
 * :c:var:`wlength` + :c:var:`rlength` bytes long.
 *
 * :param wdata: pointer to the buffer containing data to write
 * :param wlength: number of bytes to write
 * :param rdata: pointer to the buffer where incoming data should be stored
 * :param rlength: number of bytes to read
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_transfer_spi(const uint8_t *wdata, size_t wlength,
                                                 uint8_t *rdata, size_t rlength);

/**
 * :brief: Reads the current state of the interrupt line.
 *
 * :param state: Pointer to where the current state is written to
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success
 */
chipinterface_error_t chipinterface_get_interrupt_state(chipinterface_interrupt_state_t *state);

/**
 * :brief: Waits up to :c:var:`microseconds` for an interrupt
 *
 * This function blocks and waits for an interrupt for :c:var:`microseconds` time.
 * If the interrupt line is already asserted, the function returns immediately.
 * If :c:var:`microseconds` is :c:var:`CHIPINTERFACE_WAIT_FOREVER`, the function waits
 * wait forever. While waiting, the underlying implementation may put the system
 * into a low-power state or use busy-wait.
 *
 * It is enough if the underlying implementation to provide a resolution of 1 ms.
 *
 * :param microseconds: The time to wait or :c:var:`CHIPINTERFACE_WAIT_FOREVER`
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on interrupts,
 *          otherwise :c:var:`CHIPINTERFACE_TIMEOUT`
 */
chipinterface_error_t chipinterface_wait_for_interrupt(uint32_t microseconds);

/**
 * :brief: Sets an interrupt callback function
 *
 * This function sets a callback function that is called when an interrupt
 * occurs.
 *
 * This function is only needed for X7 sensors.
 *
 * :param callback: The callback function
 * :param context: Pointer to a user-defined object that is passed back to the callback.
 *                 Can be NULL if not used.
 * :return: :c:var:`CHIPINTERFACE_SUCCESS` on success,
 *          otherwise :c:var:`CHIPINTERFACE_FAILURE`
 */
chipinterface_error_t chipinterface_set_interrupt_callback(void (*callback)(void*), void *context);

#ifdef __cplusplus
}
#endif

#endif // NOVELDA_CHIPINTERFACE_H

