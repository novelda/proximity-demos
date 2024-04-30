/*
* Copyright Novelda AS 2024.
*/
#pragma once

//
// This file is considered public. It describes the communication interface and
// necessary structs between host and X4.
//

#include "stdint.h"
#include "x4_common.h"

#ifdef __cplusplus
extern "C"
{
#endif


// Bump version number when making changes to the communication interface
// to ensure that X4 firmware and host application are built with identical
// header files
enum {
    X4_INTERFACE_VERSION = 9
};

typedef enum {
    X4_RUN_MODE_STOP = 0,
    X4_RUN_MODE_AUTONOMOUS = 1, // X4_RUN_MODE_NORMAL
    X4_RUN_MODE_EVENT = 2
} x4_run_mode_t;

typedef enum {
    // Handled by bootloader and I2C assembler code
    X4_I2C_COMMAND_NOP                    = 0x00,
    X4_I2C_COMMAND_SET_DATA_POINTER,
    X4_I2C_COMMAND_WRITE_DATA,
    X4_I2C_COMMAND_SOFT_REBOOT,

    // Set the data pointer to a variable in memory with an optional offset.
    // These commands are usually followed by an I2C command to read from or
    // write to that address.
    X4_COMMAND_DPTRS_START            = 0x0a,

    X4_COMMAND_SET_DPTR_TO_INFO       = X4_COMMAND_DPTRS_START,
    X4_COMMAND_SET_DPTR_TO_PARAMETERS = 0x0b,
    X4_COMMAND_SET_DPTR_TO_RESULT     = 0x0c,
    X4_COMMAND_SET_DPTR_TO_RADAR_DATA = 0x0d,

    X4_COMMAND_DPTRS_END              = X4_COMMAND_SET_DPTR_TO_RADAR_DATA,

    // Action commands, but may set dptr as a side-effect
    X4_COMMAND_SET_RUN_MODE           = 0x0e,
    X4_COMMAND_LPOSC_MEASUREMENT      = 0x0f,
    X4_COMMAND_CLEAR_INTERRUPT        = 0x10, // Clears the interrupt line
    X4_COMMAND_INCREASE_DPTR          = 0x11, // Data: 1 byte [num bytes to increase data pointer]
    X4_SPI_COMMAND_SET_DPTR_TO_MEMORY = 0x12  // SPI specific, for I2C use X4_I2C_COMMAND_SET_DATA_POINTER
} x4_i2c_command_t;

X4_PACK_START()

#define X4_MAGIC_BYTE 0xE1

typedef struct {
    uint8_t magic_byte;
    uint8_t interface_version;
    uint8_t firmware_hash;
} x4_info_t;

#define X4_MAX_RANGE_BINS 24 // The maximum number of range bins used by any algorithm

typedef struct{
    uint16_t sensor_event_period;
    uint16_t sweep_period;
    uint8_t detector_first_range_bin;
    uint8_t detector_last_range_bin;
    uint16_t detector_thresholds[X4_MAX_RANGE_BINS];
    uint8_t app_logic_M[2];
    uint8_t app_logic_N[2];
} rw_config_t;

X4_PACK_END()

#ifdef __cplusplus
}
#endif

