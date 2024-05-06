/*
* Copyright Novelda AS 2024.
*/
#include "novelda_chipinterface.h"
#include "novelda_x4sensor_private.h"
#include "x4sensor_configuration.h"
#include "x4_algorithm_common.h"
#include "x4_interface_common.h"

#include <string.h>

#define I2C_MAX_TRANSFER_SIZE 60
#define I2C_COMMAND_WAIT_MICROSECONDS 60

static uint8_t com_buffer_i2c[I2C_MAX_TRANSFER_SIZE];
static uint32_t prev_frame_counter = 0;
static uint8_t fw_hash = 0;

static x4sensor_error_t
write_data_i2c(const uint8_t* data, size_t length)
{
    X4SENSOR_CHECK(length <= I2C_MAX_TRANSFER_SIZE - 2, return X4SENSOR_INVALID_PARAMETER);

    com_buffer_i2c[0] = X4_I2C_COMMAND_WRITE_DATA;
    com_buffer_i2c[1] = (uint8_t)length;
    memcpy(&com_buffer_i2c[2], data, length);

    chipinterface_error_t chip_stat = CHIPINTERFACE_SUCCESS;
    chip_stat = chipinterface_write_i2c(com_buffer_i2c, length + 2);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, return X4SENSOR_CHIPINTERFACE_ERROR);
    chip_stat = chipinterface_wait_us(I2C_COMMAND_WAIT_MICROSECONDS);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, return X4SENSOR_CHIPINTERFACE_ERROR);

    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
read_data_i2c(uint8_t* data, size_t length)
{
    if (chipinterface_read_i2c(data, length) == CHIPINTERFACE_SUCCESS)
        return X4SENSOR_SUCCESS;
    else
        return X4SENSOR_CHIPINTERFACE_ERROR;
}

static x4sensor_error_t
write_command_i2c(x4_i2c_command_t command, const uint8_t* data, size_t length)
{
    X4SENSOR_CHECK(length <= I2C_MAX_TRANSFER_SIZE - 1, return X4SENSOR_INVALID_PARAMETER);

    com_buffer_i2c[0] = (uint8_t)command;
    if (length > 0)
        memcpy(&com_buffer_i2c[1], data, length);

    chipinterface_error_t chip_stat = CHIPINTERFACE_SUCCESS;
    chip_stat = chipinterface_write_i2c(com_buffer_i2c, length + 1);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, return X4SENSOR_CHIPINTERFACE_ERROR);
    chip_stat = chipinterface_wait_us(I2C_COMMAND_WAIT_MICROSECONDS);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, return X4SENSOR_CHIPINTERFACE_ERROR);

    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
clear_interrupt_i2c(void)
{
    return write_command_i2c(X4_COMMAND_CLEAR_INTERRUPT, NULL, 0);
}

static x4sensor_error_t
set_register_i2c(uint16_t register_address, const uint8_t data)
{
    uint8_t address[2] = {
        [0] = (uint8_t)(register_address >> 8),
        [1] = (uint8_t)register_address
    };
    x4sensor_error_t x4_stat;
    x4_stat = write_command_i2c(X4_I2C_COMMAND_SET_DATA_POINTER, address, 2);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);
    x4_stat = write_data_i2c(&data, 1);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);

    return X4SENSOR_SUCCESS;
}

static  x4sensor_error_t
get_register_i2c(uint16_t register_address, uint8_t *data)
{
    uint8_t address[2] = {
        [0] = (uint8_t)(register_address >> 8),
        [1] = (uint8_t)register_address
    };
    x4sensor_error_t x4_stat;
    x4_stat = write_command_i2c(X4_I2C_COMMAND_SET_DATA_POINTER, address, 2);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);
    x4_stat = read_data_i2c(data, 1);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);

    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
set_dptr_i2c(x4_i2c_command_t dptr_command, uint8_t offset)
{
    return write_command_i2c(dptr_command, &offset, 1);
}

static x4sensor_error_t
start_lposc_measurement_i2c(uint8_t nticks)
{
    x4sensor_error_t x4_stat;
    x4_stat = write_command_i2c(X4_COMMAND_LPOSC_MEASUREMENT, &nticks, sizeof(uint8_t));
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);

    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
reset_sensor_i2c()
{
    chipinterface_error_t chip_stat;
    chip_stat = chipinterface_set_chip_enabled(false);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, goto error);
    chip_stat = chipinterface_wait_us(500);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, goto error);
    chip_stat = chipinterface_set_chip_enabled(true);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, goto error);
    chip_stat = chipinterface_wait_us(1000);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, goto error);

    return X4SENSOR_SUCCESS;
error:
    chipinterface_set_chip_enabled(false);
    return X4SENSOR_CHIPINTERFACE_ERROR;
}

static x4sensor_error_t
discover_sensor_i2c(x4sensor_info_t *info)
{
    x4sensor_error_t x4_stat;
    x4_stat = reset_sensor_i2c();
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);

    uint8_t *sample_id_bytes = (uint8_t*)&info->sample_id;
    const uint16_t sample_id_address = 65531;
    x4_stat = get_register_i2c(sample_id_address, &sample_id_bytes[0]);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    x4_stat = read_data_i2c(&sample_id_bytes[1], 3);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    const uint16_t revision_id_address = 65519;
    x4_stat = get_register_i2c(revision_id_address, &info->chip_revision);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);


end:
    chipinterface_set_chip_enabled(false);
    return x4_stat;
}

static x4sensor_error_t
upload_firmware_i2c(const uint8_t *firmware, size_t size)
{
    x4sensor_error_t x4_stat;
    chipinterface_error_t chip_stat;
    prev_frame_counter = 0;
    x4_stat = reset_sensor_i2c();
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    // Programming the address once is enough. The dptr is auto-incremented
    // on every write.
    uint8_t address[2] = {
        [0] = 0,
        [1] = 0
    };
    x4_stat = write_command_i2c(X4_I2C_COMMAND_SET_DATA_POINTER, address, 2);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    // Write the content in chunks
    for (size_t offset = 0; offset < size;) {
        uint8_t bytes_to_write;
        if (size - offset > I2C_MAX_TRANSFER_SIZE - 2)
            bytes_to_write = I2C_MAX_TRANSFER_SIZE - 2;
        else
            bytes_to_write = (uint8_t)(size - offset);
        x4_stat = write_data_i2c(&firmware[offset], bytes_to_write);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
        offset += bytes_to_write;
    }

    // Consume any potentially pending interrupt. This may happen when
    // the sensor was already enabled before.
    chipinterface_wait_for_interrupt(0);

    // Reboot
    x4_stat = write_command_i2c(X4_I2C_COMMAND_SOFT_REBOOT, NULL, 0);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    // Wait for IRQ to become low which means X4 is booting up
    chipinterface_interrupt_state_t irq_state;
    do{
        chip_stat = chipinterface_get_interrupt_state(&irq_state);
        X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, x4_stat = X4SENSOR_CHIPINTERFACE_ERROR; goto error);
    }while(irq_state == chipinterface_interrupt_asserted);

    chip_stat = chipinterface_wait_for_interrupt(50000);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, x4_stat = X4SENSOR_CHIPINTERFACE_ERROR; goto error);

    x4_stat = clear_interrupt_i2c();
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    // Is the firmware running properly?
    x4_stat = set_dptr_i2c(X4_COMMAND_SET_DPTR_TO_INFO, 0);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    x4_stat = read_data_i2c(com_buffer_i2c, sizeof(x4_info_t));
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    x4_info_t* x4 = (x4_info_t*)com_buffer_i2c;
    fw_hash = x4sensor_make_firmware_hash();
    if (x4->firmware_hash != fw_hash) {
        x4_stat = X4SENSOR_FIRMWARE_VERIFICATION_FAILED;
        goto error;
    }
    if (x4->magic_byte != X4_MAGIC_BYTE) {
        x4_stat = X4SENSOR_FIRMWARE_VERIFICATION_FAILED;
        goto error;
    }
    if (x4->interface_version != X4_INTERFACE_VERSION) {
        x4_stat = X4SENSOR_FIRMWARE_INCOMPATIBLE_INTERFACE_VERSION;
        goto error;
    }

    return X4SENSOR_SUCCESS;
error:
    chipinterface_set_chip_enabled(false);
    return x4_stat;
}

static x4sensor_error_t
write_config_i2c(const rw_config_t *config)
{
    x4sensor_error_t x4_stat;

    x4_stat = set_dptr_i2c(X4_COMMAND_SET_DPTR_TO_PARAMETERS, 0);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);
    const uint8_t *config_buffer = (const uint8_t*)config;
    size_t length = sizeof(rw_config_t);
   // Write the content in chunks
    for (size_t offset = 0; offset < length;) {
        uint8_t bytes_to_write;
        if (length - offset > I2C_MAX_TRANSFER_SIZE - 2)
            bytes_to_write = I2C_MAX_TRANSFER_SIZE - 2;
        else
            bytes_to_write = (uint8_t)(length - offset);
        x4_stat = write_data_i2c(&config_buffer[offset], bytes_to_write);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);
        offset += bytes_to_write;
    }

    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
set_run_mode_i2c(x4_run_mode_t mode, x4sensor_event_flags_t events)
{
    x4sensor_error_t x4_stat;
    const uint8_t mode_w_flags[2] = {
        [0] = (const uint8_t)mode,
        [1] = (const uint8_t)events
    };
    x4_stat = write_command_i2c(X4_COMMAND_SET_RUN_MODE, &mode_w_flags[0], sizeof(mode_w_flags));
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);

    return X4SENSOR_SUCCESS;
}

static bool is_sensor_alive(){
    // Check if sensor FW app is running
    x4sensor_error_t x4_stat;
    uint8_t x4_FW_hash;
    x4_stat = set_dptr_i2c(X4_COMMAND_SET_DPTR_TO_INFO, offsetof(x4_info_t,firmware_hash));
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    x4_stat = read_data_i2c(&x4_FW_hash, sizeof(uint8_t));
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);

    if(x4_FW_hash == fw_hash){
        return true;
    }
end:
    return false;
}

static x4sensor_error_t
read_recording_data_i2c(uint8_t *buffer, size_t max_size, size_t *nbytes_read)
{
    x4sensor_error_t x4_stat;
    size_t bytes_to_read;
    size_t bytes_read = 0;
    if(!x4sensor_is_recording()){
        // Due to timing limitation don't check that when in recording mode.
        if(!is_sensor_alive()){
            x4_stat = X4SENSOR_FAILURE;
            goto end;
        }
    }
    x4_stat = set_dptr_i2c(X4_COMMAND_SET_DPTR_TO_RESULT, 0);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    bytes_to_read = sizeof(payload_t);
    x4_stat = read_data_i2c(buffer + bytes_read, bytes_to_read);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    bytes_read += bytes_to_read;
    uint32_t current_frame_number = ((payload_t*)buffer)->frame_counter;
    if((prev_frame_counter < current_frame_number) || ((prev_frame_counter > current_frame_number) && ((current_frame_number & 0xffff0000) == 0xffff0000))){
        // Check if frame counter has increased or wrap-around has occured
        prev_frame_counter = current_frame_number;
    }else{
        return X4SENSOR_FRAME_COUNTER_NOT_INCREASED;
    }
    if (x4sensor_is_recording()) {
        x4_stat = set_dptr_i2c(X4_COMMAND_SET_DPTR_TO_RADAR_DATA, 0);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
        bytes_to_read = x4sensor_get_configuration()->FrameConfig_RangeBins * sizeof(x4_sample_t);
        x4_stat = read_data_i2c(buffer + bytes_read, bytes_to_read);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
        bytes_read += bytes_to_read;
    }

end:
    *nbytes_read = bytes_read;
    return x4_stat;
}

static x4sensor_error_t
deinitialize_interface_i2c(void)
{
    if (chipinterface_delete_i2c() == CHIPINTERFACE_SUCCESS)
        return X4SENSOR_SUCCESS;
    else
        return X4SENSOR_CHIPINTERFACE_ERROR;
}

const x4sensor_vtable_t x4sensor_vtable_i2c = {
    .read_recording_data = read_recording_data_i2c,
    .upload_firmware = upload_firmware_i2c,
    .write_config = write_config_i2c,
    .set_run_mode = set_run_mode_i2c,
    .get_register = get_register_i2c,
    .set_register = set_register_i2c,
    .destroy_chipinterface = deinitialize_interface_i2c,
    .start_lposc_measurement = start_lposc_measurement_i2c,
    .clear_interrupt = clear_interrupt_i2c,
    .discover_sensor = discover_sensor_i2c
};

