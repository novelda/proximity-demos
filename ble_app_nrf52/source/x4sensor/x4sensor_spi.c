/*
* Copyright Novelda AS 2024.
*/
/*
* Copyright Novelda AS 2022.
*/
#include "novelda_chipinterface.h"
#include "novelda_x4sensor_private.h"
#include "x4sensor_configuration.h"
#include "x4_algorithm_common.h"
#include "x4_interface_common.h"
#include "x4sensor_spi.h"
#include <string.h>

#define MAX_TRANSFER_SIZE 400
#define COMMAND_WAIT_MICROSECONDS 60
#define SPI_MAX_WRITE_BYTES 8
#define SPI_MAX_READ_BYTES 8

static uint32_t prev_frame_counter = 0;
static uint8_t fw_hash = 0;
static uint8_t com_buffer[MAX_TRANSFER_SIZE];
static x4sensor_error_t reset_sensor_spi();
static x4sensor_error_t read_data_spi(uint8_t* data, size_t length_w, size_t length_r, bool direct);
static x4sensor_error_t
write_data_spi(const uint8_t* data, size_t length, bool direct)
{
    chipinterface_error_t chip_stat = CHIPINTERFACE_SUCCESS;
    X4SENSOR_CHECK(length <= MAX_TRANSFER_SIZE - 2, return X4SENSOR_INVALID_PARAMETER);
    uint8_t payload_index = 0;
    if(direct == false){
        payload_index = 1;
    }else{
        payload_index = 0;

    }
    uint8_t packet_length = SPI_MAX_WRITE_BYTES - payload_index;
    for (uint32_t i = 0; i < length; i+=packet_length) {
        uint8_t bytes_to_write = packet_length;
        if(i + packet_length > length){
            bytes_to_write = (uint8_t)length - i;
        }
        memcpy(&com_buffer[payload_index], &data[i], bytes_to_write);
        if(direct == false){
            com_buffer[0] = ADDR_SPI_TO_CPU_WRITE_DATA_WE  | 0x80;
            bytes_to_write ++;
        }else{
            com_buffer[0] = com_buffer[0]  | 0x80;
        }
        chip_stat = chipinterface_transfer_spi(com_buffer, bytes_to_write, NULL, 0);
        X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, return X4SENSOR_CHIPINTERFACE_ERROR);
        if(i + packet_length <length){
            uint8_t data_mem_fifo[] = {ADDR_SPI_SPI_MEM_FIFO_STATUS_R};
            do{
                data_mem_fifo[0] = ADDR_SPI_SPI_MEM_FIFO_STATUS_R;
                chip_stat = read_data_spi(data_mem_fifo, 1, 1, true);
                if(data_mem_fifo[0] & FIFO_TO_MEM_FIFO_EMPTY) packet_length = SPI_MAX_WRITE_BYTES - payload_index;
                else packet_length = 1 + payload_index;
            }while(data_mem_fifo[0] & FIFO_TO_MEM_FIFO_FULL);
        }
    }
    if(direct == false){
        chip_stat = chipinterface_wait_us(COMMAND_WAIT_MICROSECONDS);
        X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, return X4SENSOR_CHIPINTERFACE_ERROR);
    }
    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
read_data_spi(uint8_t* data, size_t length_w, size_t length_r, bool direct)
{
    uint8_t bytes_w = (uint8_t)length_w;
    chipinterface_error_t chip_stat = CHIPINTERFACE_SUCCESS;
    size_t read_bytes = length_r;
    uint8_t direct_first_byte = data[0];
    if(direct == false){
        com_buffer[0] = (uint8_t)ADDR_SPI_FROM_CPU_READ_DATA_RE;
        bytes_w = (uint8_t)length_w + 1;
        memcpy(&com_buffer[1], data, length_w);
        if(read_bytes > SPI_MAX_READ_BYTES){
            read_bytes = SPI_MAX_READ_BYTES; //assuming FIFO is full
        }
    }else{
        memcpy(&com_buffer[0], data, length_w);
    }
    if (chipinterface_transfer_spi(com_buffer, bytes_w, data, read_bytes) == CHIPINTERFACE_SUCCESS){
        for(size_t i = read_bytes; i<length_r; i+=read_bytes){
            if(i + read_bytes > length_r ){
                read_bytes = length_r - i;
            }
            if(direct == false){
                com_buffer[0] = (uint8_t)ADDR_SPI_FROM_CPU_READ_DATA_RE;
                chip_stat = chipinterface_transfer_spi(com_buffer, 1, &data[i], read_bytes);
            }else{
                com_buffer[0] = direct_first_byte;
                chip_stat = chipinterface_transfer_spi(com_buffer, 1, &data[i], read_bytes);
            }
            if(chip_stat != CHIPINTERFACE_SUCCESS){
                return X4SENSOR_CHIPINTERFACE_ERROR;
            }
        }
        return X4SENSOR_SUCCESS;
    }
    else
        return X4SENSOR_CHIPINTERFACE_ERROR;
}

static x4sensor_error_t
clear_interrupt(void)
{
    uint8_t payload[]={
        X4_COMMAND_CLEAR_INTERRUPT
    };
    return write_data_spi(payload, sizeof(payload), false);
}
static x4sensor_error_t
set_register_spi(uint16_t register_address, const uint8_t data)
{
    x4sensor_error_t x4_stat;
    uint8_t data_write_register[] = {
        [0] = X4_SPI_COMMAND_SET_DPTR_TO_MEMORY | 0x80,
        [1] = (uint8_t)(register_address>>8),
        [2] = (uint8_t)(register_address),
        [3] = 1,
        [4] = data
    };
    x4_stat = write_data_spi(data_write_register, sizeof(data_write_register), false);
    return x4_stat;
}

static  x4sensor_error_t
get_register_spi(uint16_t register_address, uint8_t *data)
{
    x4sensor_error_t x4_stat;
    uint8_t data_read_register[] = {
        [0] = X4_SPI_COMMAND_SET_DPTR_TO_MEMORY,
        [1] = (uint8_t)(register_address>>8),
        [2] = (uint8_t)(register_address)
    };
    if((x4_stat = write_data_spi(data_read_register, sizeof(data_read_register), false)) != X4SENSOR_SUCCESS) return x4_stat;
    if((x4_stat = read_data_spi(data_read_register, 0, 1, false)) != X4SENSOR_SUCCESS) return x4_stat;
    *data = data_read_register[0];
    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
set_dptr_spi(x4_i2c_command_t dptr_command, uint8_t offset)
{
    uint8_t payload[] = {
        payload[0] = (uint8_t)dptr_command,
        payload[1] = offset
    };
    return write_data_spi(payload, sizeof(payload), false);
}

static x4sensor_error_t
start_lposc_measurement(uint8_t nticks)
{
    uint8_t payload[] = {
        payload[0] = (uint8_t)X4_COMMAND_LPOSC_MEASUREMENT,
        payload[1] = nticks
    };
    return write_data_spi(payload, sizeof(payload), false);
}

static x4sensor_error_t
reset_sensor_spi()
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

x4sensor_error_t flush_SPI_fifo(uint8_t mask, uint8_t condition){
    x4sensor_error_t x4_stat;
    uint32_t max_retries = 32;// FIFO depth is 8
    uint32_t retries = 0;
    uint8_t data_read_fifo_status[] = {ADDR_SPI_SPI_MEM_FIFO_STATUS_R};
    x4_stat = read_data_spi(data_read_fifo_status, sizeof(data_read_fifo_status), 1, true);
    while ((data_read_fifo_status[0] & mask) != condition) {
        if(retries++ > max_retries){
            goto error;
        }
        uint8_t data_read_mem[] = {ADDR_SPI_FROM_MEM_READ_DATA_RE};
        x4_stat = read_data_spi(data_read_mem, sizeof(data_read_mem), 1, true); // ignore read byte
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

        data_read_fifo_status[0] = ADDR_SPI_SPI_MEM_FIFO_STATUS_R;
        x4_stat = read_data_spi(data_read_fifo_status, 1, sizeof(data_read_fifo_status), true);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    }
    return X4SENSOR_SUCCESS;
error:
    return X4SENSOR_FAILURE;
}
x4sensor_error_t flush_CPU_fifo(uint8_t mask, uint8_t condition){
    x4sensor_error_t x4_stat;
    uint32_t max_retries = 32;// FIFO depth is 8
    uint32_t retries = 0;
    uint8_t data_read_fifo_status[] = {ADDR_SPI_SPI_MB_FIFO_STATUS_R};
    x4_stat = read_data_spi(data_read_fifo_status, sizeof(data_read_fifo_status), 1, true);
    while ((data_read_fifo_status[0] & mask) != condition) {
        if(retries++ > max_retries){
            goto error;
        }
        uint8_t data_read_mem[] = {ADDR_SPI_FROM_CPU_READ_DATA_RE};
        x4_stat = read_data_spi(data_read_mem, sizeof(data_read_mem), 1, true); // ignore read byte
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

        data_read_fifo_status[0] = ADDR_SPI_SPI_MB_FIFO_STATUS_R;
        x4_stat = read_data_spi(data_read_fifo_status, 1, sizeof(data_read_fifo_status), true);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    }
    return X4SENSOR_SUCCESS;
error:
    return X4SENSOR_FAILURE;
}

static x4sensor_error_t
discover_sensor_spi(x4sensor_info_t *info)
{
    x4sensor_error_t x4_stat;
    x4_stat = reset_sensor_spi();
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);


    uint16_t sample_id_address = 65519;
    uint8_t data_address_LSB[] = {ADDR_SPI_MEM_FIRST_ADDR_LSB_RW,(uint8_t)sample_id_address};
    x4_stat = write_data_spi(data_address_LSB, sizeof(data_address_LSB), true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    uint8_t data_address_MSB[] = {ADDR_SPI_MEM_FIRST_ADDR_MSB_RW, (uint8_t)(sample_id_address>>8)};
    x4_stat = write_data_spi(data_address_MSB, sizeof(data_address_MSB), true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    {uint8_t data_mem_readback_mode[] = {ADDR_SPI_MEM_MODE_RW, (uint8_t)(SET_NORMAL_MODE)};
    x4_stat = write_data_spi(data_mem_readback_mode, sizeof(data_mem_readback_mode), true); // no reset to CPU
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);}


    x4_stat = flush_SPI_fifo(FIFO_FROM_MEM_DATA_VALID_FLAG, 0);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    {uint8_t data_mem_readback_mode[] = {ADDR_SPI_MEM_MODE_RW, (uint8_t)(SET_READBACK_MODE)};
    x4_stat = write_data_spi(data_mem_readback_mode, sizeof(data_mem_readback_mode), true); // no reset to CPU
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);}

    x4_stat = flush_SPI_fifo(FIFO_FROM_MEM_DATA_VALID_FLAG, FIFO_FROM_MEM_DATA_VALID_FLAG);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    // Read
    uint8_t data_read_mem[8] = {ADDR_SPI_FROM_MEM_READ_DATA_RE};
    x4_stat = read_data_spi(data_read_mem, 1, 8, true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    uint8_t *sample_id_bytes = (uint8_t*)&info->sample_id;
    info->chip_revision = data_read_mem[0];
    data_read_mem[0] = ADDR_SPI_FROM_MEM_READ_DATA_RE;
    x4_stat = read_data_spi(data_read_mem, 1, 8, true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    sample_id_bytes[0] = data_read_mem[4];
    sample_id_bytes[1] = data_read_mem[5];
    sample_id_bytes[2] = data_read_mem[6];
    sample_id_bytes[3] = data_read_mem[7];
    uint8_t data_mem_normal_mode[] = {ADDR_SPI_MEM_MODE_RW, (uint8_t)(SET_NORMAL_MODE)};
    x4_stat = write_data_spi(data_mem_normal_mode, sizeof(data_mem_normal_mode), true); // no reset to CPU
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

error:
    chipinterface_set_chip_enabled(false);
    return x4_stat;
}

static x4sensor_error_t
upload_firmware_spi(const uint8_t *firmware, size_t size)
{
    x4sensor_error_t x4_stat;
    chipinterface_error_t chip_stat;
    prev_frame_counter = 0;
    x4_stat = reset_sensor_spi();
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    uint8_t data_address_LSB[] = {ADDR_SPI_MEM_FIRST_ADDR_LSB_RW,(uint8_t)START_OF_SRAM_LSB};
    x4_stat = write_data_spi(data_address_LSB, sizeof(data_address_LSB), true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    uint8_t data_address_MSB[] = {ADDR_SPI_MEM_FIRST_ADDR_MSB_RW, (uint8_t)(START_OF_SRAM_MSB)};
    x4_stat = write_data_spi(data_address_MSB, sizeof(data_address_MSB), true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    uint8_t data_mem_write_mode[] = {ADDR_SPI_MEM_MODE_RW, (uint8_t)(SET_PROGRAMMING_MODE)};
    x4_stat = write_data_spi(data_mem_write_mode, sizeof(data_mem_write_mode), true); // reset to CPU
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    // Upload firmware
    uint8_t packet_length = SPI_MAX_WRITE_BYTES - 1;
    uint8_t data_upload_fw[SPI_MAX_WRITE_BYTES] = {ADDR_SPI_TO_MEM_WRITE_DATA_WE};
    for (uint32_t i = 0; i < size; i+=packet_length) {
        data_upload_fw[0] = ADDR_SPI_TO_MEM_WRITE_DATA_WE;
        uint8_t bytes_to_write = packet_length;
        if(i + packet_length > size){
            bytes_to_write = (uint8_t)size - i;
        }
        memcpy(&data_upload_fw[1], &firmware[i], bytes_to_write);
        x4_stat = write_data_spi(data_upload_fw, bytes_to_write +1 , true);
        uint8_t data_mem_fifo[] = {ADDR_SPI_SPI_MEM_FIFO_STATUS_R};
        do{
            data_mem_fifo[0] = ADDR_SPI_SPI_MEM_FIFO_STATUS_R;
            x4_stat = read_data_spi(data_mem_fifo, 1, 1, true);
            if(data_mem_fifo[0] & FIFO_TO_MEM_FIFO_EMPTY) packet_length = SPI_MAX_WRITE_BYTES -1;
            else packet_length = 1;
        }while(data_mem_fifo[0] & FIFO_TO_MEM_FIFO_FULL);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    }

    // Consume any potentially pending interrupt. This may happen when
    // the sensor was already enabled before.
    chipinterface_wait_for_interrupt(0);

    // Reboot
    uint8_t data_reboot[] = {ADDR_SPI_BOOT_FROM_OTP_SPI_RWE, BOOT_FROM_SRAM};
    x4_stat = write_data_spi(data_reboot, sizeof(data_reboot), true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    // Verify FW
    x4_stat = write_data_spi(data_address_LSB, sizeof(data_address_LSB), true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
    x4_stat = write_data_spi(data_address_MSB, sizeof(data_address_MSB), true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    uint8_t data_mem_readback_mode[] = {ADDR_SPI_MEM_MODE_RW, (uint8_t)(SET_READBACK_MODE)};
    x4_stat = write_data_spi(data_mem_readback_mode, sizeof(data_mem_readback_mode), true); // no reset to CPU
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    x4_stat = flush_SPI_fifo(FIFO_FROM_MEM_DATA_VALID_FLAG, FIFO_FROM_MEM_DATA_VALID_FLAG);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    uint8_t data_normal_mode[] = {ADDR_SPI_MEM_MODE_RW, (uint8_t)(SET_NORMAL_MODE)};
    x4_stat = write_data_spi(data_normal_mode, sizeof(data_normal_mode), true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    // Wait for IRQ to become low which means X4 is booting up
    chipinterface_interrupt_state_t irq_state;
    do{
        chip_stat = chipinterface_get_interrupt_state(&irq_state);
        X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, x4_stat = X4SENSOR_CHIPINTERFACE_ERROR; goto error);
    }while(irq_state == chipinterface_interrupt_asserted);

    chip_stat = chipinterface_wait_for_interrupt(50000);
    X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, x4_stat = X4SENSOR_CHIPINTERFACE_ERROR; goto error);

    x4_stat = flush_CPU_fifo(FIFO_FROM_CPU_DATA_VALID, 0);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);

    x4_stat = clear_interrupt();
    X4SENSOR_CHECK(chip_stat == X4SENSOR_SUCCESS, goto error);

    // Is the firmware running properly?
    x4_stat = set_dptr_spi(X4_COMMAND_SET_DPTR_TO_INFO, 0);

    X4SENSOR_CHECK(chip_stat == X4SENSOR_SUCCESS, goto error);

    x4_stat = read_data_spi(com_buffer, 0, sizeof(x4_info_t), false);
    X4SENSOR_CHECK(chip_stat == X4SENSOR_SUCCESS, goto error);
    x4_info_t* x4 = (x4_info_t*)com_buffer;
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
write_config_spi(const rw_config_t *config)
{
    x4sensor_error_t x4_stat;
    uint8_t payload[sizeof(rw_config_t)+3] = {
        [0] = (const uint8_t)X4_COMMAND_SET_DPTR_TO_PARAMETERS | 0x80,
        [1] = (const uint8_t)0, //offset
        [2] = (const uint8_t)sizeof(rw_config_t)
    };
    memcpy(&payload[3],(const uint8_t*)config,sizeof(rw_config_t));
    x4_stat = write_data_spi((const uint8_t*)payload, sizeof(rw_config_t)+3, false);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, return x4_stat);

    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
set_run_mode_spi(x4_run_mode_t mode, x4sensor_event_flags_t events)
{
    uint8_t mode_w_flags[] = {
        [0] = (const uint8_t)X4_COMMAND_SET_RUN_MODE,
        [1] = (const uint8_t)mode,
        [2] = (const uint8_t)events
    };
    return write_data_spi(mode_w_flags, sizeof(mode_w_flags), false);
}

static bool is_sensor_alive(){
    x4sensor_error_t x4_stat;
    uint8_t data_read_force_zero[] = {ADDR_SPI_FORCE_ZERO_R};
    x4_stat = read_data_spi(data_read_force_zero, sizeof(data_read_force_zero), 1, true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    uint8_t data_read_force_one[] = {ADDR_SPI_FORCE_ONE_R};
    x4_stat = read_data_spi(data_read_force_one, sizeof(data_read_force_one), 1, true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    uint8_t data_read_fw_version[] = {ADDR_SPI_FIRMWARE_VERSION_SPI_R};
    x4_stat = read_data_spi(data_read_fw_version, sizeof(data_read_fw_version), 1, true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    if((data_read_force_zero[0] == 0) && (data_read_force_one[0] == 0xff) && (data_read_fw_version[0] == fw_hash)){
        return true;
    }
end:
    return false;
}

static x4sensor_error_t
read_recording_data_spi(uint8_t *buffer, size_t max_size, size_t *nbytes_read)
{
    x4sensor_error_t x4_stat;
    size_t bytes_to_read;
    size_t bytes_read = 0;
    uint8_t data_read_fifo_status[] = {ADDR_SPI_SPI_MB_FIFO_STATUS_R};
    x4_stat = read_data_spi(data_read_fifo_status, sizeof(data_read_fifo_status), 1, true);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    while(data_read_fifo_status[0] & 0x2){ // test from_cpu_data_valid == 1
        uint8_t tmp;
        x4_stat = read_data_spi(&tmp, 0, 1, false);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
        data_read_fifo_status[0] = ADDR_SPI_SPI_MB_FIFO_STATUS_R;
        x4_stat = read_data_spi(data_read_fifo_status, sizeof(data_read_fifo_status), 1, true);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    }
    x4_stat = set_dptr_spi(X4_COMMAND_SET_DPTR_TO_RESULT, 0);
    X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
    for(int retries = x4sensor_get_retry_count(); retries > 0; --retries){
        data_read_fifo_status[0] = ADDR_SPI_SPI_MB_FIFO_STATUS_R;
        x4_stat = read_data_spi(data_read_fifo_status, sizeof(data_read_fifo_status), 1, true);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto end);
        if(retries<x4sensor_get_retry_count() -1){
            x4sensor_inc_retries_total_count();
        }
        if(data_read_fifo_status[0] & 0x2){ // from_cpu_data_valid == 1
            break;
        }
    }
    if((data_read_fifo_status[0] & 0x2) == 0) {
        if(is_sensor_alive()){
            x4_stat = X4SENSOR_DATA_NOT_READY;
        }else{
            x4_stat = X4SENSOR_FAILURE;
        }
        goto end;
    }
    bytes_to_read = sizeof(payload_t);
    x4_stat = read_data_spi(buffer + bytes_read, 0, bytes_to_read, false);
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
        bytes_to_read = x4sensor_get_configuration()->FrameConfig_RangeBins * sizeof(x4_sample_t);
        uint16_t fetch_data_pif_reg_address = ADDR_PIF_FETCH_RADAR_DATA_SPI_W | 0x8000;
        uint8_t data_fetch_radar_data[] = {X4_SPI_COMMAND_SET_DPTR_TO_MEMORY | 0x80, (uint8_t)(fetch_data_pif_reg_address >> 8), (uint8_t)fetch_data_pif_reg_address ,1 , 0xff};
        x4_stat = write_data_spi(data_fetch_radar_data, sizeof(data_fetch_radar_data), false);

        buffer[bytes_read] = ADDR_SPI_RADAR_DATA_SPI_RE;
        x4_stat = read_data_spi(&buffer[bytes_read], 1, bytes_to_read, true);
        bytes_read += bytes_to_read;
    }

end:
    *nbytes_read = bytes_read;
    return x4_stat;
}


static x4sensor_error_t
deinitialize_interface_spi(void)
{
    if (chipinterface_delete_spi() == CHIPINTERFACE_SUCCESS)
        return X4SENSOR_SUCCESS;
    else
        return X4SENSOR_CHIPINTERFACE_ERROR;
}

const x4sensor_vtable_t x4sensor_vtable_spi = {
    .read_recording_data = read_recording_data_spi,
    .upload_firmware = upload_firmware_spi,
    .write_config = write_config_spi,
    .set_run_mode = set_run_mode_spi,
    .get_register = get_register_spi,
    .set_register = set_register_spi,
    .destroy_chipinterface = deinitialize_interface_spi,
    .start_lposc_measurement = start_lposc_measurement,
    .clear_interrupt = clear_interrupt,
    .discover_sensor = discover_sensor_spi
};

