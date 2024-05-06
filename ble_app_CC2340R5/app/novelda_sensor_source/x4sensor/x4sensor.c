/*
* Copyright Novelda AS 2024.
*/
#include "novelda_x4sensor.h"
#include "novelda_x4sensor_private.h"
#include "novelda_chipinterface.h"
#include "x4_algorithm_common.h"
#include "x4_interface_common.h"
#include "x4sensor_configuration.h"

#include <string.h>
#include <stdint.h>

#define I2C_X4_SLAVE_ADDRESS 90
#define I2C_FREQUENCY 400000
#define SPI_FREQUENCY (32000000/1)
#define DEFAULT_COMM_RETRY 5

#define NVA_MIN(i, j) (((i) < (j)) ? (i) : (j))
#define NVA_MAX(i, j) (((i) > (j)) ? (i) : (j))
#define NVA_ABS(i) (((i) < (0)) ? (-i) : (i))

static const uint32_t TicksPerSecond = 27000000 / 12 / 128;
static uint8_t comm_retry = DEFAULT_COMM_RETRY;
static uint32_t total_retries = 0;
typedef enum {
	X4_RUN_STAGE_DISABLED = 0,
	X4_RUN_STAGE_STOPPED = 1,
	X4_RUN_STAGE_RUNNING = 2
} x4_runstage_t;

static x4sensor_info_t info;
static rw_config_t algorithm_config;
static const x4sensor_configuration_t *config;
static uint8_t sensitivity_level;
static const uint8_t *firmware_data;
static size_t firmware_size;
static x4_run_mode_t run_mode;
static x4_runstage_t run_stage;
static const x4sensor_vtable_t *vtable;
static uint32_t lposc_correction_factor_1000;
static x4sensor_error_t x4_stat;
static const int16_t *range_lut;
static uint8_t range_bins;
static const uint16_t *threshold_vectors;
static uint8_t sensitivity_levels;
static const uint8_t *sensitivity_levels_indexes;
static const uint8_t *M_values;
static const uint8_t *N_values;
static const uint16_t *Range_cm;
static bool is_recording;

x4sensor_error_t x4sensor_set_retry_count(uint8_t retry_count){
    comm_retry = retry_count;
    return X4SENSOR_SUCCESS;
}

uint8_t x4sensor_get_retry_count(void){
    return comm_retry;
}
uint32_t x4sensor_get_retries_total_count(void){
    return total_retries;
}
void x4sensor_inc_retries_total_count(void){
    if(total_retries < UINT32_MAX) total_retries++;
}
static x4sensor_error_t
disable_x4()
{
    run_stage = X4_RUN_STAGE_DISABLED;
    chipinterface_error_t chip_stat = chipinterface_set_chip_enabled(false);
    X4SENSOR_CHECK_OR_RETURN(chip_stat == CHIPINTERFACE_SUCCESS, X4SENSOR_CHIPINTERFACE_ERROR);
    return X4SENSOR_SUCCESS;
}

//
// The lposc on the X4 might be up to +- 30% off. We let the X4 run
// for a certain amount of ticks and measure the actual time that
// takes. From that we can deduce a correction factor for all timing
// values. The factor is multipllied by 1000 in order to avoid
// floating point.
//
static x4sensor_error_t
measure_lposc()
{
    chipinterface_error_t chip_stat;
    uint32_t start_time;
    uint32_t end_time;
    uint8_t attempts_left;
    const uint8_t calibration_duration_ticks = 5; // increasing this value reduces
                                                  // the influence of external factors
                                                  // like irq response time, but then
                                                  // the whole procedure takes longer.
    const int32_t time_per_tick_us = 29127;
    const int32_t expected_time_us = time_per_tick_us * calibration_duration_ticks;
    const uint32_t max_margin_1000 = 350; // = (30+5)% * 1000
    const uint32_t max_measurement_time_us = 250000;

    for (attempts_left = 3; attempts_left > 0; --attempts_left) {
        chip_stat = chipinterface_get_time_microseconds(&start_time);
        X4SENSOR_CHECK(chip_stat == CHIPINTERFACE_SUCCESS, return X4SENSOR_CHIPINTERFACE_ERROR);
        // We assume that the time for starting the measurement is neglectable
        x4_stat = vtable->start_lposc_measurement(calibration_duration_ticks);
        X4SENSOR_CHECK_OR_RETURN(x4_stat == X4SENSOR_SUCCESS, x4_stat);
        chip_stat = chipinterface_wait_for_interrupt(max_measurement_time_us);
        X4SENSOR_CHECK_OR_RETURN(chip_stat == CHIPINTERFACE_SUCCESS, X4SENSOR_CHIPINTERFACE_ERROR);
        chip_stat = chipinterface_get_time_microseconds(&end_time);
        X4SENSOR_CHECK_OR_RETURN(chip_stat == CHIPINTERFACE_SUCCESS, X4SENSOR_CHIPINTERFACE_ERROR);

        int32_t actual_time_us = end_time - start_time;
        lposc_correction_factor_1000 = 1000u - ((actual_time_us - expected_time_us) * 1000 / expected_time_us);

        // Check resulting factor for plausibility
        if (lposc_correction_factor_1000 < 1000 - max_margin_1000)
            continue;
        if (lposc_correction_factor_1000 > 1000 + max_margin_1000)
            continue;
        // Next attempt if necessary
        break;
    }
    if (attempts_left == 0) {
        x4_stat = X4SENSOR_OSCILLATOR_FREQUENCY_NOT_PLAUSIBLE;
        return x4_stat;
    }

    x4_stat = vtable->clear_interrupt();
    X4SENSOR_CHECK_OR_RETURN(x4_stat == X4SENSOR_SUCCESS, x4_stat);

    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
parse_config_blob(const uint8_t *buffer, size_t nbytes)
{
    const x4sensor_blob_header_t* header = (const x4sensor_blob_header_t*)buffer;

    if (nbytes - sizeof(x4sensor_blob_header_t) < header->blob_size)
        return X4SENSOR_CONFIGURATION_INVALID_SIZE;

    if (header->magic_word != x4sensor_configuration_magic_word)
        return X4SENSOR_CONFIGURATION_INVALID_TYPE;

    // TODO: Check data integrity (header->blob_hash)

    if (header->configuration_format_hash != CONFIGURATION_FORMAT_HASH)
        return X4SENSOR_CONFIGURATION_INVALID_LAYOUT;

    info.algorithm_variant_hash = header->algorithm_variant_hash;
    info.firmware_version = header->firmware_commit_hash;
    if (header->header_version == 1) {
        info.algorithm_commit_hash = header->algorithm_commit_hash;
    } else if (header->header_version == 2) {
        info.algorithm_version[0] = header->algorithm_version[0];
        info.algorithm_version[1] = header->algorithm_version[1];
        info.algorithm_version[2] = header->algorithm_version[2];
    }
    const uint8_t* data = buffer + sizeof(x4sensor_blob_header_t);
    const uint16_t initial_offset = *(const uint16_t*)data;
    config = (const x4sensor_configuration_t*)(&data[initial_offset]);

    range_bins = config->FrameConfig_RangeBins / config->range_decimation_DecimFactor;
    range_lut = (const int16_t*)(&data[config->detector_RangeLookUpTable.offset]);
    threshold_vectors = (const uint16_t*)(&data[config->detector_DetectorThresholds.offset]);
    M_values = (const uint8_t*)(&data[config->app_logic_M.offset]);
    N_values = (const uint8_t*)(&data[config->app_logic_N.offset]);
    sensitivity_levels = config->detector_DetectorThresholds.length / range_bins;
    sensitivity_levels_indexes = (const uint8_t*)(&data[config->PublicParameters_SensitivityLevel.offset]);
    Range_cm = (const uint16_t*)(&data[config->PublicParameters_Range_cm.offset]);
    firmware_data = &data[config->firmware.offset];
    firmware_size = config->firmware.length;

    return X4SENSOR_SUCCESS;
}

static x4sensor_error_t
init_common(const uint8_t *configuration_blob, size_t configuration_blob_size)
{
    run_mode = X4_RUN_MODE_STOP;
    run_stage = X4_RUN_STAGE_DISABLED;
    is_recording = false;
    lposc_correction_factor_1000 = 0;
    config = NULL;
    memset(&info, 0, sizeof(info));

    x4_stat = parse_config_blob(configuration_blob, configuration_blob_size);
    X4SENSOR_CHECK_OR_GOTO(x4_stat == X4SENSOR_SUCCESS, end);

    x4_stat = vtable->discover_sensor(&info);

    // We override x4_stat here because it's usually the first error that users are facing
    // when ramping up. Throwing a more specific error message might be overwhelming.
    if (x4_stat != X4SENSOR_SUCCESS) {
        x4_stat = X4SENSOR_SENSOR_DISCOVERY_FAILED;
        goto end;
    }

    run_stage = X4_RUN_STAGE_STOPPED;

    x4_stat = x4sensor_set_sensitivity_level(config->detector_SensitivityLevel);
    X4SENSOR_CHECK_OR_GOTO(x4_stat == X4SENSOR_SUCCESS, end);

    x4_stat = x4sensor_set_range_cm(config->detector_Range_cm);
    X4SENSOR_CHECK_OR_GOTO(x4_stat == X4SENSOR_SUCCESS, end);

    // Set default value to 10s
    x4_stat = x4sensor_set_periodic_report_interval(10 * x4sensor_get_frame_rate());
    X4SENSOR_CHECK_OR_GOTO(x4_stat == X4SENSOR_SUCCESS, end);

end:
    if (x4_stat != X4SENSOR_SUCCESS)
        run_stage = X4_RUN_STAGE_DISABLED;
    return x4_stat;
}

static x4sensor_error_t
configure_and_start_x4(x4_run_mode_t mode, x4sensor_event_flags_t events)
{
    switch (run_stage) {
    case X4_RUN_STAGE_DISABLED: // not allowed
    case X4_RUN_STAGE_RUNNING: // not allowed
        X4SENSOR_CHECK_OR_RETURN(false, X4SENSOR_NOT_ALLOWED);
        break;
    case X4_RUN_STAGE_STOPPED:
        x4_stat = vtable->upload_firmware(firmware_data, firmware_size);
        if (x4_stat != X4SENSOR_SUCCESS){
            goto disable_chip;
        }
        if (lposc_correction_factor_1000 == 0) {
            x4_stat = measure_lposc();
            if (x4_stat != X4SENSOR_SUCCESS){
                goto disable_chip;
            }
        }

        algorithm_config.sweep_period = TicksPerSecond * lposc_correction_factor_1000 / config->ChipX4_FPS / 1000;
        for(int attempts = x4sensor_get_retry_count(); attempts > 0; --attempts){
            x4_stat = vtable->write_config(&algorithm_config);
            if(x4_stat == X4SENSOR_SUCCESS){
                break;
            }
        }
        X4SENSOR_CHECK_OR_GOTO(x4_stat == X4SENSOR_SUCCESS, disable_chip);
        x4_stat = vtable->set_run_mode(mode, events);
        X4SENSOR_CHECK_OR_GOTO(x4_stat == X4SENSOR_SUCCESS, disable_chip);
        run_stage = X4_RUN_STAGE_RUNNING;
        break;
    }
    run_mode = mode;

    return X4SENSOR_SUCCESS;
disable_chip:
    disable_x4();
    return x4_stat;
}

x4sensor_error_t
x4sensor_initialize_i2c(const uint8_t *configuration_blob, size_t configuration_blob_size)
{
    X4SENSOR_CHECK_OR_RETURN(run_stage == X4_RUN_STAGE_DISABLED, X4SENSOR_NOT_ALLOWED);
    chipinterface_error_t chip_stat;
    vtable = &x4sensor_vtable_i2c;
    chip_stat = chipinterface_create_i2c(I2C_FREQUENCY, I2C_X4_SLAVE_ADDRESS);
    X4SENSOR_CHECK_OR_RETURN(chip_stat == CHIPINTERFACE_SUCCESS, X4SENSOR_CHIPINTERFACE_ERROR);

    x4_stat = init_common(configuration_blob, configuration_blob_size);
    if (x4_stat != X4SENSOR_SUCCESS)
        goto error;

    return X4SENSOR_SUCCESS;
error:
    chipinterface_delete_i2c();
    return x4_stat;
}

x4sensor_error_t
x4sensor_initialize_spi(const uint8_t *configuration_blob, size_t configuration_blob_size)
{
    X4SENSOR_CHECK_OR_RETURN(run_stage == X4_RUN_STAGE_DISABLED, X4SENSOR_NOT_ALLOWED);
    chipinterface_error_t chip_stat;
    vtable = &x4sensor_vtable_spi;
    chip_stat = chipinterface_create_spi(SPI_FREQUENCY, &chipinterface_default_x4_spi_config);
    X4SENSOR_CHECK_OR_RETURN(chip_stat == CHIPINTERFACE_SUCCESS, X4SENSOR_CHIPINTERFACE_ERROR);

    x4_stat = init_common(configuration_blob, configuration_blob_size);
    if (x4_stat != X4SENSOR_SUCCESS)
        goto error;

    return X4SENSOR_SUCCESS;
error:
    chipinterface_delete_spi();
    return x4_stat;
}

x4sensor_error_t
x4sensor_deinitialize()
{
    chipinterface_error_t chip_stat;

    firmware_data = NULL;

    switch (run_stage) {
    case X4_RUN_STAGE_RUNNING:
        chip_stat = chipinterface_set_chip_enabled(false);
        X4SENSOR_CHECK_OR_RETURN(chip_stat == CHIPINTERFACE_SUCCESS, X4SENSOR_CHIPINTERFACE_ERROR);
        // Intentional fall-through
    case X4_RUN_STAGE_STOPPED:
        x4_stat = vtable->destroy_chipinterface();
        X4SENSOR_CHECK_OR_GOTO(x4_stat == X4SENSOR_SUCCESS, error);
        // Intentional fall-through
    case X4_RUN_STAGE_DISABLED:
        break;
    }
    run_stage = X4_RUN_STAGE_DISABLED;

    return X4SENSOR_SUCCESS;
error:
    return x4_stat;
}

const x4sensor_configuration_t *
x4sensor_get_configuration()
{
    return config;
}

const x4sensor_info_t *
x4sensor_get_info()
{
    X4SENSOR_CHECK(run_stage >= X4_RUN_STAGE_STOPPED, x4_stat = X4SENSOR_NOT_ALLOWED; return NULL);
    return &info;
}
static uint8_t
x4sensor_cm_to_bin_conv(int16_t range_cm, int16_t length, const int16_t *LUT)
{
    for( int i = 0; i < length; i++ )
    {
        if (range_cm <= LUT[i]) {
            return i;
        }
    }
    return 0;
}
static int16_t
x4sensor_bin_to_cm_conv(uint8_t bin, const int16_t *LUT)
{
    return LUT[bin];
}
uint16_t
x4sensor_get_range_cm()
{
    X4SENSOR_CHECK(run_stage >= X4_RUN_STAGE_STOPPED, x4_stat = X4SENSOR_NOT_ALLOWED; goto error;);
    return x4sensor_bin_to_cm_conv(algorithm_config.detector_last_range_bin, range_lut);
error:
    return 0;
}

uint8_t
x4sensor_get_sensitivity_level()
{
    X4SENSOR_CHECK(run_stage >= X4_RUN_STAGE_STOPPED, x4_stat = X4SENSOR_NOT_ALLOWED; goto error;);
    return sensitivity_level;
error:
    return 0;
}

x4sensor_error_t
x4sensor_set_range_cm(uint16_t centimeters)
{
    X4SENSOR_CHECK_OR_RETURN(run_stage == X4_RUN_STAGE_STOPPED, X4SENSOR_NOT_ALLOWED);

    X4SENSOR_CHECK_OR_RETURN((centimeters >= Range_cm[0]) && (centimeters <= Range_cm[1]),
        X4SENSOR_INVALID_PARAMETER);
    uint8_t det_last_range_bin = x4sensor_cm_to_bin_conv(centimeters, range_bins, range_lut);
    X4SENSOR_CHECK_OR_RETURN(det_last_range_bin != 0, X4SENSOR_INVALID_PARAMETER);

    algorithm_config.detector_first_range_bin = config->detector_FirstRangeBin;
    algorithm_config.detector_last_range_bin = det_last_range_bin;

    return X4SENSOR_SUCCESS;
}

x4sensor_error_t
x4sensor_set_sensitivity_level(uint8_t level)
{
    X4SENSOR_CHECK_OR_RETURN(run_stage == X4_RUN_STAGE_STOPPED, X4SENSOR_NOT_ALLOWED);
    uint8_t max_level = sensitivity_levels + 1;
    uint8_t sens_level = 0;
    for(uint16_t i=0; i < config->PublicParameters_SensitivityLevel.length; i++){
        if(level == sensitivity_levels_indexes[i]){
            sens_level = level; // sensitivity level has to be listed as valid index
        }
    }
    X4SENSOR_CHECK_OR_RETURN(sens_level <= max_level && sens_level > 0, X4SENSOR_INVALID_PARAMETER);
    size_t threshold_bytes = sizeof(uint16_t) * range_bins;
    size_t start_of_vector = (sens_level-1) * range_bins;
    memcpy(&algorithm_config.detector_thresholds, &threshold_vectors[start_of_vector], threshold_bytes);

    memcpy(&algorithm_config.app_logic_M, &M_values[(sens_level-1) * 2], 2);
    memcpy(&algorithm_config.app_logic_N, &N_values[(sens_level-1) * 2], 2);

    sensitivity_level = sens_level;
    return X4SENSOR_SUCCESS;
}

x4sensor_error_t
x4sensor_set_periodic_report_interval(uint16_t period_frames)
{
    X4SENSOR_CHECK_OR_RETURN(run_stage == X4_RUN_STAGE_STOPPED, X4SENSOR_NOT_ALLOWED);
    algorithm_config.sensor_event_period = period_frames;
    return X4SENSOR_SUCCESS;
}

uint16_t
x4sensor_get_periodic_report_interval()
{
    X4SENSOR_CHECK(run_stage >= X4_RUN_STAGE_STOPPED, x4_stat = X4SENSOR_NOT_ALLOWED; goto error;);
    return algorithm_config.sensor_event_period;
error:
    return 0;
}

x4sensor_error_t
x4sensor_start_normal_mode()
{
    return configure_and_start_x4(X4_RUN_MODE_AUTONOMOUS, 0);
}

x4sensor_error_t
x4sensor_start_event_mode(x4sensor_event_flags_t events)
{
    return configure_and_start_x4(X4_RUN_MODE_EVENT, events);
}

x4sensor_error_t
x4sensor_start_recording_mode()
{
    is_recording = true;
    x4sensor_set_periodic_report_interval(1);
    return configure_and_start_x4(X4_RUN_MODE_EVENT, X4SENSOR_EVENT_PERIODIC_REPORT);
}

x4sensor_error_t
x4sensor_stop()
{
    chipinterface_error_t chip_stat;
    X4SENSOR_CHECK_OR_RETURN(run_stage == X4_RUN_STAGE_RUNNING, X4SENSOR_NOT_ALLOWED);

    run_stage = X4_RUN_STAGE_STOPPED;
    is_recording = false;
    chip_stat = chipinterface_set_chip_enabled(false);
    X4SENSOR_CHECK_OR_RETURN(chip_stat == CHIPINTERFACE_SUCCESS, X4SENSOR_CHIPINTERFACE_ERROR);

    return X4SENSOR_SUCCESS;
}

size_t
x4sensor_get_sensor_data(uint8_t *buffer, size_t max_size)
{
    X4SENSOR_CHECK(run_stage == X4_RUN_STAGE_RUNNING, x4_stat = X4SENSOR_NOT_ALLOWED; goto error;);
    X4SENSOR_CHECK(run_mode == X4_RUN_MODE_EVENT, x4_stat = X4SENSOR_NOT_ALLOWED; goto error;);
    if(x4sensor_is_recording()){
        X4SENSOR_CHECK(max_size >= x4sensor_get_max_sensor_data_size_recording_mode(), x4_stat = X4SENSOR_INVALID_PARAMETER; goto error;);
    }else{
        X4SENSOR_CHECK(max_size >= x4sensor_get_max_sensor_data_size_event_mode(), x4_stat = X4SENSOR_INVALID_PARAMETER; goto error;);
    }
    size_t bytes_read = 0;
    for(int attempts = x4sensor_get_retry_count(); attempts > 0; --attempts){
        x4_stat = vtable->read_recording_data(buffer, max_size, &bytes_read);
        if((x4_stat == X4SENSOR_SUCCESS) || (x4_stat == X4SENSOR_FRAME_COUNTER_NOT_INCREASED)){
            break;
        }
    }
    if (x4_stat != X4SENSOR_SUCCESS){
        disable_x4();
    }else{
        x4_stat = vtable->clear_interrupt();
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
        return bytes_read;
    }
error:
    return 0;
}

x4sensor_error_t x4sensor_get_distance_cluster(const uint8_t *buffer, x4sensor_distance_cluster_t* dist_cluster){
    if(x4sensor_get_distance_cluster_length() == 0){
        x4_stat = X4SENSOR_FEATURE_NOT_SUPPORTED;
        return x4_stat;
    }
    const payload_t *payload=(const payload_t *)buffer;
    memset(dist_cluster, 0, sizeof(x4sensor_distance_cluster_t));
    if(payload->distanceClusterFirstBinAboveThresholdIndex == 0xff){
        dist_cluster->detector_hit = false;
        dist_cluster->first_detection_bin_distance_mm = 0;
    }else{
        dist_cluster->detector_hit = true;
        dist_cluster->first_detection_bin_distance_mm = x4sensor_bin_to_cm_conv(payload->distanceClusterFirstBinAboveThresholdIndex, range_lut) * 10;
        bool end_reached = false;
        for(int i=0; i<config->detector_DistanceClusterLength; i++){
            if(i == payload->distanceClusterIndex + 1){ // detect premature end. distanceClusterIndex counts bins after detector_hit
                end_reached = true;
            }
            if(end_reached == true){
                dist_cluster->bin_power[i] = 0;
            }else{
                dist_cluster->bin_power[i] = payload->distanceClusterBinsPower[i];
            }
        }
    }
    return X4SENSOR_SUCCESS;
}

uint8_t x4sensor_get_distance_cluster_length(){
    return config->detector_DistanceClusterLength;
}

uint8_t x4sensor_get_distance_cluster_first_bin_number(const uint8_t *buffer){
    const payload_t *payload=(const payload_t *)buffer;
    return payload->distanceClusterFirstBinAboveThresholdIndex;
}

uint16_t x4sensor_get_distance_between_bins_mm(){
    return (uint16_t)config->FrameConfig_RangeBinLength_mm*config->range_decimation_DecimFactor;
}


bool
x4sensor_get_detection_state(const uint8_t *buffer)
{
    const payload_t *payload=(const payload_t *)buffer;
    return payload->detection.presence;
}

x4sensor_event_flags_t
x4sensor_get_events(const uint8_t *buffer)
{
    const payload_t *payload=(const payload_t *)buffer;
    return payload->events;
}

uint32_t
x4sensor_get_frame_counter(const uint8_t *buffer)
{
    const payload_t *payload=(const payload_t *)buffer;
    return payload->frame_counter;
}

uint8_t
x4sensor_get_frame_rate()
{
    X4SENSOR_CHECK(run_stage >= X4_RUN_STAGE_STOPPED, x4_stat = X4SENSOR_NOT_ALLOWED; goto error;);
    return config->ChipX4_FPS;
error:
    return 0;
}

static x4sensor_error_t
set_test_mode(x4_test_mode_t mode)
{
    if((mode == X4_TEST_MODE_M1) || (mode == X4_TEST_MODE_M2) || (mode == X4_TEST_MODE_M1_KCC) || (mode == X4_TEST_MODE_M2_KCC)){
        // powerup_CrystalOSC
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x4A, 0x02)) == X4SENSOR_SUCCESS, goto error); // xosc_en = 1
        uint8_t lock_status = 0;
        do{
            X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x0072, &lock_status)) == X4SENSOR_SUCCESS, goto error);
        }while((lock_status & 0x40) != 0x40); // wait for xosc_lock == 1
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x4A, 0x62)) == X4SENSOR_SUCCESS, goto error); // auxclk_sel=1 sysclk_sel=1 en=1
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x4A, 0x63)) == X4SENSOR_SUCCESS, goto error); // above + lpclk_disable=1 (can't do all in one go)

        // powerup_CommonPLL()
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x006A, 0x60)) == X4SENSOR_SUCCESS, goto error);
        lock_status = 0;
        do{
            X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x0072, &lock_status)) == X4SENSOR_SUCCESS, goto error);
        }while((lock_status & 0x80) != 0x80);

        // power up LDOs
        uint8_t dvdd_rx_ctrl = 0;
        X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x0076, &dvdd_rx_ctrl)) == X4SENSOR_SUCCESS, goto error);
        dvdd_rx_ctrl &= 0x1F;
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0076, dvdd_rx_ctrl)) == X4SENSOR_SUCCESS, goto error);
        uint8_t dvdd_tx_ctrl = 0;
        X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x0078, &dvdd_tx_ctrl)) == X4SENSOR_SUCCESS, goto error);
        dvdd_tx_ctrl &= 0x1F;
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0078, dvdd_tx_ctrl)) == X4SENSOR_SUCCESS, goto error);

        uint8_t avdd_rx_ctrl = 0;
        X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x007A, &avdd_rx_ctrl)) == X4SENSOR_SUCCESS, goto error);
        avdd_rx_ctrl &= 0x1F;
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x007A, avdd_rx_ctrl)) == X4SENSOR_SUCCESS, goto error);
        uint8_t avdd_tx_ctrl = 0;
        X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x007B, &avdd_tx_ctrl)) == X4SENSOR_SUCCESS, goto error);
        avdd_tx_ctrl &= 0x1F;
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x007B, avdd_tx_ctrl)) == X4SENSOR_SUCCESS, goto error);

        uint8_t ldo_status_2 = 0;
        do{
            x4_stat = vtable->get_register(0x8000 | 0x007E, &ldo_status_2);
            X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
        }while((ldo_status_2 & 0xf) != 0xf);

        // powerup_RxTxPLL
        uint8_t common_pll_ctrl_4 = 0;
        x4_stat = vtable->get_register(0x8000 | 0x006D, &common_pll_ctrl_4);
        X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
        bool must_start = ((common_pll_ctrl_4 & (1<<6)) == 0);
        bool has_lock = false;
        do{
            if(must_start){ // must start
                X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x006D, &common_pll_ctrl_4)) == X4SENSOR_SUCCESS, goto error);
                X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x006D, common_pll_ctrl_4 | (1<<6))) == X4SENSOR_SUCCESS, goto error); // Enable common PLL external bypass

                //powerup_RxTxPLL
                X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0061, 0x02 | 0x01)) == X4SENSOR_SUCCESS, goto error);
                X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0066, 0x02 | 0x01)) == X4SENSOR_SUCCESS, goto error);
            }
            X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x006D, common_pll_ctrl_4 & ~(1<<6))) == X4SENSOR_SUCCESS, goto error); // Disable common PLL external bypass

            uint8_t rx_pll_status = 0;
            x4_stat = vtable->get_register(0x8000 | 0x0064, &rx_pll_status);
            X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
            uint8_t tx_pll_status = 0;
            x4_stat = vtable->get_register(0x8000 | 0x0069, &tx_pll_status);
            X4SENSOR_CHECK(x4_stat == X4SENSOR_SUCCESS, goto error);
            has_lock = ((((tx_pll_status & 0x80) == 0x80) && ((rx_pll_status & 0x80) == 0x80)));
            if(!has_lock){
                // Stop rx and tx PLLS
                X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0061, 0x03)) == X4SENSOR_SUCCESS, goto error); //rx_pll_ctrl_2
                X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0066, 0x03)) == X4SENSOR_SUCCESS, goto error); //tx_pll_ctrl_2
                must_start = true;
            }
        }while(!has_lock);
        uint8_t misc_ctrl = 0;
        X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x0075, &misc_ctrl)) == X4SENSOR_SUCCESS, goto error);
        if((mode == X4_TEST_MODE_M1) || (mode == X4_TEST_MODE_M1_KCC)){
            X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0075, misc_ctrl | 0x60)) == X4SENSOR_SUCCESS, goto error); // tx_power = 3
        }else{ // (mode == X4_TEST_MODE_M2) || (mode == X4_TEST_MODE_M2_KCC)
            X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0075, misc_ctrl & ~0x60)) == X4SENSOR_SUCCESS, goto error); // tx_power = 0
        }
        // powerup_samplers
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x000F, 0x01)) == X4SENSOR_SUCCESS, goto error);
        if((mode == X4_TEST_MODE_M1) || (mode == X4_TEST_MODE_M2)){
            X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x002C, 0x0E)) == X4SENSOR_SUCCESS, goto error); // trx_clocks_per_pulse = 14
        }else{ // (mode == X4_TEST_MODE_M1_KCC) || (mode == X4_TEST_MODE_M2_KCC)
            X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x002C, 0x05)) == X4SENSOR_SUCCESS, goto error); // trx_clocks_per_pulse = 5
        }
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0035, 0x64)) == X4SENSOR_SUCCESS, goto error); // trx_iterations = 100

        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x003B, 0x88)) == X4SENSOR_SUCCESS, goto error); // trx_send_every_pulse = 1, rx_strobe_enable = 0, tx_strobe_enable = 1
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0036, 0xff)) == X4SENSOR_SUCCESS, goto error); // trx_start

    }else if((mode == X4_TEST_MODE_M3) || (mode == X4_TEST_MODE_NORMAL)){

        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x002C, 0x0E)) == X4SENSOR_SUCCESS, goto error); // trx_clocks_per_pulse = 14
        X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0035, 0x64)) == X4SENSOR_SUCCESS, goto error); // trx_iterations = 100

        uint8_t misc_ctrl = 0;
        X4SENSOR_CHECK((vtable->get_register(0x8000 | 0x0075, &misc_ctrl)) == X4SENSOR_SUCCESS, goto error);

        if(mode == X4_TEST_MODE_M3){
            X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x0075, misc_ctrl & ~0x60)) == X4SENSOR_SUCCESS, goto error); // tx_power = 0
        }
    }else if((mode == X4_TEST_MODE_FCC10_TX0) || (mode == X4_TEST_MODE_FCC10_TX3)){
        if(mode == X4_TEST_MODE_FCC10_TX0){
            X4SENSOR_CHECK((vtable->set_register(0x8000 | 0x001C, 0x2D )) == X4SENSOR_SUCCESS, goto error); // rx_counter_lsb = 45
        }

    }
    return X4SENSOR_SUCCESS;
error:
    return X4SENSOR_FAILURE;
}

x4sensor_error_t
x4sensor_start_test_mode(x4_test_mode_t test_mode)
{
    X4SENSOR_CHECK(x4sensor_start_recording_mode() == X4SENSOR_SUCCESS, return x4_stat);
    X4SENSOR_CHECK_OR_RETURN(chipinterface_wait_for_interrupt(200000) == CHIPINTERFACE_SUCCESS, X4SENSOR_CHIPINTERFACE_ERROR);
    X4SENSOR_CHECK(set_test_mode(test_mode) == X4SENSOR_SUCCESS, return x4_stat);
    if ((test_mode != X4_TEST_MODE_M1) && (test_mode != X4_TEST_MODE_M2) && (test_mode != X4_TEST_MODE_M1_KCC) && (test_mode != X4_TEST_MODE_M2_KCC))
        x4_stat = vtable->clear_interrupt();
    return x4_stat;
}

size_t
x4sensor_get_max_sensor_data_size_recording_mode()
{
    size_t size = config->FrameConfig_RangeBins * sizeof(x4_sample_t);
    size += sizeof(payload_t);
    return size;
}

size_t
x4sensor_get_max_sensor_data_size_event_mode()
{
    size_t size = sizeof(payload_t);
    return size;
}

uint8_t
x4sensor_get_number_of_radar_bins()
{
    return config->FrameConfig_RangeBins;
}

bool
x4sensor_is_recording()
{
    return is_recording;
}

uint8_t
x4sensor_make_firmware_hash()
{
    uint8_t val = 0;
    for (size_t i = 0; i < firmware_size; ++i) {
        val = (val ^ firmware_data[i]) + 47;
    }
    return val;
}

x4sensor_error_t
x4sensor_get_last_error()
{
    return x4_stat;
}

const char*
x4sensor_convert_error_to_string(x4sensor_error_t error)
{
    if (error == X4SENSOR_SUCCESS)
        return "No error.";
    else if (error == X4SENSOR_FAILURE)
        return "An error happened.";
    else if (error == X4SENSOR_CONFIGURATION_INVALID_SIZE)
        return "The configuration blob size is wrong.";
    else if (error == X4SENSOR_CONFIGURATION_INVALID_DATA)
        return "The configuration blob data is corrupt.";
    else if (error == X4SENSOR_CONFIGURATION_INVALID_TYPE)
        return "The configuration blob has the wrong type.";
    else if (error == X4SENSOR_CONFIGURATION_INVALID_LAYOUT)
        return "The configuration blob data format is not compatible to this x4sensor build.";
    else if (error == X4SENSOR_FIRMWARE_INCOMPATIBLE_INTERFACE_VERSION)
        return "The firmware interface is not compatible to this version of x4sensor lib.";
    else if (error == X4SENSOR_FIRMWARE_VERIFICATION_FAILED)
        return "An error happened while verfiying the firmware. Check the hardware connection.";
    else if (error == X4SENSOR_CHIPINTERFACE_ERROR)
        return "An error happened in a call to a chipinterface function. This indicates either a hardware problem or a mistake in the chipinterface implementation.";
    else if (error == X4SENSOR_SENSOR_DISCOVERY_FAILED)
        return "The initial contact to the sensor hardware failed. Check the hardware connection.";
    else if (error == X4SENSOR_INVALID_PARAMETER)
        return "A function was called with one or more invalid parameter values.";
    else if (error == X4SENSOR_NOT_ALLOWED)
        return "A function was called while it was not allowed.";
    else if (error == X4SENSOR_OSCILLATOR_FREQUENCY_NOT_PLAUSIBLE)
        return "The measured X4 oscillator frequency is outside the plausible range. This indicates a problem in the chipinterface implementation.";
    else if (error == X4SENSOR_FEATURE_NOT_SUPPORTED)
        return "The feature is not configured or not available.";
    else if (error == X4SENSOR_FRAME_COUNTER_NOT_INCREASED)
        return "The frame counter has not increased.";
    else if (error == X4SENSOR_DATA_NOT_READY)
        return "Sensor data is not yet ready.";
    else
       return "Invalid error code";
}

