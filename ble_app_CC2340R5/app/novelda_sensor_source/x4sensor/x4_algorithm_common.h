/*
* Copyright Novelda AS 2024.
*/
#pragma once

//
// This file is considered Novelda-internal. It shall not be exposed to public code that
// interfaces to the X4. Use x4_interface_common.h for such structs and defines instead.
//

#include <x4_common.h>
#include <x4_interface_common.h>
#include "x4sensor_configuration.h"
#include <stdbool.h>
#include <stdint.h>

#include "x4_symbol_export.h"

#ifndef X4_SYMBOL_EXPORT
#define X4_SYMBOL_EXPORT
#endif

#define PROXIMITY_MINIMAL_NN_INPUT_SIZE 5
#define PROXIMITY_MINIMAL_NN_START_BIN 5
#define PROXIMITY_MINIMAL_NN_END_BIN 13
#define OCCUPANCY_FILTER_MAX_LENGTH 11
#define PROXIMITY_MN_PARAM_SET_SIZE 5

#ifdef __cplusplus
extern "C" {
#endif


X4_PACK_START()


typedef union {
    struct {
        int16_t i;
        int16_t q;
    };
    int16_t iq[2];
    uint8_t bytes[4];
} x4_sample_t;

typedef struct {
    uint8_t events;
    uint8_t bin;
    uint32_t frame_counter;
    uint16_t frame_delay;   // delay of the radar start of this frame
                            // due to communication delays in the previous frame
                            // in slow oscillator ticks (56,9us per tick)
    union {
        x4_sample_t sample;
        uint32_t power;
        int8_t nn_buf[5];
    };
    struct {
        uint32_t power;
        uint8_t bin;
        bool presence;
    } detection;
    uint8_t distanceClusterFirstBinAboveThresholdIndex;
    uint8_t distanceClusterIndex;
    uint32_t distanceClusterBinsPower[DISTANCE_CLUSTER_LENGTH];
} payload_t;

typedef struct {
    uint8_t range_decimation_DecimFactor;
    const int8_t X4_CODE *range_decimation_DecimFilter;
    uint8_t range_decimation_normFactor;
    bool apn_Enable;
    uint8_t apn_RefBin;
    uint8_t static_removal_StaticRemovalWeightFactor;
    uint8_t static_removal_StaticRemovalDCMapInitFilter;
    uint8_t static_removal_StaticRemovalSampleShift;
    uint8_t iir_smoothing_A;
    uint8_t iir_smoothing_B;
    uint8_t iir_init_filter;
    uint8_t app_logic_WalkTestMinRange;
    uint8_t app_logic_WalkTestThreshold;
    uint8_t FrameConfig_RangeBins;
    uint8_t detector_occupancy_N;
    uint8_t detector_occupancy_M;
    uint8_t app_logic_occupancy_BinsOutsideZone;
    uint8_t detector_DistanceClusterLength;
    rw_config_t X4_XDATA *rw_config;
} config_t;

typedef void X4_XDATA* node_t;

X4_PACK_END()

extern X4_XDATA payload_t payload;
extern X4_CODE const config_t global_config;

void algorithm_init();
bool algorithm_has_detection();
void algorithm_run_once();
uint16_t algorithm_get_sweep_period();
uint16_t algorithm_get_sensor_event_period();
void algorithm_set_frame_delay(uint16_t ticks);
void algorithm_reset_payload();
void algorithm_set_sensor_events(uint8_t events);
uint8_t algorithm_get_sensor_events();

#ifdef __cplusplus
}
#endif

