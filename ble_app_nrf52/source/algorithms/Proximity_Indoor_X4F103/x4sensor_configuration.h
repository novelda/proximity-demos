/*
* Copyright Novelda AS 2024.
*/
#ifndef X4SENSOR_CONFIGURATION_H
#define X4SENSOR_CONFIGURATION_H

#include "x4_common.h"

#include <stddef.h>
#include <stdint.h>

#define CONFIGURATION_FORMAT_HASH 0xDE497A2E

#define DISTANCE_CLUSTER_LENGTH ((uint8_t)( 6 ))
X4_PACK_START()

typedef struct {
    uint16_t offset;
    uint16_t length;
} x4sensor_configuration_array_t;

typedef struct x4sensor_configuration {
    x4sensor_configuration_array_t firmware;
    x4sensor_configuration_array_t detector_DetectorThresholds;
    x4sensor_configuration_array_t app_logic_N;
    x4sensor_configuration_array_t app_logic_M;
    x4sensor_configuration_array_t ChipX4_CoeffsQ;
    x4sensor_configuration_array_t ChipX4_CoeffsI;
    x4sensor_configuration_array_t detector_RangeLookUpTable;
    x4sensor_configuration_array_t PublicParameters_SensitivityLevel;
    x4sensor_configuration_array_t PublicParameters_Range_cm;
    uint16_t detector_Range_cm;
    uint8_t detector_DistanceClusterLength;
    uint8_t FrameConfig_RangeBins;
    uint8_t range_decimation_DecimFactor;
    uint8_t FrameConfig_RangeBinLength_mm;
    uint8_t ChipX4_FPS;
    uint8_t detector_FirstRangeBin;
    uint8_t detector_SensitivityLevel;
} x4sensor_configuration_t;

X4_PACK_END()

#endif // X4SENSOR_CONFIGURATION_H


