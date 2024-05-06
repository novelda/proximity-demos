/*
* Copyright Novelda AS 2024.
*/
#ifndef NOVELDA_X4SENSOR_PRIVATE_H
#define NOVELDA_X4SENSOR_PRIVATE_H

#include "novelda_x4sensor.h"
#include "novelda_chipinterface.h"
#include "x4_interface_common.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C"
{
#endif

#define X4SENSOR_CHECK(condition, action) \
	{ \
		if (condition) { \
			; \
		} else { \
			action; \
		} \
	}

#define X4SENSOR_CHECK_OR_GOTO(condition, label) \
	X4SENSOR_CHECK(condition, goto label)

#define X4SENSOR_CHECK_OR_RETURN(condition, error_code) \
	X4SENSOR_CHECK(condition, x4_stat = error_code; return x4_stat)

typedef x4sensor_error_t (*read_recording_data_func)(uint8_t *buffer, size_t max_size, size_t* bytes_read);
typedef x4sensor_error_t (*set_run_mode_func)(x4_run_mode_t mode, x4sensor_event_flags_t events);
typedef x4sensor_error_t (*get_register_func)(uint16_t addres, uint8_t *value);
typedef x4sensor_error_t (*set_register_func)(uint16_t addres, uint8_t value);
typedef x4sensor_error_t (*write_config_func)(const rw_config_t *config);
typedef x4sensor_error_t (*destroy_chipinterface_func)(void);
typedef x4sensor_error_t (*upload_firmware_fxn)(const uint8_t *firmware, size_t size);
typedef x4sensor_error_t (*start_lposc_measurement_func)(uint8_t nticks);
typedef x4sensor_error_t (*clear_interrupt_func)();
typedef x4sensor_error_t (*discover_sensor_func)(x4sensor_info_t *info);

typedef struct x4sensor_configuration x4sensor_configuration_t;

static const uint16_t x4sensor_configuration_magic_word = 0xDA1A;

typedef struct {
	upload_firmware_fxn upload_firmware;
	set_run_mode_func set_run_mode;
	get_register_func get_register;
	set_register_func set_register;
	write_config_func write_config;
	read_recording_data_func read_recording_data;
	destroy_chipinterface_func destroy_chipinterface;
	start_lposc_measurement_func start_lposc_measurement;
	clear_interrupt_func clear_interrupt;
	discover_sensor_func discover_sensor;
} x4sensor_vtable_t;

X4_PACK_START();

// Used for identifying and validating blobs that have been
// generated from a SignalFlow algorithm.
static const uint8_t x4sensor_blob_header_version = 2;
typedef struct {
	uint16_t magic_word;
	uint8_t header_version;
	uint8_t blob_hash;
	uint32_t blob_size;
	uint32_t algorithm_variant_hash;
	union {
		uint32_t algorithm_commit_hash; // header version 1
		uint8_t algorithm_version[3];   // header version 2
	};
	uint32_t firmware_commit_hash;
	uint32_t configuration_format_hash;
} x4sensor_blob_header_t;

X4_PACK_END();

extern const x4sensor_vtable_t x4sensor_vtable_i2c;
extern const x4sensor_vtable_t x4sensor_vtable_spi;

X4_SYMBOL_EXPORT const x4sensor_configuration_t *x4sensor_get_configuration();
X4_SYMBOL_EXPORT bool x4sensor_is_recording();
X4_SYMBOL_EXPORT uint8_t x4sensor_get_number_of_radar_bins();
X4_SYMBOL_EXPORT uint8_t x4sensor_make_firmware_hash();
X4_SYMBOL_EXPORT uint32_t x4sensor_get_frame_counter(const uint8_t *buffer);
X4_SYMBOL_EXPORT uint16_t x4sensor_get_frame_delay(const uint8_t *buffer);
X4_SYMBOL_EXPORT int16_t *x4sensor_get_radar_samples(const uint8_t *buffer);
X4_SYMBOL_EXPORT void x4sensor_inc_retries_total_count(void);

// FIXME!?!?!?: Explicitly expose aliases for the direct chipinterface calls used by X4Sensor node
X4_SYMBOL_EXPORT void x4sensor_chipinterface_set_interface_type(uint8_t interface_type);
X4_SYMBOL_EXPORT void x4sensor_chipinterface_clear_interrupt_callback( void );
X4_SYMBOL_EXPORT chipinterface_error_t x4sensor_chipinterface_wait_us( uint32_t microseconds );
X4_SYMBOL_EXPORT chipinterface_error_t x4sensor_chipinterface_get_interrupt_state( chipinterface_interrupt_state_t* state );
X4_SYMBOL_EXPORT chipinterface_error_t x4sensor_chipinterface_wait_for_interrupt( uint32_t microseconds );

#ifdef __cplusplus
}
#endif

#endif // NOVELDA_X4CONTROLLER_PRIVATE_H

