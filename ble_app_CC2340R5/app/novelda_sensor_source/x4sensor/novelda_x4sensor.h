/*
* Copyright Novelda AS 2024.
*/
#ifndef NOVELDA_X4SENSOR_H
#define NOVELDA_X4SENSOR_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "x4sensor_configuration.h"
// For Novelda internal use only
#if defined(BUILDING_NOVELDA_X4SENSOR) || defined(USING_NOVELDA_X4SENSOR)
#include "x4_symbol_export.h"
#endif

#ifndef X4_SYMBOL_EXPORT
#define X4_SYMBOL_EXPORT
#endif


#ifdef __cplusplus
extern "C"
{
#endif

/**
 * :file: novelda_x4sensor.h
 *
 * The X4Sensor C API interfaces a stand-alone detection algorithm running on
 * the X4 hardware sensor to a bare metal host application. It takes care of
 * the low-level communication with the sensor.
 *
 * The application life-cycle is as follows:
 *
 * 1. Initialize the library and the sensor with
 *    :c:func:`x4sensor_initialize_i2c` or :c:func:`x4sensor_initialize_spi`.
 *
 * 2. Optionally configure the library with :c:func:`x4sensor_set_range_cm` and
 *    :c:func:`x4sensor_set_sensitivity_level`.
 *
 * 3. Start the sensor operation
 *
 *    The sensor can operate in the following modes:
 *
 *    **Normal operation mode:**
 *
 *    - started by :c:func:`x4sensor_start_normal_mode`
 *    - reports detections via a GPIO pin
 *    - optimized for power consumption
 *
 *    **Event mode:**
 *
 *    - an event-based operation mode where the host is notified in case
 *      of a subscribed event.
 *    - started by :c:func:`x4sensor_start_event_mode`
 *    - reports events by asserting a GPIO pin
 *    - frame data must be read out via :c:func:`x4sensor_get_sensor_data`
 *
 *    **Recording mode:**
 *
 *    - reads out internal data from the hardware sensor and is used
 *      for tests and validation.
 *    - started by :c:func:`x4sensor_start_recording_mode`
 *    - reports a new frame by asserting a GPIO pin
 *    - frame data must be read out via :c:func:`x4sensor_get_sensor_data`
 *
 * 4. The sensor may be stopped by calling :c:func:`x4sensor_stop`.
 *
 */

/**
 * :brief: Error codes used in the X4Sensor API
 */
typedef int8_t x4sensor_error_t;
/** The function call was successful */
static const x4sensor_error_t X4SENSOR_SUCCESS = 0;
/** An unspecified error happened */
static const x4sensor_error_t X4SENSOR_FAILURE = -1;
/** The blob size is wrong */
static const x4sensor_error_t X4SENSOR_CONFIGURATION_INVALID_SIZE = -2;
/** Blob data corrupted */
static const x4sensor_error_t X4SENSOR_CONFIGURATION_INVALID_DATA = -3;
/** Blob seems valid but has wrong type */
static const x4sensor_error_t X4SENSOR_CONFIGURATION_INVALID_TYPE = -4;
/** Blob data format is not compatible to this x4sensor build */
static const x4sensor_error_t X4SENSOR_CONFIGURATION_INVALID_LAYOUT = -5;
/** The X4 firmware interface is not compatible to this version of x4sensor lib. */
static const x4sensor_error_t X4SENSOR_FIRMWARE_INCOMPATIBLE_INTERFACE_VERSION = -10;
/** An error happened while verifying the firmware. Check the hardware connection. */
static const x4sensor_error_t X4SENSOR_FIRMWARE_VERIFICATION_FAILED = -11;
 /** An error happened in a call to a chipinterface function */
static const x4sensor_error_t X4SENSOR_CHIPINTERFACE_ERROR = -12;
/** Initial contact to the sensor failed, most likely a connection problem */
static const x4sensor_error_t X4SENSOR_SENSOR_DISCOVERY_FAILED = -13;
/** A function was called with one or more invalid parameter values */
static const x4sensor_error_t X4SENSOR_INVALID_PARAMETER = -14;
/** A function was called while it was not allowed */
static const x4sensor_error_t X4SENSOR_NOT_ALLOWED = -15;
/** The measured X4 oscillator frequency is outside the plausible range */
static const x4sensor_error_t X4SENSOR_OSCILLATOR_FREQUENCY_NOT_PLAUSIBLE = -16;
/** The feature is not configured/supported */
static const x4sensor_error_t X4SENSOR_FEATURE_NOT_SUPPORTED = -17;
/** The frame counter not increased */
static const x4sensor_error_t X4SENSOR_FRAME_COUNTER_NOT_INCREASED = -18;
/** The sensor data is not ready to be read*/
static const x4sensor_error_t X4SENSOR_DATA_NOT_READY = -19;

/**
 * :brief: A single event flag
 *
 * The event mode is started by :c:func:`x4sensor_start_event_mode`.
 */
typedef enum x4sensor_event_flag_t {
    /**
     * Enables periodic reports from the sensor. Can be used for keep-alive
     * checks of the sensor hardware or just for periodic updates of the host.
     * The report interval is configured with
     * :c:func:`x4sensor_set_periodic_report_interval`.
     */
    X4SENSOR_EVENT_PERIODIC_REPORT = 1 << 0,

    /**
     * Enables notifications when the detetion state changes. Depending on the
     * algorithm configuration this may be the proximity or occupancy state.
     */
    X4SENSOR_EVENT_STATE_CHANGE = 1 << 1
} x4sensor_event_flag_t;

/**
 * :brief: A combination of :c:enum:`x4sensor_event_flag_t` event flags
 */
typedef uint8_t x4sensor_event_flags_t;

/**
 * :brief: Various test modes used in :c:func:`x4sensor_start_test_mode`.
 */
typedef enum x4_test_mode_t {
    X4_TEST_MODE_M1 = 0,
    X4_TEST_MODE_M2 = 1,
    X4_TEST_MODE_M3 = 2,
    X4_TEST_MODE_NORMAL = 3,
    X4_TEST_MODE_FCC10_TX0 = 4,
    X4_TEST_MODE_FCC10_TX3 = 5,
    X4_TEST_MODE_M1_KCC = 6,
    X4_TEST_MODE_M2_KCC = 7
} x4_test_mode_t;

/**
 * :brief: Meta information about the sensor and the firmware
 */
typedef struct x4sensor_info_t {
    /** The hardware sensor's serial number */
    uint32_t sample_id;
    /** The hardware sensor's revision id */
    uint8_t  chip_revision;
    /** Major, minor and patch number of the algorithm version */
    uint8_t algorithm_version[3];
    /** A firmware version identifier, little endian */
    uint32_t firmware_version;
    /** Identifies the algorithm version, little endian */
    uint32_t algorithm_commit_hash;
    /** Identifies the algorithm variant, little endian */
    uint32_t algorithm_variant_hash;
} x4sensor_info_t;


/**
 * :brief: A window of detection power values
 *
 * A distance cluster contains the signal power values per range bin around the
 * first detector hit. A detector hit occurs when the processed radar signal
 * exceeds the configured sensitivity threshold at a certain range bin.
 *
 * The distance cluster starts one range bin before the first hit and ends
 * multiple bins later. The actual length of the distance cluster is
 * preconfigured by the algorithm and can be obtained from
 * :c:func:`x4sensor_get_distance_cluster_length`.
 *
 * The :c:member:`x4sensor_distance_cluster_t.first_detection_bin_distance_mm`
 * member is the distance from the sensor to the range bin with the detection
 * hit. That is the second entry in the cluster.
 *
 * :See: :c:func:`x4sensor_set_sensitivity_level`,
 *       :c:func:`x4sensor_get_distance_cluster_length`
 */
typedef struct x4sensor_distance_cluster_t {
    /** True if the cluster has power values above the threshold */
    bool detector_hit;
    /** Distance in mm to first bin above the threshold */
    uint16_t first_detection_bin_distance_mm;
    /** Signal power per range bin */
    uint32_t bin_power[DISTANCE_CLUSTER_LENGTH];
} x4sensor_distance_cluster_t;

/**
 *  :brief: Set number of retransmition attemts
 *
 *  This function sets up the number of times the communication
 *  will be retried before the read/write function returns error.
 *  Default value of 5 is used if user doesn't set otherwise.
 *
 *  :param retry_count: a value of retry number
 *  :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_set_retry_count(uint8_t retry_count);

/**
 *  :brief: Get number of retransmition attemts
 *
 *  This function returns the number of times the communication
 *  will be retried before the read/write function returns error.
 *
 *  :return: number of retry attempts
 */
X4_SYMBOL_EXPORT uint8_t x4sensor_get_retry_count(void);

/**
 *  :brief: Get a total number of retransmition attemts made
 *
 *  This function returns the total number of retries that has been made
 *  since X4 sensor initialization.
 *
 *  :return: number of retries made
 */
X4_SYMBOL_EXPORT uint32_t x4sensor_get_retries_total_count(void);

/**
 *  :brief: Initializes and sets up the X4Sensor library for I2C communication
 *
 *  This function sets up the low-level chip interface and prepares the
 *  hardware sensor for operation. It is the first function to be called.
 *
 *  :param configuration: a pointer to the configuration binary blob
 *  :param configuration_size: the configuration size
 *  :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_initialize_i2c(const uint8_t *configuration, size_t configuration_size);

/**
 * :brief: Initializes and sets up the X4Sensor library for SPI communication
 *
 * This function sets up the low-level chip interface and prepares the
 * hardware sensor for operation. It is the first function to be called.
 *
 * :param configuration: a pointer to the configuration binary blob
 * :param configuration_size: the configuration size
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_initialize_spi(const uint8_t *configuration, size_t configuration_size);

/**
 * :brief: Frees up all resources used by X4Sensor
 *
 * This function may be called when the X4Sensor is no longer needed. After
 * that, one might start over again with :c:func:`x4sensor_initialize_i2c` or
 * :c:func:`x4sensor_initialize_spi`. This function may be called at any time,
 * even without stopping the sensor.
 *
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_deinitialize();

/**
 * :brief: Shows meta information about the sensor and the firmware
 *
 * This function returns a pointer to a struct with meta information about the
 * sensor and its firmware. It may be called at any time after initializing
 * the X4Sensor library.
 *
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT const x4sensor_info_t *x4sensor_get_info();

/**
 * :brief: Returns the configured detection range
 *
 * The value returned may differ from desired range inputted to
 * :c:func:`x4sensor_set_range_cm` function.
 * If no range has been configured, a default value is returned.
 *
 * :See: :c:func:`x4sensor_set_range_cm`
 * :return: detection range in centimeters
 */
X4_SYMBOL_EXPORT uint16_t x4sensor_get_range_cm();

/**
 * :brief: Returns the configured sensitivity level
 *
 * If no sensitivity has been configured, a default value is returned.
 *
 * :return: sensitivity level
 *
 * :See: :c:func:`x4sensor_set_sensitivity_level`
 */
X4_SYMBOL_EXPORT uint8_t x4sensor_get_sensitivity_level();

/**
 * :brief: Returns the sensor frame rate
 *
 * :return: frame rate in frames per second
 */
X4_SYMBOL_EXPORT uint8_t x4sensor_get_frame_rate();

/**
 * :brief: Sets the maximum detection range
 *
 * This function allows the user to set the maximum detection range of the
 * sensor. Since the radar sensor divides the detection zone in range bins,
 * the observed range might not match the configured value exactly.
 *
 * This function may be called after initialization and while the sensor is
 * stopped.
 *
 * :param centimeters: the desired maximum detection range in centimeters
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_set_range_cm(uint16_t centimeters);

/**
 * :brief: Sets the sensitivity level
 *
 * The sensitivity level configures the sensitivity of the sensor. Each
 * firmware comes with a set of sensitivity thresholds. The meaning of the
 * different levels depends on the actual firmware. A higher value means
 * usually "more sensitive." The maximum value depends on the actual firmware
 * and algorithm and can be obtained from the release documentation.
 *
 * This function may be called after initialization and while the sensor is
 * stopped.
 *
 * :param level: a value greater than 0
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_set_sensitivity_level(uint8_t level);

/**
 * :brief: Sets the interval length of periodic sensor reports
 *
 * This function sets the interval length of periodic sensor reports. The
 * interval length is specified in number of frames. This option is only
 * effective when the sensor is started in event mode with
 * :c:member:`x4sensor_event_flag_t.X4SENSOR_EVENT_PERIODIC_REPORT` subscribed.
 *
 * This function may be called after initialization and while the sensor is
 * stopped.
 *
 * :param frames: the interval length in frames
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 *
 * :See: :c:func:`x4sensor_get_frame_rate`, :c:func:`x4sensor_start_event_mode`
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_set_periodic_report_interval(uint16_t frames);

/**
 * :brief: Gets the interval length of periodic sensor reports
 *
 * This function gets the interval length of periodic sensor reports.
 *
 * This function may be called while the sensor is running.
 *
 * :return: interval length in frames
 *
 * :See: :c:func:`x4sensor_set_periodic_report_interval`,
 *       :c:func:`x4sensor_start_event_mode`
 */
X4_SYMBOL_EXPORT uint16_t x4sensor_get_periodic_report_interval();

/**
 * :brief: Starts the hardware sensor in normal operation mode
 *
 * This function starts the sensor in normal operation mode and returns. The
 * hardware sensor runs autonomously and reports detections by asserting an IO
 * pin. Operation can be stopped using :c:func:`x4sensor_stop`.
 *
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_start_normal_mode();

/**
 * :brief: Starts the hardware sensor in event mode
 *
 * This function starts the sensor in event mode and returns. Several events
 * may be subscribed by specifying a combination of event flags.
 *
 * When a subscribed event occurs, the hardware sensor reports it by asserting
 * the interrupt line. The frame data must then be read out using
 * :c:func:`x4sensor_get_sensor_data` before the sensor is ready to acquire the
 * next frame. The time between interrupt and reading out the frame data must
 * be kept as short as possible.
 *
 * The implementation of interrupt handling is left to the integrator. It is
 * for instance possible to use the :c:func:`chipinterface_wait_for_interrupt`
 * function in chipinterface.
 *
 * :param events: a combination of event flags
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_start_event_mode(x4sensor_event_flags_t events);

/**
 * :brief: Starts the hardware sensor in recording mode
 *
 * This function starts the sensor in recording mode and returns. The hardware
 * sensor reports each frame by asserting the interrupt line. The frame data
 * must then be read out using :c:func:`x4sensor_get_sensor_data` before the
 * sensor acquires the next frame. The time between interrupt and reading out
 * the frame data must be kept as short as possible.
 *
 * The implementation of interrupt handling is left to the integrator. It is
 * for instance possible to use the :c:func:`chipinterface_wait_for_interrupt`
 * function in chipinterface.
 *
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_start_recording_mode();

/**
 * :brief: Stops the current sensor operation
 *
 * This function stops the sensor and brings it into a power-down state. After
 * calling this function, the sensor may be reconfigured and can be started
 * again.
 *
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_stop();

/**
 * :brief: Returns the maximum buffer size needed for the frame in recording mode
 *
 * This function returns the estimated maximum buffer size needed for the call
 * of :c:func:`x4sensor_get_sensor_data` so that the application can
 * preallocate sufficient space. The utilized size may be less than that.
 *
 * :return: the maximum sensor data buffer size.
 */
X4_SYMBOL_EXPORT size_t x4sensor_get_max_sensor_data_size_recording_mode();

/**
 * :brief: Returns the maximum buffer size needed for the frame in event mode
 *
 * This function returns the estimated maximum buffer size needed for the call
 * of :c:func:`x4sensor_get_sensor_data` so that the application can
 * preallocate sufficient space. The utilized size may be less than that.
 *
 * :return: the maximum sensor data buffer size.
 */
X4_SYMBOL_EXPORT size_t x4sensor_get_max_sensor_data_size_event_mode();

/**
 * :brief: Reads out frame data and prepares the next sensor interval
 *
 * This function reads out frame data and prepares the next sensor interval.
 * The frame data contains presence information and can be queried with
 * :c:func:`x4sensor_get_detection`.
 *
 * Call this function after obtaining an interrupt in event mode or in
 * recording mode. Do not call it in normal operation mode.
 *
 * Make sure to preallocate enough space in :c:var:`buffer`. In event mode, the
 * maximum buffer size can be predicted with
 * :c:func:`x4sensor_get_max_sensor_data_size_event_mode` or in recording mode
 * with :c:func:`x4sensor_get_max_sensor_data_size_recording`, respectively.
 *
 * :param buffer: a pointer to the destination memory
 * :param max_size: the maximum size of :c:var:`buffer`
 * :return: the number of bytes written to :c:var:`buffer`
 */
X4_SYMBOL_EXPORT size_t x4sensor_get_sensor_data(uint8_t *buffer, size_t max_size);

/**
 * :brief: Returns the error code of the last function call
 *
 * Most X4Sensor functions return an error code of type
 * :c:type:`x4sensor_error_t`. Some functions return other values for
 * convenience but may still produce an error. This function allows the user to
 * retrieve the error that happened in the last function call regardless of
 * which function was called.
 *
 * :return: the error code of the last function call
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_get_last_error();

/**
 * :brief: Converts an \a error code into a human-readable string
 *
 * This function converts a x4sensor_error_t error code into a human-readable
 * C string. It is a convenience function that should be used with care.
 * Calling it will have a significant impact on the code size and is only
 * recommended if code size is not top priority.
 *
 * :param: error the error code
 * :return: a C string with more information about the error
 */
X4_SYMBOL_EXPORT const char* x4sensor_convert_error_to_string(x4sensor_error_t error);

/**
 * :brief: Starts the hardware sensor in regulatory test mode
 *
 * This function starts the sensor in test mode and returns. In case of
 * :c:member:`x4_test_mode_t.X4_TEST_MODE_M1` and
 * :c:member:`x4_test_mode_t.X4_TEST_MODE_M2` mode the hardware sensor emits
 * continuously and no further action is needed. In other test modes sensor
 * behaves like in recording mode (see
 * :c:func:`x4sensor_start_recording_mode`). :c:func:`x4sensor_stop` has to be
 * called before starting another test mode.
 *
 * :param test_mode: a test mode index
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_start_test_mode(x4_test_mode_t test_mode);

/**
 * :brief: Extracts event flags from the frame data buffer
 *
 * This function returns the event flags that caused the sensor to report data.
 * Events are subscribed to by :c:func:`x4sensor_start_event_mode`.
 *
 * :param buffer: data fetched by :c:func:`x4sensor_get_sensor_data`
 * :return: the flags that caused the sensor to report data
 */
X4_SYMBOL_EXPORT x4sensor_event_flags_t x4sensor_get_events(const uint8_t *buffer);

/**
 * :brief: Extracts frame counter from the frame data buffer
 *
 * This function returns the frame counter from the data buffer.
 * The frame counter is incremented for each processed frame.
 *
 * :param buffer: data fetched by :c:func:`x4sensor_get_sensor_data`
 * :return: value of the frame counter
 */
X4_SYMBOL_EXPORT uint32_t x4sensor_get_frame_counter(const uint8_t *buffer);

/**
 * :brief: Extracts the detection state from a sensor data buffer
 *
 * :param buffer: data fetched by :c:func:`x4sensor_get_sensor_data`
 * :return: true in case of proximity or occupancy depending on the algorithm,
 *          otherwise false
 */
X4_SYMBOL_EXPORT bool x4sensor_get_detection_state(const uint8_t *buffer);

/**
 * :brief: Extracts distance cluster data from sensor data buffer
 *
 * :param buffer: data fetched by :c:func:`x4sensor_get_sensor_data`
 * :param distance_cluster: pointer to where the cluster data is written to
 * :return: :c:var:`X4SENSOR_SUCCESS` on success, otherwise an error code
 */
X4_SYMBOL_EXPORT x4sensor_error_t x4sensor_get_distance_cluster(const uint8_t *buffer, x4sensor_distance_cluster_t* distance_cluster);

/**
 * :brief: Returns the number of range bins in a distance cluster
 *
 * The number of range bins is fixed and cannot be changed.
 *
 * :return: number of range bins in a distance cluster
 */
X4_SYMBOL_EXPORT uint8_t x4sensor_get_distance_cluster_length();

/**
 * :brief: Returns the number of the first bin in a distance cluster
 *
 * :param buffer: data fetched by :c:func:`x4sensor_get_sensor_data`
 * :return: bin index to the first bin in a distance cluster
 */
X4_SYMBOL_EXPORT uint8_t x4sensor_get_distance_cluster_first_bin_number(const uint8_t *buffer);

/**
 * :brief: Returns the distance between adjacent range bins in mm
 *
 * The X4Sensor radar divides the detection zone into range bins with a certain
 * length. This length is fixed and cannot be changed.
 *
 * :return: The distance between adjacent range bins in mm
 */
X4_SYMBOL_EXPORT uint16_t x4sensor_get_distance_between_bins_mm();


#ifdef __cplusplus
}
#endif

#endif // NOVELDA_X4CONTROLLER_H

