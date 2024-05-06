

#ifndef PROXIMITYPROFILE_H
#define PROXIMITYPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
// Profile Parameters
#define PROXIMITYPROFILE_DETECTION                   0  // R uint8 - Profile Characteristic proximity value
#define PROXIMITYPROFILE_RANGE                       1  // RW uint16 - Profile Characteristic range value
#define PROXIMITYPROFILE_SENSITIVITY                 2  // RW uint8 - Profile Characteristic sensitivity value
#define PROXIMITYPROFILE_TIMEOUT                     3  // RW uint16 - Profile Characteristic timeout value

// Simple Profile Service UUID
#define PROXIMITYPROFILE_SERV_UUID               0x20F1

// Key Pressed UUID
#define PROXIMITYPROFILE_DETECTION_UUID            0x2BAD
#define PROXIMITYPROFILE_RANGE_UUID                0x2BB1
#define PROXIMITYPROFILE_SENSITIVITY_UUID          0x2BB2
#define PROXIMITYPROFILE_TIMEOUT_UUID              0x2BB3


/*********************************************************************
 * Profile Callbacks
 */
// Callback when a characteristic value has changed
typedef void (*pfnProximityProfile_Change_t)( uint8 paramID );

typedef struct
{
  pfnProximityProfile_Change_t        pfnProximityProfile_Change;  // Called when characteristic value changes
} ProximityProfile_CBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      ProximityProfile_addService
 *
 * @brief   This function initializes the Simple GATT Server service
 *          by registering GATT attributes with the GATT server.
 *
 * @return  SUCCESS or stack call status
 */
bStatus_t ProximityProfile_addService( void );

/**
 * @fn      ProximityProfile_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   appCallbacks - pointer to application callback.
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t ProximityProfile_registerAppCBs( ProximityProfile_CBs_t *appCallbacks );

/**
 * @fn      ProximityProfile_setParameter
 *
 * @brief   Set a Simple GATT Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate
 *                  data type (example: data type of uint16 will be cast to
 *                  uint16 pointer).
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t ProximityProfile_setParameter( uint8 param, uint8 len, void *value );

/**
 * @fn      ProximityProfile_getParameter
 *
 * @brief   Get a Simple GATT Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to write. This is dependent on
 *                  the parameter ID and WILL be cast to the appropriate
 *                  data type (example: data type of uint16 will be cast to
 *                  uint16 pointer).
 *
 * @return  SUCCESS or INVALIDPARAMETER
 */
bStatus_t ProximityProfile_getParameter( uint8 param, void *value );


/**
 * @brief Handler for the Connect event.
 *
 * This function handles the Connect event by starting the sensor.
 */
extern void ProximityProfile_on_connect(void);

/**
 * @brief Handler for the Disconnect event.
 *
 * This function handles the Disconnect event by stopping the sensor if no active connections are present.
 */
extern void ProximityProfile_on_disconnect(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PROXIMITYPROFILE_H */
