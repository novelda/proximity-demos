/*
 * novelda_sensor.h
 *
 *  Created on: Jun 29, 2022
 *      Author: null
 */

#ifndef NOVELDA_SENSOR_H_
#define NOVELDA_SENSOR_H_
#include <novelda_x4sensor.h>



//EVENT bitmasks
#define EVENT_SENSOR_STOP            0x00000001
#define EVENT_SENSOR_START           0x00000002
#define EVENT_SENSOR_CHNG_SNSTIVITY  0x00000004
#define EVENT_SENSOR_CHNG_RANGE      0x00000008
#define EVENT_SENSOR_RESTART         0x00000010
#define EVENT_SENSOR_PRESENCE        0x00000020
#define EVENT_SENSOR_PROXIMITY       0x00000040
#define EVENT_SENSOR_INIT            0x00000080

#define MAIN_ASSERT(action, expected)                          \
do{ \
   if (action != expected) {                                  \
     /*while(1)*/NRF_LOG_INFO("SENSOR ASSERT!!");} }                                        \
while (0)

typedef void (*presence_callback)( );

extern void sensor_init(void);
extern void sensor_stop_remote(void);
extern void sensor_run_remote(uint8_t sensitivity, uint16_t range, presence_callback callback);

#endif /* NOVELDA_SENSOR_H_ */
