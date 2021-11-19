#ifndef _CONTROL_DIRECTION_H_
#define _CONTROL_DIRECTION_H_

#include "steering.h"

/* brief	Determine the distance in m between two GPS location
 * param	float lat1, lon1, lat2, lon2	GPS decimal coordinates : latitude and longitude of each location
 * retval	float distance	Distance between the two GPS location in meters
 * */
float get_distance(float lat1, float lon1, float lat2, float lon2);

/* brief	Control the speed of the car in real time according to the distance between the car and the location we want to join
 * param	float distance	Distance between the two GPS location in meters
 * retval	None
 * */
void movement_with_GPS(float distance);

/* brief	Calculate the movement of the car according to the distance between the car and the location we want to join
 * param	float distance	Distance between the two GPS location in meters
 * retval	None
 * */
void movement_without_GPS(float distance);



#endif
