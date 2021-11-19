
/* Includes ------------------------------------------------------------------*/

#include "control_direction.h"
#include "stdlib.h"
#include "math.h"

/* Private define ------------------------------------------------------------*/

#define DIST_MAX_STEP 0.5	// distance traveled (in meters) in 1 sec with speed of 75
#define DIST_MED_STEP 0.1	// distance traveled (in meters) in 1 sec with speed of 60


/* Programs ------------------------------------------------------------------*/

/* brief	Determine the distance in meters between two GPS location
 * param	float lat1, lon1, lat2, lon2	GPS decimal coordinates : latitude and longitude of each location
 * retval	float distance	Distance between the two GPS location in meters
 * */
float get_distance(float lat1, float lon1, float lat2, float lon2){

  // Earth radius in France
  float R = 6371.160;

  // degree to radian conversion
  float pi = 3.14159265358979;

  float lat1_r = ((2.0*pi) * lat1) / 360;
  float lon1_r = ((2.0*pi) * lon1) / 360;
  float lon2_r = ((2.0*pi) * lon2) / 360;
  float lat2_r = ((2.0*pi) * lat2) / 360;

  // distance calcul
  float dist_lat = (lat2_r - lat1_r) / 2.0;
  float dist_lon = (lon2_r - lon1_r) / 2.0;

  float a = ( sinf(dist_lat)*sinf(dist_lat) ) + cosf(lat1) * cosf(lat2) * ( sinf(dist_lon) * sinf(dist_lon) );

  float d = 2.0 * R * atan2f(sqrt(a), sqrt(1.0-a));

  return (d*1000);
}


/* brief	Control the speed of the car according to the distance between the car and the location we want to join
 * param	float distance	Distance between the two GPS location in meters
 * retval	None
 * */
void movement_with_GPS(float distance){
	if (distance > 2.0) {
		wheels_set_speed(GPIO_PIN_SET, GPIO_PIN_SET, 75, 75);
	}
	else if (distance > 0.25) {
		wheels_set_speed(GPIO_PIN_SET, GPIO_PIN_SET, 60, 60);
	}
	else {
		wheels_set_speed(GPIO_PIN_SET, GPIO_PIN_SET, 50, 50);
	}
}

/* brief	Calculate the movement of the car according to the distance between the car and the location we want to join
 * param	float distance	Distance between the two GPS location in meters
 * retval	None
 * */
void movement_without_GPS(float distance){
	while (distance > 2.0) {
		wheels_set_speed(GPIO_PIN_SET, GPIO_PIN_SET, 75, 75);
		distance = distance - DIST_MAX_STEP;
		HAL_Delay(1000);
	}
	while (distance > 0.25) {
		wheels_set_speed(GPIO_PIN_SET, GPIO_PIN_SET, 60, 60);
		distance = distance - DIST_MED_STEP;
		HAL_Delay(1000);
	}
	wheels_set_speed(GPIO_PIN_SET, GPIO_PIN_SET, 50, 50);
}

