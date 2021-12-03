/*
 * GPS.c
 *
 *  Created on: 1 déc. 2021
 *      Author: Amélie MAIER
 */

/* Includes ------------------------------------------------------------------*/

#include "GPS.h"
#include "stdlib.h"
#include "math.h"

/* Private define ------------------------------------------------------------*/
#define PI 3.14159265358979


/* Programs ------------------------------------------------------------------*/

/* brief	Conversion coordinates from DMS (degrees / minutes / seconds) to DD (decimal degrees)
 * param	double angle_rad 	Angle in radians
 * retval	double angle_deg 	Angle in degrees
 * */
double dms2dd(double deg, double min, double sec) {
	return  (deg + (min/60) + (sec/3600));
}

/* brief	Conversion degrees to radians
 * param	double angle_rad 	Angle in radians
 * retval	double angle_deg 	Angle in degrees
 * */
double deg2rad(double angle_rad) {
	return ((2.0*PI) * angle_rad) / 360;
}

/* brief	Determine the distance in meters between two GPS location
 * param	double lat1, lon1, lat2, lon2	GPS decimal coordinates : latitude and longitude of each location
 * retval	double distance	Distance between the two GPS location in meters
 * */

double get_distance(double lat1, double lon1, double lat2, double lon2){

  // Earth radius in France
  double R = 6371.160;

  // degree to radian conversion

  double lat1_r = deg2rad(lat1);
  double lon1_r = deg2rad(lon1);
  double lon2_r = deg2rad(lon2);
  double lat2_r = deg2rad(lat2);

  // distance calcul
  double dist_lat = (lat2_r - lat1_r) / 2.0;
  double dist_lon = (lon2_r - lon1_r) / 2.0;

  double a = ( sinf(dist_lat)*sinf(dist_lat) ) + cosf(lat1) * cosf(lat2) * ( sinf(dist_lon) * sinf(dist_lon) );

  double d = 2.0 * R * atan2f( sqrt(a), sqrt(1.0-a) );

  return (d*1000);
}

/* brief	Determine the angle in degrees compared to the North between two GPS location
 * param	double lat1, lon1, lat2, lon2	GPS decimal coordinates : latitude and longitude of each location
 * retval	double Angle					Angle between the two GPS location in degrees compared to the North
 * */
double get_angle_GPS(double lat1, double lon1, double lat2, double lon2) {

	double lat1_r = deg2rad(lat1);
	double lon1_r = deg2rad(lon1);
	double lon2_r = deg2rad(lon2);
	double lat2_r = deg2rad(lat2);

	// calculation of relatif angle in radian
    double dLon = (lon2_r - lon1_r);
	double y = sinf(dLon) * cosf(lat2_r);
	double x = ( cosf(lat1_r) * sinf(lat2_r) ) - ( sinf(lat1_r) * cosf(lat2_r) * cosf(dLon) ) ;
	double angle_rad = atan2f(y, x);

	//conversion in deg + conversion to have absolute angle compared to the North
	double angle_deg = ( 360 * angle_rad ) / (2*PI);
	angle_deg = fmod((angle_deg + 360.0), 360.0);

    return angle_deg;
}

