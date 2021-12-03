/*
 * GPS.h
 *
 *  Created on: 1 déc. 2021
 *      Author: Amélie MAIER
 */

#ifndef GPS_H_
#define GPS_H_

#include "steering.h"


/* brief	Conversion coordinates from DMS (degrees / minutes / seconds) to DD (decimal degrees)
 * param	double angle_rad 	Angle in radians
 * retval	double angle_deg 	Angle in degrees
 * */
double dms2dd(double deg, double min, double sec);

/* brief	Conversion degrees to radians
 * param	double angle_rad 	Angle in radians
 * retval	double angle_deg 	Angle in degrees
 * */
double deg2rad(double angle_rad);


/* brief	Determine the distance in m between two GPS location
 * param	double lat1, lon1, lat2, lon2	GPS decimal coordinates : latitude and longitude of each location
 * retval	double distance					Distance between the two GPS location in meters
 * */
double get_distance(double lat1, double lon1, double lat2, double lon2);

/* brief	Determine the angle in degrees compared to the North between two GPS location
 * param	double lat1, lon1, lat2, lon2	GPS decimal coordinates : latitude and longitude of each location
 * retval	double Angle					Angle between the two GPS location in degrees compared to the North
 * */
double get_angle_GPS(double lat1, double lon1, double lat2, double lon2);


#endif /* GPS_H_ */
