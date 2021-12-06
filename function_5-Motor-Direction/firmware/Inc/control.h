/*
 * control.h
 *
 *  Created on: 12 nov. 2021
 *      Author: Carole Meyer
 */

#ifndef CONTROL_H_
#define CONTROL_H_

extern int modeSpeed;
extern int modeSteer;

#include <stdbool.h>

/**
*	Controle les MARG, MARD et MAV à partir de modeSpeed et modeSteer recus via le CAN
**/
void car_control(int requested_speed, int requested_steer);


/* brief	Calculate the movement of the car according to the distance between the car and the location we want to join
 * param	double distance		Distance between the two GPS location in meters
 * retval	None
 * */
double go_straight_without_GPS(double distance);

/* brief	Determine the angle we need to turn to join the right location
 * param	int requested_speed
 * 			int resquested_angle
 * retval	None
 * */
double calculate_alpha(int requested_speed, int requested_steer);

/* brief	Determine the angle we need to turn to join the right location
 * param	double teta		Angle between the North and the GPS location we want to join
 * 			double alpha	Angle between the North and the axis of the car
 * retval	double beta		Angle between the axis of the car and the GPS location
 * */
double calculate_beta(double teta);


/* brief	Manage the direction of the car according to the angle between the car axis and the GPS location
 * param	double beta			Angle between the axis of the car and the GPS location in degrees
 * retval	int angle_command	Command to control the steering of the wheels
 * */
int direction_management(double beta);


/* brief	Control the speed and the steering of the car in real time according to the distance and the angle between the car and the location we want to join
 * param	double distance		Distance between the two GPS location in meters
 * 			double beta			Angle between the axis of the car and the GPS location in degrees
 * retval	None
 * */
void direction_speed(double distance, double beta);


/* brief	Manage the movement from 2 GPS coordinates
 * param	double lat1, lon1, lat2, lon2	GPS decimal coordinates : latitude and longitude of each location
 * retval	None
 * */
void movement_with_GPS(double lat1, double lon1, double lat2, double lon2);

/* brief	Make a 360 degrees turn
 * param	None
 * retval	None
 * */
void turn360(void);

#endif /* CONTROL_H_ */
