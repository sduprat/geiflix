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

/**
*	Controle les MARG, MARD et MAV à partir de modeSpeed et modeSteer recus via le CAN
**/
void car_control(int requested_speed, int requested_steer);


#endif /* CONTROL_H_ */
