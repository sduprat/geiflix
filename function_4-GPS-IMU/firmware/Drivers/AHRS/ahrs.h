/*
 * ahrs.h
 *
 *  Created on: Sep 23, 2021
 *      Author: dimercur
 */

#ifndef _AHRS_H
#define _AHRS_H

#include "stm32l4xx.h"

//define OUT_QUATERNION
#define OUT_EULER

typedef struct {
	float x;
	float y;
	float z;
} AHRS_3AxisValues;

/**
 * @brief   Initialization of AHRS internal variables
 */
void AHRS_Init(void);

/**
 * @brief   Process new sensor values and update AHRS state
 * @param   accelerometer	3-axis values of accelerometer
 * @param   gyroscope	    3-axis values of gyroscope
 * @param   magnetometer	3-axis values of magnetometer
 */
void AHRS_Update(AHRS_3AxisValues *accelerometer, AHRS_3AxisValues *gyroscope, AHRS_3AxisValues *magnetometer);

/**
 * @brief   Get eulerAngles angles .
 * @retval  structure with x, y and z axis field, given as float
 */
AHRS_3AxisValues *AHRS_GetEulerAngles(void);

/**
 * @brief   Get eulerAngles angles as an array of float.
 * @retval  euler array of float (first: x axis, then y axis and z axis)
 */
float *AHRS_GetEulerAnglesAsArray(void);

#endif // _AHRS_H
