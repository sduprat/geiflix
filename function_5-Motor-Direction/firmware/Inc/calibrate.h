/*
 * calibrate.h
 *
 *  Created on: 19 nov. 2021
 *      Author: Carole Meyer
 */

#ifndef CALIBRATE_H_
#define CALIBRATE_H_

#include "steering.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

/**
*	Performe la calibration du module de direction (action moteur, recuperation valeur capteur)
**/
void calibrate(void);

#endif /* CALIBRATE_H_ */
