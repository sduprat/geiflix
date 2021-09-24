/*
 * scheduler.c
 *
 *  Created on: Sep 24, 2021
 *      Author: dimercur
 */

#include "stm32l4xx_hal.h"

#include "globalvar.h"
#include "iks01a2.h"

#include <stdio.h>
#include <string.h>

uint8_t flag_1ms;

/*********************************
 * Task task_send_values
 * Periodic: 1hz (1000 ms)
 */
void task_send_values (void);
uint32_t counter_task_send_values;
#define PERIOD_TASK_SEND_VALUES 1000

/*********************************
 * Task task_get_acceleration
 * Periodic: 12.5hz (80 ms)
 */
void task_get_acceleration (void);
uint32_t counter_task_get_acceleration;
#define PERIOD_TASK_GET_ACCELERATION 80

/*********************************
 * Task task_get_rotation
 * Periodic: 12.5hz (80 ms)
 */
void task_get_rotation (void);
uint32_t counter_task_get_rotation;
#define PERIOD_TASK_GET_ROTATION 80

/*********************************
 * Task task_get_magnetic
 * Periodic: 10hz (100 ms)
 */
void task_get_magnetic (void);
uint32_t counter_task_get_magnetic;
#define PERIOD_TASK_GET_MAGNETIC 100

/*********************************
 * Task task_get_temperature
 * Periodic: 10hz (100 ms)
 */
void task_get_temperature (void);
uint32_t counter_task_get_temperature;
#define PERIOD_TASK_GET_TEMPERATURE 100

/*********************************
 * Task task_get_pressure
 * Periodic: 10hz (100 ms)
 */
void task_get_pressure (void);
uint32_t counter_task_get_pressure;
#define PERIOD_TASK_GET_PRESSURE 100

/*********************************
 * Task task_get_humidity
 * Periodic: 10hz (100 ms)
 */
void task_get_humidity (void);
uint32_t counter_task_get_humidity;
#define PERIOD_TASK_GET_HUMIDITY 100

/*
 * @brief  Initialize scheduler counters and flags
 *
 */
uint32_t SCHEDULER_Init(void) {
	flag_1ms =0;

	counter_task_send_values=0;
	counter_task_get_acceleration=0;
	counter_task_get_rotation=0;
	counter_task_get_magnetic=0;
	counter_task_get_pressure=0;
	counter_task_get_humidity=0;
	counter_task_get_temperature=0;

	return 1;
}

/*
 * @brief  execute scheduler
 *
 */
void SCHEDULER_Run(void) {

	while (1)
	{
		if (flag_1ms!=0)
		{
			flag_1ms=0;
			counter_task_send_values++;
			counter_task_get_acceleration++;
			counter_task_get_rotation++;
			counter_task_get_magnetic++;
			counter_task_get_temperature++;
			counter_task_get_humidity++;
			counter_task_get_pressure++;
		}

		if (counter_task_get_acceleration>= PERIOD_TASK_GET_ACCELERATION) {
			counter_task_get_acceleration=0;
			task_get_acceleration();
		}

		if (counter_task_get_rotation>= PERIOD_TASK_GET_ROTATION) {
			counter_task_get_rotation=0;
			task_get_rotation();
		}

		if (counter_task_get_magnetic>= PERIOD_TASK_GET_MAGNETIC) {
			counter_task_get_magnetic=0;
			task_get_magnetic();
		}

		if (counter_task_get_temperature>= PERIOD_TASK_GET_TEMPERATURE) {
			counter_task_get_temperature=0;
			task_get_temperature();
		}

		if (counter_task_get_humidity>= PERIOD_TASK_GET_HUMIDITY) {
			counter_task_get_humidity=0;
			task_get_humidity();
		}

		if (counter_task_get_pressure>= PERIOD_TASK_GET_PRESSURE) {
			counter_task_get_pressure=0;
			task_get_pressure();
		}

		if (counter_task_send_values>= PERIOD_TASK_SEND_VALUES) {
			counter_task_send_values=0;
			task_send_values();
		}
	}
}

void task_send_values (void) {
	printf("Acc: x=%4.2f\ty=%4.2f\tz=%4.2f\r\n",
			current_acceleration_mg.x, current_acceleration_mg.y, current_acceleration_mg.z);
	printf("Gyr: x=%4.2f\ty=%4.2f\tz=%4.2f\r\n",
			current_angular_rate_mdps.x, current_angular_rate_mdps.y, current_angular_rate_mdps.z);
	printf("Mag: x=%4.2f\ty=%4.2f\tz=%4.2f\r\n",
			current_magnetic_mG.x, current_magnetic_mG.y, current_magnetic_mG.z);
	printf("T°:%3.2f\r\nPres=%6.2f\r\nHum=%3.2f%%\r\n\r\n",
			current_temperature_degC, current_pressure_hPa, current_humidity_perc);
}

void task_get_acceleration (void) {
	IKS01A2_GetAcceleration(&current_acceleration_mg);
}

void task_get_rotation (void) {
	IKS01A2_GetRotation(&current_angular_rate_mdps);
}

void task_get_magnetic (void) {
	IKS01A2_GetMagnetic(&current_magnetic_mG);
}

void task_get_temperature (void) {
	IKS01A2_GetTemperature(&current_temperature_degC);
}

void task_get_humidity (void) {
	IKS01A2_GetHumidity(&current_humidity_perc);
}

void task_get_pressure (void) {
	IKS01A2_GetPressure(&current_pressure_hPa);
}
