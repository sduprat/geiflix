/*
 * iks01a2.h
 *
 *  Created on: Sep 23, 2021
 *      Author: dimercur
 */

#ifndef BSP_COMMON_IKS01A2_H_
#define BSP_COMMON_IKS01A2_H_

#include "stm32l4xx_hal.h"

typedef struct {
	float x;
	float y;
	float z;
} values3d_t;

uint32_t IKS01A2_Init(void);

/********************************************************
 * Functions related to lsm6dsl
 */
uint32_t IKS01A2_GetAcceleration(values3d_t *acceleration_mg);
uint32_t IKS01A2_GetRotation(values3d_t *angular_rate_mdps);
//uint32_t IKS01A2_GetTemperature(float *temperature_degC);

/********************************************************
 * Functions related to lsm303agr
 */
uint32_t IKS01A2_GetMagnetic(values3d_t *magnetic_mG);

/********************************************************
 * Functions related to hts221
 */
uint32_t IKS01A2_GetTemperature(float *temperature_degC);
uint32_t IKS01A2_GetHumidity(float *humidity_perc);

/********************************************************
 * Functions related to lps22hb
 */
uint32_t IKS01A2_GetPressure(float *pressure_hPa);

#endif /* BSP_COMMON_IKS01A2_H_ */
