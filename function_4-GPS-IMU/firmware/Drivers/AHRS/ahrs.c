/*
 * iks01a2.h
 *
 *  Created on: Sep 23, 2021
 *      Author: dimercur
 */

#include "ahrs.h"
#include <math.h>

#define USE_MAG

#define USE_MADGWICK_AHRS
//#define USE_MAHONY_AHRS

#ifdef USE_MAHONY_AHRS
#include "MahonyAHRS.h"
#else
#include "MadgwickAHRS.h"
#endif

//#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
//#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
//#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
//
//#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
//#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
//#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
//#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

//void AHRS_GetValues(float * val);
//void readAllSensors(uint8_t *GyroTempBuf, uint8_t *AccTempBuf, uint8_t *MagTempBuf);
//void readRawGyro(uint8_t *GyroTempBuf);
//void processGyroData(float *GyroBuf, uint8_t *GyroTempBuf);
//void processAccelData(float *AccBuf, uint8_t *AccTempBuf);
//void processMagnetoData(float *MagBuf, uint8_t *MagTempBuf);
//uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;

//float GyroSensitivity = 0;
//uint8_t Gyro_LBEFlag = 0;
//uint8_t GyroDRDFlag = 0;
//uint8_t Accel_LBEorFIFOFlag = 0;
//float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
//uint8_t Accel_cDivider;

//void myFusion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

//float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
//float fTiltedX,fTiltedY = 0.0f;

//float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f};
//uint8_t MagTempBuffer[6] = {0.0f}, AccTempBuffer[6] = {0.0f}, GyroTempBuffer[6] = {0.0f};
//float QuaternionsBuffer[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float q0, q1, q2, q3;
float eulerAngles[3];
AHRS_3AxisValues eulerAnglesStruct;
//float GyroCorrectionCoeffs[3] = {0.0f};

#define QW q0
#define QX q1
#define QY q2
#define QZ q3

#define PI                         (float)     3.14159265f
//void myFusion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
//	float recipNorm;
//	q0 = 0;
//	q1 = gx;
//	q2 = gy;
//	q3 = gz;
//
//	// Normalise quaternion
//	recipNorm = 1/sqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
//	q0 *= recipNorm;
//	q1 *= recipNorm;
//	q2 *= recipNorm;
//	q3 *= recipNorm;
//}

/**
 * @brief   Initialization of AHRS internal variables
 */
void AHRS_Init(void) {
	// Initialization of quaternion of sensor frame relative to auxiliary frame
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;

	// Initialization of Euler angle array and structure
	eulerAngles[0] = eulerAnglesStruct.x = 0.0f; // X axis
	eulerAngles[1] = eulerAnglesStruct.y = 0.0f; // Y axis
	eulerAngles[2] = eulerAnglesStruct.z = 0.0f; // Z axis
}

/**
 * @brief   Process new sensor values and update AHRS state
 * @param   accelerometer	3-axis values of accelerometer
 * @param   gyroscope	    3-axis values of gyroscope
 * @param   magnetometer	3-axis values of magnetometer
 */
void AHRS_Update(AHRS_3AxisValues *accelerometer, AHRS_3AxisValues *gyroscope, AHRS_3AxisValues *magnetometer) {
	//float val[9] = {0.0f};

#ifdef USE_MADGWICK_AHRS
#ifdef USE_MAG
	MadgwickAHRSupdate (
			accelerometer->x*PI/180.0,
			accelerometer->y*PI/180.0,
			accelerometer->z*PI/180.0,
			gyroscope->x,
			gyroscope->y,
			gyroscope->z,
			magnetometer->x,
			magnetometer->y,
			magnetometer->z);
#else
	MadgwickAHRSupdate(
			accelerometer->x*PI/180.0,
			accelerometer->y*PI/180.0,
			accelerometer->z*PI/180.0,
			gyroscope->x,
			gyroscope->y,
			gyroscope->z,
			0.0f,
			0.0f,
			0.0f);
	//MadgwickAHRSupdate(0.0f, 0.0f, 0.0f, 100, 100, 1100, 0.0f, 0.0f, 0.0f);
#endif //#ifdef USE_MAG

#elif defined USE_MAHONY_AHRS

#ifdef USE_MAG
	MahonyAHRSupdate(
			accelerometer->x*PI/180.0,
			accelerometer->y*PI/180.0,
			accelerometer->z*PI/180.0,
			gyroscope->x,
			gyroscope->y,
			gyroscope->z,
			magnetometer->x,
			magnetometer->y,
			magnetometer->z);
#else
	MahonyAHRSupdate(
			accelerometer->x*PI/180.0,
			accelerometer->y*PI/180.0,
			accelerometer->z*PI/180.0,
			gyroscope->x,
			gyroscope->y,
			gyroscope->z,
			0.0f,
			0.0f,
			0.0f);
#endif //#ifdef USE_MAG

#else
	myFusion(
			accelerometer->x*PI/180.0,
			accelerometer->y*PI/180.0,
			accelerometer->z*PI/180.0,
			gyroscope->x,
			gyroscope->y,
			gyroscope->z,
			magnetometer->x,
			magnetometer->y,
			magnetometer->z);
#endif

    //	QuaternionsBuffer[0] = q0;
	//	QuaternionsBuffer[1] = q1;
	//	QuaternionsBuffer[2] = q2;
	//	QuaternionsBuffer[3] = q3;
}

/**
 * @brief   Update eulerAngles angles array for AHRS internal state
 */
static void AHRS_UpdateEulerAngles(void) {
	//float *q = QuaternionsBuffer;

	/*
	 * eulerAngles[0] = atan2(2*q[1]*q[2]-2*q[0]*q[3], 2*q[0]*q[0]+2*q[1]*q[1]-1)*180/PI; // heading, yaw, phi
	 * euler[1] = -asin(2*q[1]*q[3]+2*q[0]*q[2])*180/PI; // attitude, elevation, pitch, theta
	 * euler[2] = atan2(2*q[2]*q[3]-2*q[0]*q[1], 2*q[0]*q[0]+2*q[3]*q[3]-1)*180/PI; // bank, roll, psi
	 */

	float test = QX*QY+QZ*QW;

	if (test > 0.499) {
		eulerAngles[0] = 2*atan2(QX, QW)*180/PI;
		eulerAngles[1] = PI*180/(2*PI);
		eulerAngles[2] = 0;
	} else if (test< -0.499) {
		eulerAngles[0] = -2*atan2(QX, QW)*180/PI;
		eulerAngles[1] = -PI*180/(2*PI);
		eulerAngles[2] = 0;
	} else {
		eulerAngles[0] = atan2(2*QY*QW - 2*QX*QZ, 1 - 2*QY*QY - 2*QZ*QZ)*180/PI;
		eulerAngles[1] = asin(2*QX*QY + 2*QZ*QW)*180/PI;
		eulerAngles[2] = atan2(2*QX*QW - 2*QY*QZ, 1 - 2*QX*QX - 2*QZ*QZ)*180/PI;
	}
}

/**
 * @brief   Get eulerAngles angles .
 * @retval  structure with x, y and z axis field, given as float
 */
AHRS_3AxisValues *AHRS_GetEulerAngles(void) {
	// Update Euler angles from internal state
	AHRS_UpdateEulerAngles();

	eulerAnglesStruct.x = eulerAngles[0];
	eulerAnglesStruct.y = eulerAngles[1];
	eulerAnglesStruct.z = eulerAngles[2];

	return &eulerAnglesStruct;
}

/**
 * @brief   Get eulerAngles angles as an array of float.
 * @retval  euler array of float (first: x axis, then y axis and z axis)
 */
float *AHRS_GetEulerAnglesAsArray(void) {
	// Update Euler angles from internal state
	AHRS_UpdateEulerAngles();

	return eulerAngles;
}

//void AHRS_GetValues(float * val) {
//#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG
//	uint8_t USART_TempBuf[100];
//	uint8_t byteCounter = 0;
//#endif //#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG
//
//
//	processGyroData(GyroBuffer, GyroTempBuffer);
//	processAccelData(AccBuffer, AccTempBuffer);
//	processMagnetoData(MagBuffer, MagTempBuffer);
//
//	val[0] = - (GyroBuffer[1] - GyroCorrectionCoeffs[1]);
//	val[1] = GyroBuffer[0] - GyroCorrectionCoeffs[0];
//	val[2] = GyroBuffer[2] - GyroCorrectionCoeffs[2];
//#ifdef OUT_GYRO
//	byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[0], val[1], val[2]);
//	//USART_printfWithDMA("%f,%f,%f,", val[0], val[1], val[2]);
//#endif //#ifdef OUT_GYRO
//
//	val[3] = AccBuffer[0];
//	val[4] = AccBuffer[1];
//	val[5] = AccBuffer[2];
//#ifdef OUT_ACCEL
//	byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[3], val[4], val[5]);
//	//USART_printfWithDMA("%f,%f,%f,", val[3], val[4], val[5]);
//#endif //#ifdef OUT_ACCEL
//
//	val[6] = MagBuffer[0];
//	val[7] = MagBuffer[1];
//	val[8] = MagBuffer[2];
//#ifdef OUT_MAG
//	byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[6], val[7], val[8]);
//	//USART_printfWithDMA("%f,%f,%f,", val[6], val[7], val[8]);
//#endif //#ifdef OUT_MAG
//
//#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG
//	sprintf((char*)(USART_TempBuf + byteCounter), "\r\n");
//	USART_printfWithDMA("%s", USART_TempBuf);
//	//USART_printfWithDMA("\r\n");
//#endif
//}

//void UpdateGyroBias() {
//
//	int i = 0;
//
//	HAL_Delay(1000);
//
//	for (i = 0; i < 100; i++) {
//		readRawGyro(GyroTempBuffer);
//		processGyroData(GyroBuffer, GyroTempBuffer);
//		GyroCorrectionCoeffs[0] += GyroBuffer[0];
//		GyroCorrectionCoeffs[1] += GyroBuffer[1];
//		GyroCorrectionCoeffs[2] += GyroBuffer[2];
//		HAL_Delay(10);
//	}
//	GyroCorrectionCoeffs[0] /= 100.0f;
//	GyroCorrectionCoeffs[1] /= 100.0f;
//	GyroCorrectionCoeffs[2] /= 100.0f;
//}

/**
 * @brief  Configure the Mems to gyroscope application.
 * @param  None
 * @retval None
 */
//void Demo_GyroConfig(void)
//{
//	L3GD20_InitTypeDef L3GD20_InitStructure;
//	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
//
//	/* Configure Mems L3GD20 */
//	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
//	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_3;
//	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
//	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
//	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Single;
//	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
//	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
//	L3GD20_Init(&L3GD20_InitStructure);
//
//	L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_REF_SIGNAL;
//	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_5;
//	L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
//
//	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_DISABLE);
//
//	L3GD20_INT2InterruptCmd(L3GD20_INT2INTERRUPT_ENABLE);
//
//	uint8_t tmpreg = 0;
//	L3GD20_Read(&tmpreg, L3GD20_CTRL_REG4_ADDR, 1);
//	/* Switch the sensitivity value set in the CRTL4 */
//	switch(tmpreg & 0x30)
//	{
//	case 0x00:
//		GyroSensitivity=L3G_Sensitivity_250dps;
//		break;
//
//	case 0x10:
//		GyroSensitivity=L3G_Sensitivity_500dps;
//		break;
//
//	case 0x20:
//		GyroSensitivity=L3G_Sensitivity_2000dps;
//		break;
//	}
//	Gyro_LBEFlag = tmpreg & 0x40;
//}


/**
 * @brief  Basic management of the timeout situation.
 * @param  None.
 * @retval None.
 */
//uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
//{
//	return 0;
//}

/**
 * @brief  Basic management of the timeout situation.
 * @param  None.
 * @retval None.
 */
//uint32_t L3GD20_TIMEOUT_UserCallback(void)
//{
//	return 0;
//}
//
//void Sensors_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	readAllSensors(GyroTempBuffer, AccTempBuffer, MagTempBuffer);
//	AHRS_Update(QuaternionsBuffer);
//
//	GyroDRDFlag=1;
//}
