/**
 *******************************************************************************
 * @file    teseo_liv3f_class.h
 * @author  AST
 * @version V1.0.0
 * @date    Jan-2019
 *
 *******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        www.st.com/software_license_agreement_liberty_v2
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************
 * Converted to plain C by S.DI MERCURIO 2021
 */

#ifndef TESEO_LIV3F_H_
#define TESEO_LIV3F_H_

#include "stm32l4xx_hal.h"

#include "teseo.h"
#include "gnss_parser.h"
#include "NMEA_parser.h"

//#define TESEO_USE_I2C
#define TESEO_USE_UART

#if defined (TESEO_USE_I2C)
#define DEFAULT_I2C_HANDLER hi2c1
#define DEFAULT_DEVICE_ADDRESS (0x3A<<1)
#define DEFAULT_DEVICE_PORT 0xFF
#else /* TESEO_USE_UART */
#define DEFAULT_UART_HANDLER huart1
#endif /* TESEO_USE_I2C */

#define BUFFER_SIZE 32
#define MAX_FIELD_LENGTH 30
#define MAX_STRING_LENGTH 100
#define MAX_RESPONSE_LENGTH 40

#if !defined bool
typedef uint8_t bool;
#define false 0
#define true !false
#endif /* bool */

typedef struct
{
	char inputString[MAX_STRING_LENGTH];
	char inputString2[MAX_STRING_LENGTH];
	bool stringComplete;
	char inChar[BUFFER_SIZE];
	int index;
	int end;
} I2CHandler;

typedef struct
{
	char inputString[MAX_STRING_LENGTH];
	bool stringComplete;
	int index;
	int end;
} UARTHandler;

typedef enum
{
	TENS = 0,
	HUNDREDS,
	THOUSANDS
} Decimal_t;

/**
 * @brief       Reset chip and wait 3s
 */
void TESEO_ResetChip(void);

/**
 * @brief       Wakeup chip from hibernate mode
 */
void TESEO_WakeupChip(void);

/**
 * @brief       Switch chip into hibernate mode
 */
void TESEO_HibernateChip(void);

/**
 * @brief       Initialize the sensor and the data structures
 * @note		in case of I2C communication, I2C peripheral should be initialized before calling this function
 * @return      GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_Init();

/**
 * @brief       Update the internal data structures of the sensor using the appropriate communication method
 * @note		To prevent data loss, this function should be called at least 20 times per second
 * @return      GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_Update();

/**
 * @brief       	Send a command to the device
 * @param command	The command to send
 * @return      	GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_SendCommand(char *command);

/**
 * @brief    	    Ask the device for a specific message
 * @param message	The message to receive
 * @return   	    GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_AskMessage(char* message);

/**
 * @brief       Ask the device if the message requested by @a askMessage() was received
 * @return      1 if the message was received, 0 otherwise
 */
int TESEO_GetMessageDone ();

/**
 * @brief       Get the complete data structure
 * @return      The full data structure
 */
GNSSParser_Data_t TESEO_GetData();

/**
 * @brief       Get the wakeup status of the device
 * @return      1 if the device is enabled, 0 otherwise
 */
int TESEO_GetWakeupStatus();

/**
 * @brief       Get the GPGGA coordinates in DMM
 * @return      The coordinates structure
 * @remark      Coordinates are given in Decimale Degree Minute (DMM): See google :https://support.google.com/maps/answer/18539?hl=fr&co=GENIE.Platform%3DAndroid)
 * @remark      lon and lat read as this: DDD MM.MMMM, with 1,2 or 3 digits for degree, 2 digit only for minute and after the dot, fraction of minute
 * @remark      the format can be used under google maps , with the space between degree and minute.
 * @remark      Separate lat and lon with period like this: 1 27.96845, 43 34.25801
 * @remark      See also http://tvaira.free.fr/bts-sn/activites/activite-peripherique-usb/conversions.html
 */
Coords_t TESEO_GetCoords_DMM();

/**
 * @brief       Get the GPGGA coordinates in DD
 * @return      The coordinates structure
 * @remark      Convert coordinates in Decimale Degree Minute (DMM) to Decimal Degree (DD)
 */
Coords_t TESEO_GetCoords_DD();

/**
 * @brief       Get the debug status of the device
 * @return      DEBUG_ON if debug is enabled, DEBUG_OFF otherwise
 */
Debug_State TESEO_GetDebugStatus ();

/**
 * @brief       Get the GPGGA message data structure
 * @return      The required data structure
 */
GPGGA_Info_t TESEO_GetGPGGAData();

/**
 * @brief       Get the --GNS message data structure
 * @return      The required data structure
 */
GNS_Info_t TESEO_GetGNSData ();

/**
 * @brief       Get the GPGTS message data structure
 * @return      The required data structure
 */
GPGST_Info_t TESEO_GetGPGSTData();

/**
 * @brief       Get the GPRMC message data structure
 * @return      The required data structure
 */
GPRMC_Info_t TESEO_GetGPRMCData ();

/**
 * @brief       Get the --GSA message data structure
 * @return      The required data structure
 */
GSA_Info_t TESEO_GetGSAData ();

/**
 * @brief       Get the --GSV message data structure
 * @return      The required data structure
 */
GSV_Info_t TESEO_GetGSVData ();

/**
 * @brief       Get the PSTMVER message data structure
 * @return      The required data structure
 */
PSTMVER_Info_t TESEO_GetVERData ();

/**
 * @brief       Get the PSTMPASSRTN message data structure
 * @return      The required data structure
 */
PSTMPASSRTN_Info_t TESEO_GetPASSData ();

/**
 * @brief       Get the PSTMAGPS message data structure
 * @return      The required data structure
 */
PSTMAGPS_Info_t TESEO_GetAGPSData ();

/**
 * @brief       Get the Geofence message data structure
 * @return      The required data structure
 */
Geofence_Info_t TESEO_GetGeofenceData ();

/**
 * @brief       Get the Odometer message data structure
 * @return      The required data structure
 */
Odometer_Info_t TESEO_GetOdometerData ();

/**
 * @brief       Get the Datalog structure
 * @return      The required data structure
 */
Datalog_Info_t TESEO_GetDatalogData ();
/**
 * @brief       Get the result of the last command sent
 * @return      GNSS_OP_OK if it was a success, GNSS_OP_ERROR otherwise
 */
OpResult_t TESEO_GetResult ();

/**
 * @brief       Activate/deactivate the debug flag
 * @return      The current state of the debug flag
 */
Debug_State TESEO_ToggleDebug();

#endif /*TESEO_LIV3F_H_*/
