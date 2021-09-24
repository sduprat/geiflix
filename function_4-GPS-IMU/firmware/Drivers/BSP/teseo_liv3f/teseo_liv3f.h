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
 */

#include "stm32l4xx_hal.h"

#include "teseo.h"
#include "gnss_parser.h"
#include "NMEA_parser.h"

#define DEFAULT_BUS 0
#define DEFAULT_I2C NULL
#define DEFAULT_UART NULL

#define DEFAULT_DEVICE_ADDRESS 0x3A
#define DEFAULT_DEVICE_PORT 0xFF

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

//extern int useI2C = DEFAULT_BUS;
////extern TwoWire *dev_i2c = DEFAULT_I2C;
////extern HardwareSerial *dev_uart = DEFAULT_UART;
//extern int commandDone;
//extern char compareMessage[MAX_RESPONSE_LENGTH];
//extern I2CHandler i2ch;
//extern UARTHandler uarth;
//extern GNSSParser_Data_t data;
//extern uint8_t app[MAX_MSG_LEN][MAX_FIELD_LENGTH];

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
 * @param message	The message to recieve
 * @return   	    GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_AskMessage(char* message);

/**
 * @brief       Ask the device if the message requested by @a askMessage() was recieved
 * @return      1 if the message was recieved, 0 otherwise
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
 * @brief       Get the GPGGA coordinates
 * @return      The coordinates structure
 */
Coords_t TESEO_GetCoords();

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

