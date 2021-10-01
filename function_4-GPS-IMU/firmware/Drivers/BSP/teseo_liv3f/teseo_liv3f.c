/**
 *******************************************************************************
 * @file    teseo_liv3f_class.cpp
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

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "teseo_liv3f.h"
#include "globalvar.h"

/*
 * Constant for strtol base param
 */
#define BASE 10
#define strtof(A, B) strtod(A, B)

#define MAX_TRANSFER_SIZE 200

//TwoWire *dev_i2c = DEFAULT_I2C;
//HardwareSerial *dev_uart = DEFAULT_UART;
static int commandDone;
static char compareMessage[MAX_RESPONSE_LENGTH];

#if defined (TESEO_USE_I2C)
I2CHandler i2ch;
#else
#define MAX_DMA_BUFFER_SIZE 512
UART_HandleTypeDef huart1;
UARTHandler uarth;
DMA_HandleTypeDef hdma_usart1_rx;

uint8_t uart_dma_buffer[MAX_DMA_BUFFER_SIZE];
uint32_t buffer_end;
uint32_t buffer_index;
#endif /* TESEO_USE_I2C */

static GNSSParser_Data_t data;
static uint8_t app[MAX_MSG_LEN][MAX_FIELD_LENGTH];
static Coords_t xyz_coords_dd;

static void MSP_TESEO_Init(void);

#if defined (TESEO_USE_I2C)
/**
 * @brief       Update the internal data structures of the sensor using I2C communication
 * @return      GNSS_OK on Success
 */
static GNSS_StatusTypeDef TESEO_I2CUpdate();
#endif /* TESEO_USE_I2C */

#if defined (TESEO_USE_UART)
/**
 * @brief       Update the internal data structures of the sensor using UART communication
 * @return      GNSS_OK on Success
 */
static GNSS_StatusTypeDef TESEO_UARTUpdate();
#endif /* TESEO_USE_UART */

#if defined (TESEO_USE_I2C)
/**
 * @brief       	Sends the string to the I2C device
 * @param strToWr	The string to write
 * @return      	GNSS_OK on Success
 */
static GNSS_StatusTypeDef platform_WrWord(char *strToWr);
/**
 * @brief       Recieves 32 bytes from the I2C device
 * @param data	The buffer used to memorize the incoming bytes
 * @return      GNSS_OK on Success
 * @return		data pointer contains the recieved data
 */
static GNSS_StatusTypeDef platform_RdWord(char *data);

static GNSS_StatusTypeDef platform_I2CRead(uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
static GNSS_StatusTypeDef platform_I2CWrite(uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToWrite);
//#else
//static GNSS_StatusTypeDef platform_UARTRead(uint8_t* pBuffer, uint16_t NumByteToRead);
//static GNSS_StatusTypeDef platform_UARTWrite(uint8_t* pBuffer, uint16_t NumByteToWrite);
#endif /* TESEO_USE_I2C */

/**
 * @brief  This function initializes the agent handling parsed GNSS data
 * @param  pGNSSParser_Data The agent
 * @retval GNSS_PARSER_OK on success GNSS_PARSER_ERROR otherwise
 */
static GNSSParser_Status_t GNSS_PARSER_Init(GNSSParser_Data_t *pGNSSParser_Data);

/**
 * @brief  This function computes the checksum and checks the sanity of a GNSS sentence
 * @param  pSentence The sentence
 * @param  len The sentence length
 * @retval GNSS_PARSER_OK on success GNSS_PARSER_ERROR otherwise
 */
static GNSSParser_Status_t GNSS_PARSER_CheckSanity(uint8_t *pSentence, uint64_t len);

/**
 * @brief  This function dispatches a GNSS sentence to be parsed
 * @param  pGNSSParser_Data The agent
 * @param  msg The message type
 * @param  pBuffer The message to be dispatched
 * @retval GNSS_PARSER_OK on success GNSS_PARSER_ERROR otherwise
 */
static GNSSParser_Status_t GNSS_PARSER_ParseMsg(GNSSParser_Data_t *pGNSSParser_Data, uint8_t msg, uint8_t *pBuffer);

/**
 * @brief  Function that makes the parsing of the $GPGGA NMEA string with all Global Positioning System Fixed data.
 * @param  pGPGGAInfo     Pointer to GPGGA_Info_t struct
 * @param  NMEA	          NMEA string read by the Gps expansion
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParseGPGGA(GPGGA_Info_t *pGPGGAInfo, uint8_t NMEA[]);

static void NMEA_ScanUtc(uint8_t *pUTCStr, UTC_Info_t *pUTC);

static uint32_t NMEA_Checksum(const uint8_t buf[]);

/**
 * @brief  Function that makes the parsing of the string read by the Gps expansion, capturing the right parameters from it.
 * @param  pGNSInfo      Pointer to GNS_Info_t struct
 * @param  NMEA[]        NMEA string read by the Gps expansion
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParseGNS(GNS_Info_t *pGNSInfo, uint8_t NMEA[]);

static int32_t NMEA_CheckGNSMsg(const char header[]);

/**
 * @brief  Function that makes the parsing of the $GPGST NMEA string with GPS Pseudorange Noise Statistics.
 * @param  pGPGSTInfo    Pointer to a GPGST_Info_t struct
 * @param  NMEA	         NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParseGPGST(GPGST_Info_t *pGPGSTInfo, uint8_t NMEA[]);

/**
 * @brief  Function that makes the parsing of the $GPRMC NMEA string with Recommended Minimum Specific GPS/Transit data.
 * @param  pGPRMCInfo    Pointer to a GPRMC_Info_t struct
 * @param  NMEA	         NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParseGPRMC(GPRMC_Info_t *pGPRMCInfo, uint8_t NMEA[]);

/**
 * @brief  Function that makes the parsing of the $GSA NMEA string.
 * @param  pGSAInfo      Pointer to a GSA_Info_t struct
 * @param  NMEA	         NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParseGSA(GSA_Info_t *pGSAInfo, uint8_t NMEA[]);

static int32_t NMEA_CheckGSAMsg(const char header[]);

/**
 * @brief  Function that makes the parsing of the $GSV NMEA string.
 * @param  pGSVInfo      Pointer to a GSV_Info_t struct
 * @param  NMEA	         NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParseGSV(GSV_Info_t *pGSVInfo, uint8_t NMEA[]);

static int32_t NMEA_CheckGSVMsg(const char header[]);

static void NMEA_ResetGSVMsg(GSV_Info_t *pGSVInfo);

/**
 * @brief  Function that parses of the $PSTMVER NMEA string with version data.
 * @param  pPSTMVER      Pointer to PSTMVER_Info_t struct
 * @param  NMEA	         NMEA string read by the Gps expansion
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParsePSTMVER(PSTMVER_Info_t *pPSTMVER, uint8_t NMEA[]);

/**
 * @brief  This function parses the geofence related messages
 * @param  pGeofence     Pointer to Geofence_Info_t
 * @param  NMEA	         NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParsePSTMGeofence(Geofence_Info_t *pGeofence, uint8_t NMEA[]);

static int32_t NMEA_CheckGeofenceMsg(const char header[]);

static void NMEA_ScanTimestampTime(uint8_t buf[], Timestamp_Info_t *pTimestamp);

static void NMEA_ScanTimestampDate(uint8_t buf[], Timestamp_Info_t *pTimestamp);

static uint32_t NMEA_Digit2int(uint8_t buf[], int32_t offset, Decimal_t d);

/**
 * @brief  This function parses the odometer related messages
 * @param  pOdo          Pointer to a Odometer_Info_t struct
 * @param  NMEA          NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParsePSTMOdo(Odometer_Info_t *pOdo, uint8_t NMEA[]);

static int32_t NMEA_CheckOdoMsg(const char header[]);

/**
 * @brief  This function parses the datalog related messages
 * @param  pDatalog      Pointer to a Datalog_Info_t struct
 * @param  NMEA          NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParsePSTMDatalog(Datalog_Info_t *pDatalog, uint8_t NMEA[]);

static int32_t NMEA_CheckDatalogMsg(const char header[]);

/**
 * @brief  This function parses the list configuration message
 * @param  pResult             Ack from Teseo
 * @param  NMEA                NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParsePSTMsgl(OpResult_t *pResult, uint8_t NMEA[]);

static int32_t NMEA_CheckListMsg(const char header[]);

/**
 * @brief  This function parses the SavePar messages
 * @param  pResult             Ack from Teseo
 * @param  NMEA                NMEA string read by the Gps expansion.
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParsePSTMSavePar(OpResult_t *pResult, uint8_t NMEA[]);

static int32_t NMEA_CheckSaveparMsg(const char header[]);

/**
 * @brief  Function that parses of the $PSTMSTAGPSPASSRTN NMEA string with version data.
 * @param  pPSTMPASSRTN  Pointer to PSTMPASSRTN_Info_t struct
 * @param  NMEA	         NMEA string read by the Gps expansion
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParsePSTMPASSRTN(PSTMPASSRTN_Info_t *pPSTMPASSRTN, uint8_t NMEA[]);

static int32_t NMEA_CheckPassMsg(const char header[]);

/**
 * @brief  Function that parses of the $PSTMSTAGPS NMEA string with version data.
 * @param  pPSTMAGPS Pointer to PSTMAGPS_Info_t struct
 * @param  NMEA	     NMEA string read by the Gps expansion
 * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
static ParseStatus_t NMEA_ParsePSTMAGPS(PSTMAGPS_Info_t *pPSTMAGPS, uint8_t NMEA[]);

static int32_t NMEA_CheckAGPSMsg(const char header[]);

/**
 * @brief  This function makes a copy of the datas stored into GPGGAInfo into the pInfo param
 * @param  pInfo     Pointer to GPGGA_Info_t object where there are the GPGGA_Info_t to be copied
 * @param  GPGGAInfo Instance of a GPGGA_Info_t object pointer where the GPGGA_Info_t stored into pInfo have to be copied
 * @retval None
 */
//static void NMEA_Copy_Data(GPGGA_Info_t *pInfo, GPGGA_Info_t GPGGAInfo);

/**
 * @brief  This function converts a character to unsigned integer
 * @param  c The character to convert
 * @retval The returned unsigned integer
 */
static uint32_t NMEA_Char2int(uint8_t c);

/**
 * @brief       Initialize the sensor and the data structures
 * @note		in case of I2C communication, I2C peripheral should be initialized before calling this function
 * @return      GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_Init()
{
	MSP_TESEO_Init();
	TESEO_WakeupChip();
	TESEO_ResetChip();

#if defined (TESEO_USE_I2C)
	i2ch.stringComplete = false;
	i2ch.index = 0;
	i2ch.end = 0;
#else
	uarth.stringComplete=false;
	uarth.index=0;
	uarth.end =0;
#endif /* TESEO_USE_I2C */
	commandDone = 1;

	GNSS_PARSER_Init(&data);

#if defined (TESEO_USE_I2C)
	memset(i2ch.inputString, 0, MAX_STRING_LENGTH);
	memset(i2ch.inputString2, 0, MAX_STRING_LENGTH);
#endif /* TESEO_USE_I2C */

	TESEO_SendCommand((char *)"$PSTMRESTOREPAR");
	TESEO_SendCommand((char *)"$PSTMSRR");
	HAL_Delay(4000);

	return GNSS_OK;
}

/**
 * @brief       Update the internal data structures of the sensor using the appropriate communication method
 * @note		To prevent data loss, this function should be called at least 20 times per second
 * @return      GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_Update()
{
#if defined (TESEO_USE_I2C)
	return TESEO_I2CUpdate();
#else
	return TESEO_UARTUpdate();
#endif /* TESEO_USE_I2C */

	return GNSS_TIMEOUT;
}

/**
 * @brief       	Send a command to the device
 * @param command	The command to send
 * @return      	GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_SendCommand(char *command)
{
#if defined (TESEO_USE_I2C)
	platform_WrWord(command);
#else
	HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 1000);
#endif /* TESEO_USE_I2C */

	return GNSS_OK;
}

/**
 * @brief    	    Ask the device for a specific message
 * @param message	The message to receive
 * @return   	    GNSS_OK on Success
 */
GNSS_StatusTypeDef TESEO_AskMessage(char* message)
{
	memset(compareMessage, 0, MAX_RESPONSE_LENGTH);
	strncpy(compareMessage, message, strlen(message)+1);
	commandDone=0;
	return GNSS_OK;
}

/**
 * @brief       Ask the device if the message requested by @a askMessage() was recieved
 * @return      1 if the message was received, 0 otherwise
 */
int TESEO_GetMessageDone ()
{
	return commandDone;
}

/**
 * @brief       Get the complete data structure
 * @return      The full data structure
 */
GNSSParser_Data_t TESEO_GetData()
{
	return data;
}

/**
 * @brief       Get the wakeup status of the device
 * @return      1 if the device is enabled, 0 otherwise
 */
int TESEO_GetWakeupStatus()
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) return 1;
	else return 0;
}

/**
 * @brief       Get the GPGGA coordinates
 * @return      The coordinates structure
 * @remark      Coordinates are given in Decimal Degree Minute (DMM) (also known as GPS format)
 * @remark      See google :https://support.google.com/maps/answer/18539?hl=fr&co=GENIE.Platform%3DAndroid)
 */
Coords_t TESEO_GetCoords_DMM()
{
	return data.gpgga_data.xyz;
}

/**
 * @brief       Get the GPGGA coordinates in DD
 * @return      The coordinates structure
 * @remark      Convert coordinates in Decimal Degree Minute (DMM) to Decimal Degree (DD)
 */
Coords_t TESEO_GetCoords_DD()
{
	float degrees;
	float minutes;
	float tmp;

	memcpy ((void*)&xyz_coords_dd, (void*)&data.gpgga_data.xyz, sizeof(xyz_coords_dd));

	// Convert Latitude
	// convert DD MM.mmm to DD.MMmmm
	tmp = data.gpgga_data.xyz.lat/100.0;
	// Split fractional part (0.MMmmm) from integer part (DD.0)
	minutes=modff(tmp, &degrees);
	// Restore integer minutes (0.MMmmm => MM.mmm)
	minutes = minutes*100.0;

	// Convert to decimal degree (DD.ddddd)
	xyz_coords_dd.lat = degrees+(minutes/60.0);

	// Convert Longitude
	// convert DD MM.mmm to DD.MMmmm
	tmp = data.gpgga_data.xyz.lon/100.0;
	// Split fractional part (0.MMmmm) from integer part (DD.0)
	minutes=modff(tmp, &degrees);
	// Restore integer minutes (0.MMmmm => MM.mmm)
	minutes = minutes*100.0;

	// Convert to decimal degree (DD.ddddd)
	xyz_coords_dd.lon = degrees+(minutes/60.0);

	// Others fields don't need correction
	return xyz_coords_dd;
}

/**
 * @brief       Get the debug status of the device
 * @return      DEBUG_ON if debug is enabled, DEBUG_OFF otherwise
 */
Debug_State TESEO_GetDebugStatus ()
{
	return data.debug;
}

/**
 * @brief       Get the GPGGA message data structure
 * @return      The required data structure
 */
GPGGA_Info_t TESEO_GetGPGGAData()
{
	return data.gpgga_data;
}

/**
 * @brief       Get the --GNS message data structure
 * @return      The required data structure
 */
GNS_Info_t TESEO_GetGNSData ()
{
	return data.gns_data;
}

/**
 * @brief       Get the GPGTS message data structure
 * @return      The required data structure
 */
GPGST_Info_t TESEO_GetGPGSTData()
{
	return data.gpgst_data;
}

/**
 * @brief       Get the GPRMC message data structure
 * @return      The required data structure
 */
GPRMC_Info_t TESEO_GetGPRMCData ()
{
	return data.gprmc_data;
}

/**
 * @brief       Get the --GSA message data structure
 * @return      The required data structure
 */
GSA_Info_t TESEO_GetGSAData ()
{
	return data.gsa_data;
}

/**
 * @brief       Get the --GSV message data structure
 * @return      The required data structure
 */
GSV_Info_t TESEO_GetGSVData ()
{
	return data.gsv_data;
}

/**
 * @brief       Get the PSTMVER message data structure
 * @return      The required data structure
 */
PSTMVER_Info_t TESEO_GetVERData ()
{
	return data.pstmver_data;
}

/**
 * @brief       Get the PSTMPASSRTN message data structure
 * @return      The required data structure
 */
PSTMPASSRTN_Info_t TESEO_GetPASSData ()
{
	return data.pstmpass_data;
}

/**
 * @brief       Get the PSTMAGPS message data structure
 * @return      The required data structure
 */
PSTMAGPS_Info_t TESEO_GetAGPSData ()
{
	return data.pstmagps_data;
}

/**
 * @brief       Get the Geofence message data structure
 * @return      The required data structure
 */
Geofence_Info_t TESEO_GetGeofenceData ()
{
	return data.geofence_data;
}

/**
 * @brief       Get the Odometer message data structure
 * @return      The required data structure
 */
Odometer_Info_t TESEO_GetOdometerData ()
{
	return data.odo_data;
}

/**
 * @brief       Get the Datalog structure
 * @return      The required data structure
 */
Datalog_Info_t TESEO_GetDatalogData ()
{
	return data.datalog_data;
}

/**
 * @brief       Get the result of the last command sent
 * @return      GNSS_OP_OK if it was a success, GNSS_OP_ERROR otherwise
 */
OpResult_t TESEO_GetResult ()
{
	return data.result;
}

/**
 * @brief       Activate/deactivate the debug flag
 * @return      The current state of the debug flag
 */
Debug_State TESEO_ToggleDebug()
{
	data.debug = (data.debug == DEBUG_ON ? DEBUG_OFF : DEBUG_ON);
	return data.debug;
}

/**
 * @brief       Low-level hardware initialisation
 */
static void MSP_TESEO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configure Teseo_reset Output Level (not under reset)*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	/* Configure Teseo_Wakup Output Level (chip is enabled (wakeup))*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	/* Configure GPIO pin : Teseo_Reset */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configure GPIO pins : Teseo_Wakup */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configure UART1 */
	__HAL_RCC_USART1_CLK_ENABLE();

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while (1);
	}

	/* Configure UART1 GPIO
	 * PA9: UART1 TX
	 * PA10: UART1 RX
	 */
	/* Configure GPIO pin : Teseo_Reset */
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate=GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Enable UART1 RX DMA mode */
	memset(uart_dma_buffer, 0, MAX_DMA_BUFFER_SIZE);
	buffer_end=0;
	buffer_index=0;

	/* USART1 DMA Init */
	/* USART1_RX Init */
	__HAL_RCC_DMA1_CLK_ENABLE();

	hdma_usart1_rx.Instance = DMA1_Channel5;
	hdma_usart1_rx.Init.Request = DMA_REQUEST_2;
	hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
	if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
	{
		while(1);
	}

	__HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);

	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	/* Initializes Rx sequence using Reception To Idle event API.
	     As DMA channel associated to UART Rx is configured as Circular,
	     reception is endless.
	     If reception has to be stopped, call to HAL_UART_AbortReceive() could be used.

	     Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
	     user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
	     following events :
	     - DMA RX Half Transfer event (HT)
	     - DMA RX Transfer Complete event (TC)
	     - IDLE event on UART Rx line (indicating a pause is UART reception flow)
	 */
	if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_dma_buffer, MAX_DMA_BUFFER_SIZE))
	{
		while (1);
	}
}

#if defined (TESEO_USE_UART)
void DMA1_Channel5_IRQHandler(void)
{
	/* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

	/* USER CODE END DMA1_Channel6_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
	/* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

	/* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
 * @brief  User implementation of the Reception Event Callback
 *         (Rx event notification called after use of advanced reception service).
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	buffer_end = Size;
}
#endif /* TESEO_USE_UART */

/**
 * @brief       Reset chip and wait 3s
 */
void TESEO_ResetChip(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(3000);
}

/**
 * @brief       Wakeup chip from hibernate mode
 */
void TESEO_WakeupChip(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(100);
}

/**
 * @brief       Switch chip into hibernate mode
 */
void TESEO_HibernateChip(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_Delay(100);
}

/************************************************************
 * Private functions
 ************************************************************/
#if defined (TESEO_USE_I2C)
static GNSS_StatusTypeDef TESEO_I2CUpdate()
{
	memset (i2ch.inChar, 0, BUFFER_SIZE);
	platform_RdWord(i2ch.inChar);
	for (int i=0; i<BUFFER_SIZE; i++)
	{
		if (i2ch.inChar[i] != (char) 0x00)
		{
			if (i2ch.stringComplete)
				i2ch.inputString2[i2ch.index] = i2ch.inChar[i];
			else
				i2ch.inputString[i2ch.index] = i2ch.inChar[i];
			i2ch.index = i2ch.index + 1;
		}
		if (i2ch.inChar[i] == '\n')
		{
			i2ch.stringComplete = true;
			i2ch.end = i2ch.index;
			i2ch.index = 0;
		}
	}

	HAL_Delay(1);

	if (i2ch.stringComplete)
	{
		uint8_t buffer[MAX_STRING_LENGTH];
		memset (buffer, 0, MAX_STRING_LENGTH);
		for (int i = 0; i < MAX_STRING_LENGTH; i++)
			buffer[i] = (uint8_t) i2ch.inputString[i];
		GNSSParser_Status_t status = GNSS_PARSER_CheckSanity(buffer, strlen((char *) buffer));
		if (status != GNSS_PARSER_ERROR)
		{
			if ((commandDone == 0) && (strncmp(compareMessage, (char *) buffer, strlen(compareMessage)) == 0))
			{
				commandDone = 1;
			}
			for(int m = 0; m < NMEA_MSGS_NUM; m++)
			{
				GNSS_PARSER_ParseMsg(&data, (eNMEAMsg)m, buffer);
			}
		}
		strncpy(i2ch.inputString, i2ch.inputString2, sizeof(i2ch.inputString));
		memset(i2ch.inputString2, 0, sizeof(i2ch.inputString2));
		i2ch.stringComplete = false;
	}
	return GNSS_OK;
}
#endif /* TESEO_USE_I2C */

#if defined (TESEO_USE_UART)
static uint32_t plateform_UARTAvailableData(void) {
	if (buffer_end < buffer_index)
		return (MAX_DMA_BUFFER_SIZE + buffer_end - buffer_index);
	else return (buffer_end - buffer_index);
}

static char plateform_UARTRead(void) {
	char tmp=0;

	if (plateform_UARTAvailableData()>0) {
		tmp= uart_dma_buffer[buffer_index];
		buffer_index++;
		if (buffer_index>MAX_DMA_BUFFER_SIZE) buffer_index=0;
	}

	return tmp;
}

static GNSS_StatusTypeDef TESEO_UARTUpdate()
{
	if (uarth.stringComplete)
	{
		uint8_t buffer[MAX_STRING_LENGTH];
		memset(buffer, 0, MAX_STRING_LENGTH);
		for (int i = 0; i < MAX_STRING_LENGTH; i++)
			buffer[i] = (uint8_t) uarth.inputString[i];
		GNSSParser_Status_t status = GNSS_PARSER_CheckSanity(buffer, uarth.end);

		if (status != GNSS_PARSER_ERROR)
		{
			if ((commandDone == 0) && (strncmp(compareMessage, (char *) buffer, strlen(compareMessage)) == 0))
			{
				commandDone = 1;
			}
			for(int m = 0; m < NMEA_MSGS_NUM; m++)
			{
				GNSS_PARSER_ParseMsg(&data, (eNMEAMsg)m, buffer);
			}
		}
		memset(uarth.inputString, 0, sizeof(uarth.inputString));
		uarth.stringComplete = false;
	}

	/* A revoir */
	while (plateform_UARTAvailableData())
	{
		char inputChar = plateform_UARTRead();
		uarth.inputString[uarth.index]= inputChar;
		if (inputChar == '\n')
		{
			uarth.stringComplete = true;
			uarth.end = uarth.index+1;
			uarth.index = 0;
			break;
		}
		uarth.index = uarth.index + 1;
	}
	return GNSS_OK;
}
#endif /* TESEO_USE_UART */

#if defined (TESEO_USE_I2C)
static GNSS_StatusTypeDef platform_I2CRead(uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
	int status = 0;
	HAL_StatusTypeDef HAL_Status;

	HAL_Status = HAL_I2C_Mem_Read(&hi2c1, DEFAULT_DEVICE_ADDRESS, RegisterAddr,
			I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead, 1000);

	if (HAL_Status == HAL_OK) status=1;
	else status=0;

	return (GNSS_StatusTypeDef) status;
}

static GNSS_StatusTypeDef platform_I2CWrite(uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
	int status = 0;
	HAL_StatusTypeDef HAL_Status;

	HAL_Status = HAL_I2C_Mem_Write(&hi2c1, DEFAULT_DEVICE_ADDRESS, RegisterAddr,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) pBuffer, NumByteToWrite, 1000);

	if (HAL_Status == HAL_OK) status=1;
	else status=0;

	return (GNSS_StatusTypeDef) status;
}
#endif /* TESEO_USE_I2C */

static GNSSParser_Status_t GNSS_PARSER_Init(GNSSParser_Data_t *pGNSSParser_Data)
{
	if (pGNSSParser_Data == NULL)
	{
		return GNSS_PARSER_ERROR;
	}

	pGNSSParser_Data->debug = DEBUG_ON;
	(void)memset((void *)(&pGNSSParser_Data->gpgga_data), 0, sizeof(GPGGA_Info_t));
	pGNSSParser_Data->gpgga_data.xyz.ew = (uint8_t)' ';
	pGNSSParser_Data->gpgga_data.xyz.ns = (uint8_t)' ';
	pGNSSParser_Data->gpgga_data.xyz.mis = (uint8_t)' ';

	(void)memset((void *)(&pGNSSParser_Data->gns_data), 0, sizeof(GNS_Info_t));
	pGNSSParser_Data->gns_data.xyz.ew = (uint8_t)' ';
	pGNSSParser_Data->gns_data.xyz.ns = (uint8_t)' ';

	(void)memset((void *)(&pGNSSParser_Data->gpgst_data), 0, sizeof(GPGST_Info_t));

	(void)memset((void *)(&pGNSSParser_Data->gprmc_data), 0, sizeof(GPRMC_Info_t));
	pGNSSParser_Data->gprmc_data.xyz.ew = (uint8_t)' ';
	pGNSSParser_Data->gprmc_data.xyz.ns = (uint8_t)' ';

	(void)memset((void *)(&pGNSSParser_Data->gsa_data), 0, sizeof(GSA_Info_t));
	(void)memset((void *)(&pGNSSParser_Data->gsv_data), 0, sizeof(GSV_Info_t));
	(void)memset((void *)(&pGNSSParser_Data->pstmver_data), 0, sizeof(PSTMVER_Info_t));
	(void)memset((void *)(&pGNSSParser_Data->pstmpass_data), 0, sizeof(PSTMPASSRTN_Info_t));
	(void)memset((void *)(&pGNSSParser_Data->pstmagps_data), 0, sizeof(PSTMAGPS_Info_t));
	(void)memset((void *)(&pGNSSParser_Data->geofence_data), 0, sizeof(Geofence_Info_t));
	(void)memset((void *)(&pGNSSParser_Data->odo_data), 0, sizeof(Odometer_Info_t));
	(void)memset((void *)(&pGNSSParser_Data->datalog_data), 0, sizeof(Datalog_Info_t));
	(void)memset((void *)(&pGNSSParser_Data->result), 0, sizeof(OpResult_t));

	return GNSS_PARSER_OK;
}

static GNSSParser_Status_t GNSS_PARSER_CheckSanity(uint8_t *pSentence, uint64_t len)
{
	uint32_t checksum, check = 0U;

	if((len > 0U) && (len < 5U))
	{
		return GNSS_PARSER_ERROR;
	}
	if(len == 0U)
	{
		return GNSS_PARSER_OK;
	}
	checksum = (NMEA_Char2int(pSentence[len-4U]) << 4) | NMEA_Char2int(pSentence[len-3U]);

	for(uint64_t c = 1U; c < (len-5U); c++)
	{
		check = (check ^ pSentence[c]);
	}

	return (check == checksum) ? GNSS_PARSER_OK : GNSS_PARSER_ERROR;
}

static GNSSParser_Status_t GNSS_PARSER_ParseMsg(GNSSParser_Data_t *pGNSSParser_Data, uint8_t msg, uint8_t *pBuffer)
{
	ParseStatus_t status = PARSE_FAIL;

	switch(msg)
	{
	case GPGGA:
		status = NMEA_ParseGPGGA(&pGNSSParser_Data->gpgga_data, pBuffer);
		break;
	case GNS:
		status = NMEA_ParseGNS(&pGNSSParser_Data->gns_data, pBuffer);
		break;
	case GPGST:
		status = NMEA_ParseGPGST(&pGNSSParser_Data->gpgst_data, pBuffer);
		break;
	case GPRMC:
		status = NMEA_ParseGPRMC(&pGNSSParser_Data->gprmc_data, pBuffer);
		break;
	case GSA:
		status = NMEA_ParseGSA(&pGNSSParser_Data->gsa_data, pBuffer);
		break;
	case GSV:
		status = NMEA_ParseGSV(&pGNSSParser_Data->gsv_data, pBuffer);
		break;
	case PSTMVER:
		status = NMEA_ParsePSTMVER(&pGNSSParser_Data->pstmver_data, pBuffer);
		break;
	case PSTMPASSRTN:
		status = NMEA_ParsePSTMPASSRTN(&pGNSSParser_Data->pstmpass_data, pBuffer);
		break;
	case PSTMAGPSSTATUS:
		status = NMEA_ParsePSTMAGPS(&pGNSSParser_Data->pstmagps_data, pBuffer);
		break;
	case PSTMGEOFENCE:
		status = NMEA_ParsePSTMGeofence(&pGNSSParser_Data->geofence_data, pBuffer);
		break;
	case PSTMODO:
		status = NMEA_ParsePSTMOdo(&pGNSSParser_Data->odo_data, pBuffer);
		break;
	case PSTMDATALOG:
		status = NMEA_ParsePSTMDatalog(&pGNSSParser_Data->datalog_data, pBuffer);
		break;
	case PSTMSGL:
		status = NMEA_ParsePSTMsgl(&pGNSSParser_Data->result, pBuffer);
		break;
	case PSTMSAVEPAR:
		status = NMEA_ParsePSTMSavePar(&pGNSSParser_Data->result, pBuffer);
		break;
	default:
		break;
	}

	return ((status == PARSE_FAIL) ? GNSS_PARSER_ERROR : GNSS_PARSER_OK);
}

static ParseStatus_t NMEA_ParseGPGGA(GPGGA_Info_t *pGPGGAInfo, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (strcmp((char *)app[0], "$GPGGA") == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			int32_t valid = strtol((char *)app[6], NULL, BASE);
			if((valid == 1) || (valid == 0))
			{
				pGPGGAInfo->valid = (uint8_t)valid;
			}

			NMEA_ScanUtc(app[1], &pGPGGAInfo->utc);
			pGPGGAInfo->xyz.lat = strtod((char *)app[2], NULL);
			pGPGGAInfo->xyz.ns = *((uint8_t*)app[3]);
			pGPGGAInfo->xyz.lon = strtod((char *)app[4], NULL);
			pGPGGAInfo->xyz.ew = *((uint8_t*)app[5]);
			pGPGGAInfo->sats = strtol((char *)app[7], NULL, BASE);
			pGPGGAInfo->acc = strtof((char *)app[8], NULL);
			pGPGGAInfo->xyz.alt = strtof((char *)app[9], NULL);
			pGPGGAInfo->xyz.mis = *((uint8_t*)app[10]);
			pGPGGAInfo->geoid.height = strtol((char *)app[11], NULL, BASE);
			pGPGGAInfo->geoid.mis = *((uint8_t*)app[12]);
			// This field is reserved
			//pGPGGAInfo->update = strtol((char *)app[13], NULL, BASE);
			pGPGGAInfo->checksum = NMEA_Checksum(app[15]);

			status = PARSE_SUCC;
		}
	}

	return status;
}

static void NMEA_ScanUtc(uint8_t *pUTCStr, UTC_Info_t *pUTC)
{
	pUTC->utc = strtol((char *)pUTCStr,NULL,10);

	pUTC->hh = (pUTC->utc / 10000);
	pUTC->mm = (pUTC->utc - (pUTC->hh * 10000)) / 100;
	pUTC->ss = pUTC->utc - ((pUTC->hh * 10000) + (pUTC->mm * 100));

	return;
}

static uint32_t NMEA_Checksum(const uint8_t buf[])
{
	return ((NMEA_Char2int(buf[0]) << 4) | (NMEA_Char2int(buf[1])));
}


static ParseStatus_t NMEA_ParseGNS(GNS_Info_t *pGNSInfo, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{
		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckGNSMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			(void)strncpy((char *)pGNSInfo->constellation, (char *)app[0], MAX_STR_LEN);
			NMEA_ScanUtc(app[1], &pGNSInfo->utc);
			pGNSInfo->xyz.lat = strtod((char *)app[2], NULL);
			pGNSInfo->xyz.ns = *((uint8_t*)app[3]);
			pGNSInfo->xyz.lon = strtod((char *)app[4], NULL);
			pGNSInfo->xyz.ew = *((uint8_t*)app[5]);
			pGNSInfo->gps_mode = *((uint8_t*)app[6]);
			pGNSInfo->glonass_mode = *((uint8_t*)app[7]);
			pGNSInfo->sats = strtol((char *)app[8], NULL, BASE);
			pGNSInfo->hdop = strtof((char *)app[9], NULL);
			pGNSInfo->xyz.alt = strtof((char *)app[10], NULL);
			pGNSInfo->geo_sep = strtof((char *)app[11], NULL);
			pGNSInfo->dgnss_age = *((uint8_t*)app[12]);
			pGNSInfo->dgnss_ref = *((uint8_t*)app[13]);
			pGNSInfo->checksum = NMEA_Checksum(app[14]);

			status = PARSE_SUCC;
		}
	}

	return status;
}

static int32_t NMEA_CheckGNSMsg(const char header[])
{
	int32_t is_gnsmsg = 1;

	if (strcmp(header, "$GPGNS") == 0)
	{
		is_gnsmsg = 0;
	}
	if (strcmp(header, "$GAGNS") == 0)
	{
		is_gnsmsg = 0;
	}
	if (strcmp(header, "$BDGNS") == 0)
	{
		is_gnsmsg = 0;
	}
	if (strcmp(header, "$QZGNS") == 0)
	{
		is_gnsmsg = 0;
	}
	if (strcmp(header, "$GNGNS") == 0)
	{
		is_gnsmsg = 0;
	}

	return is_gnsmsg;
}

static ParseStatus_t NMEA_ParseGPGST(GPGST_Info_t *pGPGSTInfo, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (strcmp((char *)app[0], "$GPGST") == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			NMEA_ScanUtc(app[1], &pGPGSTInfo->utc);
			pGPGSTInfo->EHPE = strtof((char *)app[2], NULL);
			pGPGSTInfo->semi_major_dev = strtof((char *)app[3], NULL);
			pGPGSTInfo->semi_minor_dev = strtof((char *)app[4], NULL);
			pGPGSTInfo->semi_major_angle = strtof((char *)app[5], NULL);
			pGPGSTInfo->lat_err_dev = strtof((char *)app[6], NULL);
			pGPGSTInfo->lon_err_dev = strtof((char *)app[7], NULL);
			pGPGSTInfo->alt_err_dev = strtof((char *)app[8], NULL);
			pGPGSTInfo->checksum = NMEA_Checksum(app[9]);

			status = PARSE_SUCC;
		}
	}

	return status;
}

static ParseStatus_t NMEA_ParseGPRMC(GPRMC_Info_t *pGPRMCInfo, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (strcmp((char *)app[0], "$GPRMC") == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			NMEA_ScanUtc(app[1],  &pGPRMCInfo->utc);
			pGPRMCInfo->status = *((uint8_t*)app[2]);
			pGPRMCInfo->xyz.lat = strtod((char *)app[3], NULL);
			pGPRMCInfo->xyz.ns = *((uint8_t*)app[4]);
			pGPRMCInfo->xyz.lon = strtod((char *)app[5], NULL);
			pGPRMCInfo->xyz.ew = *((uint8_t*)app[6]);
			pGPRMCInfo->speed = strtof((char *)app[7], NULL);
			pGPRMCInfo->trackgood = strtof((char *)app[8], NULL);
			pGPRMCInfo->date = strtol((char *)app[9], NULL, BASE);
			pGPRMCInfo->mag_var = strtof((char *)app[10], NULL);
			pGPRMCInfo->mag_var_dir = *((uint8_t*)app[11]);
			/* WARNING: from received msg, it seems there is another data (app[12]) before the checksum */
			pGPRMCInfo->checksum = NMEA_Checksum(app[13]);

			status = PARSE_SUCC;
		}
	}

	return status;
}

static ParseStatus_t NMEA_ParseGSA(GSA_Info_t *pGSAInfo, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)(uint8_t)(uint8_t)(uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)(uint8_t)(uint8_t)',') || (NMEA[i] == (uint8_t)(uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckGSAMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			(void)strncpy((char *)pGSAInfo->constellation, (char *)app[0], MAX_STR_LEN);
			pGSAInfo->operating_mode = *((uint8_t*)app[1]);
			pGSAInfo->current_mode = strtol((char *)app[2], NULL, BASE);

			int32_t *sat_prn = pGSAInfo->sat_prn;
			for (int8_t i = 0; i < MAX_SAT_NUM; i++)
			{
				*(&sat_prn[i]) = strtol((char *)app[3+i], NULL, BASE);
			}

			pGSAInfo->pdop = strtof((char *)app[15], NULL);
			pGSAInfo->hdop = strtof((char *)app[16], NULL);
			pGSAInfo->vdop = strtof((char *)app[17], NULL);
			pGSAInfo->checksum = NMEA_Checksum(app[18]);

			status = PARSE_SUCC;
		}
	}

	return status;
}

static int32_t NMEA_CheckGSAMsg(const char header[])
{
	int32_t is_gsamsg = 1;

	if (strcmp(header, "$GPGSA") == 0)
	{
		is_gsamsg = 0;
	}
	if (strcmp(header, "$GLGSA") == 0)
	{
		is_gsamsg = 0;
	}
	if (strcmp(header, "$GAGSA") == 0)
	{
		is_gsamsg = 0;
	}
	if (strcmp(header, "$BDGSA") == 0)
	{
		is_gsamsg = 0;
	}
	if (strcmp(header, "$GNGSA") == 0)
	{
		is_gsamsg = 0;
	}

	return is_gsamsg;
}

static ParseStatus_t NMEA_ParseGSV(GSV_Info_t *pGSVInfo, uint8_t NMEA[])
{
	int8_t app_idx;
	int32_t gsv_idx = 0;
	int32_t new_field;
	BOOL valid_gsv_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckGSVMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_gsv_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_gsv_msg == TRUE)
		{
			NMEA_ResetGSVMsg(pGSVInfo);

			(void)strncpy((char *)pGSVInfo->constellation, (char *)app[0], MAX_STR_LEN);
			pGSVInfo->amount = strtol((char *)app[1], NULL, BASE);
			pGSVInfo->number = strtol((char *)app[2], NULL, BASE);
			pGSVInfo->tot_sats = strtol((char *)app[3], NULL, BASE);
			app_idx = 4;
			for (int8_t i = 1; i <= GSV_MSG_SATS; i++)
			{
				pGSVInfo->gsv_sat_i[gsv_idx].prn = strtol((char *)app[app_idx*i], NULL, BASE);
				pGSVInfo->gsv_sat_i[gsv_idx].elev = strtol((char *)app[(app_idx*i)+1], NULL, BASE);
				pGSVInfo->gsv_sat_i[gsv_idx].azim = strtol((char *)app[(app_idx*i)+2], NULL, BASE);
				pGSVInfo->gsv_sat_i[gsv_idx].cn0 = strtol((char *)app[(app_idx*i)+3], NULL, BASE);

				if(pGSVInfo->gsv_sat_i[gsv_idx].prn != 0)
				{
					pGSVInfo->current_sats++;
				}
				gsv_idx++;
			}

			status = PARSE_SUCC;
		}

	}

	return status;
}

static int32_t NMEA_CheckGSVMsg(const char header[])
{
	int32_t is_gsvmsg = 1;

	if (strcmp(header, "$GPGSV") == 0)
	{
		is_gsvmsg = 0;
	}
	if (strcmp(header, "$GLGSV") == 0)
	{
		is_gsvmsg = 0;
	}
	if (strcmp(header, "$GAGSV") == 0)
	{
		is_gsvmsg = 0;
	}
	if (strcmp(header, "$BDGSV") == 0)
	{
		is_gsvmsg = 0;
	}
	if (strcmp(header, "$QZGSV") == 0)
	{
		is_gsvmsg = 0;
	}
	if (strcmp(header, "$GNGSV") == 0)
	{
		is_gsvmsg = 0;
	}

	return is_gsvmsg;
}

static void NMEA_ResetGSVMsg(GSV_Info_t *pGSVInfo)
{
	(void)memset(pGSVInfo->constellation, 0, (size_t)MAX_STR_LEN);
	pGSVInfo->amount = 0;
	pGSVInfo->number = 0;
	pGSVInfo->current_sats = 0;
	pGSVInfo->tot_sats = 0;
	for (int8_t i = 0; i < MAX_SAT_NUM; i++)
	{
		(void)memset(&pGSVInfo->gsv_sat_i[i], 0, sizeof(GSV_SAT_Info_t));
	}
}

static ParseStatus_t NMEA_ParsePSTMVER(PSTMVER_Info_t *pPSTMVER, uint8_t NMEA[])
{
	int8_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (strcmp((char *)app[0], "$PSTMVER") == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			(void)strncpy((char *)pPSTMVER->pstmver_string, (char *)app[1], MAX_STR_LEN);

			status = PARSE_SUCC;
		}
	}
	return status;
}

static ParseStatus_t NMEA_ParsePSTMGeofence(Geofence_Info_t *pGeofence, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckGeofenceMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			/* Enabling */
			if (strcmp((char *)app[0], "$PSTMCFGGEOFENCEOK") == 0)
			{
				pGeofence->op = GNSS_FEATURE_EN_MSG;
				pGeofence->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMCFGGEOFENCEERROR") == 0)
			{
				pGeofence->op = GNSS_FEATURE_EN_MSG;
				pGeofence->result = GNSS_OP_ERROR;
			}
			/* Configuring */
			else if (strcmp((char *)app[0], "$PSTMGEOFENCECFGOK") == 0)
			{
				pGeofence->op = GNSS_GEOFENCE_CFG_MSG;
				pGeofence->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMGEOFENCECFGERROR") == 0)
			{
				pGeofence->op = GNSS_GEOFENCE_STATUS_MSG;
				pGeofence->result = GNSS_OP_ERROR;
			}
			/* Querying Status */
			else if (strcmp((char *)app[0], "$PSTMGEOFENCESTATUS") == 0)
			{
				pGeofence->op = GNSS_GEOFENCE_STATUS_MSG;
				NMEA_ScanTimestampTime(app[1], &pGeofence->timestamp);
				NMEA_ScanTimestampDate(app[2], &pGeofence->timestamp);

				int32_t *geofence_status = pGeofence->status;
				for(uint8_t i = 0; i < MAX_GEOFENCES_NUM; i++)
				{
					*(&geofence_status[i]) = strtol((char *)app[3+i], NULL, BASE);
				}
			}
			/* Alarm Msg */
			else if (strcmp((char *)app[0], "$PSTMGEOFENCE") == 0)
			{
				pGeofence->op = GNSS_GEOFENCE_ALARM_MSG;
				NMEA_ScanTimestampTime(app[1], &pGeofence->timestamp);
				NMEA_ScanTimestampDate(app[2], &pGeofence->timestamp);
				pGeofence->idAlarm = strtol((char *)app[3], NULL, BASE);
				pGeofence->coords.lat = strtod((char *)app[4], NULL);
				pGeofence->coords.lon = strtod((char *)app[5], NULL);
				pGeofence->coords.radius = strtod((char *)app[6], NULL);
				pGeofence->coords.distance = strtod((char *)app[7], NULL);
				pGeofence->coords.tolerance = strtod((char *)app[8], NULL);
				pGeofence->status[pGeofence->idAlarm] = strtol((char *)app[9], NULL, BASE);
			}
			else
			{
				/* do nothing */
			}

			status = PARSE_SUCC;
		}
	}

	return status;
}

static int32_t NMEA_CheckGeofenceMsg(const char header[])
{
	int32_t is_geofencemsg = 1;

	if (strcmp(header, "$PSTMCFGGEOFENCEOK") == 0)
	{
		is_geofencemsg = 0;
	}
	if (strcmp(header, "$PSTMCFGGEOFENCEERROR") == 0)
	{
		is_geofencemsg = 0;
	}
	if (strcmp(header, "$PSTMGEOFENCECFGOK") == 0)
	{
		is_geofencemsg = 0;
	}
	if (strcmp(header, "$PSTMGEOFENCECFGERROR") == 0)
	{
		is_geofencemsg = 0;
	}
	if (strcmp(header, "$PSTMGEOFENCESTATUS") == 0)
	{
		is_geofencemsg = 0;
	}
	if (strcmp(header, "$PSTMGEOFENCE") == 0)
	{
		is_geofencemsg = 0;
	}
	if (strcmp(header, "$PSTMGEOFENCEREQERROR") == 0)
	{
		is_geofencemsg = 0;
	}

	return is_geofencemsg;
}

static void NMEA_ScanTimestampTime(uint8_t buf[], Timestamp_Info_t *pTimestamp)
{
	/* FORMAT: HHMMSS */
	pTimestamp->hh = NMEA_Digit2int(buf, 0, TENS);
	pTimestamp->mm = NMEA_Digit2int(buf, 2, TENS);
	pTimestamp->ss = NMEA_Digit2int(buf, 4, TENS);
}

static void NMEA_ScanTimestampDate(uint8_t buf[], Timestamp_Info_t *pTimestamp)
{
	/* FORMAT: YYYYMMDD */
	pTimestamp->year = NMEA_Digit2int(buf, 0, THOUSANDS);
	pTimestamp->month = NMEA_Digit2int(buf, 4, TENS);
	pTimestamp->day = NMEA_Digit2int(buf, 6, TENS);
}

static uint32_t NMEA_Digit2int(uint8_t buf[], int32_t offset, Decimal_t d)
{
	uint32_t ret = (unsigned char)0;
	uint32_t hu, hd, hc, hm;

	switch (d)
	{
	case TENS:
		hd = NMEA_Char2int(buf[offset]);
		hu = NMEA_Char2int(buf[offset+1]);

		ret = (hd * (unsigned)10) + hu;
		break;

	case HUNDREDS:
		hc = NMEA_Char2int(buf[offset]);
		hd = NMEA_Char2int(buf[offset+1]);
		hu = NMEA_Char2int(buf[offset+2]);

		ret = (hc * (unsigned)100) + (hd * (unsigned)10) + hu;
		break;

	case THOUSANDS:
		hm = NMEA_Char2int(buf[offset]);
		hc = NMEA_Char2int(buf[offset+1]);
		hd = NMEA_Char2int(buf[offset+2]);
		hu = NMEA_Char2int(buf[offset+3]);

		ret = (hm * (unsigned)1000) + (hc * (unsigned)100) + (hd * (unsigned)10) + hu;
		break;

	default:
		break;
	}

	return ret;
}

static ParseStatus_t NMEA_ParsePSTMOdo(Odometer_Info_t *pOdo, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckOdoMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			/* Enabling */
			if (strcmp((char *)app[0], "$PSTMCFGODOOK") == 0)
			{
				pOdo->op = GNSS_FEATURE_EN_MSG;
				pOdo->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMCFGODOERROR") == 0)
			{
				pOdo->op = GNSS_FEATURE_EN_MSG;
				pOdo->result = GNSS_OP_ERROR;
			}
			/* Start */
			else if (strcmp((char *)app[0], "$PSTMODOSTARTOK") == 0)
			{
				pOdo->op = GNSS_ODO_START_MSG;
				pOdo->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMODOSTARTERROR") == 0)
			{
				pOdo->op = GNSS_ODO_START_MSG;
				pOdo->result = GNSS_OP_ERROR;
			}
			/* Stop */
			else if (strcmp((char *)app[0], "$PSTMODOSTOPOK") == 0)
			{
				pOdo->op = GNSS_ODO_STOP_MSG;
				pOdo->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMODOSTOPERROR") == 0)
			{
				pOdo->op = GNSS_ODO_STOP_MSG;
				pOdo->result = GNSS_OP_ERROR;
			}
			else
			{
				/* do nothing */
			}

			status = PARSE_SUCC;
		}
	}
	return status;
}

static int32_t NMEA_CheckOdoMsg(const char header[])
{
	int32_t is_odomsg = 1;

	if (strcmp(header, "$PSTMCFGODOOK") == 0)
	{
		is_odomsg = 0;
	}
	if (strcmp(header, "$PSTMCFGODOERROR") == 0)
	{
		is_odomsg = 0;
	}
	if (strcmp(header, "$PSTMODOSTARTOK") == 0)
	{
		is_odomsg = 0;
	}
	if (strcmp(header, "$PSTMODOSTARTERROR") == 0)
	{
		is_odomsg = 0;
	}
	if (strcmp(header, "$PSTMODOSTOPOK") == 0)
	{
		is_odomsg = 0;
	}
	if (strcmp(header, "$PSTMODOSTOPERROR") == 0)
	{
		is_odomsg = 0;
	}

	return is_odomsg;
}

static ParseStatus_t NMEA_ParsePSTMDatalog(Datalog_Info_t *pDatalog, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckDatalogMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			/* Enabling */
			if (strcmp((char *)app[0], "$PSTMCFGLOGOK") == 0)
			{
				pDatalog->op = GNSS_FEATURE_EN_MSG;
				pDatalog->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMCFGLOGERROR") == 0)
			{
				pDatalog->op = GNSS_FEATURE_EN_MSG;
				pDatalog->result = GNSS_OP_ERROR;
			}
			/* Configuring */
			else if (strcmp((char *)app[0], "$PSTMLOGCREATEOK") == 0)
			{
				pDatalog->op = GNSS_DATALOG_CFG_MSG;
				pDatalog->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMLOGCREATEERROR") == 0)
			{
				pDatalog->op = GNSS_DATALOG_CFG_MSG;
				pDatalog->result = GNSS_OP_ERROR;
			}
			/* Start */
			else if (strcmp((char *)app[0], "$PSTMLOGSTARTOK") == 0)
			{
				pDatalog->op = GNSS_DATALOG_START_MSG;
				pDatalog->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMLOGSTARTERROR") == 0)
			{
				pDatalog->op = GNSS_DATALOG_START_MSG;
				pDatalog->result = GNSS_OP_ERROR;
			}
			/* Stop */
			else if (strcmp((char *)app[0], "$PSTMLOGSTOPOK") == 0)
			{
				pDatalog->op = GNSS_DATALOG_STOP_MSG;
				pDatalog->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMLOGSTOPERROR") == 0)
			{
				pDatalog->op = GNSS_DATALOG_STOP_MSG;
				pDatalog->result = GNSS_OP_ERROR;
			}
			/* Erase */
			else if (strcmp((char *)app[0], "$PSTMLOGERASEOK") == 0)
			{
				pDatalog->op = GNSS_DATALOG_ERASE_MSG;
				pDatalog->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMLOGERASEERROR") == 0)
			{
				pDatalog->op = GNSS_DATALOG_ERASE_MSG;
				pDatalog->result = GNSS_OP_ERROR;
			}
			else
			{
				/* do nothing */
			}

			status = PARSE_SUCC;
		}
	}
	return status;
}

static int32_t NMEA_CheckDatalogMsg(const char header[])
{
	int32_t is_datalogmsg = 1;

	if (strcmp(header, "$PSTMCFGLOGOK") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMCFGLOGERROR") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMLOGCREATEOK") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMLOGCREATEERROR") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMLOGSTARTOK") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMLOGSTARTERROR") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMLOGSTOPOK") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMLOGSTOPERROR") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMLOGERASEOK") == 0)
	{
		is_datalogmsg = 0;
	}
	if (strcmp(header, "$PSTMLOGERASEERROR") == 0)
	{
		is_datalogmsg = 0;
	}

	return is_datalogmsg;
}

static ParseStatus_t NMEA_ParsePSTMsgl(OpResult_t *pResult, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckListMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			/* Enabling */
			if (strcmp((char *)app[0], "$PSTMCFGMSGLOK") == 0)
			{
				*pResult = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMCFGMSGLERROR") == 0)
			{
				*pResult = GNSS_OP_ERROR;
			}
			else
			{
				/* do nothing */
			}

			status = PARSE_SUCC;
		}

	}
	return status;
}

static int32_t NMEA_CheckListMsg(const char header[])
{
	int32_t is_listmsg = 1;

	if (strcmp(header, "$PSTMCFGMSGLOK") == 0)
	{
		is_listmsg = 0;
	}
	if (strcmp(header, "$PSTMCFGMSGLERROR") == 0)
	{
		is_listmsg = 0;
	}

	return is_listmsg;
}

static ParseStatus_t NMEA_ParsePSTMSavePar(OpResult_t *pResult, uint8_t NMEA[])
{
	int32_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckSaveparMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			if (strcmp((char *)app[0], "$PSTMSAVEPAROK") == 0)
			{
				*pResult = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMSAVEPARERROR") == 0)
			{
				*pResult = GNSS_OP_ERROR;
			}
			else
			{
				/* do nothing */
			}

			status = PARSE_SUCC;
		}
	}
	return status;
}

static int32_t NMEA_CheckSaveparMsg(const char header[])
{
	int32_t is_savevarmsg = 1;

	if (strcmp(header, "$PSTMSAVEPAROK") == 0)
	{
		is_savevarmsg = 0;
	}
	if (strcmp(header, "$PSTMSAVEPARERROR") == 0)
	{
		is_savevarmsg = 0;
	}

	return is_savevarmsg;
}

static ParseStatus_t NMEA_ParsePSTMPASSRTN(PSTMPASSRTN_Info_t *pPSTMPASSRTN, uint8_t NMEA[])
{
	int8_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{

		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckPassMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			if (strcmp((char *)app[0], "$PSTMSTAGPS8PASSRTN") == 0)
			{
				(void)strncpy((char *)pPSTMPASSRTN->deviceId, (char *)app[1], MAX_STR_LEN);
				(void)strncpy((char *)pPSTMPASSRTN->pwd, (char *)app[2], 64);
				pPSTMPASSRTN->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMSTAGPS8PASSGENERROR") == 0)
			{
				pPSTMPASSRTN->result = GNSS_OP_ERROR;
			}
			else
			{
				/* do nothing */
			}

			status = PARSE_SUCC;
		}
	}
	return status;
}

static int32_t NMEA_CheckPassMsg(const char header[])
{
	int32_t is_passmsg = 1;

	if (strcmp(header, "$PSTMSTAGPS8PASSRTN") == 0)
	{
		is_passmsg = 0;
	}
	if (strcmp(header, "$PSTMSTAGPS8PASSGENERROR") == 0)
	{
		is_passmsg = 0;
	}

	return is_passmsg;
}

static ParseStatus_t NMEA_ParsePSTMAGPS(PSTMAGPS_Info_t *pPSTMAGPS, uint8_t NMEA[])
{
	int8_t new_field;
	BOOL valid_msg = FALSE;

	ParseStatus_t status = PARSE_FAIL;

	if(NMEA != NULL)
	{
		/* clear the app[][] buffer */
		for (int8_t i = 0; i < MAX_MSG_LEN; i++)
		{
			(void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
		}

		for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
		{
			new_field = 0;

			if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
			{
				app[j][k] = (uint8_t)'\0';
				new_field = 1;

				if (NMEA_CheckAGPSMsg((char *)app[0]) == 0)
				{
					j++;
					k = 0;
					valid_msg = TRUE;
				}
				else
				{
					break;
				}
			}
			if(new_field == 0)
			{
				app[j][k] = NMEA[i];
				k++;
			}
		}

		if (valid_msg == TRUE)
		{
			/* Status */
			if (strcmp((char *)app[0], "$PSTMAGPSSTATUS") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_STATUS_MSG;
				pPSTMAGPS->status = strtol((char *)app[1], NULL, BASE);
			}
			/* Begin */
			else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDBEGINOK") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_BEGIN_MSG;
				pPSTMAGPS->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDBEGINERROR") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_BEGIN_MSG;
				pPSTMAGPS->result = GNSS_OP_ERROR;
			}
			/* Block type */
			else if (strcmp((char *)app[0], "$PSTMSTAGPSBLKTYPEOK") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_BLKTYPE_MSG;
				pPSTMAGPS->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMSTAGPSBLKTYPEERROR") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_BLKTYPE_MSG;
				pPSTMAGPS->result = GNSS_OP_ERROR;
			}
			/* Slot freq */
			else if (strcmp((char *)app[0], "$PSTMSTAGPSSLOTFRQOK") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_SLOTFRQ_MSG;
				pPSTMAGPS->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMSTAGPSSLOTFRQERROR") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_SLOTFRQ_MSG;
				pPSTMAGPS->result = GNSS_OP_ERROR;
			}
			/* Seed pkt */
			else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDPKTOK") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_SEEDPKT_MSG;
				pPSTMAGPS->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDPKTERROR") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_SEEDPKT_MSG;
				pPSTMAGPS->result = GNSS_OP_ERROR;
			}
			/* Propagate */
			else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDPROPOK") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_PROP_MSG;
				pPSTMAGPS->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDPROPERROR") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_PROP_MSG;
				pPSTMAGPS->result = GNSS_OP_ERROR;
			}
			/* Init time */
			else if (strcmp((char *)app[0], "$PSTMINITTIMEOK") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_INITTIME_MSG;
				pPSTMAGPS->result = GNSS_OP_OK;
			}
			else if (strcmp((char *)app[0], "$PSTMINITTIMEERROR") == 0)
			{
				pPSTMAGPS->op = GNSS_AGPS_INITTIME_MSG;
				pPSTMAGPS->result = GNSS_OP_ERROR;
			}
			else
			{
				/* do nothing */
			}

			status = PARSE_SUCC;
		}
	}
	return status;
}

static int32_t NMEA_CheckAGPSMsg(const char header[])
{
	int32_t is_passmsg = 1;

	/* Status */
	if (strcmp(header, "$PSTMAGPSSTATUS") == 0)
	{
		is_passmsg = 0;
	}
	/* Begin */
	if (strcmp(header, "$PSTMSTAGPSSEEDBEGINOK") == 0)
	{
		is_passmsg = 0;
	}
	if (strcmp(header, "$PSTMSTAGPSSEEDBEGINERROR") == 0)
	{
		is_passmsg = 0;
	}
	/* Block type */
	if (strcmp(header, "$PSTMSTAGPSBLKTYPEOK") == 0)
	{
		is_passmsg = 0;
	}
	if (strcmp(header, "$PSTMSTAGPSBLKTYPEERROR") == 0)
	{
		is_passmsg = 0;
	}
	/* Slot freq */
	if (strcmp(header, "$PSTMSTAGPSSLOTFRQOK") == 0)
	{
		is_passmsg = 0;
	}
	if (strcmp(header, "$PSTMSTAGPSSLOTFRQERROR") == 0)
	{
		is_passmsg = 0;
	}
	/* Seed pkt */
	if (strcmp(header, "$PSTMSTAGPSSEEDPKTOK") == 0)
	{
		is_passmsg = 0;
	}
	if (strcmp(header, "$PSTMSTAGPSSEEDPKTERROR") == 0)
	{
		is_passmsg = 0;
	}
	/* Propagate */
	if (strcmp(header, "$PSTMSTAGPSSEEDPROPOK") == 0)
	{
		is_passmsg = 0;
	}
	if (strcmp(header, "$PSTMSTAGPSSEEDPROPERROR") == 0)
	{
		is_passmsg = 0;
	}
	/* Init time */
	if (strcmp(header, "$PSTMINITTIMEOK") == 0)
	{
		is_passmsg = 0;
	}
	if (strcmp(header, "$PSTMINITTIMEERROR") == 0)
	{
		is_passmsg = 0;
	}
	return is_passmsg;
}

//static void NMEA_Copy_Data(GPGGA_Info_t *pInfo, GPGGA_Info_t GPGGAInfo)
//{
//	pInfo->acc          = GPGGAInfo.acc;
//	pInfo->geoid.height = GPGGAInfo.geoid.height;
//	pInfo->geoid.mis    = GPGGAInfo.geoid.mis;
//	pInfo->sats         = GPGGAInfo.sats;
//	pInfo->update       = GPGGAInfo.update;
//	pInfo->utc.hh       = GPGGAInfo.utc.hh;
//	pInfo->utc.mm       = GPGGAInfo.utc.mm;
//	pInfo->utc.ss       = GPGGAInfo.utc.ss;
//	pInfo->utc.utc      = GPGGAInfo.utc.utc;
//	pInfo->valid        = GPGGAInfo.valid;
//	pInfo->xyz.alt      = GPGGAInfo.xyz.alt;
//	pInfo->xyz.lat      = GPGGAInfo.xyz.lat;
//	pInfo->xyz.lon      = GPGGAInfo.xyz.lon;
//	pInfo->xyz.ew       = GPGGAInfo.xyz.ew;
//	pInfo->xyz.ns       = GPGGAInfo.xyz.ns;
//	pInfo->xyz.mis      = GPGGAInfo.xyz.mis;
//	pInfo->checksum     = GPGGAInfo.checksum;
//}

uint32_t NMEA_Char2int(uint8_t c)
{
	uint32_t ret = (unsigned char)0;

	if((c >= (uint8_t)'0') && (c <= (uint8_t)'9'))
	{
		ret = (unsigned char)(c - (uint8_t)'0');
	}

	if((c >= (uint8_t)'A') && (c <= (uint8_t)'F'))
	{
		ret = (unsigned char)(c - (uint8_t)'A') + (unsigned)10;
	}

	if((c >= (uint8_t)'a') && (c <= (uint8_t)'f'))
	{
		ret = (unsigned char)(c - (uint8_t)'a') + (unsigned)10;
	}

	return ret;
}
