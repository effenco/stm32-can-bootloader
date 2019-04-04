/* IO_MGMT.c -----------------------------------------------------------------*/
/*
 * IO_MGMT.c
 *
 *  Created on: 3 nov. 2017
 *      Author: frainville
 */


/* Includes ------------------------------------------------------------------*/

#include "IO_MGMT.h"
#include "main.h"

#include "stm32f3xx_hal.h"

#include "globalvars.h"
#include "globaldefines.h"

#include "CANTX_FIFO.h"
/*------------------------------------------------------------------ Includes */

/* Private defines -----------------------------------------------------------*/

/*----------------------------------------------------------- Private defines */

/* Private macros ------------------------------------------------------------*/

/*------------------------------------------------------------ Private macros */

/* Private variables ---------------------------------------------------------*/

/*--------------------------------------------------------- Private variables */

/* Private function prototypes -----------------------------------------------*/

/*----------------------------------------------- Private function prototypes */

/* Private function definitions ----------------------------------------------*/

/*---------------------------------------------- Private function definitions */

/* Public function definitions -----------------------------------------------*/

void CAN_SEND_1000ms(){

#if 0
	CanTxMsgTypeDef CAN_SEND_MSG;
	uint32_t tempbitstream;
#endif

}

void CAN_SEND_100ms(){

#if 0
	CanTxMsgTypeDef CAN_SEND_MSG;
	uint32_t tempbitstream;
#endif

}

void CAN_SEND_20ms(){
	CanTxMsgTypeDef CAN_SEND_MSG;

	static uint8_t heartbeat=0;

	// Increment Heartbeat
	heartbeat++;
	if (heartbeat >= 255)heartbeat = 0;

	//=========================================================================================
	// This message is designed to keep compatibility with ECU that was programmed to work with the PIC32 version
	CAN_SEND_MSG.header.IDE = CAN_ID_EXT;
	CAN_SEND_MSG.header.ExtId = 0x18FFFF00 + CAN_SA;
	CAN_SEND_MSG.header.RTR = CAN_RTR_DATA;
	CAN_SEND_MSG.header.DLC = 8;

	CAN_SEND_MSG.Data[0]=0x00;
	CAN_SEND_MSG.Data[1]=0x00;
	CAN_SEND_MSG.Data[2]=0x00;
	CAN_SEND_MSG.Data[3]=0x00;
	CAN_SEND_MSG.Data[4]=0x00;
	CAN_SEND_MSG.Data[5]=0x00;
	CAN_SEND_MSG.Data[6]=0x00;
	CAN_SEND_MSG.Data[7]=heartbeat;

	CANTX_FIFO_intellipush(&CanTxList, &CAN_SEND_MSG);

}
/*----------------------------------------------- Public function definitions */

/*----------------------------------------------------------------- IO_MGMT.c */
