/*globalvars.c---------------------------------------------------------------*/
/*
 * globalvars.c
 *
 *  Created on: Nov 3, 2017
 *      Author: frainville
 */

/* Includes ------------------------------------------------------------------*/
/* Include all headers containing the needed typedefs here */
#include "globaldefines.h"
#include "CANTX_FIFO.h"

/*------------------------------------------------------------------ Includes */

__attribute__((__section__(".board_info"))) const unsigned char BOARD_NAME[10] = "APP";

/* Public variables ----------------------------------------------------------*/
/* Declare all global variables here */

/* Global management -------------------*/
uint32_t G_mSCounter=0;
/*------------------- Global management */



/* CAN management ----------------------*/
CANTX_FIFO CanTxList;
/*---------------------- CAN management */

/*---------------------------------------------------------- Public variables */

/*---------------------------------------------------------------globalvars.c*/
