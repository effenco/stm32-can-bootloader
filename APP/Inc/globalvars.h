/*globalvars.h---------------------------------------------------------------*/
/*
 * globalvars.h
 *
 *  Created on: Nov 3, 2017
 *      Author: frainville
 */

#ifndef INC_GLOBALVARS_H_
#define INC_GLOBALVARS_H_

/* Includes ------------------------------------------------------------------*/
/* Include all headers containing the needed typedefs here */
#include "globaldefines.h"
#include "CANTX_FIFO.h"

/*------------------------------------------------------------------ Includes */

/* Public variables ----------------------------------------------------------*/
/* Define all extern global variables here
 * Do not initialize value here */

/* Global management -------------------*/
extern uint32_t G_mSCounter;
/*------------------- Global management */


/* CAN management ----------------------*/
extern CANTX_FIFO CanTxList;
/*---------------------- CAN management */

/*---------------------------------------------------------- Public variables */


#endif /* INC_GLOBALVARS_H_ */
/*---------------------------------------------------------------globalvars.h*/
