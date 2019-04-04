/**
  ******************************************************************************
  * STM32F373 Bootloader
  ******************************************************************************
  * @authors Akos Pasztor, Francois Rainville
  * @initial file   bootloader.c
  * @file 	btld.c
  * @brief  Bootloader implementation
  *	        This file contains the functions of the bootloader. The bootloader 
  *	        implementation uses the official HAL library of ST.
  * @see    Please refer to README for detailed information.
  ******************************************************************************
  * Copyright (c) 2018 Akos Pasztor.                    https://akospasztor.com
  * Copyright (c) 2018 François Rainville.
  ******************************************************************************
**/

#include <btld.h>
#include "stm32f3xx_hal.h"

extern CRC_HandleTypeDef hcrc;


/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);

/* Private variables ---------------------------------------------------------*/
static uint32_t flash_ptr = APP_ADDRESS;

/* Initialize bootloader and flash -------------------------------------------*/
void Bootloader_Init(void){
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    //__HAL_RCC_FLASH_CLK_ENABLE();

    /* Clear flash flags */
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    HAL_FLASH_Lock();
}

/* Erase flash ---------------------------------------------------------------*/
uint8_t btld_EraseFlash(void){
    uint32_t NbrOfPages = 0;
    uint32_t PageError  = 0;
    FLASH_EraseInitTypeDef  pEraseInit;
    HAL_StatusTypeDef       status = HAL_OK;

    HAL_FLASH_Unlock();

    /* Get the number of pages to erase */
    NbrOfPages = (FLASH_BASE + FLASH_SIZE - APP_ADDRESS) / FLASH_PAGE_SIZE;

    if(status == HAL_OK)
    {
        //pEraseInit.Banks = FLASH_BANK_2;
        pEraseInit.NbPages = NbrOfPages;
        //pEraseInit.Page = FLASH_PAGE_NBPERBANK - pEraseInit.NbPages;
        pEraseInit.PageAddress = APP_ADDRESS; //FLASH_PAGE_NBPERBANK - pEraseInit.NbPages;
        pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
    }

    HAL_FLASH_Lock();
    
    return (status == HAL_OK) ? BL_OK : BL_ERASE_ERROR;
}

/* Flash Begin ---------------------------------------------------------------*/
void btld_FlashBegin(void){
    /* Reset flash destination address */
    flash_ptr = APP_ADDRESS;
    
    /* Unlock flash */
    HAL_FLASH_Unlock();
}

/* Program 32bit data into flash ---------------------------------------------*/
uint8_t btld_FlashNext_32(uint32_t data,uint32_t* index){

    if( !(flash_ptr <= (END_ADDRESS - 3)) || (flash_ptr < APP_ADDRESS) )
    {
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }

    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_ptr,(uint64_t) data) == HAL_OK)
    {
        /* Check the written value */
        if(*(uint32_t*)flash_ptr != data)
        {
            /* Flash content doesn't match source content */
            HAL_FLASH_Lock();
            return BL_WRITE_ERROR;
        }
        /* Increment Flash destination address */
        flash_ptr += 4;
        *index=(flash_ptr-APP_ADDRESS);
    }
    else
    {
        /* Error occurred while writing data into Flash */
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }

    return BL_OK;
}

/* Program 64bit data into flash ---------------------------------------------*/
uint8_t btld_FlashNext(uint64_t data){
    if( !(flash_ptr <= (FLASH_BASE + FLASH_SIZE - 8)) || (flash_ptr < APP_ADDRESS) )
    {
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }
    
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_ptr, data) == HAL_OK)      
    {
        /* Check the written value */
        if(*(uint64_t*)flash_ptr != data)
        {
            /* Flash content doesn't match source content */
            HAL_FLASH_Lock();
            return BL_WRITE_ERROR;
        }   
        /* Increment Flash destination address */
        flash_ptr += 8;
    }
    else
    {
        /* Error occurred while writing data into Flash */
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }
    
    return BL_OK;
}

/* Finish flash programming --------------------------------------------------*/
void btld_FlashEnd(void){
    /* Lock flash */
    HAL_FLASH_Lock();
}

/* Configure flash write protection ------------------------------------------------*/
uint8_t btld_ConfigProtection(uint32_t protection){
    FLASH_OBProgramInitTypeDef  OBStruct = {0};
    HAL_StatusTypeDef           status = HAL_ERROR;
    
    status = HAL_FLASH_Unlock();
    status |= HAL_FLASH_OB_Unlock();

    /* Bank 1 */

    OBStruct.OptionType = OPTIONBYTE_RDP;

    if(protection & BL_PROTECTION_RDP)
    {
    	/* Enable RDP protection */
    	OBStruct.RDPLevel = OB_RDP_LEVEL_1;


    }
    else
    {
        /* Remove RDP protection */
    	OBStruct.RDPLevel = OB_RDP_LEVEL_0;

    }
    status |= HAL_FLASHEx_OBProgram(&OBStruct);



    if(status == HAL_OK)
    {
        /* Loading Flash Option Bytes - this generates a system reset. */ 
        status |= HAL_FLASH_OB_Launch();
    }
    
    status |= HAL_FLASH_OB_Lock();
    status |= HAL_FLASH_Lock();

    return (status == HAL_OK) ? BL_OK : BL_OBP_ERROR;
}

/* Check if application fits into user flash ---------------------------------------*/
uint8_t btld_CheckSize(uint32_t appsize){
    return ((FLASH_BASE + FLASH_SIZE - APP_ADDRESS) >= appsize) ? BL_OK : BL_SIZE_ERROR;
}

/* Verify checksum of bootloader ---------------------------------*/
uint32_t btld_GetBootChecksum(void){

    uint32_t calculatedCrc = 0;

    calculatedCrc = HAL_CRC_Calculate(&hcrc, (uint32_t*)BL_ADDRESS, BL_SIZE);

    return calculatedCrc;

}

/* Verify checksum of application located in flash ---------------------------------*/
uint32_t btld_GetChecksum(void){

    uint32_t calculatedCrc = 0;

    calculatedCrc = HAL_CRC_Calculate(&hcrc, (uint32_t*)APP_ADDRESS, APP_SIZE);

    return calculatedCrc;
}

/* Save checksum to flash ----------------------------------------------------------*/
uint8_t btld_SaveChecksum(void){
	uint32_t calculatedCrc = 0;
	HAL_StatusTypeDef returnedERR=HAL_OK;

	calculatedCrc=btld_GetChecksum();

	HAL_FLASH_Unlock();

	returnedERR=HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,CRC_ADDRESS, (uint64_t)calculatedCrc);

	HAL_FLASH_Lock();

	return returnedERR;
}

/* Check for application in user flash ---------------------------------------------*/
uint8_t btld_CheckForApplication(void){

    return ( ((*(__IO uint32_t*)APP_ADDRESS) - RAM_SIZE) == 0x20000000 ) ? BL_OK : BL_NO_APP;
}


/* Jump to application -------------------------------------------------------------*/
void btld_JumpToApp(void){
    uint32_t  JumpAddress = *(__IO uint32_t*)(APP_ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;
    

    HAL_RCC_DeInit();
    HAL_DeInit();
    
    //HAL_NVIC_DisableIRQ();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    
#if (SET_VECTOR_TABLE)
    SCB->VTOR = APP_ADDRESS;
#endif
    
    __set_MSP(*(__IO uint32_t*)APP_ADDRESS);
    Jump();
}

/* Jump to System Memory (ST Bootloader) -------------------------------------------*/
void btld_JumpToSysMem(void){
    uint32_t  JumpAddress = *(__IO uint32_t*)(SYSMEM_ADDRESS + 4);
    pFunction Jump = (pFunction)JumpAddress;
    
    HAL_RCC_DeInit();
    HAL_DeInit();
    
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    
    __set_MSP(*(__IO uint32_t*)SYSMEM_ADDRESS);
    Jump();
    
    while(1);
}
