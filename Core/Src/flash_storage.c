/*
 * flash_storage.c
 *
 *  Created on: May 11, 2026
 *      Author: Vilem Broucek
 */
#define FLASH_STORAGE_ADDR 0x0801FC00
#include "gui.h"

//CODE for EEPROM EMULATION IN FLASH WAS DESIGNED BY AI
void flash_saveSettings(uint16_t addr, Mode_t mode, Transmit_t trans) //ToDo: ADD RESERVATION TO LINKER
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = FLASH_STORAGE_ADDR;
    eraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return;
    }

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + 0, (uint32_t)addr);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + 4, (uint32_t)mode);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + 8, (uint32_t)trans);

    HAL_FLASH_Lock();
}

void flash_loadSettings(uint16_t* addr, Mode_t* mode, Transmit_t* transmit)
{
    uint32_t rawAddr = *(__IO uint32_t*)FLASH_STORAGE_ADDR;
    uint32_t rawMode = *(__IO uint32_t*)(FLASH_STORAGE_ADDR + 4);
    uint32_t rawTrans = *(__IO uint32_t*)(FLASH_STORAGE_ADDR + 8);

    if (rawAddr != 0xFFFFFFFF)
    {
        *addr = (uint16_t)rawAddr;
        *mode = (Mode_t)rawMode;
        *transmit = (Transmit_t)rawTrans;
    }
    else
    {
    	//DEFAULT VALUES AFTER FLASH
        *addr = 20;
        *mode = DMX512;
        *transmit = BROADCAST;
    }
}

/*void flash_loadSettings_checkEmpty(uint16_t* addr, Mode_t* mode, Transmit_t* trans)
{
    uint32_t rawData = *(__IO uint32_t*)FLASH_STORAGE_ADDR;

    if (rawData == 0xFFFFFFFF)
    {
        *addr = 20;
        *mode = DMX512;
        *trans = BROADCAST;

        //flash_saveSettings(*addr, *mode, *trans);
    }
    else
    {
        *addr = (uint16_t)(*(__IO uint32_t*)(FLASH_STORAGE_ADDR + 0));
        *mode = (Mode_t)(*(__IO uint32_t*)(FLASH_STORAGE_ADDR + 4));
        *trans = (Transmit_t)(*(__IO uint32_t*)(FLASH_STORAGE_ADDR + 8));
    }
}*/


