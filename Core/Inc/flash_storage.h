/*
 * flash_storage.h
 *
 *  Created on: May 11, 2026
 *      Author: Vilem Broucek
 */

#ifndef INC_FLASH_STORAGE_H_
#define INC_FLASH_STORAGE_H_

#include "main.h"

void flash_loadSettings(uint16_t*, Mode_t*, Transmit_t*);
void flash_saveSettings(uint16_t, Mode_t, Transmit_t);



#endif /* INC_FLASH_STORAGE_H_ */
