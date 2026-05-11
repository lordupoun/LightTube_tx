/*
 * gui.h
 *
 *  Created on: May 10, 2026
 *      Author: Vilem Broucek
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include <stdbool.h>
#include "stm32f1xx_hal.h"

void gui_drawBase(void);

typedef enum
{
    MAIN,
    SET_MODE,
	SET_TRANSMIT,
    SET_ADDRESS,
	CHANGE_MODE,
	CHANGE_TRANSMIT,
	CHANGE_ADDRESS,
	SET_PRESETS,
	CHANGE_PRESETS,
	SET_SAVE,
	CHANGE_SAVE

} Screen_t;

typedef struct ScreenItem //ToDo: predelat na union, usetrilo by FLASH
{
	const Screen_t name;
	const bool editable;
    const struct ScreenItem* up;
    const struct ScreenItem* down;
    const struct ScreenItem* enter;
    const struct ScreenItem* prev;
    const void (*up_action)();
    const void (*down_action)();
    const void (*enter_action)();

} ScreenItem_t;

typedef enum {
    BROADCAST,
	INDIVIDUAL
} Transmit_t;

typedef enum {
    DMX512,
    BLUETOOTH
} Mode_t;



#endif /* INC_GUI_H_ */
