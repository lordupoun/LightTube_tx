/*
 * gui.h
 *
 *  Created on: May 10, 2026
 *      Author: Vilem Broucek
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

#include <stdbool.h>
#include "main.h"
#include "i2c_lcd.h"

void gui_drawBase(void);

enum
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
	CHANGE_SAVE,
	SET_TUBES_ASSIGN,
	CHANGE_TUBES_ASSIGN
};
typedef uint8_t Screen_t;


typedef struct ScreenItem //ToDo: predelat na union, usetrilo by FLASH; preskladat dle zarovnani - usetri FLASH
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

//THE REST OF TYPEDEFS IS IN MAIN.H

void gui_buttonUp(void);
void gui_buttonDown(void);
void gui_buttonLeft(void);
void gui_buttonRight(void);
void gui_drawSettings(void);
void gui_init(Mode_t*, Transmit_t*, uint16_t*, I2C_LCD_HandleTypeDef*, uint8_t*);
void gui_setScreen(void);



#endif /* INC_GUI_H_ */
