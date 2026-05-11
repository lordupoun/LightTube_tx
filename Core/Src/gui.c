/*
 * gui.c
 *
 *  Created on: May 10, 2026
 *      Author: Vilem Broucek
 */

#include "gui.h"
#include "i2c_lcd.h"

extern const ScreenItem_t mainScreen, setModeScreen, setTransmitScreen, setAddressScreen;
extern const ScreenItem_t changeModeScreen, changeTransmitScreen, changeAddressScreen;
extern const ScreenItem_t setPresetsScreen, changePresetsScreen, setSaveScreen, changeSaveScreen;

static void incModeValue(void);
static void decModeValue(void);
static void confirmModeValue(void);
static void incTransmitValue(void);
static void decTransmitValue(void);
static void confirmTransmitValue(void);
static void incAddrValue(void);
static void decAddrValue(void);
static void confirmAddrValue(void);
//static void incPresetsValue(void);
//static void decPresetsValue(void);
//static void confirmPresetsValue(void);
static void incSaveValue(void);
static void decSaveValue(void);
static void confirmSaveValue(void);

static I2C_LCD_HandleTypeDef* lcd1;

static const ScreenItem_t* currentScreen = &mainScreen;

static uint16_t* dmxAddrPtr = 0;
static Mode_t* currentMode;
static Transmit_t* currentTransmit;

static uint16_t localDMXAddress = 20; //Local address only for GUI, before accepting changes
static Mode_t localCurrentMode;
static Transmit_t localCurrentTransmit;
static bool localSave;

//DEFINES BEHAVIOR OF BUTTONS ON EACH SCREEN
const ScreenItem_t mainScreen =
{
		.name = MAIN,
		.editable = false,
	    .up = NULL,
	    .down = NULL,
	    .enter = &setModeScreen,
	    .prev = NULL
};

const ScreenItem_t setModeScreen =
{
		.name = SET_MODE,
		.editable = false,
	    .up = &setSaveScreen,
	    .down = &setTransmitScreen,
	    .enter = &changeModeScreen,
	    .prev = &mainScreen
};

const ScreenItem_t setTransmitScreen =
{
		.name = SET_TRANSMIT,
		.editable = false,
	    .up = &setModeScreen,
	    .down = &setAddressScreen,
	    .enter = &changeTransmitScreen,
	    .prev = &mainScreen
};

const ScreenItem_t setAddressScreen =
{
		.name = SET_ADDRESS,
		.editable = false,
	    .up = &setTransmitScreen,
	    .down = &setPresetsScreen,
	    .enter = &changeAddressScreen,
	    .prev = &mainScreen
};

const ScreenItem_t changeModeScreen =
{
		.name = CHANGE_MODE,
		.editable = true,
	    .up = NULL, //Union by umoznil volani stejnym prikazem
	    .down = NULL,
	    .enter = NULL,
	    .prev = &setModeScreen,
		.up_action = incModeValue,
		.down_action = decModeValue,
		.enter_action = confirmModeValue,
};

const ScreenItem_t changeTransmitScreen =
{
		.name = CHANGE_TRANSMIT,
		.editable = true,
	    .up = NULL,
	    .down = NULL,
	    .enter = NULL,
	    .prev = &setTransmitScreen,
		.up_action = incTransmitValue,
		.down_action = decTransmitValue,
		.enter_action = confirmTransmitValue,
};
const ScreenItem_t changeAddressScreen =
{
		.name = CHANGE_ADDRESS,
		.editable = true,
	    .up = NULL,
	    .down = NULL,
	    .enter = NULL,
	    .prev = &setAddressScreen,
		.up_action = incAddrValue,
		.down_action = decAddrValue,
		.enter_action = confirmAddrValue,
		//.value = 20
};

const ScreenItem_t setPresetsScreen =
{
		.name = SET_PRESETS,
		.editable = false,
	    .up = &setAddressScreen,
	    .down = &setSaveScreen,
	    //.enter = &changePresetsScreen,
		.enter = NULL,
	    .prev = &mainScreen
};

const ScreenItem_t changePresetsScreen =
{
		.name = CHANGE_PRESETS,
		.editable = true,
	    .up = NULL,
	    .down = NULL,
	    .enter = NULL,
	    .prev = &setPresetsScreen,
		//.up_action = incPresetsValue,
		//.down_action = decPresetsValue,
		//.enter_action = confirmPresetsValue,
};

const ScreenItem_t setSaveScreen =
{
		.name = SET_SAVE,
		.editable = false,
	    .up = &setPresetsScreen,
	    .down = &setModeScreen,
	    .enter = &changeSaveScreen,
	    .prev = &mainScreen
};

const ScreenItem_t changeSaveScreen =
{
		.name = CHANGE_SAVE,
		.editable = true,
	    .up = NULL,
	    .down = NULL,
	    .enter = NULL,
	    .prev = &setSaveScreen,
		.up_action = incSaveValue,
		.down_action = decSaveValue,
		.enter_action = confirmSaveValue,
		//.value = 20
};


//Pointer to current DMX512 address, Currently used LCD
void gui_init(Mode_t* mode, Transmit_t* transmit,uint16_t* addr, I2C_LCD_HandleTypeDef* lcd)
{
	currentMode=mode;
	currentTransmit=transmit;
	dmxAddrPtr=addr;
	lcd1=lcd;
	localDMXAddress=*dmxAddrPtr;
	localCurrentMode=*mode;
	localCurrentTransmit=*transmit;
}

//Defines GRAPHICS and TEXTS for EACH SCREEN
void gui_setScreen()
{
	char text[17];
	switch(currentScreen->name)
	{
		case MAIN:
			gui_drawBase();
			lcd_gotoxy(lcd1, 0, 1);
			if(*currentMode==DMX512)
				lcd_puts(lcd1, "Mode: DMX512");
			if(*currentMode==BLUETOOTH)
				lcd_puts(lcd1, "Mode: BLUETOOTH");
			break;
		case SET_MODE:
			gui_drawSettings();
			lcd_gotoxy(lcd1, 0, 1);
			if(*currentMode==DMX512)
				lcd_puts(lcd1, "MODE: DMX512");
			if(*currentMode==BLUETOOTH)
				lcd_puts(lcd1, "MODE: BLUETOOTH");
			break;
		case SET_TRANSMIT:
			gui_drawSettings();
			lcd_gotoxy(lcd1, 0, 1);
			if(*currentTransmit==BROADCAST)
				lcd_puts(lcd1, "TX: BROADCAST");
			if(*currentTransmit==INDIVIDUAL)
				lcd_puts(lcd1, "TX: INDIVIDUAL");
			break;
		case SET_ADDRESS:
			gui_drawSettings();
			lcd_gotoxy(lcd1, 0, 1);
			snprintf(text, sizeof(text), "ADDRESS: %u", *dmxAddrPtr);
			lcd_puts(lcd1, text);
			break;
		case CHANGE_MODE:
			lcd_clear(lcd1);
			lcd_gotoxy(lcd1, 0, 0);
			lcd_puts(lcd1, "CHANGE MODE TO");
			lcd_gotoxy(lcd1, 0, 1);
			if(*currentMode==DMX512)
				lcd_puts(lcd1, "MODE: DMX512");
			if(*currentMode==BLUETOOTH)
				lcd_puts(lcd1, "MODE: BLUETOOTH");
			break;
		case CHANGE_TRANSMIT:
			lcd_clear(lcd1);
			lcd_gotoxy(lcd1, 0, 0);
			lcd_puts(lcd1, "CHANGE TX TO");
			lcd_gotoxy(lcd1, 0, 1);
			if(*currentTransmit==BROADCAST)
				lcd_puts(lcd1, "TX: BROADCAST");
			if(*currentTransmit==INDIVIDUAL)
				lcd_puts(lcd1, "TX: INDIVIDUAL");
			break;
		case CHANGE_ADDRESS:
			lcd_clear(lcd1);
			lcd_gotoxy(lcd1, 0, 0);
			lcd_puts(lcd1, "CHANGE ADDRESS");
			lcd_gotoxy(lcd1, 0, 1);
			snprintf(text, sizeof(text), "ADDRESS: %u", *dmxAddrPtr);
			lcd_puts(lcd1, text);
			break;
		case SET_PRESETS:
			gui_drawSettings();
			lcd_gotoxy(lcd1, 0, 1);
			lcd_puts(lcd1, "PRESET CH: WIP");
			/*if(*currentPresets==true)
				lcd_puts(lcd1, "PRESET CH: YES");
			if(*currentPresets==false)
				lcd_puts(lcd1, "PRESETS CH: NO");*/
			break;
		case CHANGE_PRESETS:
			gui_drawSettings();
			lcd_gotoxy(lcd1, 0, 1);
			lcd_puts(lcd1, "NOT AVAILABLE");
			break;
		case SET_SAVE:
			localSave=false;
			gui_drawSettings();
			lcd_gotoxy(lcd1, 0, 1);
			lcd_puts(lcd1, "SAVE TO EEPROM?");
			break;
		case CHANGE_SAVE:
			lcd_clear(lcd1);
			lcd_gotoxy(lcd1, 0, 0);
			lcd_puts(lcd1, "SAVE TO EEPROM:");
			lcd_gotoxy(lcd1, 0, 1);
			lcd_puts(lcd1, "NO");
			break;

	}
}

void gui_buttonUp()
{
	if(currentScreen->editable==true)
	{
		currentScreen->up_action();
	}
	else if(currentScreen->up != NULL)
	{
		currentScreen = currentScreen->up;
		gui_setScreen();
	}
}
void gui_buttonDown()
{
	if(currentScreen->editable==true)
	{
		currentScreen->down_action();
	}
	else if(currentScreen->down != NULL)
	{
		currentScreen = currentScreen->down;
		gui_setScreen();
	}
}
void gui_buttonLeft()
{
	if(currentScreen->prev != NULL)
	{
		currentScreen = currentScreen->prev;
		gui_setScreen();
	}
}
void gui_buttonRight()
{
	if(currentScreen->editable==true)
	{
		currentScreen->enter_action();
	}
	else if(currentScreen->enter != NULL)
	{
		currentScreen = currentScreen->enter;
		gui_setScreen();
	}
}


void gui_drawBase()
{
	lcd_clear(lcd1);
	lcd_gotoxy(lcd1, 0, 0);
	lcd_puts(lcd1, "LightTube");
}

void gui_drawSettings()
{
	lcd_clear(lcd1);
	lcd_gotoxy(lcd1, 0, 0);
	lcd_puts(lcd1, "-SETTINGS-");
}

void incAddrValue()
{
	char text[6];
	lcd_gotoxy(lcd1, 9, 1);
	lcd_puts(lcd1, "     ");
	if(localDMXAddress<512)
	{
		localDMXAddress+=1;
	}
	snprintf(text, sizeof(text), "%u", localDMXAddress);
	lcd_gotoxy(lcd1, 9, 1);
	lcd_puts(lcd1, text);
}
void decAddrValue()
{
	char text[6];
	lcd_gotoxy(lcd1, 10, 1);
	lcd_puts(lcd1, "    ");
	if(localDMXAddress>1)
	{
		localDMXAddress-=1;
	}
	snprintf(text, sizeof(text), "%u", localDMXAddress);
	lcd_gotoxy(lcd1, 9, 1);
	lcd_puts(lcd1, text);
}
void confirmAddrValue()
{
	*dmxAddrPtr=localDMXAddress;
	currentScreen=&setAddressScreen;
	gui_setScreen();
}

void incModeValue()
{
	lcd_gotoxy(lcd1, 6, 1);
	lcd_puts(lcd1, "         ");
	if(localCurrentMode==DMX512)
	{
		localCurrentMode=BLUETOOTH;
		lcd_gotoxy(lcd1, 6, 1);
		lcd_puts(lcd1, "BLUETOOTH");
	}
	else
	{
		localCurrentMode=DMX512;
		lcd_gotoxy(lcd1, 6, 1);
		lcd_puts(lcd1, "DMX512");
	}
}
void decModeValue()
{
	incModeValue();
}
void confirmModeValue()
{
	if(localCurrentMode==DMX512) //ToDo: change this, in the context of main and gui code it is not logical
	{
		dmx_activate(); //ToDo: Change to a pointer to function, can be initialized in init
	}
	else
	{
		bluetooth_activate();
	}
	//*currentMode=localCurrentMode;
	currentScreen=&setModeScreen;
	gui_setScreen();
}

void incTransmitValue()
{
	lcd_gotoxy(lcd1, 4, 1);
	lcd_puts(lcd1, "         ");
	if(localCurrentTransmit==BROADCAST)
	{
		localCurrentTransmit=INDIVIDUAL;
		lcd_gotoxy(lcd1, 4, 1);
		lcd_puts(lcd1, "INDIVIDUAL");
	}
	else
	{
		localCurrentTransmit=BROADCAST;
		lcd_gotoxy(lcd1, 4, 1);
		lcd_puts(lcd1, "BROADCAST");
	}
}
void decTransmitValue()
{
	incTransmitValue();
}
void confirmTransmitValue()
{
	if(localCurrentTransmit==BROADCAST) //ToDo: change this, in the context of main and gui code it is not logical
	{
		broadcast_mode(); //ToDo: Change to a pointer to function, can be initialized in init
	}
	else
	{
		individual_mode();
	}
	//*currentTransmit=localCurrentTransmit;
	currentScreen=&setTransmitScreen;
	gui_setScreen();
}

void incSaveValue()
{
	lcd_gotoxy(lcd1, 0, 1);
	lcd_puts(lcd1, "         ");
	if(localSave==false)
	{
		localSave=true;
		lcd_gotoxy(lcd1, 0, 1);
		lcd_puts(lcd1, "YES");
	}
	else
	{
		localSave=false;
		lcd_gotoxy(lcd1, 0, 1);
		lcd_puts(lcd1, "NO");
	}
}
void decSaveValue()
{
	incSaveValue();
}
void confirmSaveValue()
{
	lcd_clear(lcd1);
	lcd_gotoxy(lcd1, 0, 0);
	lcd_puts(lcd1, "SAVING TO EEPROM");
	lcd_gotoxy(lcd1, 0, 1);
	lcd_puts(lcd1, "VALUES SAVED!");
	currentScreen=&setSaveScreen;
	HAL_Delay(500);
	gui_setScreen();
}
