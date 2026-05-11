/*
 * gui.c
 *
 *  Created on: May 10, 2026
 *      Author: Vilem Broucek
 */

#include "gui.h"
#include "i2c_lcd.h"
#include "flash_storage.h"

extern const ScreenItem_t mainScreen, setModeScreen, setTransmitScreen, setAddressScreen;
extern const ScreenItem_t changeModeScreen, changeTransmitScreen, changeAddressScreen;
extern const ScreenItem_t setPresetsScreen, changePresetsScreen, setSaveScreen, changeSaveScreen, setTubesAssignScreen, changeTubesAssignScreen;

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
static void showTubeNumber(void);
static void nullFunction(void);

static I2C_LCD_HandleTypeDef* lcd1;

static const ScreenItem_t* currentScreen = &mainScreen;

static uint16_t* dmxAddrPtr = 0;
static Mode_t* currentMode; //ToDo: change to currentModePtr
static Transmit_t* currentTransmit; //ToDo: change to currentTransmitPtr

static uint16_t localDMXAddress; //Local address only for GUI, before accepting changes
static Mode_t localCurrentMode;
static Transmit_t localCurrentTransmit;
static bool localSave;
static uint8_t* numOfReceiversPtr;

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
	    .up = &setTubesAssignScreen,
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
	    .down = &setTubesAssignScreen,
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
const ScreenItem_t setTubesAssignScreen =
{
		.name = SET_TUBES_ASSIGN,
		.editable = false,
	    .up = &setSaveScreen,
	    .down = &setModeScreen,
	    .enter = &changeTubesAssignScreen,
	    .prev = &mainScreen,
		.up_action = NULL,
		.down_action = NULL,
		.enter_action = NULL,
		//.value = 20
};

const ScreenItem_t changeTubesAssignScreen =
{
		.name = CHANGE_TUBES_ASSIGN,
		.editable = true,
	    .up = NULL,
	    .down = NULL,
	    .enter = NULL,
	    .prev = &setTubesAssignScreen,
		.up_action = nullFunction,
		.down_action = nullFunction,
		.enter_action = showTubeNumber,
		//.value = 20
};


//Pointer to current DMX512 address, Currently used LCD
void gui_init(Mode_t* mode, Transmit_t* transmit, uint16_t* addr, I2C_LCD_HandleTypeDef* lcd, uint8_t* receiversNum)
{
	currentMode=mode;
	currentTransmit=transmit;
	dmxAddrPtr=addr;
	lcd1=lcd;
	localDMXAddress=*dmxAddrPtr;
	localCurrentMode=*mode;
	localCurrentTransmit=*transmit;
	numOfReceiversPtr=receiversNum;
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
			localCurrentMode=*currentMode;
			lcd_clear(lcd1);
			lcd_gotoxy(lcd1, 0, 0);
			lcd_puts(lcd1, "CHANGE MODE TO:");
			lcd_gotoxy(lcd1, 0, 1);
			if(*currentMode==DMX512)
				lcd_puts(lcd1, "   -DMX512-");
			if(*currentMode==BLUETOOTH)
				lcd_puts(lcd1, "  -BLUETOOTH-");
			break;
		case CHANGE_TRANSMIT:
			localCurrentTransmit=*currentTransmit;
			lcd_clear(lcd1);
			lcd_gotoxy(lcd1, 0, 0);
			lcd_puts(lcd1, "CHANGE TX TO:");
			lcd_gotoxy(lcd1, 0, 1);
			if(*currentTransmit==BROADCAST)
				lcd_puts(lcd1, "  -BROADCAST-");
			if(*currentTransmit==INDIVIDUAL)
				lcd_puts(lcd1, "  -INDIVIDUAL-");
			break;
		case CHANGE_ADDRESS:
			localDMXAddress=*dmxAddrPtr;
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
			lcd_puts(lcd1, "SAVE TO FLASH?");
			break;
		case CHANGE_SAVE:
			lcd_clear(lcd1);
			lcd_gotoxy(lcd1, 0, 0);
			lcd_puts(lcd1, "SAVE TO FLASH:");
			lcd_gotoxy(lcd1, 0, 1);
			lcd_puts(lcd1, "     -NO-");
			break;
		case SET_TUBES_ASSIGN:
			gui_drawSettings();
			lcd_gotoxy(lcd1, 0, 1);
			lcd_puts(lcd1, "IDENTIFY TUBES?");
			break;
		case CHANGE_TUBES_ASSIGN:
			lcd_clear(lcd1);
			lcd_gotoxy(lcd1, 0, 0);
			lcd_puts(lcd1, "UNPLUG DMX!");
			lcd_gotoxy(lcd1, 0, 1);
			lcd_puts(lcd1, " -PRESS ENTER-");
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
	if(localDMXAddress<512-(NUM_CHANNELS*(*numOfReceiversPtr)+NUM_COMMON_CHANNELS))
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
	lcd_gotoxy(lcd1, 0, 1);
	lcd_puts(lcd1, "               ");
	if(localCurrentMode==DMX512)
	{
		localCurrentMode=BLUETOOTH;
		lcd_gotoxy(lcd1, 0, 1);
		lcd_puts(lcd1, "  -BLUETOOTH-");
	}
	else
	{
		localCurrentMode=DMX512;
		lcd_gotoxy(lcd1, 0, 1);
		lcd_puts(lcd1, "   -DMX512-");
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
	lcd_gotoxy(lcd1, 0, 1);
	lcd_puts(lcd1, "               ");
	if(localCurrentTransmit==BROADCAST)
	{
		localCurrentTransmit=INDIVIDUAL;
		lcd_gotoxy(lcd1, 0, 1);
		lcd_puts(lcd1, "  -INDIVIDUAL-");
	}
	else
	{
		localCurrentTransmit=BROADCAST;
		lcd_gotoxy(lcd1, 0, 1);
		lcd_puts(lcd1, "  -BROADCAST-");
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
		lcd_puts(lcd1, "     -YES-");
	}
	else
	{
		localSave=false;
		lcd_gotoxy(lcd1, 0, 1);
		lcd_puts(lcd1, "     -NO- ");
	}
}
void decSaveValue()
{
	incSaveValue();
}
void confirmSaveValue()
{
	if(localSave==true)
	{
		lcd_clear(lcd1);
		lcd_gotoxy(lcd1, 0, 0);
		lcd_puts(lcd1, "SAVING TO FLASH");
		flash_saveSettings(*dmxAddrPtr, *currentMode, *currentTransmit);
		lcd_gotoxy(lcd1, 0, 1);
		lcd_puts(lcd1, "VALUES SAVED!");
		HAL_Delay(2000);
	}
	currentScreen=&setSaveScreen;
	gui_setScreen();
}
void showTubeNumber()
{
	identify_tube_address();
	lcd_clear(lcd1);
	lcd_gotoxy(lcd1, 0, 0);
	lcd_puts(lcd1, "IDENTIFYING");
	lcd_gotoxy(lcd1, 0, 1);
	lcd_puts(lcd1, "");
	HAL_Delay(500);

	currentScreen=&setTubesAssignScreen;
	gui_setScreen();
}

void nullFunction()
{

}
