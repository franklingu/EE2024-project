/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"

#include "led7seg.h"
#include "joystick.h"
#include "pca9532.h"
#include "light.h"
#include "acc.h"
#include "oled.h"
#include "temp.h"
#include "rgb.h"

#include <stdio.h>

typedef enum {
    Calibration,
    StandBy,
    Active,
} MachineMode;

static uint32_t msTicks = 0;  //global variable for timing
MachineMode currentMode = Calibration;

static void ssp_init(void)
{
    SSP_CFG_Type SSP_ConfigStruct;
    PINSEL_CFG_Type PinCfg;

    /*
     * Initialize SPI pin connect
     * P0.7 - SCK;
     * P0.8 - MISO
     * P0.9 - MOSI
     * P2.2 - SSEL - used as GPIO
     */
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 7;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 8;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 9;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Funcnum = 0;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 2;
    PINSEL_ConfigPin(&PinCfg);

    SSP_ConfigStructInit(&SSP_ConfigStruct);

    // Initialize SSP peripheral with parameter given in structure above
    SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

    // Enable SSP peripheral
    SSP_Cmd(LPC_SSP1, ENABLE);

}

static void i2c_init(void)
{
    PINSEL_CFG_Type PinCfg;

    /* Initialize I2C2 pin connect */
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);

    // Initialize I2C peripheral
    I2C_Init(LPC_I2C2, 100000);

    /* Enable I2C1 operation */
    I2C_Cmd(LPC_I2C2, ENABLE);
}

void resetBtn_init(void) {
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 4;
    PinCfg.Funcnum = 0;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(0, (1<<4), 0);
}

void calibratedBtn_init(void) {
	PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 31;
    PinCfg.Funcnum = 0;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(1, (1<<31), 0);
}

int resetBtn_read(void) {
	uint32_t state;

    state = GPIO_ReadValue(0);
    return state & (1 << 4);
}

int calibratedBtn_read(void) {
	uint32_t state;

	state = GPIO_ReadValue(1);
	return state & (1 << 31);
}

void SysTick_Handler(void) {
    msTicks++;
}

static uint32_t getTicks(void) {
    return msTicks;
}

int8_t x, y, z;

void doCalibration(){
	oled_clearScreen(OLED_COLOR_BLACK);
	while(calibratedBtn_read() != 0){
		char oledOutput[256];
		acc_read(&x, &y, &z);
		sprintf(oledOutput, "Acc: %d %d %d", x, y, z);
		oled_putString(10, 10, (uint8_t *) "CALIBRATION", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(10, 20, (uint8_t *) oledOutput, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
	currentMode = StandBy;
}


uint8_t numberToCharUint(int number) {
	return (uint8_t)(number + 48);
}

void doStandByMode() {
	oled_clearScreen(OLED_COLOR_BLACK);
	int standByTiming = 5;
	int prevCountingTicks = getTicks();

	oled_putString(0, 0, (uint8_t *)"STANDBY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	led7seg_setChar(numberToCharUint(standByTiming), FALSE);
	while (1) {
	    if (resetBtn_read() == 0) {
	        currentMode = Calibration;
	        break;
	    }

	    // light sensor interrupt code

	    if (standByTiming > 0 && getTicks() - prevCountingTicks > 1000) {
	    	standByTiming--;
	    	led7seg_setChar(numberToCharUint(standByTiming), FALSE);
	    	prevCountingTicks = getTicks();
	    }
	    if (standByTiming == 0) {
	    	uint32_t luminance = light_read();
	    	float temperature = temp_read() / 10.0;
	    	uint8_t risky = (luminance >= 800);
	    	uint8_t hot = (temperature >= 26);

	    	// if conditions met, go to the active mode

	    	if (risky) {
	    		oled_putString(0, 10, (uint8_t *)"RISKY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	    	} else {
	    		oled_putString(0, 10, (uint8_t *)"SAFE ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	    	}
	    	if (hot) {
	    		oled_putString(0, 20, (uint8_t *)"HOT   ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	    	} else {
	    		oled_putString(0, 20, (uint8_t *)"NORMAL", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	    	}
	    }
	}
}

void all_init() {
    i2c_init();
    ssp_init();
    resetBtn_init();
    calibratedBtn_init();

    pca9532_init();
    led7seg_init();
    acc_init();
    oled_init();
    rgb_init();

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    oled_clearScreen(OLED_COLOR_BLACK);

    temp_init(&getTicks);
    SysTick_Config(SystemCoreClock / 1000);
    light_init();
    light_enable();
    light_setRange(LIGHT_RANGE_4000);
}

int main (void) {
	all_init();
    while (1) {
        if (currentMode == Calibration) {
        	doCalibration();
        }

        if (currentMode == StandBy) {
            doStandByMode();
        }

        if (currentMode == Active) {
            if (resetBtn_read() == 0) {
                currentMode = Calibration;
                break;
            }
            // to do for active
        }
    }
}

