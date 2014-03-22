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

static const int TicksInOneSecond = 1000;
static const int SensorOperatingTimeInterval = 25;
static const int TemperatureThreshold = 26;
static const int LuminanceThreshold = 800;
static const int UnsafeFrequencyLowerBound = 2;
static const int UnsafeFrequencyUpperBound = 10;
static const int TimeWindow = 3000;

static uint32_t msTicks = 0;
static uint32_t luminance;
int8_t x, y, z;
int8_t x_prev, y_prev, z_prev;
int32_t xoff, yoff, zoff;
MachineMode currentMode = Calibration;
int8_t warningOn = 0;

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

void shouldUpdateXYZ(){
    if (x - x_prev <= 3 && x - x_prev >= -3){
        x = x_prev;
    }
    if (y - y_prev <= 3 && y - y_prev >= -3){
        y = y_prev;
        }
    if (z - z_prev <= 3 && z - z_prev >= -3){
        z = z_prev;
        }
    if (x >= -3 && x <= 3
        && y >= -3 && y <= 3
        && z >= -3 && z <= 3){
        x = y = z = 0;
    }
}

uint8_t numberToCharUint(int number) {
    return (uint8_t)(number + 48);
}

void turnOnWarning() {
    // TODO: turn on warning
    oled_putString(0, 60, (uint8_t *)"WARNING", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    warningOn = 1;
}

void turnOffWarning() {
    // TODO: turn off warning
    oled_putString(0, 60, (uint8_t *)"       ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    warningOn = 0;
}

void accReadSelfImproved() {
    x_prev = x;
    y_prev = y;
    z_prev = z;
    acc_read(&x, &y, &z);
    x = x + xoff;
    y = y + yoff;
    z = z + zoff;
    shouldUpdateXYZ();
}

void doCalibration() {
    GPIO_ClearValue( 2, 0 );
    char oledOutput1[15];
    char oledOutput2[15];
    char oledOutput3[15];
    uint32_t prevCountingTicks = getTicks();

    led7seg_setChar('0', FALSE);
    oled_clearScreen(OLED_COLOR_BLACK);
    oled_putString(0, 0, (uint8_t *) "CALIBRATION", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    while(currentMode == Calibration){
        if (calibratedBtn_read() == 0) {
            currentMode = StandBy;
            break;
        }
        if (getTicks() - prevCountingTicks >= SensorOperatingTimeInterval) {
            accReadSelfImproved();
            sprintf(oledOutput1, "Acc: %d   ", x);
            sprintf(oledOutput2, "     %d   ", y);
            sprintf(oledOutput3, "     %d   ", z);
            oled_putString(0, 10, (uint8_t *)oledOutput1, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(0, 20, (uint8_t *)oledOutput2, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(0, 30, (uint8_t *)oledOutput3, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            prevCountingTicks = getTicks();
        }
    }
}

void doStandByMode() {
    // TODO: clear red, next line not working for now
    GPIO_SetValue( 2, 0);

    oled_clearScreen(OLED_COLOR_BLACK);
    int standByTiming = 5;
    uint32_t prevCountingTicks = getTicks();

    oled_putString(0, 0, (uint8_t *)"STANDBY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    led7seg_setChar(numberToCharUint(standByTiming), FALSE);
    while (currentMode == StandBy) {
        if (standByTiming > 0 && getTicks() - prevCountingTicks >= TicksInOneSecond) {
            // TODO: set up connection to PC
            standByTiming--;
            led7seg_setChar(numberToCharUint(standByTiming), FALSE);
            prevCountingTicks = getTicks();
        }
        if (standByTiming == 0 && getTicks() - prevCountingTicks >= SensorOperatingTimeInterval) {
            float temperature = temp_read() / 10.0;
            uint8_t isRisky = (luminance >= LuminanceThreshold);
            uint8_t isHot = (temperature >= TemperatureThreshold);

            // TODO: if conditions met, go to the active mode

            if (isRisky) {
                oled_putString(0, 10, (uint8_t *)"RISKY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            } else {
                oled_putString(0, 10, (uint8_t *)"SAFE ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            }
            if (isHot) {
                oled_putString(0, 20, (uint8_t *)"HOT   ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            } else {
                oled_putString(0, 20, (uint8_t *)"NORMAL", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            }
            prevCountingTicks = getTicks();
        }
    }
}

void doActiveMode() {
    uint32_t prevCountingTicks = getTicks();
    uint32_t prevPCTimingTicks = getTicks();
    uint32_t prevTimingForWarningOn = getTicks();
    uint32_t prevTimingForWarningOff = getTicks();
    uint32_t prevTimingForZAxisRecorded = getTicks();
    uint8_t countForFrequency = 0;
    int8_t prevZAxisIsNonNegative = (z >= 0);
    int8_t isTimingForWarningOn = 0;
    float frequency;

    oled_clearScreen(OLED_COLOR_BLACK);
    oled_putString(0, 0, (uint8_t *)"ACTIVE", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    while (currentMode == Active) {
        if (getTicks() - prevCountingTicks >= SensorOperatingTimeInterval) {
            float temperature = temp_read() / 10.0;
            uint8_t isRisky = (luminance >= LuminanceThreshold);
            uint8_t isHot = (temperature >= TemperatureThreshold);
            if (isRisky || isHot) {
                currentMode = StandBy;
                break;
            }
            if (isRisky) {
                 oled_putString(0, 10, (uint8_t *)"RISKY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            } else {
                 oled_putString(0, 10, (uint8_t *)"SAFE ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            }
            if (isHot) {
                 oled_putString(0, 20, (uint8_t *)"HOT   ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            } else {
                 oled_putString(0, 20, (uint8_t *)"NORMAL", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            }
            accReadSelfImproved();
            if ((prevZAxisIsNonNegative && z < 0) || (!prevZAxisIsNonNegative && z >= 0)) {
                prevZAxisIsNonNegative = !prevZAxisIsNonNegative;
                countForFrequency ++;
                if (countForFrequency % 2 == 0) {
                    frequency = (float)TicksInOneSecond / (getTicks() - prevTimingForZAxisRecorded);
                    if (frequency < UnsafeFrequencyLowerBound || frequency > UnsafeFrequencyUpperBound) {
                        isTimingForWarningOn = 0;
                        prevTimingForWarningOff = getTicks();
                    } else if (frequency >= UnsafeFrequencyLowerBound && frequency <= UnsafeFrequencyUpperBound) {
                        isTimingForWarningOn = 1;
                        prevTimingForWarningOn = getTicks();
                    } else if (!warningOn &&
                            isTimingForWarningOn && (getTicks() - prevTimingForWarningOn > TimeWindow)) {
                        turnOnWarning();
                        isTimingForWarningOn = 0;
                        prevTimingForWarningOff = getTicks();
                        prevTimingForWarningOn = getTicks();
                    } else if (warningOn &&
                            !isTimingForWarningOn && (getTicks() - prevTimingForWarningOff > TimeWindow)) {
                        turnOnWarning();
                        isTimingForWarningOn = 1;
                        prevTimingForWarningOn = getTicks();
                        prevTimingForWarningOff = getTicks();
                    }
                }
            }
            prevCountingTicks = getTicks();
        }
        if (getTicks() - prevPCTimingTicks >= TicksInOneSecond) {
            // TODO: report to PC using UART
            prevPCTimingTicks = getTicks();
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
    //rgb_init();
    GPIO_SetDir( 2, 0, 0 );

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    oled_clearScreen(OLED_COLOR_BLACK);

    temp_init(&getTicks);
    SysTick_Config(SystemCoreClock / TicksInOneSecond);

    //light interrupt
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 0;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 5;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(2, (1 << 5), 0);
    light_init();

    light_enable();
    light_setRange(LIGHT_RANGE_4000);


    light_setHiThreshold(150);
    light_setLoThreshold(50);
    light_setIrqInCycles(LIGHT_CYCLE_8);
    light_clearIrqStatus();


    luminance = light_read();

    // Enable GPIO Interrupt P2.5 for light sensor
    LPC_GPIOINT->IO2IntEnF |= 1 << 5;
    // Enable GPIO Interrupt P0.4 for SW3 (reset button)
    LPC_GPIOINT->IO0IntEnF |= 1 << 4;
    NVIC_EnableIRQ(EINT3_IRQn);

    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 0-z;
}

void EINT3_IRQHandler(void){
    // SW3
    if ((LPC_GPIOINT->IO0IntStatF >> 4) & 0x1){
        LPC_GPIOINT->IO0IntClr |= 1 << 4;
        currentMode = Calibration;
    }

    // light
    if ((LPC_GPIOINT->IO2IntStatF >> 5)& 0x1){
        LPC_GPIOINT->IO2IntClr |= 1 << 5;
        light_clearIrqStatus();
        luminance = light_read();
    }
}

int main (void) {
    all_init();
    currentMode = Active;
    while (1) {
        if (currentMode == Calibration) {
            doCalibration();
        }

        if (currentMode == StandBy) {
            doStandByMode();
        }

        if (currentMode == Active) {
            doActiveMode();
        }
    }
}
