/*****************************************************************************
 *
 *   EE2024 Assignment 2 Program
 *
 *   Monday Lab Group
 *   Gu Junchao (A0105750N)
 *   XIE KAI (A0102016E)
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"
#include "led7seg.h"
#include "joystick.h"
#include "pca9532.h"
#include "light.h"
#include "acc.h"
#include "oled.h"
#include "temp.h"
#include "rgb.h"
#include "rotary.h"

#include <stdio.h>
#include <string.h>

#define UART_PORT   (LPC_UART_TypeDef *)LPC_UART3   // Select UART3
uint8_t rev_buf[255];                               // Reception buffer
uint32_t rev_cnt = 0;                               // Reception counter

uint32_t isReceived = 0;                            // Init to be not received

typedef enum {
    Calibration,
    StandBy,
    Active,
} MachineMode;

static const int TicksInOneSecond = 1000;
static const int SensorOperatingTimeInterval = 20;
static const int TemperatureThreshold = 26;
static const int LuminanceThreshold = 800;
static const int UnsafeFrequencyLowestBound = 0;
static const int UnsafeFrequencyHighestBound = 25;
static const int TimeWindowHighestBound = 10000;
static const int ReportingPeriodLowestBound = 500;
static const int StepForTimeAdjustment = 100;

MachineMode currentMode = Calibration;
int unsafeFrequencyLowerBound = 2;
int unsafeFrequencyHigherBound = 10;
uint32_t msTicks = 0;
uint32_t luminance;
int timeWindow = 3000;
int reportingPeriod = 1000;
int8_t x, y, z;
int8_t x_prev, y_prev, z_prev;
int32_t xoff, yoff, zoff;
int8_t warningOn = 0;
int8_t isBuzzerSet = 0;

int recentValCounter = 0;
int8_t recentX[5] = {0,0,0,0,0};
int8_t recentY[5] = {0,0,0,0,0};
int8_t recentZ[5] = {0,0,0,0,0};

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

//SW3 Reset Button will shift FFS back to Calibration mode
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

//SW4 Calibration Button will move FFS to StandBy mode,
//if current mode is Calibration mode
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

// UART Receive Callback Function
// It will be called when a message is received
void UART_IntReceive(void)
{
    /* Read the received data */
    if(UART_Receive(UART_PORT, &rev_buf[rev_cnt], 1, NONE_BLOCKING) == 1) {
        if(rev_buf[rev_cnt] == '\r'){
            isReceived = 1;
        }
        rev_cnt++;
        if(rev_cnt == 255) rev_cnt = 0;
    }
}

//set up UART interrupt
void UART_Receive_Int_Init()
{
    // UART Config
    UART_CFG_Type UARTConfigStruct;
    // UART FIFO config
    UART_FIFO_CFG_Type UARTFIFOConfigStruct;
    // Pin config
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
    /* Init UART Config to default state:
     * Baudrate = 9600bps 8N1 */
    UART_ConfigStructInit(&UARTConfigStruct);
    /* Set Baudrate to 115200 */
    UARTConfigStruct.Baud_rate = 115200;
    // Init UART3
    UART_Init(UART_PORT, &UARTConfigStruct);
    //--------------------------------------

    /* Init FIFOConfig to default state,
     * using FIFO will allow LPC to have more time to handle interrupts,
     * and also prevent data loss at high rate
     */
    UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
    // Init FIFO for UART3
    UART_FIFOConfig(UART_PORT, &UARTFIFOConfigStruct);
    //------------------------------------------------

    // Setup callback for Receive Message
    UART_SetupCbs(UART_PORT, 0, (void *)UART_IntReceive);
    // Enable UART Transmit
    UART_TxCmd(UART_PORT, ENABLE);
    /* Enable UART Rx interrupt */
    UART_IntConfig(UART_PORT, UART_INTCFG_RBR, ENABLE);
    /* Enable Interrupt for UART3 */
    NVIC_EnableIRQ(UART3_IRQn);
}

//UART3 interrupt handler
void UART3_IRQHandler(void)
{
    UART3_StdIntHandler();
}

//UART Sent, which includes message pre-processing that
//change '\0' to '\r\n'
void UART_Send_Message(char* msg){
    //handling message, eg \0 to \r\n
    int len = strlen(msg);
    msg[len] = '\r';// \0 -> \r
    msg[++len] = '\n';// append \n behind
    UART_Send(UART_PORT, (uint8_t*)msg, (uint32_t)len, BLOCKING);
}

//UART Receive Post-processing that
//change '\r' to '\0'
void UART_RcvMsgHandling(){
    rev_buf[rev_cnt-1] = '\0';
    rev_cnt = 0;
}

//unused, since right now we use interrupt to detect SW3
int resetBtn_read(void) {
    uint32_t state;

    state = GPIO_ReadValue(0);
    return state & (1 << 4);
}

//read whether SW4 Calibration Button is pressed or not
int calibratedBtn_read(void) {
    uint32_t state;

    state = GPIO_ReadValue(1);
    return state & (1 << 31);
}

//this handler occurs every 1ms
void SysTick_Handler(void) {
    msTicks++;
    //to produce continuous warning sounds
    if (warningOn) {
        if (!isBuzzerSet) {
            GPIO_SetValue(0, 1 << 26);
            isBuzzerSet = !isBuzzerSet;
        } else if (isBuzzerSet) {
            GPIO_ClearValue(0, 1 << 26);
            isBuzzerSet = !isBuzzerSet;
        }
    }
}

//get current tick (time)
static uint32_t getTicks(void) {
    return msTicks;
}

//improve Acc data by using MEAN Filter method
void shouldUpdateXYZ(){
    //we use Mean Filter here to remove noise
    recentX[recentValCounter] = x;
    recentY[recentValCounter] = y;
    recentZ[recentValCounter] = z;
    recentValCounter++;
    if(recentValCounter == 5)
        recentValCounter = 0;
    int8_t meanX = (recentX[0] + recentX[1] + recentX[2] + recentX[3] + recentX[4]) / 5;
    int8_t meanY = (recentY[0] + recentY[1] + recentY[2] + recentY[3] + recentY[4]) / 5;
    int8_t meanZ = (recentZ[0] + recentZ[1] + recentZ[2] + recentZ[3] + recentZ[4]) / 5;
    x = meanX;
    y = meanY;
    z = meanZ;
}

//Acc read that includes data pre-processing and post-processing
void accReadSelfImproved() {
    x_prev = x;
    y_prev = y;
    z_prev = z;
    acc_read(&x, &y, &z);
    x = x + xoff;
    y = y + yoff;
    z = z + zoff;
    shouldUpdateXYZ();
    if (x >= -1 && x <= 1 && y >= -1 && y <= 1 && z >= -1 && z <= 1) {
        x = y = z = 0;
    }
}

void clearRecentXYZ(){
    int i;
    for(i = 0; i < 5; i++){
        recentX[i] = 0;
        recentY[i] = 0;
        recentZ[i] = 0;
    }
}

uint8_t numberToCharUint(int number) {
    return (uint8_t)(number + 48);
}

void turnOnWarning() {
    GPIO_SetValue(2, (1 << 0));
    oled_putString(30, 40, (uint8_t*)"WARNING", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    warningOn = 1;
    pca9532_setLeds(0xffff, 0xffff);
}

void turnOffWarning() {
    GPIO_ClearValue( 2, (1<<0));
    oled_putString(30, 40, (uint8_t *)"       ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    warningOn = 0;
    pca9532_setLeds(0x0000, 0xffff);
    GPIO_ClearValue(0, 1<<26);
}

//Calibration mode
void doCalibration() {
    char oledOutput1[15];
    char oledOutput2[15];
    char oledOutput3[15];
    char timeWindowString[20];
    uint32_t prevCountingTicks = getTicks();

    //clear-up
    acc_setMode(ACC_MODE_MEASURE);
    led7seg_setChar('0', FALSE);
    oled_clearScreen(OLED_COLOR_BLACK);
    oled_putString(0, 0, (uint8_t *) "CALIBRATION", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    sprintf(timeWindowString, "timeWin: %d  ", timeWindow);
    oled_putString(0, 50, (uint8_t *)timeWindowString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    while(currentMode == Calibration){
        //is calibrated?
        if (calibratedBtn_read() == 0) {
            currentMode = StandBy;
            break;
        }
        //core of Calibration mode
        if (getTicks() - prevCountingTicks >= SensorOperatingTimeInterval) {
            acc_read(&x, &y, &z);
            shouldUpdateXYZ();
            sprintf(oledOutput1, "Acc: %d   ", x);
            sprintf(oledOutput2, "     %d   ", y);
            sprintf(oledOutput3, "     %d   ", z);
            oled_putString(0, 10, (uint8_t *)oledOutput1, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(0, 20, (uint8_t *)oledOutput2, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            oled_putString(0, 30, (uint8_t *)oledOutput3, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

            //Rotary Switch Control (Extra Feature)
            uint8_t rotaryStatus = rotary_read();
            if (rotaryStatus == ROTARY_RIGHT && timeWindow < TimeWindowHighestBound) {
                timeWindow += StepForTimeAdjustment;
                sprintf(timeWindowString, "timeWin: %d", timeWindow);
                oled_putString(0, 50, (uint8_t *)timeWindowString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            } else if (rotaryStatus == ROTARY_LEFT && timeWindow > reportingPeriod) {
                timeWindow -= StepForTimeAdjustment;
                sprintf(timeWindowString, "timeWin: %d", timeWindow);
                oled_putString(0, 50, (uint8_t *)timeWindowString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            }
            prevCountingTicks = getTicks();
        }
    }
    //setup Offset values
    clearRecentXYZ();
    xoff = 0-x;
    yoff = 0-y;
    zoff = 0-z;
}

//StandBy mode
void doStandByMode() {
    int standByTiming = 5;
    uint32_t prevCountingTicks = getTicks();
    uint32_t prevPCReportingTicks = getTicks();
    char msg[255];
    int isHandshakeEstablished = 0;
    int isReadyMsgSent = 0;
    int UART_waitTime = 0;

    //clean-up
    acc_setMode(ACC_MODE_STANDBY);
    oled_clearScreen(OLED_COLOR_BLACK);
    oled_putString(0, 0, (uint8_t *)"STANDBY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    led7seg_setChar(numberToCharUint(standByTiming), FALSE);
    while (currentMode == StandBy) {
        //Communication with Station
        if (getTicks() - prevPCReportingTicks >= TicksInOneSecond) {
            //UART handling in StandBy
            if(isReceived){
                isReceived = 0;
                UART_RcvMsgHandling();

                if(!isHandshakeEstablished && strcmp(rev_buf, "RNACK") == 0){
                    isReadyMsgSent = 0;
                } else if(!isHandshakeEstablished && strcmp(rev_buf, "RACK") == 0){
                    strcpy(msg, "HSHK 056");
                    UART_Send_Message(msg);
                    isHandshakeEstablished = 1;
                }

                if(isHandshakeEstablished && strcmp(rev_buf, "RSTC") == 0){
                    strcpy(msg, "CACK");
                    UART_Send_Message(msg);
                    currentMode = Calibration;
                    break;
                }
            }
            if(!isHandshakeEstablished){
                if(!isReadyMsgSent){
                    strcpy(msg, "RDY 056");
                    UART_Send_Message(msg);
                    isReadyMsgSent = 1;
                } else{
                    UART_waitTime++;
                    if(UART_waitTime == 5){
                        UART_waitTime = 0;
                        strcpy(msg, "RDY 056");
                        UART_Send_Message(msg);
                    }
                }
            }
            //Counter down in 7Seg Display
            if(standByTiming > 0){
                standByTiming--;
                led7seg_setChar(numberToCharUint(standByTiming), FALSE);
            }
            prevPCReportingTicks = getTicks();
        }
        //Core of StandBy mode
        if (standByTiming == 0 && getTicks() - prevCountingTicks >= SensorOperatingTimeInterval) {
            float temperature = temp_read() / 10.0;
            uint8_t isRisky = (luminance >= LuminanceThreshold);
            uint8_t isHot = (temperature >= TemperatureThreshold);

            if (!isRisky && !isHot && isHandshakeEstablished) {
                currentMode = Active;
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
            prevCountingTicks = getTicks();
        }
    }
}

//Active mode
void doActiveMode() {
    uint32_t prevCountingTicks = getTicks();
    uint32_t prevPCReportingTicks = getTicks();
    uint32_t prevTimingForWarningOn = getTicks();
    uint32_t prevTimingForWarningOff = getTicks();
    uint32_t countForFrequency = 0;
    int8_t isTimingForWarningOn = 0;
    float temperature = temp_read() / 10.0;
    uint8_t isRisky = (luminance >= LuminanceThreshold);
    uint8_t isHot = (temperature >= TemperatureThreshold);
    uint8_t joystickStatus;
    char freqString[255];
    char msg[255];

    //clean-up
    acc_setMode(ACC_MODE_MEASURE);
    oled_clearScreen(OLED_COLOR_BLACK);
    led7seg_setChar('0', FALSE);
    oled_putString(0, 0, (uint8_t *)"ACTIVE", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
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
    sprintf(freqString, "Low: %d;Upp: %d ", unsafeFrequencyLowerBound, unsafeFrequencyHigherBound);
    oled_putString(0, 50, (uint8_t *)freqString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    while (currentMode == Active) {
        if (getTicks() - prevCountingTicks >= SensorOperatingTimeInterval) {
            isRisky = (luminance >= LuminanceThreshold);

            if (isRisky || isHot) {
                currentMode = StandBy;
                break;
            }

            accReadSelfImproved();

            if ((z < 0 && z_prev >= 0) || (z >= 0 && z_prev < 0)) {
                countForFrequency++;
            }

            //Joystick Control (Extra Feature)
            joystickStatus = joystick_read();
            if (joystickStatus == JOYSTICK_CENTER) {
                // nothing
            } else if (joystickStatus == JOYSTICK_UP && unsafeFrequencyHigherBound < UnsafeFrequencyHighestBound) {
                unsafeFrequencyHigherBound++;
                sprintf(freqString, "Low: %d;Upp: %d ", unsafeFrequencyLowerBound, unsafeFrequencyHigherBound);
                oled_putString(0, 50, (uint8_t *)freqString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            } else if (joystickStatus == JOYSTICK_DOWN && unsafeFrequencyHigherBound > unsafeFrequencyLowerBound + 1) {
                unsafeFrequencyHigherBound--;
                sprintf(freqString, "Low: %d;Upp: %d ", unsafeFrequencyLowerBound, unsafeFrequencyHigherBound);
                oled_putString(0, 50, (uint8_t *)freqString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            } else if (joystickStatus == JOYSTICK_LEFT && unsafeFrequencyLowerBound > UnsafeFrequencyLowestBound) {
                unsafeFrequencyLowerBound--;
                sprintf(freqString, "Low: %d;Upp: %d ", unsafeFrequencyLowerBound, unsafeFrequencyHigherBound);
                oled_putString(0, 50, (uint8_t *)freqString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            } else if (joystickStatus == JOYSTICK_RIGHT && unsafeFrequencyHigherBound > unsafeFrequencyLowerBound + 1) {
                unsafeFrequencyLowerBound++;
                sprintf(freqString, "Low: %d;Upp: %d ", unsafeFrequencyLowerBound, unsafeFrequencyHigherBound);
                oled_putString(0, 50, (uint8_t *)freqString, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
            }

            prevCountingTicks = getTicks();
        }
        //Communication with Station
        if (getTicks() - prevPCReportingTicks >= reportingPeriod) {
            temperature = temp_read() / 10.0;
            isHot = (temperature >= TemperatureThreshold);

            //UART handling in Active
            if (isReceived) {
                isReceived = 0;
                UART_RcvMsgHandling();

                if (strcmp(rev_buf, "RSTC") == 0) {
                    strcpy(msg, "CACK");
                    UART_Send_Message(msg);
                    currentMode = Calibration;
                    break;
                } else if(strcmp(rev_buf, "RSTS") == 0){
                    strcpy(msg, "SACK");
                    UART_Send_Message(msg);
                    currentMode = StandBy;
                    break;
                }
            }
            char reportContent[255];
            char warningContent[255];
            if(warningOn)
                strcpy(warningContent, "WARNING");
            else
                strcpy(warningContent, "");
            sprintf(reportContent, "REPT 056 %02d %s", countForFrequency / 2, warningContent);
            UART_Send_Message(reportContent);

            //Core of Active mode
            if (countForFrequency / 2 >= unsafeFrequencyLowerBound && countForFrequency / 2 <= unsafeFrequencyHigherBound) {
                if (isTimingForWarningOn) {
                    if (getTicks() - prevTimingForWarningOn >= timeWindow) {
                        turnOnWarning();
                    }
                } else {
                    prevTimingForWarningOn = getTicks() - reportingPeriod;
                    isTimingForWarningOn = 1;
                }
            } else {
                if (isTimingForWarningOn) {
                    prevTimingForWarningOff = getTicks() - reportingPeriod;
                    isTimingForWarningOn = 0;
                } else {
                    if (getTicks() - prevTimingForWarningOff >= timeWindow) {
                        turnOffWarning();
                    }
                }
            }
            countForFrequency = 0;
            prevPCReportingTicks = getTicks();
        }
    }
    turnOffWarning();
}

void all_init() {
    i2c_init();
    ssp_init();

    resetBtn_init();
    calibratedBtn_init();

    //general peripherals init
    pca9532_init();
    led7seg_init();
    acc_init();
    oled_init();
    joystick_init();
    rotary_init();

    // GPIO settings generally
    GPIO_SetDir( 2, (1<<0), 1 );
    GPIO_SetDir( 2, 0, 0 );
    GPIO_SetDir(2, 1<<1, 1);
    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    oled_clearScreen(OLED_COLOR_BLACK);
    //temperature sensor init
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
    UART_Receive_Int_Init();
    // set up interrupt priority
    NVIC_SetPriority(EINT3_IRQn, (1 << __NVIC_PRIO_BITS));  // 1 << 5 = 16, first priority for external interrupt
    NVIC_SetPriority(UART3_IRQn, (1 << __NVIC_PRIO_BITS) + 1);
    NVIC_SetPriority(TIMER0_IRQn, (1 << __NVIC_PRIO_BITS) + 2);
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
