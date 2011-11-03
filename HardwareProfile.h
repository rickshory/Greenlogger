/******************************************************************************
 *
 *                Microchip Memory Disk Drive File System
 *
 ******************************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    None
 * Processor:       PIC18/PIC24/dsPIC30/dsPIC33/PIC32
 * Compiler:        C18/C30/C32
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
*****************************************************************************/
/*
board layout versions, and corresponding prototypes (Pn):
00: P1, this is the only one of this board version
01: P2, P3
02: P4, P5, & P6
The breadboarded prototypes, P(-1) and P0, can change and are updated
 to the latest board vesion
*/
//#define BOARD_VERSION_00
//#define BOARD_VERSION_01
#define BOARD_VERSION_02

#ifndef _HARDWAREPROFILE_H_
#define _HARDWAREPROFILE_H_

// Define your clock speed here

// Sample clock speed for PIC18
#if defined (__18CXX)

    #define GetSystemClock()        40000000                        // System clock frequency (Hz)
    #define GetPeripheralClock()    GetSystemClock()                // Peripheral clock freq.
    #define GetInstructionClock()   (GetSystemClock() / 4)          // Instruction clock freq.

// Sample clock speed for a 16-bit processor
#elif defined (__C30__)

//    #define GetSystemClock()        32000000 // original example
    #define GetSystemClock()        4000000 // 4MHz internal RC osc
    #define GetPeripheralClock()    GetSystemClock()
    #define GetInstructionClock()   (GetSystemClock() / 2)

    // Clock values
    #define MILLISECONDS_PER_TICK       10                      // Definition for use with a tick timer
    #define TIMER_PRESCALER             TIMER_PRESCALER_8       // Definition for use with a tick timer
    #define TIMER_PERIOD                20000                   // Definition for use with a tick timer

#endif
    

// Select your interface type
// This library currently only supports a single physical interface layer at a time


// Description: Macro used to enable the SD-SPI physical layer (SD-SPI.c and .h)
#define USE_SD_INTERFACE_WITH_SPI

/*********************************************************************/
/******************* Pin and Register Definitions ********************/
/*********************************************************************/

// definitions to read device ID and Revision
#define DEVpage         0xff
#define DEVIDadr       0x0000
#define DEVREVadr      0x0002

/* SD Card definitions: Change these to fit your application when using
   an SD-card-based physical layer                                   */

#ifdef USE_SD_INTERFACE_WITH_SPI
    #ifdef __PIC24F__

        // Registers for the SPI module you want to use

        // Description: The main SPI control register
        #define SPICON1             SPI1CON1
        // Description: The SPI status register
        #define SPISTAT             SPI1STAT
        // Description: The SPI Buffer
        #define SPIBUF              SPI1BUF
        // Description: The receive buffer full bit in the SPI status register
        #define SPISTAT_RBF         SPI1STATbits.SPIRBF

        // Description: The bitwise define for the SPI control register (i.e. _____bits)
        #define SPICON1bits         SPI1CON1bits
        // Description: The bitwise define for the SPI status register (i.e. _____bits)
        #define SPISTATbits         SPI1STATbits
        // Description: The enable bit for the SPI module
        #define SPIENABLE           SPISTATbits.SPIEN

        #ifdef BOARD_VERSION_00
            // Description: SD-SPI Chip Select Output bit
            #define SD_CS				LATBbits.LATB15
            // Description: SD-SPI Chip Select TRIS bit
            #define SD_CS_TRIS          TRISBbits.TRISB15
/* Card Detect (SD_CD) and Write Enable (SD_WE) don't apply to microSD cards, RS 2011-02-02      
            // Description: SD-SPI Card Detect Input bit
            #define SD_CD               PORTAbits.RA0
            // Description: SD-SPI Card Detect TRIS bit
            #define SD_CD_TRIS          TRISAbits.TRISA0
        
            // Description: SD-SPI Write Protect Check Input bit
            #define SD_WE               PORTAbits.RA2
            // Description: SD-SPI Write Protect Check TRIS bit
            #define SD_WE_TRIS          TRISAbits.TRISA2
*/

        // Tris pins for SCK/SDI/SDO lines
            // Description: The TRIS bit for the SCK pin
            #define SPICLOCK            TRISBbits.TRISB13
            // Description: The TRIS bit for the SDI pin
            #define SPIIN               TRISBbits.TRISB12
            // Description: The TRIS bit for the SDO pin
            #define SPIOUT              TRISBbits.TRISB14

        // weak pull-ups for SDI/CS lines
            // pin 23, CN14 (RP12 mapped as SDI, card DAT0 pin 7)
            #define PULL_UP_SDI _CN14PUE
            // pin 26, CN11 (RP15 mapped as _SS, card DAT3 pin 1)
            #define PULL_UP_CS _CN11PUE

        // SD card power pin
            #define SD_POWER_TRIS TRISBbits.TRISB11
            #define SD_POWER _RB11
        #endif // BOARD_VERSION_00

        #ifdef BOARD_VERSION_01
            // Description: SD-SPI Chip Select Output bit
            #define SD_CS				LATBbits.LATB15
            // Description: SD-SPI Chip Select TRIS bit
            #define SD_CS_TRIS          TRISBbits.TRISB15

        // Tris pins for SCK/SDI/SDO lines
            // Description: The TRIS bit for the SCK pin
            #define SPICLOCK            TRISBbits.TRISB12
            // Description: The TRIS bit for the SDI pin
            #define SPIIN               TRISBbits.TRISB11
            // Description: The TRIS bit for the SDO pin
            #define SPIOUT              TRISBbits.TRISB14

        // weak pull-ups for SDI/CS lines
            // pin 22, CN15 (RP11 mapped as SDI, card DAT0 pin 7)
            #define PULL_UP_SDI _CN15PUE
            // pin 26, CN11 (RP15 mapped as _SS, card DAT3 pin 1)
            #define PULL_UP_CS _CN11PUE

        // SD card power pin
            #define SD_POWER_TRIS TRISBbits.TRISB13
            #define SD_POWER _RB13
        #endif // BOARD_VERSION_01

        #ifdef BOARD_VERSION_02
            // Description: SD-SPI Chip Select Output bit
            #define SD_CS				LATBbits.LATB10
            // Description: SD-SPI Chip Select TRIS bit
            #define SD_CS_TRIS          TRISBbits.TRISB10

        // Tris pins for SCK/SDI/SDO lines
            // Description: The TRIS bit for the SCK pin
            #define SPICLOCK            TRISBbits.TRISB13
            // Description: The TRIS bit for the SDI pin
            #define SPIIN               TRISBbits.TRISB14
            // Description: The TRIS bit for the SDO pin
            #define SPIOUT              TRISBbits.TRISB11

        // weak pull-ups for SDI/CS lines
            // pin 25, CN12 (RP14 mapped as SDI, card DAT0 pin 7)
            #define PULL_UP_SDI _CN12PUE
            // pin 21, CN16 (RP10 mapped as _SS, card DAT3 pin 1)
            #define PULL_UP_CS _CN11PUE

        // SD card power pin
            #define SD_POWER_TRIS TRISBbits.TRISB12
            #define SD_POWER _RB12

        // Bluetooth module control pins
            #define BT_PWR_CTL_TRIS _TRISA1
            #define BT_PWR_CTL _RA1

        #endif // BOARD_VERSION_02


        // Will generate an error if the clock speed is too low to interface to the card
        #if (GetSystemClock() < 100000)
            #error Clock speed must exceed 100 kHz
        #endif    

    #elif defined (__PIC32MX__)
    
    #endif

#endif


// USART baud rate divisor calculations assume 4MHz clock
// following are for UxMODE<3>, BRGH = 1 for high-speed mode, clock/4
#define	Baud300	0x0D04 // 0.010% err
#define	Baud1200	0x0340 // 0.040% err
#define	Baud2400	0x01A0 // -0.080% err
#define	Baud4800	0x00CF // 0.160% err
#define	Baud9600	0x0067 // 0.160% err
#define	Baud19200	0x0033 // 0.160% err
#define	Baud38400	0x0019 // 0.160% err
#define	Baud115200	0x0008 // -3.549% err	
#define	Baud250000	0x0003 // 0.000% err

// following are for UxMODE<3>, BRGH = 0 for standard mode, clock/16
//#define	Baud300	0x0340 // 0.040% err
//#define	Baud1200	0x00CF // 0.160% err
//#define	Baud2400	0x0067 // 0.160% err
//#define	Baud4800	0x0033 // 0.160% err
//#define	Baud9600	0x0019 // 0.160% err
//#define	Baud19200	0x000C // 0.160% err
//#define	Baud38400	0x0006 // -6.994% err
//#define	Baud115200	0x0001 // 8.507% err

// #include <uart2.h>

// I2C clock rates assume 4MHz oscillator

//#define I2C_POWER_TRIS _TRISA1
//#define I2C_POWER _RA1

// SDA2 physical pin 6
#define I2C_SDA_TRIS _TRISB2
// SCK2 physical pin 7
#define I2C_SCK_TRIS _TRISB3


//                               sys osc    actual
//#define	I2C100kHz  0x009D //  16 MHz    100 kHz
//#define	I2C100kHz  0x004E //  8 MHz     100 kHz
#define	I2C100kHz  0x0027 //  4 MHz        99 kHz
//#define	I2C100kHz  0x0013 //  2 MHz      99 kHz

//#define	I2C400kHz  0x0025 //  16 MHz    404 kHz
//#define	I2C400kHz  0x0012 //  8 MHz     404 kHz
#define	I2C400kHz  0x0009 //  4 MHz     385 kHz
//#define	I2C400kHz  0x0004 //  2 MHz     385 kHz (closest possible)

//#define	I2C1MHz    0x000D //  16 MHz   1026 kHz
//#define	I2C1MHz    0x0006 //  8 MHz    1026 kHz
// below this are invalid, too slow
//#define	I2C1MHz    0x0003 //  4 MHz     909 kHz

// defines for the ADXL345 accellerometer

#define ADXL345_ADDR_WRITE 0x3A
#define ADXL345_ADDR_READ 0x3B

#define ADXL345_REG_DEVID  0x00 // R  11100101  Device ID. 
#define ADXL345_REG_THRESH_TAP  0x1D // R/W  00000000  Tap threshold. 
#define ADXL345_REG_OFSX  0x1E // R/W  00000000  X-axis offset. 
#define ADXL345_REG_OFSY  0x1F // R/W  00000000  Y-axis offset. 
#define ADXL345_REG_OFSZ  0x20 // R/W  00000000  Z-axis offset. 
#define ADXL345_REG_DUR  0x21 // R/W  00000000  Tap duration. 
#define ADXL345_REG_Latent  0x22 // R/W  00000000  Tap latency. 
#define ADXL345_REG_Window  0x23 // R/W  00000000  Tap window. 
#define ADXL345_REG_THRESH_ACT  0x24 // R/W  00000000  Activity threshold. 
#define ADXL345_REG_THRESH_INACT  0x25 // R/W  00000000  Inactivity threshold. 
#define ADXL345_REG_TIME_INACT  0x26 // R/W  00000000  Inactivity time. 
#define ADXL345_REG_ACT_INACT_CTL  0x27 // R/W  00000000  Axis enable control for activity and inactivity detection. 
#define ADXL345_REG_THRESH_FF  0x28 // R/W  00000000  Free-fall threshold. 
#define ADXL345_REG_TIME_FF  0x29 // R/W  00000000  Free-fall time. 
#define ADXL345_REG_TAP_AXES  0x2A // R/W  00000000  Axis control for tap/double tap. 
#define ADXL345_REG_ACT_TAP_STATUS  0x2B // R  00000000  Source of tap/double tap. 
#define ADXL345_REG_BW_RATE  0x2C // R/W  00001010  Data rate and power mode control. 
#define ADXL345_REG_POWER_CTL  0x2D // R/W  00000000  Power-saving features control. 
#define ADXL345_REG_INT_ENABLE  0x2E // R/W  00000000  Interrupt enable control. 
#define ADXL345_REG_INT_MAP  0x2F // R/W  00000000  Interrupt mapping control. 
#define ADXL345_REG_INT_SOURCE  0x30 // R  00000010  Source of interrupts. 
#define ADXL345_REG_DATA_FORMAT  0x31 // R/W  00000000  Data format control. 
#define ADXL345_REG_DATAX0  0x32 // R  00000000  X-Axis Data 0. 
#define ADXL345_REG_DATAX1  0x33 // R  00000000  X-Axis Data 1. 
#define ADXL345_REG_DATAY0  0x34 // R  00000000  Y-Axis Data 0. 
#define ADXL345_REG_DATAY1  0x35 // R  00000000  Y-Axis Data 1. 
#define ADXL345_REG_DATAZ0  0x36 // R  00000000  Z-Axis Data 0. 
#define ADXL345_REG_DATAZ1  0x37 // R  00000000  Z-Axis Data 1. 
#define ADXL345_REG_FIFO_CTL  0x38 // R/W  00000000  FIFO control.

//Data rate codes.
#define ADXL345_3200HZ      0x0F
#define ADXL345_1600HZ      0x0E
#define ADXL345_800HZ       0x0D
#define ADXL345_400HZ       0x0C
#define ADXL345_200HZ       0x0B
#define ADXL345_100HZ       0x0A
#define ADXL345_50HZ        0x09
#define ADXL345_25HZ        0x08
#define ADXL345_12HZ5       0x07
#define ADXL345_6HZ25       0x06

#endif
