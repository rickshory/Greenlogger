/******************************************************************************
*
*               Inter-Integrated Circuit (I2C) functions
*
******************************************************************************
* FileName:           I2C.c
* Dependencies:       GenericTypeDefs.h
*                     string.h
*                     stdlib.h
*                     ctype.h
* Processor:          PIC24
* Compiler:           C30
* Company:            EcoTronics
* Version:            1.0.0
*
*****************************************************************************/

#include "Compiler.h"
#include "GenericTypeDefs.h"
#include "stdlib.h"
#include "ctype.h"
#include "HardwareProfile.h"


/*****************************************************************************/
/*                         Global Variables                                  */
/*****************************************************************************/


/************************************************************************/
/*                        Structures and defines                        */
/************************************************************************/


/************************************************************************************/
/*                               Prototypes                                         */
/************************************************************************************/

void I2C_Idle (void); // blocks until I2C bus is idle
void I2C_Init (void); // initialize I2C peripheral
void I2C_Start (void); // creates a Start condition on the I2C bus
void I2C_Stop (void); // creates a Stop condition on the I2C bus
BYTE I2C_Write (BYTE data); // writes a single byte out on the I2C bus, returns 1 if ACK otherwise 0
void I2C_ReStart (void); // do a ReStart, setup to Read
BYTE I2C_Read (BYTE ack); // reads a single byte from the I2C bus, set ack = 1 if to be acknowledged, 0 in NACK
void I2C_SendACK (void); // master, create an Acknowledge (ACK) condition on the I2C bus
void I2C_SendNACK (void); // master, create an No Acknowledge (NACK) condition on the I2C bus


/*************************************************************************
  Function:
    void I2C_Idle (void)
  Summary:
    holds until I2C bus is idle
  Conditions:
    I2C should be enabled
  Input:
    None
  Return Values:
    None
  Side Effects:
    None
  Description:
    Holds until I2C bus is idle
  Remarks:
    None
  *************************************************************************/

void I2C_Idle (void)
{
//    __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
    while (I2C2STATbits.TRSTAT) { // hardware clears this bit when done transmitting
        ; // timout or other exit here?
    }
//    __asm__ volatile("disi #0x0000"); // re-enable interrupts
}



/*************************************************************************
  Function:
    void I2C_Init (void)
  Summary:
    initializes the I2C (2) peripheral
  Conditions:
    I2C SCK rate defined in HardwareProfile.h
  Input:
    None
  Return Values:
    None
  Side Effects:
    None
  Description:
    None
  Remarks:
    Sets up Master mode, No slew rate control, 100Khz
  *************************************************************************/
void I2C_Init (void)
{
//    __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
/*    if (!I2C_POWER_TRIS) // assure the pin that powers the I2C bus is an output (TRIS = 0)
        I2C_POWER_TRIS = 0;
    __asm__ ("nop");
    __asm__ ("nop");
    __asm__ ("nop");
    __asm__ ("nop");
    I2C_POWER = 1; // assure power is on
    __asm__ ("nop");
    __asm__ ("nop");
    __asm__ ("nop");
    __asm__ ("nop");
*/
    if (_I2C2MD) { // I2C module 2 was disabled to save power during sleep
         _I2C2MD = 0; // remove disable from I2C module 2
         __asm__ ("nop");
    }
    // assure physical pins are inputs, output overrides I2C functionality and causes bus collisions
    // (this is at variance with the data sheet, which says I2C takes priority over port function)
    I2C_SDA_TRIS = 1;
    I2C_SCK_TRIS = 1;
    I2C2BRG = I2C100kHz;
    // set bits for Master Mode, No Slew Rate Control
	// do not enable yet
	I2C2CON = 0x1200;
	I2C2TRN = 0x0000; // clear transmit & receive buffers
	I2C2RCV = 0x0000;
	// enable I2C
	I2C2CON = 0x9200;
//    __asm__ volatile("disi #0x0000"); // re-enable interrupts
}

/*************************************************************************
  Function:
    void I2C_Start (void)
  Summary:
    creates a Start condition on the I2C bus
  Conditions:
    I2C should be enabled
  Input:
    None
  Return Values:
    None
  Side Effects:
    None
  Description:
    Holds until Start condition is established
  Remarks:
    None
  *************************************************************************/

void I2C_Start (void)
{
//    __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
    I2C2CONbits.SEN = 1; // initate Start sequence
//    __asm__ volatile("nop"); // seems like a good idea to wait a bit
    while (I2C2CONbits.SEN) { // hardware clears this bit when Start sequence done
        ; // timout or other exit here?
    }
//    __asm__ volatile("disi #0x0000"); // re-enable interrupts
}


/*************************************************************************
  Function:
    void I2C_Stop (void)
  Summary:
    creates a Stop condition on the I2C bus
  Conditions:
    I2C should be enabled
  Input:
    None
  Return Values:
    None
  Side Effects:
    None
  Description:
    Holds until Stop condition is established
  Remarks:
    None
  *************************************************************************/

void I2C_Stop (void)
{
//    __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
    I2C2CONbits.PEN = 1; // initate Stop sequence
//    __asm__ volatile("nop"); // seems like a good idea to wait a bit
    while (I2C2CONbits.PEN) { // hardware clears this bit when Stop sequence done
        ; // timout or other exit here?
    }
//    __asm__ volatile("disi #0x0000"); // re-enable interrupts    
}

/*************************************************************************
  Function:
    BYTE I2C_Write (BYTE data)
  Summary:
    writes a single byte out on the I2C bus
  Conditions:
    I2C should be enabled and Start condition on the bus
  Input:
    byte to be written
  Return Values:
    1 if ACK, otherwise 0
  Side Effects:
    None
  Description:
    Holds until byte is written
  Remarks:
    None
  *************************************************************************/

BYTE I2C_Write (BYTE data)
{
    BYTE r;
//    __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
    while (I2C2STATbits.TRSTAT) { // wait for bus idle
        ; // timeout or other exit here?
    }
    I2C2TRN = data; // put byte in transmit register
    // TBF is set on write to I2C2TRN, somewhat later ...
    // TRSTAT is set when BRG starts, when first data bit asserted on SDA (SCL still low)
    // TBF clears on 8th falling clock (8th data bit sent)
    // TRSTAT clears on 9th falling clock, which latches ACK or NACK
    while (I2C2STATbits.TBF || I2C2STATbits.TRSTAT) {
        ; // timeout or other exit here?
    }
    r = (!I2C2STATbits.ACKSTAT);
/*
    while (I2C2STATbits.TRSTAT) { // hardware clears this bit when transmit complete, bus idle
        ; // timeout or other exit here?
    }
*/
//    __asm__ volatile("disi #0x0000"); // re-enable interrupts
    return r;
} // return (!I2C2STATbits.ACKSTAT);

/*************************************************************************
  Function:
    void I2C_ReStart (void)
  Summary:
    do a ReStart, setup to Read
  Conditions:
    previously wrote to a device that we are now going to read from
  Input:
     None
  Return Values:
    None
  Side Effects:
    None
  Description:
    Holds until ReStart finished
  Remarks:
    Unlike Start, ignores the state of signals
  *************************************************************************/

void I2C_ReStart (void)
{
//    __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
    I2C2CONbits.RSEN = 1; // initate ReStart sequence
    while (I2C2CONbits.RSEN) { // hardware clears this bit when ReStart sequence done
        ; // timout or other exit here?
    }     
//    __asm__ volatile("disi #0x0000"); // re-enable interrupts
}


/*************************************************************************
  Function:
    BYTE I2C_Read (BYTE ack)
  Summary:
    reads a single byte from the I2C bus
  Conditions:
    I2C should be enabled, then Write (device address, and any setup), 
         then ReStart preparatory to reading from slave device
  Input:
     set ack = 1 if to be acknowledged, 0 if NACK
  Return Values:
    contents of I2C2RCV
  Side Effects:
    None
  Description:
    Holds until byte is read, also until Acknowledge process is complete
  Remarks:
    None
  *************************************************************************/

BYTE I2C_Read (BYTE ack)
{
//    __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
    I2C2CONbits.RCEN = 1; // enable master receive
    __asm__ volatile("nop"); // seems like a good idea to wait a bit
    while (!I2C2STATbits.RBF) // hardware sets this bit when all bits received
    {
        ; // timout or other exit here?
    }
    if (ack) // master does an Acknowledge, either ACK or NACK
         I2C2CONbits.ACKDT = 0; // Send ACK during Acknowledge (tell slave not to send any more)
    else
         I2C2CONbits.ACKDT = 1; // Send NACK during Acknowledge (all except last byte)
    I2C2CONbits.ACKEN = 1; // initialize Acknowledge process
    while (I2C2CONbits.ACKEN) { // hardware clears this bit when done with Acknowledge process
        ; // timout or other exit here?
    }
    I2C2CONbits.ACKDT = 0; // restore this to the default state of ACK
//    __asm__ volatile("disi #0x0000"); // re-enable interrupts  
    return(I2C2RCV); // return byte received
}


