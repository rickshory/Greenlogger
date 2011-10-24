/******************************************************************************
*
*               Inter-Integrated Circuit (I2C) functions
*
******************************************************************************
* FileName:           I2C.h
* Processor:          PIC24
* Compiler:           C30
* Company:            EcoTronics
* Version:            1.0.0
*
*****************************************************************************/


/*******************************************************************/
/*                     Strunctures and defines                     */
/*******************************************************************/

/***************************************************************************
* Prototypes                                                               *
***************************************************************************/

void I2C_Idle (void); // blocks until I2C bus is idle
void I2C_Init (void); // initialize I2C peripheral
void I2C_Start (void); // creates a Start condition on the I2C bus
void I2C_Stop (void); // creates a Stop condition on the I2C bus
BYTE I2C_Write (BYTE data); // writes a single byte out on the I2C bus, returns 1 if ACK otherwise 0
void I2C_ReStart (void); // do a ReStart, setup to Read
BYTE I2C_Read (BYTE ack); // reads a single byte from the I2C bus, set ack = 1 if to be acknowledged, 0 in NACK
void I2C_SendACK (void); // master, create an Acknowledge (ACK) condition on the I2C bus
void I2C_SendNACK (void); // master, create an No Acknowledge (NACK) condition on the I2C bus
