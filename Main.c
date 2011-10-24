/******************************************************************************
 ******************************************************************************
 * FileName:        Main.c
 * Dependencies:    FSIO.h, PinMapping.h
 * Processor:       PIC24F
 * Compiler:        C30
 * Company:         EcoTronics
 * Version:         1.0.0
 *
 *
*****************************************************************************/

#include <stdio.h>
#include "FSIO.h"
#include "PinMapping.h"
#include "I2C.h"

void outputStringToUSART (char* St);
void checkForCommands (void);
void initMain (void);
void setupAlarm (int N);
int writeCharsToFile (char* St, int N);
void tellFileWriteError (int err);
void assureSDCardIsOn (void);
void assureSDCardIsOff (void);
void initSPI (void);
void initUSART (void);
void initBTModule(void);
void findAccelerometer (void);
void initAccelerometer (void);
void readAccelerometer (void);
void clearAnyADXL345TapInterrupt (void);
void setTimeFromCharBuffer (char *c);
void createTimestamp (void);
int getCellVoltage(void);
void assureDataHeadersLogged(void);
void getDataReadings(void);
void startTimer1ToRunThisManySeconds(unsigned int numSecondsToRun);
void setSDCardPowerControl(void);
int isValidTimestamp(char* p);

#define USART1_Buffer_Length 0x800
#define commandBufferLen 30
#define MAX_IRRAD_COUNT 65535
#define CELL_VOLTAGE_THRESHOLD_WRITE_SD_CARD 370
#define CELL_VOLTAGE_THRESHOLD_READ_IRRADIANCE 350

int DEVID, DEVREV;
unsigned long bbIrradThresholdDark = 2000 * 16;
int machState;
extern int safely_set_ALCFGRPT(int);
extern volatile int safe_ALCFGRPT; // we don't access this from C yet
extern volatile char  *USART1_inputbuffer_head;
extern volatile char  *USART1_inputbuffer_tail;
extern volatile char  USART1_inputbuffer[USART1_Buffer_Length];
extern volatile char  *USART1_outputbuffer_head;
extern volatile char  *USART1_outputbuffer_tail;
extern volatile char  USART1_outputbuffer[USART1_Buffer_Length];

//extern volatile unsigned long irrData;

extern void _RTCCInterrupt(void);
extern void _U1TXInterrupt(void);
extern void _U1RXInterrupt(void);
//extern void _T1Interrupt(void);
//extern void _T2Interrupt(void);
//extern void _T3Interrupt(void);
//extern void _CompInterrupt(void);
extern void _INT1Interrupt(void);

extern void Delayms(BYTE milliseconds);

enum machStates
{
 Asleep = 0, 
 Idle,  // done with work but not yet allowed to go to sleep
 GettingTimestamp, // first step towards acquiring data
 ReadingSensors, // in the process of acquiring sensor data
 WritingData, // in the process of writing acquired data
 ServicingCommand, // servicing an input command
 TransferringData, // in the process of sending logged data
 WritingFile // in the process of saving data to SD card
};

struct { // finer details of machine state
 unsigned timeHasBeenSet:1; // false when coming out of reset, true after RTCC has been set
 unsigned timerHasBeenSynchronized:1; // alarm interrupt happens on 0 second boundary (00, 10, 20, ...)
 unsigned isReadingSensors:1; // has been awakened by RTCC interrupt and is reading data
 unsigned isRoused:1; // triggered by external interrupt, and re-triggered by any activity while awake
 unsigned reRoused:1; // re-triggered while awake, used to reset timeout
 unsigned isLeveling:1; // diagnostics show accelerometer output rather than light sensors, used for leveling the system
 unsigned isSendingData:1; // in the process of sending logged data
 unsigned accelerometerIsThere:1; // set if system finds ADXL345 on I2C bus
} stateFlags;

struct {
 unsigned turnSDCardOffBetweenDataWrites:1; // power the SD card up for each write and down afterwards
 unsigned isDataWriteTime:1; // a time point to write data to the SD card
 unsigned irrSensorGain:1; // irradiance sensor gain, 0=low gain; 1= high gain, 16x low gain
 unsigned irrSensorIntegrationPeriodHi:1; // individual bits make some testing easier later
 unsigned irrSensorIntegrationPeriodLo:1;
// 00=low,    *(322/11)  to correct (29.2927, ~29, 0.93% error)
// 01=medium, *(322/81)  to correct (3.9753, ~4, 0.62% error)
// 10=high,   *(322/322) to correct (1)
 unsigned unused3:1; // not used yet
 unsigned unused2:1; // not used yet
 unsigned unused1:1; // not used yet
} stateFlags_2;


enum fileWriteResults
{
 NoProblem = 0, // return value if it happend with no issues
 NoCard, // SD card not present or not detected
 NoInit, // could not initialize file system
 NoMkDir, // could not create the requested directory
 NoChDir, // could not change to the requested directory
 NoFileOpen, // could not open the requested file
 NoFileWrite, // could not write to the file
 NoFileSeek, // could not seek as requested
 NoClose, // could not close file
 PowerTooLowForSDCard, // cell volatage is below threshold to safely write card
 IgnoreCard // flag is set to ignore SD card
};

struct {
 unsigned useSDcard:1;
 unsigned sleepBetweenReadings:1;
 unsigned isDark:1; // state is Dark, and device reads data on long intervals 
 unsigned wasDark:1; // state was Dark on previous iteration
 unsigned writeDataHeaders:1; // flag to write column headers to SD card
                              //  done on init, reset, time change, and midnight rollover
} flags1;

// union to allow reading in Lo and Hi bytes of irradiance, and reading out whole word
typedef struct {
    union {
        struct {
            unsigned char irrLoByte;
            unsigned char irrHiByte;
        };
        struct {
            unsigned int irrWholeWord;
        };
    };
    unsigned int irrMultiplier;
} irrData;

irrData irrReadings[4];
unsigned int irrReadingNumber;
BYTE d, r, irrSensorNumber, irrSensorReadAddr, irrSensorWriteAddr, irrChannelNumber, irrChannel;
int cellVoltage;

char commandBuffer[commandBufferLen];
char *commandBufferPtr;
char send2[] = "2";
char receiveBuffer[50];
char dirNameBuffer[] ="\\00-00";
char fileNameBuffer[] = "00.TXT";
char intervalShort[] = "00:00:10"; // default ten seconds
char intervalLong[] = "00:10:00"; // default one hour
int len, err = 0;
char str[128]; // generic space for strings to be output
int intTmp;
unsigned int unsignedIntTmp;
// default date/time is winter solstice 2010
char timeStampBuffer[] = "\r\n2010-12-21 10:47:13";

int byteVal;
unsigned long secsSince1Jan2000, timeLastSet = 0;

    _CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & ICS_PGx1 & FWDTEN_OFF)
/*  
//{
   JTAG:
     JTAGEN_OFF           Disabled
     JTAGEN_ON            Enabled  ; default

   Code Protect:
     GCP_ON               Enabled
     GCP_OFF              Disabled  ; default

   Write Protect:
     GWRP_ON              Enabled
     GWRP_OFF             Disabled  ; default

   Background Debugger:
     BKBUG_ON             Enabled
     BKBUG_OFF            Disabled  ; default

   Clip-on Emulation mode:
     COE_ON               Enabled
     COE_OFF              Disabled  ; default

   ICD pins select:
     ICS_PGx3             EMUC/EMUD share PGC3/PGD3
     ICS_PGx2             EMUC/EMUD share PGC2/PGD2
     ICS_PGx1             EMUC/EMUD share PGC1/PGD1  ; default

   Watchdog Timer:
     FWDTEN_OFF           Disabled
     FWDTEN_ON            Enabled  ; default

   Windowed WDT:
     WINDIS_ON            Enabled
     WINDIS_OFF           Disabled  ; default

   Watchdog prescaler:
     FWPSA_PR32           1:32
     FWPSA_PR128          1:128  ; default

   Watchdog postscale:
     WDTPS_PS1            1:1
     WDTPS_PS2            1:2
     WDTPS_PS4            1:4
     WDTPS_PS8            1:8
     WDTPS_PS16           1:16
     WDTPS_PS32           1:32
     WDTPS_PS64           1:64
     WDTPS_PS128          1:128
     WDTPS_PS256          1:256
     WDTPS_PS512          1:512
     WDTPS_PS1024         1:1, 024
     WDTPS_PS2048         1:2, 048
     WDTPS_PS4096         1:4, 096
     WDTPS_PS8192         1:8, 192
     WDTPS_PS16384        1:16, 384
     WDTPS_PS32768        1:32, 768  ; default
//}*/    
    

    _CONFIG2(IESO_OFF & SOSCSEL_LPSOSC & WUTSEL_LEG & FNOSC_FRC & FCKSM_CSDCMD & OSCIOFNC_ON & IOL1WAY_OFF & POSCMOD_NONE) // uses internal osc
/*
//{
     IESO_OFF             Two Speed Start-up:Disabled
     IESO_ON              Two Speed Start-up:Enabled  ; default

  The following 2 configuration settings (SOSCSEL and WUTSEL) are not implemented on
   earlier versions of the PIC24FJ64GA002.  They are minor power saving options.

   Secondary Oscillator:
     SOSCSEL_LPSOSC       Low Power Secondary Oscillator
     SOSCSEL_SOSC         Default Secondary Oscillator  ; default

   Voltage Regulator Standby-mode Wake-up Timer:
     WUTSEL_FST           Fast Wake-up timer selected
     WUTSEL_LEG           Legacy Wake-up timer selected
      In some versions of the Header file, this setting is symbolized as below
     WUPTSEL_LPSOSC       Disabled
     WUPTSEL_SOSC         Enabled  ; default

   Oscillator Selection:
     FNOSC_FRC            Fast RC oscillator
     FNOSC_FRCPLL         Fast RC oscillator w/ divide and PLL
     FNOSC_PRI            Primary oscillator (XT, HS, EC)
     FNOSC_PRIPLL         Primary oscillator (XT, HS, EC) w/ PLL
     FNOSC_SOSC           Secondary oscillator
     FNOSC_LPRC           Low power RC oscillator
     FNOSC_FRCDIV         Fast RC oscillator with divide  ; default

   Clock switching and clock monitor:
     FCKSM_CSECME         Both enabled
     FCKSM_CSECMD         Only clock switching enabled
     FCKSM_CSDCMD         Both disabled  ; default

   OSCO/RC15 function:
     OSCIOFNC_ON          RC15
     OSCIOFNC_OFF         OSCO or Fosc/2  ; default

   RP Register Protection:
     IOL1WAY_OFF          Unlimited Writes To RP Registers
     IOL1WAY_ON           Write RP Registers Once  ; default

   I2C1 pins Select:
     I2C1SEL_SEC          Use Secondary I2C1 pins
     I2C1SEL_PRI          Use Primary I2C1 pins  ; default

   Oscillator Selection:
     POSCMOD_EC           External clock
     POSCMOD_XT           XT oscillator
     POSCMOD_HS           HS oscillator
     POSCMOD_NONE         Primary disabled  ; default

}// */
    
int main (void) {
    initMain();
    initUSART();
    initBTModule(); // initialized Bluetooth module
    stateFlags.isRoused = 1;
    stateFlags.isLeveling = 0; // default until set
    stateFlags_2.turnSDCardOffBetweenDataWrites = 1; // default while testing
    initSPI();
    I2C_Init(); // enable I2C module

    // display Device ID and Revision 
    TBLPAG = DEVpage;
    DEVID = __builtin_tblrdl(DEVIDadr);
    DEVREV = __builtin_tblrdl(DEVREVadr);
    len = sprintf(str, "\n\r microcontroller Device ID: 0x%x, Revision 0x%x\n\r", (int)DEVID, (int)DEVREV);
    outputStringToUSART(str);
//    len = sprintf(str, "\n\r microcontroller Revision ID: 0x%x\n\r";
//    outputStringToUSART(str);

    initAccelerometer();

/*
//{
***** Enable Interrupts ***************
 INTCON1: INTERRUPT CONTROL REGISTER 1
bit 15 NSTDIS: Interrupt Nesting Disable bit
 1 = Interrupt nesting is disabled
 0 = Interrupt nesting is enabled
 other bits flag various errors, or are unimplemented
//}*/
 INTCON1 = 0; // enable interrupt nesting, clear other flags
/*
//{
 INTCON2: INTERRUPT CONTROL REGISTER 2
bit 15 ALTIVT: Enable Alternate Interrupt Vector Table bit
 1 = Use Alternate Interrupt Vector Table
 0 = Use standard (default) vector table (default)
bit 14 DISI: DISI Instruction Status bit (read only)
 1 = DISI instruction is active
 0 = DISI instruction is not active
bit 13-3 Unimplemented: Read as 0
bit 2 INT2EP: External Interrupt 2 Edge Detect Polarity Select bit
 1 = Interrupt on negative edge
 0 = Interrupt on positive edge (default)
bit 1 INT1EP: External Interrupt 1 Edge Detect Polarity Select bit
 1 = Interrupt on negative edge
 0 = Interrupt on positive edge (default)
bit 0 INT0EP: External Interrupt 0 Edge Detect Polarity Select bit
 1 = Interrupt on negative edge
 0 = Interrupt on positive edge (default)
//} */
 INTCON2 = 0;

 _RTCIF = 0; // clear RTCC interrupt flag
 _RTCIE = 1; // enable RTCC real time clock calendar interrupt

 _INT1IF = 0; // clear INT1 interrupt flag 
 _INT1IE = 1; // Enable INT1 external interrupt

 // init the processor priority level
 _IPL = 0; // not strictly necessary to do; 0 is the default value, set at power on
 
 flags1.useSDcard = 1; // can toggle this with the O (on/off) command
 flags1.sleepBetweenReadings = 0; // can toggle this with the S (on/off) command
 flags1.isDark = 0; // default is not Dark, set irradiance below threshold
 flags1.wasDark = 0; // previous Dark state
 flags1.writeDataHeaders = 1; // log column headers on 1st SD card write
 
    stateFlags.reRoused = 1; // start Roused for the default timeout before going to sleep
    machState = Idle;

    while (1) { // main program loop
	    while (stateFlags.isLeveling) { // show Leveling diagnostics
            while (*USART1_outputbuffer_head != *USART1_outputbuffer_tail) ; // in case something is printing
            for (unsignedIntTmp = ~0; (unsignedIntTmp); unsignedIntTmp--) ; // wait a while
	        readAccelerometer();
//	        outputStringToUSART("\r\n");
	        outputStringToUSART(str);
	    }   

        setSDCardPowerControl();
        while (machState == Idle) { // RTCC interrupt will break out of this
            setSDCardPowerControl();
            if (stateFlags.reRoused) {
                stateFlags.reRoused = 0; // clear this flag
                stateFlags.isRoused = 1;
                startTimer1ToRunThisManySeconds(30);
            }
            if (stateFlags.isRoused) { // timer1 timeout will turn off
                // clean up after any external interrupt
                clearAnyADXL345TapInterrupt(); // otherwise will keep repeating interrupt
                _INT1IF = 0; // clear INT1 interrupt flag 
                _INT1IE = 1; // Enable INT1 external interrupt; was disabled within ISR, to keep from repeating
                flags1.isDark = 0; // wake up, if asleep due to darkness
            }

            if (!stateFlags.timerHasBeenSynchronized) { // if alarm has not been matched to 0 of seconds
                _RTCPTR1 = 0; // point to  Minute/Second slot in RTCC registers
                _RTCPTR0 = 0;
                intTmp = RTCVAL;
                if ((char)(0xf & intTmp) + 0x30 == '0') { // see if we are on the 0 of seconds
                            // reset alarm to happen starting now
                            /*   bit 13-10 AMASK<3:0>: Alarm Mask Configuration bits, 0010 = Every 10 seconds
                                 111111 bit positions
                                 5432109876543210     */
                    setupAlarm(0b1100101111111111);
                    stateFlags.timerHasBeenSynchronized = 1; // flag done
                }
            }

//
        // do following on first call to SD card
        //if (_SPI1MD) // SPI module 1 was disabled to save power during sleep
        //     _SPI1MD = 0; // remove disable from SPI module 1

        // uncomment following lines if trying the explicit disable of these modules during Sleep
        //if (_U1MD) // UART module 1 was disabled to save power during sleep
        //     _U1MD = 0; // remove disable from UART module 1
        //if (_I2C2MD) // I2C module 2 was disabled to save power during sleep
        //     _I2C2MD = 0; // remove disable from I2C module 2

        checkForCommands();
//  if (flags1.isDark)
//   flags1.sleepBetweenReadings = 1;
//  else 
//   flags1.sleepBetweenReadings = 0;
  
//  if (flags1.isDark != flags1.wasDark) // if state has changed; change Alarm interval
  if (1) // currently, don't look at any previous state
  /*
  bit 15: 1 = enable Alarm
  bit 14: 1 = enable Chime, so alarm will roll over and continue after Repeat counts down to 0
  bit 9-8 ALRMPTR<1:0>: Alarm Value Register Window Pointer bits (don't care)
  bit 7-0 ARPT<7:0>: Alarm Repeat Counter, 11111111 = Alarm will repeat 255 more times
  see comments in setupAlarm fn for more details
  */
  {
   if (flags1.isDark) // if it has changed to Dark, set long intervals
/*   bit 13-10 AMASK<3:0>: Alarm Mask Configuration bits, 0101 = Every hour
                 111111 bit positions
                 5432109876543210     */
    setupAlarm(0b1101011111111111);
   else // Dark has ended, change to short intervals
/*   bit 13-10 AMASK<3:0>: Alarm Mask Configuration bits, 0010 = Every 10 seconds
                 111111 bit positions
                 5432109876543210     */
    setupAlarm(0b1100101111111111);
    
   // update flag
   if (flags1.isDark)
    flags1.wasDark = 1;
   else 
    flags1.wasDark = 0;
  } // end if (isDark != wasDark)
/**/
//    if (flags1.sleepBetweenReadings) { // go to sleep

    if (!stateFlags.isRoused) { // go to sleep
        outputStringToUSART("\r\n   system timout, going to sleep\r\n");
        assureSDCardIsOff();
        while (*USART1_outputbuffer_head != *USART1_outputbuffer_tail) ; // allow any output to finish
        // if following cause lockup, fix
        _U1MD = 1; // disable UART module 1
        //
        //_I2C2MD = 1; // disable I2C module 2
       Sleep();
       __asm__ ("nop");
    }
  
 } // end (machState == Idle)
 // when (machState != Idle) execution passes on from this point

 // when RTCC occurs, changes machState to GettingTimestamp
    while (1) // various test may break early, 
    {
        setSDCardPowerControl();
        // monitor cell voltage, to decide whether there is enough power to proceed
        cellVoltage = getCellVoltage();
        if (stateFlags.isRoused) { 
//            outputStringToUSART("\r\n system roused\r\n");
            // timer diagnostics

            len = sprintf(str, "\r\n timer 1: %u seconds\r\n", (int)(TMR1/128));
            outputStringToUSART(str);
        }
        createTimestamp();
        outputStringToUSART(timeStampBuffer);
        if (cellVoltage < CELL_VOLTAGE_THRESHOLD_READ_IRRADIANCE) { 
            len = sprintf(str, " power too low\n\r");
            outputStringToUSART(str);
            machState = Idle;
            break;
        }
//        len = sprintf(str, " char 17=%u, 17&0x01=%u, char 19=%u \n\r", timeStampBuffer[17], ((char)timeStampBuffer[17] & 0x01), timeStampBuffer[19]);
//        outputStringToUSART(str);
        if (((!((char)timeStampBuffer[17] & 0x01)) && ((char)timeStampBuffer[19] == '0')) || (flags1.isDark)) {
            // if an even number of minutes, and zero seconds
            // or the once-per-hour wakeup while dark
            stateFlags_2.isDataWriteTime = 1;
        } else {
            stateFlags_2.isDataWriteTime = 0;
        }
        if (stateFlags_2.isDataWriteTime) {
            assureDataHeadersLogged(); // writes headers on reset, time change, or midnight rollover
            err = writeCharsToFile (timeStampBuffer, 21);
            if (err) {
                tellFileWriteError (err);
                stateFlags_2.isDataWriteTime = 0; // prevent trying to write anything later
            }
        }
        machState = ReadingSensors;
//        getDataReadings(); // may make part or all of this a separate function later, but for now do straight-through
        // read broadband and infrared from down- and up-pointing sensors via I2C
       //  __asm__ volatile("disi #0x3FFF"); // temporarily disable interrupts
        // explicitly initialize data to defaults
        for (irrReadingNumber = 0; irrReadingNumber < 4; irrReadingNumber++) {
            irrReadings[irrReadingNumber].irrWholeWord = 0; // default value
            irrReadings[irrReadingNumber].irrMultiplier = 1; // default gain
        }
//        len = sprintf(str, " diagnostics\n\r");
//        outputStringToUSART(str);
        len = sprintf(str, " cell voltage %u\n\r", cellVoltage);
        outputStringToUSART(str);

        I2C_Init(); // enable I2C module
        // first few Bus Collision tests should be sufficient to determine if I2C is working
        if (I2C2STATbits.BCL)
        {
            len = sprintf(str, " I2C bus collision during Init\n\r");
            outputStringToUSART(str);
            machState = Idle;
            break;
        }
        I2C_Stop(); // create a Stop condition on the I2C bus
        if (I2C2STATbits.BCL)
        {
            len = sprintf(str, " I2C bus collision during Stop\n\r");
            outputStringToUSART(str);
            machState = Idle;
            break;
        }
        for (irrSensorNumber = 0; irrSensorNumber < 2; irrSensorNumber++) {
        // 0 = down-looking sensor, 1 = up-looking sensor
             if (irrSensorNumber == 0) {
                 irrSensorWriteAddr = 0x52;
                 irrSensorReadAddr = 0x53;
             } else {
                 irrSensorWriteAddr = 0x92;
                 irrSensorReadAddr = 0x93;
             }
            I2C_Start(); // create a Start condition on the I2C bus
            if (I2C2STATbits.BCL)
            {
                len = sprintf(str, " I2C bus collision during Start\n\r");
                outputStringToUSART(str);
                machState = Idle;
                break;
            }
            r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
//            d = irrSensorWriteAddr; // address the device, say we are going to write
//            r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
            if (!r) { // device did not acknowledge
                I2C_Stop();
                if (irrSensorWriteAddr == 0x52)
                    len = sprintf(str, " Down-pointing sensor not present, or not responding\n\r");
                else
                    len = sprintf(str, " Up-pointing sensor not present, or not responding\n\r");
                outputStringToUSART(str);
                continue;
            }
            d = 0x8A; // write; a byte command, setting the register to "ID" (Part number, Rev ID) = 0x0a
            r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
            r = I2C_Write(irrSensorReadAddr);
            I2C_ReStart(); // restart, preparatory to reading
            r = I2C_Read(0); // do NACK, since this is the last and only byte read
            I2C_Stop();
// len = sprintf(str, " result of reading part ID: 0x%x\n\r", r);
// outputStringToUSART(str);
            //if ((r & 0xF0) == 0x50) { // part number for TSL2561 (datasheet says 0x1n, but actually reads 0x5n)
// len = sprintf(str, " matched correct part number\n\r");
// outputStringToUSART(str);
                for (irrChannelNumber = 0; irrChannelNumber < 2; irrChannelNumber++) {
                    // 0 = broadband channel, 1 = infrared channel
// len = sprintf(str, " entered FOR loop for channels\n\r");
// outputStringToUSART(str);
                    if (irrChannelNumber == 0)
                        irrChannel = 0xAC;
                    else
                        irrChannel = 0xAE;
                    irrReadingNumber = irrChannelNumber + (irrSensorNumber + irrSensorNumber);
                    irrReadings[irrReadingNumber].irrWholeWord = 0; // default value
                    irrReadings[irrReadingNumber].irrMultiplier = 1; // default gain

                    // start with high gain and high integration time
                    stateFlags_2.irrSensorGain = 1;
                    stateFlags_2.irrSensorIntegrationPeriodHi = 1;
                    stateFlags_2.irrSensorIntegrationPeriodLo = 0;
                    while (1) { // read sensor, adjusting gain and sensitivity till OK or topped out
                        // power up device
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
                        d = 0x80; // write; a byte command, setting the register to CONTROL = 0x00
                        r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                        d = 0x03; // write to CONTROL, power-up code
                        r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                        I2C_Stop();
                        // set up gain and integration time, prep to read sensor
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
                        d = 0x81; // write; a byte command, setting the register to TIMING = 0x01
                        r = I2C_Write(d);
                        d = 0x00;
                        if (stateFlags_2.irrSensorGain)
                            d |= 0x10; // set high gain bit
                        if (stateFlags_2.irrSensorIntegrationPeriodHi)
                            d |= 0x02; // irrSensorIntegrationPeriod code 11 is disallowed so this is unambiguous
                        else {
                            if (stateFlags_2.irrSensorIntegrationPeriodLo)
                                d |= 0x01; // irrSensorIntegrationPeriod code 01, medium
                        }
// len = sprintf(str, "\n\r TIMING code, d = 0x%x\n\r", d);
// outputStringToUSART(str);

//
//                        // set sensor gain 0=low, 1=high
//                        if (stateFlags_2.irrSensorGain)
//                            d = 0x12; // write to TIMING, high gain, long integration time
//                        else
//                            d = 0x10; 
                        r = I2C_Write(d); // write to TIMING, the gain and integration time
                        I2C_Stop();
                        // get reading
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr);
// d = irrSensorWriteAddr;
// r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                        r = I2C_Write(irrChannel);
// d = irrChannel;
// r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                        for (intTmp = 1; intTmp < 100; intTmp++) {  // poll device up to 100 times
                            I2C_ReStart(); // restart, preparatory to reading
                            r = I2C_Write(irrSensorReadAddr); // address the device, say we are going to read
// d = irrSensorReadAddr;
// r = I2C_Write(d);
// len = sprintf(str, " result of sending 0x%x = %x\n\r", d, r);
// outputStringToUSART(str);
                            irrReadings[irrReadingNumber].irrLoByte = I2C_Read(1); // do ACK, because not last byte
                            irrReadings[irrReadingNumber].irrHiByte = I2C_Read(0); // do NACK, since this is the last byte
                            I2C_Stop();
// len = sprintf(str, "\n\r after %i tries, reading[%u] = %u\n\r", intTmp, irrReadingNumber, irrReadings[irrReadingNumber].irrWholeWord);
// outputStringToUSART(str);
                            if ((irrReadings[irrReadingNumber].irrWholeWord > 0))
                                break;
                            for (unsignedIntTmp = ~0; (unsignedIntTmp); unsignedIntTmp--) ; // wait a while
                            while (*USART1_outputbuffer_head != *USART1_outputbuffer_tail) ; // in case something is printing
                        } // end of polling FOR loop, either zero or value
                        I2C_Stop();
 //len = sprintf(str, "\n\r exited FOR loop, reading[%u] = %u, Gain = %c, IntegrCode = %c%c\n\r", 
 //         irrReadingNumber, irrReadings[irrReadingNumber].irrWholeWord, (stateFlags_2.irrSensorGain ? '1' : '0'),
 //         (stateFlags_2.irrSensorIntegrationPeriodHi ? '1' : '0'), 
 //         (stateFlags_2.irrSensorIntegrationPeriodLo ? '1' : '0'));
 //outputStringToUSART(str);
                        // turn off device
                        I2C_Start();
                        r = I2C_Write(irrSensorWriteAddr); // address the device, say we are going to write
                        d = 0x80; // write; a byte command, setting the register to CONTROL = 0x00
                        r = I2C_Write(d);
                        d = 0x00; // write to CONTROL, power-down code
                        r = I2C_Write(d);
                        I2C_Stop();
                       // exit loop here, one way or another
                        if (irrReadings[irrReadingNumber].irrWholeWord < 0xFFFF)
                            break; // if zero (dark) or valid reading less than topped out at 2^16-1
                        if ((!stateFlags_2.irrSensorGain) && 
                                  (!stateFlags_2.irrSensorIntegrationPeriodHi) && 
                                  (!stateFlags_2.irrSensorIntegrationPeriodLo))
                            break; // also if topped out but we have done everything possible,
                                   // low gain and minimum integration period
                        // if we didn't exit, adjust bits, try not to top out
                        if (!stateFlags_2.irrSensorGain) { // if gain is already low
                            if (stateFlags_2.irrSensorIntegrationPeriodHi) { // integ period high, code 10
                                stateFlags_2.irrSensorIntegrationPeriodHi = 0; // set to medium code 01
                                stateFlags_2.irrSensorIntegrationPeriodLo = 1;
                            } else { // integ period is medium or low, 01 or 00
                                if (stateFlags_2.irrSensorIntegrationPeriodLo) // if medium, code 01
                                    stateFlags_2.irrSensorIntegrationPeriodLo = 0; // set to low, code 00
                            }
                        }
                        if (stateFlags_2.irrSensorGain) // if gain is high
                            stateFlags_2.irrSensorGain = 0; // set gain low
                    } // end of attempts at reading device, either OK or topped out, or zero
                    // calculate multiplier
                    // if high integration time, code 10, leave as default for now
                    if (!stateFlags_2.irrSensorIntegrationPeriodHi) {
                        if (stateFlags_2.irrSensorIntegrationPeriodLo) // medium, code 01
                            irrReadings[irrReadingNumber].irrMultiplier = 4; // (322/81), 0.62% error
                        else // low, code 00
                            irrReadings[irrReadingNumber].irrMultiplier = 29; // (322/11), 0.93% error
                    }
                    if (!stateFlags_2.irrSensorGain) // if gain is low
                        irrReadings[irrReadingNumber].irrMultiplier *= 16; // multiply by 16

                } // end of bb or ir channels
             
            //} // end of if correct device part number
            // else {} // determine if TSL2581 and work with that

        } // end of up- or down-looking device

//    I2C2CONbits.I2CEN = 0; // disable I2C module
// diagnostic for testing
//len = sprintf(str, "\r\n %u\t%u\r\n ", irrReadings[0].irrWholeWord, irrReadings[0].irrMultiplier);
//outputStringToUSART(str);

    // prepare data string
    len = sprintf(str, "\t%lu\t%lu\t%lu\t%lu\t%u",
          (unsigned long)((unsigned long)irrReadings[0].irrWholeWord * (unsigned long)irrReadings[0].irrMultiplier),
          (unsigned long)((unsigned long)irrReadings[1].irrWholeWord * (unsigned long)irrReadings[1].irrMultiplier),
          (unsigned long)((unsigned long)irrReadings[2].irrWholeWord * (unsigned long)irrReadings[2].irrMultiplier),
          (unsigned long)((unsigned long)irrReadings[3].irrWholeWord * (unsigned long)irrReadings[3].irrMultiplier),
           cellVoltage);
    outputStringToUSART(str);
    // if broadband reference is less than threshold, flag that it is dark enough
    //   to go to sleep, to save power overnight
    if ((unsigned long)((unsigned long)irrReadings[2].irrWholeWord * (unsigned long)irrReadings[2].irrMultiplier) < 
             bbIrradThresholdDark)
        flags1.isDark = 1;
    else
        flags1.isDark = 0;
    if (stateFlags_2.isDataWriteTime) {
        // log data to SD card
        err = writeCharsToFile (str, len);
        if (err)
            tellFileWriteError (err);
        outputStringToUSART("\r\n SD card procedure done\r\n");
    }
/*    if (stateFlags.isLeveling) { // show Leveling diagnostics
        readAccelerometer();
        outputStringToUSART("\r\n");
        outputStringToUSART(str);
    }   */
    machState = Idle; // done with everything, return to Idle state
        break; // if did everything, break here
    } // end of getting data readings
    ;
 } // main program loop
} // fn Main

/*****************************************
* set the RTCC Alarm register
*  options include setting automatic interval
*  
*****************************************/
void setupAlarm (int N)
{
 // set up RTCC alarm & interrupts
 // unlock the RTCC registers, including alarm register ALCFGRPT
 //__builtin_write_RTCWEN; // maybe try this format
 __asm__ ("mov #NVMKEY,W0");
 __asm__ ("mov #0x55,W1");
 __asm__ ("mov #0xAA,W2");
 __asm__ ("mov W1,[W0]");
 __asm__ ("nop");
 __asm__ ("mov W2,[W0]");
 __asm__ ("bset RCFGCAL,#13");
 __asm__ ("nop");
 __asm__ ("nop");
 
 /*
 bit 15 ALRMEN: Alarm Enable bit
  1 = Alarm is enabled (cleared automatically after an alarm event whenever ARPT<7:0> = 00 and CHIME = 0)
  0 = Alarm is disabled
 bit 14 CHIME: Chime Enable bit
  1 = Chime is enabled; ARPT<7:0> bits are allowed to roll over from 00h to FFh
  0 = Chime is disabled; ARPT<7:0> bits stop once they reach 00h
 bit 13-10 AMASK<3:0>: Alarm Mask Configuration bits
  0000 = Every half second
  0001 = Every second
  0010 = Every 10 seconds
  0011 = Every minute
  0100 = Every 10 minutes
  0101 = Every hour
  0110 = Once a day
  0111 = Once a week
  1000 = Once a month
  1001 = Once a year (except when configured for February 29th, once every 4 years)
  101x = Reserved ? do not use
  11xx = Reserved ? do not use
  ( default, use 0010, mask for every 10 seconds)
 bit 9-8 ALRMPTR<1:0>: Alarm Value Register Window Pointer bits
 Points to the corresponding Alarm Value registers when reading ALRMVALH and ALRMVALL
 registers; the ALRMPTR<1:0> value decrements on every read or write of ALRMVALH until it reaches 00.
 ALRMVAL<15:8>:
  00 = ALRMMIN
  01 = ALRMWD
  10 = ALRMMNTH
  11 = Unimplemented
 ALRMVAL<7:0>:
  00 = ALRMSEC
  01 = ALRMHR
  10 = ALRMDAY
  11 = Unimplemented
 bit 7-0 ARPT<7:0>: Alarm Repeat Counter Value bits
  11111111 = Alarm will repeat 255 more times
  ...
  00000000 = Alarm will not repeat
 The counter decrements on any alarm event. The counter is prevented from rolling over from 00h to
 FFh unless CHIME = 1.
*/

 N = safely_set_ALCFGRPT(N); // setup Alarm register in an assember fn
 _RTCWREN = 0; // relock RTCC registers, to protect values from accidental change
}

/*****************************************
* write N characters from the passed char
*  pointer to the file specified by the
*  dir and filename buffers
*****************************************/
int writeCharsToFile (char* St, int N) {
    FSFILE * pointer;
    if (!flags1.useSDcard)
        return IgnoreCard; // flag is set to ignore SD card
    if (getCellVoltage() < CELL_VOLTAGE_THRESHOLD_WRITE_SD_CARD)
        return PowerTooLowForSDCard;
    assureSDCardIsOn();
    if (!MDD_MediaDetect())
        return NoCard; // SD card not present or not detected
    // Initialize the library
    if (!FSInit())
        return NoInit; // could not initialize file system;
    // Create a directory if it doesn't already exist
    if (FSmkdir (dirNameBuffer))
        return NoMkDir; // could not create the requested directory
    // Change to the directory
    if (FSchdir (dirNameBuffer + 1))
        return NoChDir; // could not change to the requested directory
    // Create a file
    pointer = FSfopen (fileNameBuffer, "a");
    if (pointer == NULL)
        return NoFileOpen; // could not open the requested file
    // Write objects from pointer into the file
    if (FSfwrite (St, 1, N, pointer) != N)
        return NoFileWrite; // could not write to the file
    if (FSfseek(pointer, 0, SEEK_END))
        return NoFileSeek; // could not seek as requested
    // Close the file
    if (FSfclose (pointer))
        return NoClose; // could not close file 
    return NoProblem;
}


/*****************************************
* send diagnostic message about file
*  write error out the USART
*****************************************/
void tellFileWriteError (int err)
{
 switch (err) {
  case IgnoreCard: {
   outputStringToUSART("\r\n SD card ignored (\"O\" toggles)\r\n");
   break;
  }
  case PowerTooLowForSDCard: {
   outputStringToUSART("\r\n power too low, SD write skipped\r\n");
   break;
  }
  case NoCard: {
   outputStringToUSART("\r\n SD card not present or not detected\r\n");
/*   {
    extern MMC_RESPONSE SendMMCCmd(BYTE cmd, DWORD address);
//    extern void Delayms(BYTE milliseconds);
    char str[50];
    int l;
    MMC_RESPONSE    response;
    l = sprintf(str, "Write protect state = %i\r\n", MDD_SDSPI_WriteProtectState());
    outputStringToUSART(str);
    l = sprintf(str, "SPIENABLE state = %i\r\n", (int)SPIENABLE);
    outputStringToUSART(str);
    response = SendMMCCmd(SEND_STATUS,0x0);
    l = sprintf(str, "card response to SEND_STATUS = %x\r\n", response.r2._word);
    outputStringToUSART(str);
    SD_CS = 0;
    Delayms(1);
    l = sprintf(str, "result of setting SD_CS to 0: = %i\r\n", (int)SD_CS);
    outputStringToUSART(str);
    SD_CS = 1;
    Delayms(1);
    l = sprintf(str, "result of setting SD_CS to 1: = %i\r\n", (int)SD_CS);
    outputStringToUSART(str);

//    MDD_SDSPI_MediaInitialize();
//    l = sprintf(str, "MDD_SDSPI_MediaInitialize errorcode = %x\r\n", mediaInformation.errorCode);
//    outputStringToUSART(str);
   }
*/

   break;
  }
  case NoInit: {
   outputStringToUSART("\r\n could not initialize file system\r\n");
   break;
  }
  case NoMkDir: {
   outputStringToUSART("\r\n could not create the requested directory\r\n");
   break;
  }
  case NoChDir: {
   outputStringToUSART("\r\n could not change to the requested directory\r\n");
   break;
  }
  case NoFileOpen: {
   outputStringToUSART("\r\n could not open the requested file\r\n");
   break;
  }
  case NoFileWrite: {
   outputStringToUSART("\r\n could not write to the file\r\n");
   break;
  }
  case NoFileSeek: {
   outputStringToUSART("\r\n could not seek as requested\r\n");
   break;
  }
  case NoClose: {
   outputStringToUSART("\r\n could not close file\r\n");
   break;
  }
  default: {
   outputStringToUSART("\r\n unknown error\r\n");
   break;
  } 
 } // switch err
} // tellFileWriteError
/*****************************************
* copy characters from the passed null-terminated
*  string to the USART output buffer and
*  flag to start transmitting
*****************************************/
void outputStringToUSART (char* St) {
    char *tmp;
    if (!stateFlags.isRoused) // if system is asleep, or about to go to sleep
        return; // don't bother with any output till wakened
    if (_U1MD) { // UART module 1 was disabled to save power during sleep
        _U1MD = 0; // remove disable from UART module 1
        initUSART();
        __asm__ ("nop"); // allow time for wakeup
        __asm__ ("nop");
    }
    _U1TXIE = 1; // enable Tx interrupt so all bytes in buffer will be sent
    for (; *St != '\0'; St++) { // make sure there is room in USART1_outputbuffer
        __asm__ volatile("disi #0x3FFF"); // disable interrupts while we check & update the buffer
        tmp = (char*)(USART1_outputbuffer_head + 1);
        if (tmp >= (&USART1_outputbuffer[0] + USART1_Buffer_Length))
            tmp = (char*)(&USART1_outputbuffer[0]); // rollover to beginning if necessary
        // head=tail is the test for a full buffer
        if (tmp != USART1_outputbuffer_tail) { // there is room in USART1_outputbuffer
            *USART1_outputbuffer_head = *St;
            USART1_outputbuffer_head = (char*)tmp;
        }
        __asm__ volatile("disi #0x0000"); // enable interrupts
        _U1TXIE = 1; // enable Tx interrupt; interrupt will then occur as soon as Transmit 
        //  buffer has moved contents into Transmit Shift Register
    }
} 


/*****************************************
* check USART receive buffer for commands
*  
*  
*****************************************/
void checkForCommands (void) {
    char c;
    while (1) {
        __asm__ volatile("disi #0x3FFF"); // disable interrupts before first pointer test
        //  if Rx ISR called while buffer pointers in the works, will cause glitches
        if (USART1_inputbuffer_head == USART1_inputbuffer_tail) { // no (more) chars in buffer
            __asm__ volatile("disi #0x0000"); // re-enable interrupts before exit
            return;   
        }
        c = *USART1_inputbuffer_tail++; // get char & start generating new tail by incrementing
        if (USART1_inputbuffer_tail >= &USART1_inputbuffer[0] + USART1_Buffer_Length) // if end of buffer
            USART1_inputbuffer_tail = &USART1_inputbuffer[0]; // rollover to start
        __asm__ volatile("disi #0x0000"); // done updating pointers, can re-enable interrupts
        if (c == 0x0d) // if carriage-return
            c = 0x0a; // substitute linefeed
        if (c == 0x0a) { // if linefeed, attempt to parse the command
            *commandBufferPtr++ = '\0'; // null terminate
            switch (commandBuffer[0]) { // command is 1st char in buffer
//                case 0x0a: {
//                    break; // ignore multiple linefeeds
//                }
                case 'T': { // set time
                    if (!isValidTimestamp(commandBuffer + 1)) {
                         outputStringToUSART("\r\n Invalid timestamp\r\n");
                         break;
                    }    
                    outputStringToUSART("\r\n Time changed from ");
                    createTimestamp();
                    outputStringToUSART(timeStampBuffer + 2);
                    setTimeFromCharBuffer(commandBuffer + 3);
                    outputStringToUSART(" to ");
                    createTimestamp();
                    outputStringToUSART(timeStampBuffer + 2);
                    outputStringToUSART("\r\n");
                    flags1.writeDataHeaders = 1; // log data column headers on next SD card write
                    stateFlags.timeHasBeenSet = 1; // presumably is now the correct time
                    stateFlags.timerHasBeenSynchronized = 0; // but timer not guaranteed to happen on 0 of seconds
                    startTimer1ToRunThisManySeconds(30); // keep system Roused
                    break;
                }
                case 'O': { // toggle to use SD card, or ignore it
                    if (flags1.useSDcard) {
                        flags1.useSDcard = 0;
                        outputStringToUSART("\r\n Will now ignore SD card\r\n");
                    } else {
                        flags1.useSDcard = 1;
                        outputStringToUSART("\r\n Will now write to SD card\r\n");
                    }
                    break;
                }

                case 'P': { // SD card power control
                    if (stateFlags_2.turnSDCardOffBetweenDataWrites) {
                        stateFlags_2.turnSDCardOffBetweenDataWrites = 0;
                        outputStringToUSART("\r\n No SD card power control \r\n");
                    } else { 
                        stateFlags_2.turnSDCardOffBetweenDataWrites = 1;
                        SD_POWER_TRIS = 0; // assure the pin is an output, source of the SD card power under software control
                    }
                    setSDCardPowerControl(); // based on flag
                    break;
                } 

                case 'L': { // toggle Leveling diagnostics
                    if (stateFlags.isLeveling) {
                        stateFlags.isLeveling = 0;;
                        outputStringToUSART("\r\n Ignore accelerometer output\r\n");
                    } else {
                        stateFlags.isLeveling = 1;
                        outputStringToUSART("\r\n Accelerometer output enabled\r\n");
                    }
                    break;
                } 

                case 'S': { // toggle to sleep between readings, or not
                    if (flags1.sleepBetweenReadings) {
                        flags1.sleepBetweenReadings = 0;
                        outputStringToUSART("\r\n Will stay awake continually\r\n");
                    } else {
                        flags1.sleepBetweenReadings = 1;
                        outputStringToUSART("\r\n Will now sleep between readings\r\n");
                    }
                    break;
                } 

                case 'R': { // test the 'Rouse' bit
                    if (stateFlags.isRoused) {
                        stateFlags.isRoused = 0;
                        outputStringToUSART("\r\n Rouse bit cleared\r\n");
                    } else {
                        stateFlags.isRoused = 1;
                        outputStringToUSART("\r\n Rouse bit set\r\n");
                    }
                    break;
                } 

/*
                case 'G': { // toggle irradiance gain
                    if (stateFlags_2.irrSensorGain) {
                        stateFlags_2.irrSensorGain = 0;
                        outputStringToUSART("\r\n Low gain\r\n");
                    } else {
                        stateFlags_2.irrSensorGain = 1;
                        outputStringToUSART("\r\n High gain\r\n");
                    }
                    break;
                } 
*/
                // put other commands here
                default: { // if no valid command, echo back the input
                    outputStringToUSART("\r\n> ");
                    outputStringToUSART(commandBuffer);
                    outputStringToUSART("\r\n");
                    startTimer1ToRunThisManySeconds(30); // keep system Roused another two minutes
                }
            } // switch (commandBuffer[0])
            commandBuffer[0] = '\0';
            commandBufferPtr = commandBuffer; // "empty" the command buffer
         } else { // some other character
             // ignore repeated linefeed (or linefeed following carriage return) or carriage return
             if (!((c == 0x0a) || (c == 0x0a))) { 
                 if (commandBufferPtr < (commandBuffer + commandBufferLen - 1)) // if there is room
                     *commandBufferPtr++ = c; // append char to the command buffer
             }
         } // done parsing character
    } // while (1)
} // checkForCommands

/*****************************************
* set up app specifics such as IO, peripheral pin mapping, etc.
*****************************************/
void initMain (void)
{
/*
 set up ports
*/
 ODCA = 0; //no open drain outputs
 ODCB = 0;
/*
//{ disable analog on all except AN0, physical pin 2
 usually safe to set all digital bits to 1
  in some silcon revisions, in chips that do not have AN13 and AN14 (including 28-pin PIC24FJ64GA002), 
  setting those bits can allow unimplemented channels to be selected //}
             111111 bit positions
             5432109876543210     */
 AD1PCFG = 0b1001111111111110;
/*
//set bit 3 of Port A as output, all others as inputs
//bit 3, physical pin 10; RTCC interrupt toggles it for diagnostic
//       111111 bit positions
//       5432109876543210
// mov #0b1111111111110111, W0

  Port A
 bit 0, physical pin 2
 bit 1, physical pin 3, power supply for I2C bus
 bit 2, physical pin 9
 bit 3, physical pin 10
 bit 4, physical pin 12, RTCC xtal SOSCO, unavaliable for I/O
 bit 5 & up do not exist in 28-DIP pin package

;          111111 bit positions
;          5432109876543210     */
 TRISA = 0b1111111111111111;  // for testing, leave bit1, pin3 an input and hardwire the power
// TRISA = 0b1111111111111101;

/*
//{  Port B
 bit 0, physical pin 4
 bit 1, physical pin 5
 bit 2, physical pin 6; I2C port #2, SDA2, must be input or will override I2C and cause bus collisions
 bit 3, physical pin 7; I2C port #2, SCK2, must be input or will override I2C and cause bus collisions
 bit 4, physical pin 11; RTCC xtal SOSCI, unavaliable for I/O
 bit 5, physical pin 14, varies by BOARD_VERSION, see HardwareProfile.h
 bit 6, physical pin 15, UART RTS output --- UNUSED ---
 bit 7, physical pin 16, UART CTS input --- UNUSED ---
 bit 8, physical pin 17, UART RX input
 bit 9, physical pin 18, UART TX output
// following vary by BOARD_VERSION, see HardwareProfile.h
 bit 10, physical pin 21
 bit 11, physical pin 22
 bit 12, physical pin 23
 bit 13, physical pin 24
 bit 14, physical pin 25
     pin 25 can be temporarily set to Output, to use for RTCC diagnostics
 bit 15, physical pin 26
//}
           111111 bit positions
           5432109876543210     */
 TRISB = 0b1111110101111111; // SPI pins default to inputs
// TRISB = 0b0101010100111111; // this way defintely works, with 13 (SPI SCK) and 15 (SPI CS) explicitly outputs
// TRISB = 0b0101010110111111; // when using CTS & RTX

    // set outputs here based on BOARD_VERSION (see HardwareProfile.h)
    SD_POWER_TRIS = 0;
    SPICLOCK = 0;
    SD_CS_TRIS = 0;

    SD_POWER = 1; // turn SD card power on

// _LATB2 = 0;
// _LATB3 = 0;
_RB6 = 0; // default low, minimize power
_RB7 = 0; // default low, minimize power


    PPSUnLock; // unlock peripheral pin select registers
    // map INT1, wake-up sent by the accelerometer, when it detects a double-tap
    #ifdef BOARD_VERSION_00
        CONFIG_INT1_TO_RP(5); // map INT1 to pin RP5, physical pin 14
    #endif
    #ifdef BOARD_VERSION_01
        CONFIG_INT1_TO_RP(5); // map INT1 to pin RP5, physical pin 14
    #endif
    #ifdef BOARD_VERSION_02
        CONFIG_INT1_TO_RP(15); // map INT1 to pin RP15, physical pin 26
    #endif 
    PPSLock; // lock registers back again

   // Turn on the secondary oscillator
   //__builtin_write_OSCCONL(0x02); // maybe try this format
   __asm__ ("MOV #OSCCON,w1");
   __asm__ ("MOV.b #0x02, w0");
   __asm__ ("MOV #0x46, w2");
   __asm__ ("MOV #0x57, w3");
   __asm__ ("MOV.b w2, [w1]");
   __asm__ ("MOV.b w3, [w1]");
   __asm__ ("MOV.b w0, [w1]");

   // Activate the RTCC module
   // set default time
   setTimeFromCharBuffer(timeStampBuffer + 4);
    stateFlags.timeHasBeenSet = 0; // default set, but not correct time
   
   // set up RTCC alarm & interrupts
   //__builtin_write_RTCWEN; // maybe try this format
   __asm__ ("mov #NVMKEY,W0");
   __asm__ ("mov #0x55,W1");
   __asm__ ("mov #0xAA,W2");
   __asm__ ("mov W1,[W0]");
   __asm__ ("nop");
   __asm__ ("mov W2,[W0]");
   __asm__ ("bset RCFGCAL,#13");
   __asm__ ("nop");
   __asm__ ("nop");

// to test when clock is running, enable output on the RTCC pin, physical pin 25
//_RTCOE = 1; // enable output on pin
//_RTSECSEL = 1; // direct seconds clock (rather than alarm) to output
//_RTSECSEL = 0; // direct alarm (rather than seconds clock) to output

  _RTCEN = 1; // start the clock
  _RTCWREN = 0; // relock RTCC registers, to protect values from accidental change
  _RTCIP = 4; // use priority 4, default between 0 (off) and 7 (high)

// at this point, clock should be running, but no output yet
// set up Alarm
/*
 default, set up mask to match on 10 seconds, then repeat alarm every 10 seconds with no stop
 bit 15: 1 = enable Alarm
 bit 14: 1 = enable Chime, so alarm will roll over and continue after Repeat counts down to 0
 bit 13-10 AMASK<3:0>: Alarm Mask Configuration bits, 0010 = Every 10 seconds
 bit 9-8 ALRMPTR<1:0>: Alarm Value Register Window Pointer bits (don't care)
 bit 7-0 ARPT<7:0>: Alarm Repeat Counter, 11111111 = Alarm will repeat 255 more times
 see comments in setupAlarm fn for more details
*/
/*            111111 bit positions
              5432109876543210     */
 setupAlarm(0b1100101111111111);

// set up I2C
// set SCK rate, 100 kHz
 I2C1BRG = I2C100kHz;
// set defaults; will set MSB to enable, don't do it yet
// I2C1CON = 0x00;


    _INT1EP = 0; // set INT1 to occur on rising edge (may not be necessary as this is the default)
    _INT1IF = 0; // clear INT1 interrupt flag 
    _INT1IE = 1; // enable external interrupt on pin 14, INT1

} // initMain

/*****************************************
* make sure the SD card is ON
*****************************************/

void assureSDCardIsOn (void) {
    int lenLocal;
    char strLocal[50];

    if (stateFlags_2.turnSDCardOffBetweenDataWrites) { // otherwise, assume power is hardwired, no software control
        SD_POWER_TRIS = 0; // assure the pin is an output
        if (SD_POWER) { // card power is NOT off, give message
            lenLocal = sprintf(strLocal, "\n\r SD card power is on\n\r");
            outputStringToUSART(strLocal);
        }
        if (!SD_POWER) { // card power is off, turn on
//            lenLocal = sprintf(strLocal, "\n\r about to turn SD card power on\n\r");
//            outputStringToUSART(strLocal);
            SD_POWER = 1; // turn SD card power on
            for (unsignedIntTmp = 4000; (unsignedIntTmp); unsignedIntTmp--) {
                __asm__ ("nop");
                __asm__ ("nop");
            }
            lenLocal = sprintf(strLocal, "\n\r SD card powered on\n\r");
            outputStringToUSART(strLocal);
            // wait about a millisecond to stabilize
//            lenLocal = sprintf(strLocal, "\n\r after delay following SD card power on\n\r");
//            outputStringToUSART(strLocal);
        } 
    } else {
        SD_POWER_TRIS = 1; // assure the pin is an input, not involved in power control
    }

    if (_SPI1MD) { // SPI module 1 was disabled to save power during sleep
//        lenLocal = sprintf(strLocal, "\n\r SPI was disabled to save power during sleep\n\r");
//        outputStringToUSART(strLocal);
         _SPI1MD = 0; // remove disable from SPI module 1
         __asm__ ("nop");
         __asm__ ("nop");
         __asm__ ("nop");
//        lenLocal = sprintf(strLocal, "\n\r SPI re-enabled\n\r");
//        outputStringToUSART(strLocal);
        initSPI();
        lenLocal = sprintf(strLocal, "\n\r SPI re-initialized\n\r");
        outputStringToUSART(strLocal);
        for (unsignedIntTmp = 4000; (unsignedIntTmp); unsignedIntTmp--) {
            __asm__ ("nop");
            __asm__ ("nop");
        }
    }
} // end of assureSDCardIsOn

/*****************************************
* make sure the SD card is OFF
*****************************************/

void assureSDCardIsOff (void) {
    _SPI1MD = 1; // disable SPI module 1
    // power down SD card, uses ~2.16mA quiescent when on
    if (stateFlags_2.turnSDCardOffBetweenDataWrites) {
        while (SD_POWER) // can take multiple tries due to capacitance
            SD_POWER = 0; // turn off power to SD card
    }
    SPICLOCK = 1; // explicitly make SCK an input, no power
    SD_CS_TRIS = 1; // explicitly make CS an input, no power
    // see if this works
    // following 2 lines without preceeding 2 lines causes failure to find SD card
    // disable weak pull-ups
    PULL_UP_SDI = 0;
    PULL_UP_CS = 0;

} // end of assureSDCardIsOff


/*****************************************
* clear any interrupt from the ADXL345 Accelerometer
*   uses I2C bus, and that should be initialized and stopped before calling this fn
*****************************************/

void clearAnyADXL345TapInterrupt (void) {
    char r;
    if (!stateFlags.accelerometerIsThere) {
        return;
    }
    do {
        // clear the interrupt by reading the INT_SOURCE register
        I2C_Start();
        r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
        r = I2C_Write(ADXL345_REG_INT_SOURCE); // tell the device the register we are going to want
        I2C_ReStart(); // restart, preparatory to reading
        r = I2C_Write(ADXL345_ADDR_READ); // address the device, say we are going to read
        r = I2C_Read(0); // do NACK, since this is the last and only byte read
        I2C_Stop();
// len = sprintf(str, " result of clearing ADXL345 interrupt: 0x%x\n\r", r);
// outputStringToUSART(str);
    } while (r & 0x40); // tap = 0x40, double tap = 0x20
} // end of clearAnyADXL345TapInterrupt

/*****************************************
* read Accellerometer, for system leveling
*****************************************/

void readAccelerometer (void) {
    char r, x0, x1, y0, y1, z0, z1; // accelerometer readings
    if (!stateFlags.accelerometerIsThere) {
        // globals "len" and "str" available on return
        len = sprintf(str, "\n\r no accelerometer");
        return;
    }
    // bring out of low power mode
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
    r = I2C_Write(ADXL345_REG_BW_RATE); // tell the device the register we are going to want
    r = I2C_Write(0x0a); // use 100Hz for now ; bit 4 set = reduced power, higher noise
    I2C_Stop();
    // read data
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
    r = I2C_Write(ADXL345_REG_DATAX0); // tell the device the register we are going to want
    I2C_ReStart(); // restart, preparatory to reading
    r = I2C_Write(ADXL345_ADDR_READ); // address the device, say we are going to read
    x0 = I2C_Read(1); // do ACK, since not the last byte
    x1 = I2C_Read(1); // do ACK, since not the last byte
    y0 = I2C_Read(1); // do ACK, since not the last byte
    y1 = I2C_Read(1); // do ACK, since not the last byte
    z0 = I2C_Read(1); // do ACK, since not the last byte
    z1 = I2C_Read(0); // do NACK, since this is the last byte
    I2C_Stop();
    // globals "len" and "str" available on return
    len = sprintf(str, "\n\r X = %i, Y = %i, Z = %i\n\r", (unsigned int)((int)x1 << 8 | (int)x0),
              (unsigned int)((int)y1 << 8 | (int)y0),  (unsigned int)((int)z1 << 8 | (int)z0));
    // put back in low power mode
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
    r = I2C_Write(ADXL345_REG_BW_RATE); // tell the device the register we are going to want
//                d = 0x16; // set low power bit (4) and slowest sampling rate, 6.25Hz, for 40uA current
    r = I2C_Write(0x18); // set low power bit (4) and 25Hz sampling rate, for 40uA current
    I2C_Stop();
} // end of readAccelerometer

/*****************************************
* try to detect whether Accellerometer is there, and functioning
* returns with global "stateFlags.accelerometerIsThere" set or cleared
*****************************************/
void findAccelerometer (void) {
    char r;
    len = sprintf(str, "\n\r entered findAccelerometer routine \n\r");
    outputStringToUSART(str);
    stateFlags.accelerometerIsThere = 0; // until found
    I2C_Stop();
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE); // address the device, say we are going to write
    if (r)
        stateFlags.accelerometerIsThere = 1; // if NACK, assume device is not there
    if (stateFlags.accelerometerIsThere) {  // get the device ID, can use if needed for further test
        r = I2C_Write(ADXL345_REG_DEVID); // tell the device the register we are going to want
        I2C_ReStart(); // restart, preparatory to reading
        r = I2C_Write(ADXL345_ADDR_READ); // address the device, say we are going to read
        r = I2C_Read(0); // do NACK, since this is the last byte
        // result should be 0xE5
        I2C_Stop();
        len = sprintf(str, "\n\r got Accelerometer device ID: 0x%x\n\r", (int)r);
        outputStringToUSART(str);
    } else {
        len = sprintf(str, " Accelerometer not found: 0x%x\n\r", r);
        outputStringToUSART(str);
    }
}

/*****************************************
* initialize Accellerometer, for system leveling and to detect taps
*****************************************/

void initAccelerometer (void) {
    char r;
 len = sprintf(str, "\n\r entered initAccelerometer routine \n\r");
 outputStringToUSART(str);
    findAccelerometer();
    if (!stateFlags.accelerometerIsThere) {
        return;
    }
 len = sprintf(str, "\n\r accelerometer found \n\r");
 outputStringToUSART(str);
    // put device in standby to configure
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE);
    r = I2C_Write(ADXL345_REG_POWER_CTL);
    r = I2C_Write(0x00); // 
    I2C_Stop();
    // set data format
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE);
    r = I2C_Write(ADXL345_REG_DATA_FORMAT);
    r = I2C_Write(0x0b); // Full resolution, +/-16g, 4mg/LSB.
    I2C_Stop();
    // set data rate
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE);
    r = I2C_Write(ADXL345_REG_BW_RATE);
    r = I2C_Write(0x0a); // use 100Hz for now ; bit 4 set = reduced power, higher noise
    I2C_Stop();
    // set up tap detection
    // enable Tap Axes
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE);
    r = I2C_Write(ADXL345_REG_TAP_AXES);
    r = I2C_Write(0x0f); // bits: 3= supresses detection between double taps, 2=X, 1=Y, 0=Z
    I2C_Stop();
    // set tap threshold
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE);
    r = I2C_Write(ADXL345_REG_THRESH_TAP);
//    d = 0x01; // most sensitive, 62.5 mg/LSB (0xFF = +16 g)
    r = I2C_Write(0x30); // try ~3 g
    I2C_Stop();
    // set tap duration
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE);
    r = I2C_Write(ADXL345_REG_DUR);
//    d = 0x01; // most brief, 625 &#956;s/LSB.
    r = I2C_Write(0xa0); // try 0.1s
    I2C_Stop();
    // enable the Single Tap interrupt (later will change to Double Tap)
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE);
    r = I2C_Write(ADXL345_REG_INT_ENABLE);
    r = I2C_Write(0x40); //  0x40 = single tap,  0x20 = double tap,  0x60 = both
    I2C_Stop();
    // initialize by clearing any interrupts
    clearAnyADXL345TapInterrupt();
    //bring out of standby and set device to measure
    I2C_Start();
    r = I2C_Write(ADXL345_ADDR_WRITE);
    r = I2C_Write(ADXL345_REG_POWER_CTL);
    r = I2C_Write(0x08); // 
    I2C_Stop();
 len = sprintf(str, "\n\r accelerometer initialized \n\r");
 outputStringToUSART(str);

} // end of initAccelerometer

/*****************************************
* initialize SPI, to talk to the SD card
*****************************************/

void initSPI (void) {
    // all these pin definitions vary by BOARD_VERSION, see HardwareProfile.h
    // without following two lines, first few writes to the SD card fail, though then it works OK
    SPICLOCK = 0; // explicitly make SCK an output
    SD_CS_TRIS = 0; // explicitly make CS an output
    // see if this works
    // following 2 lines without preceeding 2 lines causes failure to find SD card
    // enable weak pull-ups
    PULL_UP_SDI = 1;
    PULL_UP_CS = 1;

    PPSUnLock; // unlock peripheral pin select registers
    // map SPI functions to specific pins
    #ifdef BOARD_VERSION_00
        CONFIG_SDI1_TO_RP(12); // map SDI1 to pin RP12, physical pin 23
        CONFIG_SDO1_TO_RP(14); // map SDO1 to pin RP14, physical pin 25
        CONFIG_SCK1OUT_TO_RP(13); // map SCK1OUT to pin RP13, physical pin 24
        CONFIG_SS1OUT_TO_RP(15); // map SS1OUT to pin RP15, physical pin 26
    #endif
    #ifdef BOARD_VERSION_01
        CONFIG_SDI1_TO_RP(11); // map SDI1 to pin RP11, physical pin 22
        CONFIG_SCK1OUT_TO_RP(12); // map SCK1OUT to pin RP12, physical pin 23
        CONFIG_SDO1_TO_RP(14); // map SDO1 to pin RP14, physical pin 25
        CONFIG_SS1OUT_TO_RP(15); // map SS1OUT to pin RP15, physical pin 26
    #endif
    #ifdef BOARD_VERSION_02
        CONFIG_SDI1_TO_RP(14); // map SDI1 to pin RP14, physical pin 25
        CONFIG_SCK1OUT_TO_RP(13); // map SCK1OUT to pin RP13, physical pin 24
        CONFIG_SDO1_TO_RP(11); // map SDO1 to pin RP11, physical pin 22
        CONFIG_SS1OUT_TO_RP(10); // map SS1OUT to pin RP10, physical pin 21
    #endif

    PPSLock; // lock registers back again

} // end of initSPI

/*****************************************
* initialize the USART
*****************************************/
void initUSART (void) {
    PPSUnLock; // unlock peripheral pin select registers
    // map USART functions to specific pins
//    CONFIG_U1RTS_TO_RP(6); // map RTS to pin RP6, physical pin 15 --UNUSED--
//    CONFIG_U1CTS_TO_RP(7); // map CTS to pin RP7, physical pin 16 --UNUSED--
    CONFIG_U1RX_TO_RP(8); // map RX to pin RP8, physical pin 17
    CONFIG_U1TX_TO_RP(9); // map TX to pin RP9, physical pin 18

    PPSLock; // lock registers back again

/*
 >>>>>>>>>> set up UART1 
  U1MODE: UART1 MODE REGISTER (in nomenclature below, x=1)
bit 15 UARTEN: UARTx Enable bit
 Set up peripherial pins before enabling this
 Enable this before setting UTXEN (transmit enable) or transmit will not be enabled
 Allow a minimum 1 clock cycle delay between setting the UARTEN bit and writing data to UxTXREG
 1 = UARTx is enabled; all UARTx pins are controlled by UARTx as defined by UEN<1:0>
 0 = UARTx is disabled; all UARTx pins are controlled by PORT latches; UARTx power consumption is minimal
bit 14 Unimplemented: Read as 0
bit 13 USIDL: Stop in Idle Mode bit
 1 = Discontinue module operation when device enters Idle mode
 0 = Continue module operation in Idle mode (default)
bit 12 IREN: IrDA Encoder and Decoder Enable bit(2)
 1 = IrDA encoder and decoder enabled
 0 = IrDA encoder and decoder disabled (default)
bit 11 RTSMD: Mode Selection for UxRTS Pin bit
 1 = UxRTS pin in Simplex mode
 0 = UxRTS pin in Flow Control mode (default)
bit 10 Unimplemented: Read as 0
bit 9-8 UEN1:UEN0: UARTx Enable bits(3)
 11 = UxTX, UxRX and BCLKx pins are enabled and used; UxCTS pin controlled by PORT latches
 10 = UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
 01 = UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin controlled by PORT latches
 00 = UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLKx pins controlled by PORT latches (default)
bit 7 WAKE: Wake-up on Start Bit Detect During Sleep Mode Enable bit
 1 = UARTx will continue to sample the UxRX pin; interrupt generated on falling edge, bit cleared in 
     hardware on following rising edge
 0 = No wake-up enabled (default)
bit 6 LPBACK: UARTx Loopback Mode Select bit
 1 = Enable Loopback mode
 0 = Loopback mode is disabled (default)
bit 5 ABAUD: Auto-Baud Enable bit
 1 = Enable baud rate measurement on the next character - requires reception of a Sync field (55h); cleared 
     in hardware upon completion
 0 = Baud rate measurement disabled or completed (default)
bit 4 RXINV: Receive Polarity Inversion bit
 1 = UxRX Idle state is 0
 0 = UxRX Idle state is 1 (default)
bit 3 BRGH: High Baud Rate Enable bit
 1 = BRG generates 4 clocks per bit period (4x baud clock, High-Speed mode)
 0 = BRG generates 16 clocks per bit period (16x baud clock, Standard mode) (default)
bit 2-1 PDSEL1:PDSEL0: Parity and Data Selection bits
 11 = 9-bit data, no parity
 10 = 8-bit data, odd parity
 01 = 8-bit data, even parity
 00 = 8-bit data, no parity (default)
bit 0 STSEL: Stop Bit Selection bit
 1 = Two Stop bits
 0 = One Stop bit (default)

            111111 bit positions
            5432109876543210 */
 U1MODE = 0b1000000000001000; //  enable UART, BRGH=1, 8N1
 // Baud divisor calculations assume BRGH = 1
 //  if changed, comment out high-speed equates and enable std speed equates in HardwareProfile.h
// U1BRG = Baud9600;
// U1BRG = Baud19200;
// U1BRG = Baud38400;
 U1BRG = Baud115200;
// 
/* temporarily disable for testing
*/

/*
   UxSTA: UARTx STATUS AND CONTROL REGISTER

bit 15, 13 UTXISEL1:UTXISEL0: Transmission Interrupt Mode Selection bits
 11 = Reserved; do not use
 10 = Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a 
     result, the transmit buffer becomes empty
 01 = Interrupt when the last character is shifted out of the Transmit Shift Register; all 
     transmit operations are completed
 00 = Interrupt when a character is transferred to the Transmit Shift Register (this implies 
     there is at least one character open in the transmit buffer); default
bit 14 UTXINV: IrDA® Encoder Transmit Polarity Inversion bit
 If IREN = 0: (default)
  1 = UxTX Idle 0
  0 = UxTX Idle 1
 If IREN = 1:
  1 = UxTX Idle 1
  0 = UxTX Idle 0
bit 12 Unimplemented: Read as 0
bit 11 UTXBRK: Transmit Break bit
 1 = Send Sync Break on next transmission - Start bit, followed by twelve 0 bits, followed by Stop bit;
     cleared by hardware upon completion
 0 = Sync Break transmission disabled or completed (default)
bit 10 UTXEN: Transmit Enable bit. Set UARTEN before setting this or transmit will not be enabled.  See 
     notes on setting UARTEN
 1 = Transmit enabled, UxTX pin controlled by UARTx
 0 = Transmit disabled, any pending transmission is aborted and buffer is reset. UxTX pin controlled by
     the PORT register. (default)
bit 9 UTXBF: Transmit Buffer Full Status bit (read-only)
 1 = Transmit buffer is full
 0 = Transmit buffer is not full, at least one more character can be written (POR default)
bit 8 TRMT: Transmit Shift Register Empty bit (read-only)
 1 = Transmit Shift Register is empty and transmit buffer is empty (the last transmission has 
     completed) (default)
 0 = Transmit Shift Register is not empty, a transmission is in progress or queued
bit 7-6 URXISEL1:URXISEL0: Receive Interrupt Mode Selection bits
 11 = Interrupt is set on RSR transfer, making the receive buffer full (i.e., has 4 data characters)
 10 = Interrupt is set on RSR transfer, making the receive buffer 3/4 full (i.e., has 3 data characters)
 0x = Interrupt is set when any character is received and transferred from the RSR to the receive buffer.
     Receive buffer has one or more characters. (default)
bit 5 ADDEN: Address Character Detect bit (bit 8 of received data = 1)
  1 = Address Detect mode enabled. If 9-bit mode is not selected, this control bit has no effect.
  0 = Address Detect mode disabled (default)
bit 4 RIDLE: Receiver Idle bit (read-only)
 1 = Receiver is Idle
 0 = Data is being received
bit 3 PERR: Parity Error Status bit (read-only)
 1 = Parity error has been detected for the current character
 0 = Parity error has not been detected
bit 2 FERR: Framing Error Status bit (read-only)
 1 = Framing error has been detected for the current character
 0 = Framing error has not been detected
bit 1 OERR: Receive Buffer Overrun Error Status bit (clear/read-only)
 1 = Receive buffer has overflowed
 0 = Receive buffer has not overflowed (clearing a previously set OERR bit will reset the receiver buffer
     and RSR to empty state)
bit 0 URXDA: Receive Buffer Data Available bit (read-only)
 1 = Receive buffer has data, at least one more character can be read
 0 = Receive buffer is empty

           111111 bit positions
           5432109876543210 */
 U1STA = 0b0000010000000000; // set UTXEN
 _U1RXIP = 5; // increase UART1 Receiver Interrupt Priority from 4 to 5
/* temporarily disable for testing
*/
 // initialize buffers USART I/O will use
 USART1_inputbuffer_head = &USART1_inputbuffer[0];
 USART1_inputbuffer_tail = &USART1_inputbuffer[0];
 USART1_outputbuffer_head = &USART1_outputbuffer[0];
 USART1_outputbuffer_tail = &USART1_outputbuffer[0];
 
 commandBufferPtr = &commandBuffer[0];

 //_U1TXIE = 1; // enable USART1 Transmit interrupt
 _U1RXIE = 1; // enable USART1 Receive interrupt


} // initUSART

/*****************************************
* initialize the attached Bluetooth module
*  Preconditon: USART initialized
*  If no BT module is attached, programming will go to any COM port attached
*   may be cryptic, but no problem
* 
*****************************************/
void initBTModule(void) {  // initialize the Bluetooth module
    for (unsignedIntTmp = ~0; (unsignedIntTmp); unsignedIntTmp--) ; // wait a while
    // turn on power to unit
    // unit can draw up to 30mA, which is > the 25mA each pin can supply, so
    //  try 2 pins in parallel; may change to 1 pin driving load switch
    // use Port A, bit 2 (physical pin 9) and bit 3 (physical pin 10)
    // do 2 pins simultaneously 
    TRISA &= 0b1111111111110011;  // make outputs, set the state of these 2 pins to 0
    PORTA |= 0b0000000000001100;  // drive high, set level of these 2 pins to 1
    for (unsignedIntTmp = ~0; (unsignedIntTmp); unsignedIntTmp--) ; // wait a while
    

    ;
} // end of initBTModule

/*****************************************
* verify that string in text buffer is a valid date/time
*  Input: char pointer
*  Output: int, used as boolean; 1 = true, 0 = false
*  Preconditon: date/time starting at pointer, in strict format:
*  YY-MM-DD HH:MM:SS (assumes century is 2000 which is the only valid century on this device)
*  use this fn before trying to set the time
* 
*****************************************/
int isValidTimestamp(char* p)
{
    int i, m, d, v;
    // example: "2010-12-21 10:47:13"
    // year
    if (*p++ != '2') // must always start with '20'
        return 0;
    if (*p++ != '0')
        return 0;
    i = *p++;
    if ((i < '0') || (i > '9')) // tens of year
        return 0;
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of year
        return 0;
    if (*p++ != '-') // correct delimiter
        return 0;
    // month
    i = *p++;
    if ((i < '0') || (i > '1')) // tens of month
        return 0;
    m = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of month
        return 0;
    m = m + (i & 0xf); // strip all but low nybble to convert to BCD
    if (m > 12) 
        return 0;
    if (*p++ != '-') // correct delimiter
        return 0;
    // day
    i = *p++;
    if ((i < '0') || (i > '3')) // tens of day
        return 0;
    d = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of day
        return 0;
    d = d + (i & 0xf); // strip all but low nybble to convert to BCD
    if (d > 31) // takes care of January, March, May, July, August, October, and December
        return 0;
    if ((m = 2) && (d > 29)) // February
        return 0;
    if ((m = 4) && (d > 30)) // April
        return 0;
    if ((m = 6) && (d > 30)) // June
        return 0;
    if ((m = 9) && (d > 30)) // September
        return 0;
    if ((m = 11) && (d > 30)) // November
        return 0;
    if (*p++ != ' ') // correct delimiter
        return 0;
    // hour
    i = *p++;
    if ((i < '0') || (i > '2')) // tens of hour
        return 0;
    v = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of hour
        return 0;
    v = v + (i & 0xf); // strip all but low nybble to convert to BCD
    if (v > 23) 
        return 0;
    if (*p++ != ':') // correct delimiter
        return 0;
    // minute
    i = *p++;
    if ((i < '0') || (i > '5')) // tens of minute
        return 0;
    v = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of minute
        return 0;
    v = v + (i & 0xf); // strip all but low nybble to convert to BCD
    if (v > 59) 
        return 0;
    if (*p++ != ':') // correct delimiter
        return 0;
    // second
    i = *p++;
    if ((i < '0') || (i > '5')) // tens of second
        return 0;
    v = 10 * (i & 0xf); // strip all but low nybble to convert to BCD
    i = *p++;
    if ((i < '0') || (i > '9')) // ones of second
        return 0;
    v = v + (i & 0xf); // strip all but low nybble to convert to BCD
    if (v > 59) 
        return 0;
    // passed all tests
    return 1;
}

/*****************************************
* set device date/time from date and time as text in buffer
*  Input: char pointer
*  Output: none
*  Preconditon: date/time starting at pointer, in strict format:
*  YY-MM-DD HH:MM:SS (assumes century is 2000 which is the only valid centry on this device)
* this fn will usually set a valid, though incorrect, date even from scrambled data
* 
*****************************************/
void setTimeFromCharBuffer (char *p)
{
 int i;
 // unlock the timer registers using this special sequence
 //__builtin_write_RTCWEN; // maybe try this format
 __asm__ ("mov #NVMKEY,W0");
 __asm__ ("mov #0x55,W1");
 __asm__ ("mov #0xAA,W2");
 __asm__ ("mov W1,[W0]");
 __asm__ ("nop");
 __asm__ ("mov W2,[W0]");
 __asm__ ("bset RCFGCAL,#13");
 __asm__ ("nop");
 __asm__ ("nop");

 RCFGCALbits.RTCPTR0 = 1; // point to year slot
 RCFGCALbits.RTCPTR1 = 1;

 // set date/time from char buffer; all d/t vals are BCD (binary coded decimal)
 // example: timeStampBuffer[] = "\r\n2009-12-21 10:47:13";
 // this fn receives (timeStampBuffer+4)
 i = 0;
 i |= (*p++ & 0xf); // tens-of-year char, strip all but low nybble to convert to BCD
 i <<= 4;
 i |= (*p++ & 0xf); // one-of-year
 RTCVAL = i; // set year, defined in the chip architecture to be 20xx
 // RTCVAL  pointer decrements with each write to point to next lower values
 p++; // skip date delimiter
 i = 0;
 i |= (*p++ & 0xf); // tens-of-month
 i <<= 4;
 i |= (*p++ & 0xf); // one-of-month
 i <<= 4;
 p++;
 i |= (*p++ & 0xf); // tens-of-day
 i <<= 4;
 i |= (*p++ & 0xf); // ones-of-day
 RTCVAL = i; // set month and day
 p++;
 i = 0; // day of week is don't-care value 0
 i |= (*p++ & 0xf); // tens-of-hour
 i <<= 4;
 i |= (*p++ & 0xf); // one-of-hour
 RTCVAL = i; // set hour
 p++;
 i = 0;
 i |= (*p++ & 0xf); // tens-of-minute
 i <<= 4;
 i |= (*p++ & 0xf); // one-of-minute
 i <<= 4;
 p++;
 i |= (*p++ & 0xf); // tens-of-second
 i <<= 4;
 i |= (*p++ & 0xf); // ones-of-second
 RTCVAL = i; // set minute and second
 _RTCWREN = 0; // relock RTCC registers, to protect values from accidental change
} // setTimeFromCharBuffer

/*****************************************
* create text formatted in timeStampBuffer from device date and time settings
*  also set up dirNameBuffer and fileNameBuffer based on date/time
*****************************************/
void createTimestamp (void)
{
 char Yr, Mo, Da, Hr, Mn, Sc, nybVal, i;
 char numDaysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  // create timestamp

//  timeStampBuffer[2] = '2'; // device RTCC only runs from 2000 to 2099
//  timeStampBuffer[3] = '0';
//  secsSince1Jan2000 = 0;
  _RTCPTR1 = 1; // point to Year slot in RTCC registers
  _RTCPTR0 = 1;
  intTmp = RTCVAL;
  nybVal = (char)(0xf & (intTmp >> 4)); // tens-of-year
  timeStampBuffer[4] = (nybVal + 0x30); // set character
  Yr = 10 * nybVal; // accumulate year
  nybVal = (char)(0xf & intTmp); // ones-of-year
  timeStampBuffer[5] = (nybVal + 0x30); // set character
  Yr += nybVal; // accumulate year

//  timeStampBuffer[4] = (char)(0xf & (intTmp >> 4)) + 0x30; // tens-of-year
//  timeStampBuffer[5] = (char)(0xf & intTmp) + 0x30; // ones-of-year
  intTmp = RTCVAL; // next read gets Month/Day
  // check for rollover of day
  if (!((timeStampBuffer[10] == (char)(0xf & (intTmp >> 4)) + 0x30) && 
          (timeStampBuffer[11] == (char)(0xf & intTmp) + 0x30)))
      // change of day will cause new file to be created 
      flags1.writeDataHeaders = 1; // re-log data column headers to the file on the next SD card write

  nybVal = (char)(0xf & (intTmp >> 12)); // tens-of-month
  timeStampBuffer[7] = (nybVal + 0x30); // set character
  Mo = 10 * nybVal; // accumulate month
  nybVal = (char)(0xf & (intTmp >> 8)); // ones-of-month
  timeStampBuffer[8] = (nybVal + 0x30); // set character
  Mo += nybVal; // accumulate month
  nybVal = (char)(0xf & (intTmp >> 4)); // tens-of-day
  timeStampBuffer[10] = (nybVal + 0x30); // set character
  Da = 10 * nybVal; // accumulate day
  nybVal = (char)(0xf & intTmp); // ones-of-day
  timeStampBuffer[11] = (nybVal + 0x30); // set character
  Da += nybVal; // accumulate day

//  timeStampBuffer[7] = (char)(0xf & (intTmp >> 12)) + 0x30; // tens-of-month
//  timeStampBuffer[8] = (char)(0xf & (intTmp >> 8)) + 0x30; // ones-of-month
//  timeStampBuffer[10] = (char)(0xf & (intTmp >> 4)) + 0x30; // tens-of-day
//  timeStampBuffer[11] = (char)(0xf & intTmp) + 0x30; // ones-of-day

  intTmp = RTCVAL; // next read gets Weekday/Hour
  nybVal = (char)(0xf & (intTmp >> 4)); // tens-of-hour
  timeStampBuffer[13] = (nybVal + 0x30); // set character
  Hr = 10 * nybVal; // accumulate hour
  nybVal = (char)(0xf & intTmp); // ones-of-hour
  timeStampBuffer[14] = (nybVal + 0x30); // set character
  Hr += nybVal; // accumulate hour

//  timeStampBuffer[13] = (char)(0xf & (intTmp >> 4)) + 0x30; // tens-of-hour
//  timeStampBuffer[14] = (char)(0xf & intTmp) + 0x30; // ones-of-hour
  intTmp = RTCVAL; // next read gets Minute/Second
  // presumably this will happen on a whole-second RTCC interrupt and in real time
  //   there would be no chance of a rollover between reads

  nybVal = (char)(0xf & (intTmp >> 12)); // tens-of-minute
  timeStampBuffer[16] = (nybVal + 0x30); // set character
  Mn = 10 * nybVal; // accumulate minute
  nybVal = (char)(0xf & (intTmp >> 8)); // ones-of-minute
  timeStampBuffer[17] = (nybVal + 0x30); // set character
  Mn += nybVal; // accumulate minute
  nybVal = (char)(0xf & (intTmp >> 4)); // tens-of-second
  timeStampBuffer[19] = (nybVal + 0x30); // set character
  Sc = 10 * nybVal; // accumulate second
  nybVal = (char)(0xf & intTmp); // ones-of-second
  timeStampBuffer[20] = (nybVal + 0x30); // set character
  Sc += nybVal; // accumulate second

//  timeStampBuffer[16] = (char)(0xf & (intTmp >> 12)) + 0x30; // tens-of-minute
//  timeStampBuffer[17] = (char)(0xf & (intTmp >> 8)) + 0x30; // ones-of-minute
//  timeStampBuffer[19] = (char)(0xf & (intTmp >> 4)) + 0x30; // tens-of-second
//  timeStampBuffer[20] = (char)(0xf & intTmp) + 0x30; // ones-of-second

 // set up the directory name and filename based on the timestamp
  dirNameBuffer[1] = timeStampBuffer[4];
  dirNameBuffer[2] = timeStampBuffer[5];
  dirNameBuffer[4] = timeStampBuffer[7];
  dirNameBuffer[5] = timeStampBuffer[8];

  fileNameBuffer[0] = timeStampBuffer[10];
  fileNameBuffer[1] = timeStampBuffer[11];

 // calculate seconds since midnight between 31 December 1999 and 1 January 2000
  // use 365-day years, tweaking days total to make this correct
  // adjust for leap year days
  Da += (Yr >> 2); // this comes up one short by not including year 2000 leap day
  // add that in if current date is in a leap year and after 1 March
  if (!(0x3 & Yr) && (Mo > 2)) Da++;
  // first, add up days
  secsSince1Jan2000 = (Yr * 365) + Da;
  // add in days from months
  i = 0;
  while (i < (Mo-1))
  {
   secsSince1Jan2000 += numDaysInMonth[i];
   i++;
  }
  // convert days to seconds and add in hours, minute, and seconds
  secsSince1Jan2000 = (((((secsSince1Jan2000 * 24) + Hr) * 60) + Mn) * 60) + Sc;

} // end of createTimestamp

//void getDataReadings(void);

/*************************************************************************
  Function:
    void getDataReadings(void)
  Summary:
    attempts to acquire data readings, output any diagnostics, and write results to SD card
  Conditions:
    
  Input:
    None
  Return Values:
    None
  Side Effects:
    Uses many global variables
  Description:
    Called when system state goes from Idle to ReadingSensors
  Remarks:
    None
  *************************************************************************/
void getDataReadings(void)
{

} // end of getDataReadings

/*************************************************************************
  Function:
    int getCellVoltage(void)
  Summary:
    digitizes nominal 1.2 volts from NiMH cell the system runs on, for tracking charge state
  Conditions:
    cell - connected to system ground
    cell + connected to AN0, physical pin 2
    pin configured as analog inupt
  Input:
    None
  Return Values:
    cell voltage as 10-bit value
  Side Effects:
    None
  Description:
    Reads pin 16 times and returns the average
  Remarks:
    None
  *************************************************************************/

int getCellVoltage(void)
{
    int ADCValue, count;
    int *ADC16Ptr;
//   AD1PCFG = 0xFFFE; // Only AN0 as analog input; done during initMain
    _ADC1MD = 0; // clear any disable of the entire A/D module
    AD1CON1 = 0x00e0; // Internal counter triggers conversion
    AD1CHS = 0x0000; // Connect AN0 as positive input
    AD1CSSL = 0; // no sequential scanning
    AD1CON3 = 0x0f00; // Sample time = 15Tad, Tad = Tcy
    AD1CON2 = 0x003c; // Set AD1IF after every 16 samples
    AD1CON1bits.ADON = 1; // turn ADC ON
    // following would be the loop if repeated
    ADCValue = 0; // clear value
    ADC16Ptr = &ADC1BUF0; // initialize ADC1BUF pointer
    IFS0bits.AD1IF = 0; // clear ADC interrupt flag
    AD1CON1bits.ASAM = 1; // auto start sampling for 31Tad
    // then go to conversion
    while (!IFS0bits.AD1IF){}; // conversion done?
    AD1CON1bits.ASAM = 0; // yes then stop sample/convert
    for (count = 0; count < 16; count++) // average the 16 ADC value
    {
        ADCValue = ADCValue + *ADC16Ptr++;
    }
    ADCValue = ADCValue >> 4;
    AD1CON1bits.ADON = 0; // turn ADC off
    _ADC1MD = 1; // disable the A/D module entirely, to save power
    return ADCValue ;
    // end of loop, if repeated
} // end of getCellVoltage

/*************************************************************************
  Function:
    void assureDataHeadersLogged(void)
  Summary:
    attempt to write data column headers to SD card
  Conditions:
    None, only proceeds if flags1.writeDataHeaders = 1
  Input:
    None
  Return Values:
    None
  Side Effects:
    uses global variables err, len, str
    if succesful write, clears flags1.writeDataHeaders, otherwise flag remains set
  Description:
    this should happen on initialization, reset, time change, and midnight rollover
  Remarks:
    None
  *************************************************************************/

void assureDataHeadersLogged(void)
{
    if (flags1.writeDataHeaders)
    {
        len = sprintf(str, "\n\r\n\rTime\tbbDn\tirDn\tbbUp\tirUp\tBatt\n\r");
        err = writeCharsToFile (str, len);
        if (err)
            tellFileWriteError (err);
        if (err == NoProblem) // flag successful
            flags1.writeDataHeaders = 0;
    }
} // end of assureDataHeadersLogged

void startTimer1ToRunThisManySeconds(unsigned int numSecondsToRun) {
    len = sprintf(str, "\n\r Timer1 set for %u seconds \n\r", numSecondsToRun);
    outputStringToUSART(str);
    // set up and enable Timer1
    T1CON = 0x00; //Stop Timer1 and reset control reg
    TMR1 = 0x00; //Clear contents of the timer register
    // 128 counts/sec, 32.768kHz RTCC xtal divided by 256 prescale
    PR1 = (numSecondsToRun * 128); // 128 counts per seconds
    // e.g. PR1 = (120 * 128); // period = 2 minutes
    _T1IF = 0; //Clear the Timer1 interrupt status flag
    _T1IE = 1; //Enable Timer1 interrupts

/*
 T1CON
bit 15 TON: Timerx On bit
 1 = Starts the timer
 0 = Stops the timer
bit 14 Unimplemented: Read as ‘0’
bit 13 TSIDL: Stop in Idle Mode bit
 1 = Discontinue timer operation when device enters Idle mode
 0 = Continue timer operation in Idle mode
bit 12-7 Unimplemented: Read as ‘0’
bit 6 TGATE: Timerx Gated Time Accumulation Enable bit
 When TCS = 1:
  This bit is ignored.
 When TCS = 0:
  1 = Gated time accumulation enabled
  0 = Gated time accumulation disabled
bit 5-4 TCKPS<1:0>: Timerx Input Clock Prescale Select bits
 11 = 1:256 prescale value
 10 = 1:64 prescale value
 01 = 1:8 prescale value
 00 = 1:1 prescale value
bit 3 Unimplemented: Read as ‘0’
bit 2 TSYNC: Timerx External Clock Input Synchronization Select bit
 When TCS = 1:
  1 = Synchronize external clock input
  0 = Do not synchronize external clock input
 When TCS = 0:
  This bit is ignored. Read as ‘0’. Timerx uses the internal clock when TCS = 0.
bit 1 TCS: Timerx Clock Source Select bit
 1 = External clock from TxCK pin
 0 = Internal clock (FOSC/2)
bit 0 Unimplemented: Read as ‘0’

              111111 bit positions
              5432109876543210 */
    T1CON = 0b1000000000110010; // set T1CON

    //Start Timer1 with prescaler settings at 1:256
    //clock source set to external; since RTCC is enabled, it is the 32.768kHz RTCC xtal
    // 32,768 Hz divided by 256 prescale = 128 counts/sec

} // end of startTimer1ToRunThisManySeconds


/*****************************************
*  RTCC interrupt
*****************************************/
void __attribute__ ((interrupt, no_auto_psv)) _RTCCInterrupt(void)
{
 machState = GettingTimestamp;
 _RTCIF = 0; //  clear RTCC interrupt flag
} // _InterruptVector _RTCCInterrupt


/*****************************************
*  External #1 interrupt
*****************************************/
void __attribute__ ((interrupt, no_auto_psv)) _INT1Interrupt(void)
{
    _INT1IE = 0; // disable this INT1 external interrupt until we can clear
    // the device that caused it, outside this ISR, in the main program loop
    stateFlags.reRoused = 1; // re-Roused, or Roused for the first time
    _INT1IF = 0; //  clear INT1 interrupt flag
} // _InterruptVector _INT1Interrupt


/*****************************************
*  Timer 1 interrupt
*****************************************/
void __attribute__ ((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    T1CON = 0x00; //Stop Timer1 and reset control reg
    _T1IE = 0; // disable this interrupt, one shot
    stateFlags.isRoused = 0; // can go back to sleep
    _T1IF = 0; // clear T1 interrupt flag

} // _InterruptVector _T1Interrupt



/*****************************************
*  set SD card power control
*****************************************/
void setSDCardPowerControl(void)
{
    if (stateFlags_2.turnSDCardOffBetweenDataWrites) {
        SD_POWER_TRIS = 0; // assure the pin is an output, source of the SD card power under software control
    } else {
        SD_POWER_TRIS = 1; // assure the pin is an input, not involved in power control
    }

} // end of setSDCardPowerControl



/****************************************
 *
 * Function:  _U1TXInterrupt
 *  Overview:        
 *  PreCondition: none
 *  Input:          character(s) in USART1_outputbuffer
 *  Output:         none
 *  Side Effects:   none
 * Changes:  USART1_outputbuffer_head and USART1_outputbuffer_tail
 *  Calls:          none
 *
 * * * * * *
 * from AN774, page 13, in Asynchronous Communications with the PICmicro® USART
 *TRANSMIT INTERRUPT OPERATION
 *  The TXIF bit is cleared when data is written to TXREG and gets set when this data moves into the Transmit
 * Shift Register to get transmitted. This means that the interrupt occurs when new data can be written to TXREG.
 *
 *  Whenever TXREG is empty, the TXIF bit will be set and an interrupt will occur if the interrupt is enabled. This
 * provides a useful way to transmit data as fast as possible, but it is necessary to have the data available when
 * the interrupt occurs. It is common to use a buffer that is read by the interrupt routine, one byte being written to
 
 * TXREG each time the interrupt occurs. When the last byte of data (from the buffer) has been written to
 * TXREG, the TXIE bit must be cleared to stop further interrupts from occurring. The interrupt can be enabled
 * again later when new data needs to be transmitted and this will immediately cause an interrupt. If the code disables
 * transmit interrupts and any other interrupts can occur, the interrupt routine must test for both the interrupt
 * flag and the enable bit, because the interrupt flag can be set regardless of whether the interrupt is
 * enabled.
 * * * * * *
 *  Therefore, this function will put at most one byte into the Transmit register on each call, though
 * it may be called repeatedly in quick succession
 ****************************************/
/*

void __attribute__ ((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
 char s;
 if (USART1_outputbuffer_head == USART1_outputbuffer_tail)
 {
  _U1TXIE = 0; // buffer is empty, disable TX interrupts
 } else { // generate new tail pointer
  s = (char)*USART1_outputbuffer_tail; // get byte we are going to send
  USART1_outputbuffer_tail++; // generate new Tail address, start by incrementing
  U1TXREG = s; // put byte to send in transmit buffer
  if (USART1_outputbuffer_tail >= (&USART1_outputbuffer[0] + USART1_Buffer_Length))
   USART1_outputbuffer_tail = (char*)(&USART1_outputbuffer[0]); // rollover to beginning if necessary
  _U1TXIF = 0; // clear interrupt flag
 }
} // _U1TXInterrupt


*/








