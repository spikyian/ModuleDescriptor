/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - main processing loop

 This code is for a CANPanel CBUS module, to control up to 64 LEDs (or 8 x 7 segment displays)
 and up to 64 push buttons or on/off switches

 This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                   and indicate if changes were made. You may do so in any reasonable manner,
                   but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                  your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                  legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

**************************************************************************************************************
  Note:   This source code has been written using a tab stop and indentation setting
          of 4 characters. To see everything lined up correctly, please set your
          IDE or text editor to the same settings.
******************************************************************************************************
	
 For version number and revision history see CANPanel.h

*/


        
#include "canpanel.h"       // Also contains current version and build number
// #include "max6951.h"
#include "paneltest.h"
#include "buttonscan.h"
#include <FLiM.h>




#pragma romdata EEPROM
// Define data stored in EEPROM
// Initialisation here ensures newly programmed part is in virgin state with defaults set
// Note: see eeprom.h for EEPROM usage definition

rom BYTE mlaReserved[MLA_INT_EE_SIZE];

rom WORD eeOpState = 0;
rom BYTE eeEvFree = EVT_NUM-1;
rom BYTE eeEvCount = 0;
rom BYTE eeFlimMode = fsSLiM;
rom WORD eeNodeId = DEFAULT_NN;
rom BYTE eeCanId = DEFAULT_CANID;
rom BYTE eeBootFlag = 0;


#pragma udata access VARS


#pragma udata MAIN_VARS


unsigned char   FlashInterval;             // Number of heartbeats between flashes of green LED to show alive
BOOL            Doubleflash;               // Control double flashing for FLiM
unsigned char   FlashTime;                 // Number of main loops to hold flash of green LED on
unsigned char	tmr0_reload;

PanelStatus     mainStatus;



// #pragma romdata

// local function prototypes
/*
 */
void __init(void);
BOOL checkCBUS( BOOL display );
void ISRHigh(void);
void high_irq_errata_fix(void);

/*
 * Interrupt vectors (moved higher when bootloader present)
 */

// High priority interrupt vector

//#ifdef BOOTLOADER_PRESENT
    #pragma code high_vector=0x808
//#else
//    #pragma code high_vector=0x08
//#endif


//void interrupt_at_high_vector(void)

void HIGH_INT_VECT(void)
{
    _asm
        CALL high_irq_errata_fix, 1
    _endasm
}

/*
 * See 18F2480 errata
 */
void high_irq_errata_fix(void) {
    _asm
        POP
        GOTO ISRHigh
    _endasm
}

// low priority interrupt vector

//#ifdef BOOTLOADER_PRESENT
    #pragma code low_vector=0x818
//#else
//    #pragma code low_vector=0x18
//#endif

void LOW_INT_VECT(void)
{
//    _asm GOTO ISRLow _endasm
}


// MAIN APPLICATION

#pragma code APP

void main(void)
{

    BYTE        i;
    BYTE        button;
    TickValue   startTime, stateTime, timeNow;
    DWORD       timeDiff;
  
    canPanelInit(mainStatus);
    startTime.Val = tickGet();
 
    mainStatus.started = FALSE;
    mainStatus.panelMode = testOff;
   
    while (TRUE)
    {
        // Startup delay for CBUS about 2 seconds to let other modules get powered up - ISR will be running so incoming packets processed

        if (!mainStatus.started && (tickTimeSince(startTime) > (NV->sendSodDelay * HUNDRED_MILI_SECOND) + TWO_SECOND))
        {
            mainStatus.started = TRUE;
            if (NV->sendSodDelay > 0)
                sendStartupSod(START_SOD_EVENT);
            
            if (NV->testFlags.startInTest)
            {
                flimState = fsTestMode;
                mainStatus.panelMode = NV->testFlags.panelTestMode;
            }    
        }
               
 
      // Test mode checks here will be in NV - for this test build start in test display mode and cycle test modes when button pressed.
      //TODO - add in parsing CBUS command in approp place
        mainStatus.msgReceived = checkCBUS(mainStatus.panelMode == displayMSG);    // Consume any CBUS message - display it if not display message mode

        FLiMSWCheck();  // Check FLiM switch for any mode changes

        
        if (flimState == fsNextTest)
        {
            flimState = fsTestMode;
            mainStatus.panelMode++;
            
            if ( mainStatus.panelMode == backToStart)
            {    
                mainStatus.panelMode = displayTest;
                mainStatus.testCount = 0;
            }    

            setLedTestMode( FALSE );
            clearAllLeds();
            mainStatus.eventCount = 0;
            stateTime.Val = tickGet();
        }
        else if (flimState == fsTestInput) 
        {
            mainStatus.testInput = TRUE;
            flimState = fsTestMode;
        }    
            
        if (flimState == fsTestMode)
            panelTest();
 
        // Strobe keyboard for button presses

        if (mainStatus.started)
        {
            button = keyScan();

            if (button != 0xFF)
                cbusSendEvent( 0, -1, button, TRUE );
        }
      
        // Check for any flashing status LEDs
        checkFlashing();
        
     } // main loop
} // main
 

void canPanelInit(PanelStatus mainStatus)
{
    unsigned char i;

    initIO();
    initKeyscan();
    panelTestInit();
    panelFlimInit();
    initLedDriver(NV->brightness);

// enable interrupts, all init now done

    INTEN = 1;  // Single priority for now

  //  INTCONbits.GIEH = 1;
  //  INTCONbits.GIEL = 1;
 
    setStatusLed(flimState == fsFLiM);
}

BOOL checkCBUS( BOOL display )

{
    BOOL    msgReceived;
    char    msg[20];
    BYTE    msglen, i;
    
    if (msgReceived = cbusMsgReceived( 0, (BYTE *) &msg ))
    {
 //      LED2G = BlinkLED( 1 );     // Blink LED on whilst processing messages - to give indication how busy module is

        if (display)
        {
            msglen = (msg[d0] >> 5)+1;
            if (msglen > 4)
                msglen = 4;

            for (i=0; i<msglen; i++)
              displayByte(msg[d0+i], i*2);

            for (i=msglen*2; i<8; i++)
              displayChar(' ',i);
        }
        else
            parseCBUSMsg(msg);                // Process the incoming message

    }
    
    return(msgReceived);
}




// C intialisation - declare a copy here so the library version is not used as it may link down in bootloader area

void __init(void)

{
}

// Interrupt service routines

#if defined(__18CXX)
    #pragma interruptlow ISRHigh
    void ISRHigh(void)
#elif defined(__dsPIC30F__) || defined(__dsPIC33F__) || defined(__PIC24F__) || defined(__PIC24FK__) || defined(__PIC24H__)
    void _ISRFAST __attribute__((interrupt, auto_psv)) _INT1Interrupt(void)
#elif defined(__PIC32MX__)
    void __ISR(_EXTERNAL_1_VECTOR, ipl4) _INT1Interrupt(void)
#else
    void _ISRFAST _INT1Interrupt(void)
#endif

{
    tickISR();
    canInterruptHandler();
}