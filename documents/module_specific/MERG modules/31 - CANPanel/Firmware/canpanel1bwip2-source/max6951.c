/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - MAX6951 LED chip hardware driver

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
	
 For full CANPanel version number and revision history see CANPanel.h

*/

// Revision History
//
// 25/01/14     1.0     PNB Coding started

#include "max6951.h"
#include "hwsettings.h"
#include "canpanel.h"

// Character generator for ascii characters onto 7 seg display (quite a few compromises!)

#pragma romdata CHARGEN

// Starts from "0" 0x30

//                              0    1    2    3    4    5    6    7    8    9
const rom BYTE  charGen[] = { 0x7E,0x30,0x6d,0x79,0x33,0x5b,0x5F,0x70,0x7F,0x7B,0,0,0,0,0,0,
//                 Upper case   A    B    C    D    E    F    G    H    I    J    K    L    M    N    O
                            0,0x77,0x1F,0x4E,0x3D,0x4F,0x47,0x5E,0x17,0x06,0x3C,0x07,0x0E,0x76,0x15,0x7E,
//                              P    Q    R    S    T    U    V    W    X    Y    Z
                              0x67,0x73,0x05,0x5B,0x0F,0x3E,0x1C,0x3F,0x31,0x3B,0x6D,0,0,0,0,0,
//                 Lower case   A    B    C    D    E    F    G    H    I    J    K    L    M    N    O
                            0,0x77,0x1F,0x4E,0x3D,0x4F,0x47,0x5E,0x17,0x06,0x3C,0x07,0x0E,0x76,0x15,0x7E,
//                              P    Q    R    S    T    U    V    W    X    Y    Z
                              0x67,0x73,0x05,0x5B,0x0F,0x3E,0x1C,0x3F,0x31,0x3B,0x6D,0,0,0,0,0  };

const rom BYTE HELLO[] = "HELLO";

#pragma romdata

// In memory status of LEDs

#pragma udata MAIN_VARS

// Copy of MAX chip registers, we need to keep copies in RAM because MAX chip is write only
LedsMap ledsMap;
BYTE    decodeMode;

// Local function prototypes

void sendMxCmd( BYTE mxRegister, BYTE mxValue );



void initLedDriver(BYTE brightness)

{
    BYTE    regadr, regval;

    SCK_TRIS = 0;       // SPI clock is an output
    SDO_TRIS = 0;       // SPI data output
    MX_CS_TRIS = 0;     // MAX chip select is an output
    MX_CS_IO = 1;       // Deselect to start with

    SPI_DONEFLAG = 0;
    SSPCON1 = SPI_MASTER_FOSCd4 + SPI_CKP_LOW;      // Set clock rate to 16MHz (ok for MAX chip) with idle clock low. SSP not enabled yet.
    SSPSTATbits.CKE = 1;                            // Transmit data on rising edge of clock
    SSPSTATbits.SMP = 0;                            // Input sampled at middle of data output time

    regadr = MX_CONF;
    regval = MX_CONF_FASTBLINK + MX_CONF_BLINKON;

    sendMxCmd( MX_TEST, 0);                                     // Make sure test mode is off
    sendMxCmd( MX_CONF, MX_CONF_CLEAR );                        // Initialise with outputs shut down and all outputs off
    sendMxCmd( MX_SCAN_LIMIT, 0XFF );                           // Show all LEDs/digits
    sendMxCmd( MX_INTENSITY, brightness &0x0F);                                // Brightness passed in as parameter (0-15))
    clearAllLeds();
    decodeMode = 0;
    sendMxCmd( MX_CONF, MX_CONF_FASTBLINK + MX_CONF_BLINKON + MX_CONF_ENABLE );  // Enable outputs with blink feature enabled
  //sendMxCmd( MX_TEST, 1);     // put into test mode to prove initalisation worked
}


// Pass true or false to set test mode on or off

void setLedTestMode(BOOL testMode)

{
    // In the Maxim chip, test mode is all segments on at 50% duty cycle (half brightness)
    sendMxCmd( MX_TEST, (BYTE) testMode);
}


// Run full LED test, cycling each LED on in turn, for the number of passes specified, with software delay

void runLedTest( BYTE testPasses )

{
    BYTE passCount, digCount, segCount;
    
    for (passCount = 0; passCount < testPasses; passCount++)
    {
        for (digCount = 0; digCount < 8; digCount++)
        {
            segCount = 0x80;
  
            while (segCount != 0)
            {
                sendMxCmd( MX_DIG_BOTH + digCount, segCount);
                doSwDelay( 500 );   // ?? Convert to heartbeat delay once ISR done
                segCount = (segCount == 0x80 ? 1 : segCount<<1);
                if (segCount == 0x80)
                    segCount = 0;
            }    
            sendMxCmd( MX_DIG_BOTH + digCount, 0 );
        }    
    }    
}

// Increment LED test to next LED - returns after each LED for other main loop processing

WORD_VAL ledTestCycle( WORD_VAL testStatus )

{
    BYTE    digCount, segCount;
    
    // Digit count is in status MSBYTE and segment count is in status LSBYTE
    
    if (testStatus.Val == 0xFFFFFFFF)   // Start of test
    {    
        digCount = 0;
        segCount = 0;
    }    
    else
    {   
        digCount = testStatus.byte.HB;
        segCount = testStatus.byte.LB;
        if (segCount == 0)       // start of digit
        {
//            sendMxCmd( MX_DIG_BOTH + digCount++, 0 );   // Clear last digit and move to next
            if (++digCount > 7)
                digCount = 0;            
        }
    }
    
    segCount = (segCount == 0 ? 1 : segCount<<1);    // Next segment bit in digit byte
   
    sendMxCmd( MX_DIG_BOTH + digCount, segCount);   // Turn on one segment
    
    testStatus.byte.HB = digCount;
    testStatus.byte.LB = segCount;
    
    return( testStatus );
}


void showTestX(void)

{
    sendMxCmd( MX_DIG_BOTH, 0xC0);
    sendMxCmd( MX_DIG_BOTH + 1, 0x21);
    sendMxCmd( MX_DIG_BOTH + 2, 0x12);
    sendMxCmd( MX_DIG_BOTH + 3, 0x0C);
    sendMxCmd( MX_DIG_BOTH + 4, 0x0c);
    sendMxCmd( MX_DIG_BOTH + 5, 0x12);
    sendMxCmd( MX_DIG_BOTH + 6, 0x21);
    sendMxCmd( MX_DIG_BOTH + 7, 0xC0);
}


void clearAllLeds(void)

{
    BYTE    digCount;
    BYTE    regadr;

    sendMxCmd( MX_DECODE, 0 );  //. turn off character decoding

    for (digCount = 0; digCount < 8; digCount++)
    {
        regadr = MX_DIG_BOTH + digCount;
        sendMxCmd( regadr, 0);
     }
     memset( (void *) ledsMap, 0, sizeof(ledsMap) );                     // Set in memory map to all zeroes
       
}

void setLed( BYTE ledNumber, BOOL ledState )

{
    BYTE    digNum, segNum, digValue;

    digNum = --ledNumber/8;
    segNum = ledNumber % 8;
    
    // ??? update in memory status arrays, then send to chip
    sendMxCmd( MX_DIG_BOTH + digNum, digValue);
}


// Set LED to flashing state (use setLed to clear flashing state)

void flashLed( BYTE ledNumber )

{
    BYTE    digNum, segNum, digValueOn, digValueOff;

    digNum = --ledNumber/8;
    segNum = ledNumber % 8;
    // ??? update in memory status arrays, then send to chip
    sendMxCmd( MX_DIG_P0 + digNum, digValueOn);
    sendMxCmd( MX_DIG_P1 + digNum, digValueOff);
}


// Display a 16 bit number on 7 segment displays

void displayNumber( WORD toDisplay, BYTE offset, BYTE digits, BYTE format )

{
    WORD    byteToDisplay;   //  display value in LS byte of this word

    byteToDisplay = toDisplay >> 8;        // Get the first byte to display, before the value gets truncated to a byte in the parameter
    displayByte(byteToDisplay,offset);
    byteToDisplay = toDisplay & 0xFF;
    displayByte(byteToDisplay, offset+2);
}


// Display LS nibble of toDisplay as a hex digit on the 7 segment display digit given by offset

void displayDigit( BYTE toDisplay, BYTE offset )

{
    toDisplay &= 0x0F;
    decodeMode |= (1<<offset);

    // ?? Put in validation check for offset value
    sendMxCmd( MX_DECODE, decodeMode);
    sendMxCmd( MX_DIG_BOTH + offset, toDisplay);
}

// Display byte as 2 hex digits on the 7 segment display starting at the digit given by offset

void displayByte( BYTE toDisplay, BYTE offset )
{
    displayDigit( toDisplay>>4, offset );
    displayDigit( toDisplay, offset + 1);
}


void displayChar( unsigned char  toDisplay, BYTE offset )
{
    unsigned char genChar;

    decodeMode &= ~(1<<offset);
    sendMxCmd( MX_DECODE, decodeMode);   // Turn off decode for alphanumerics

    genChar = (toDisplay == ' ' ? 0 : charGen[toDisplay - 0x30]);
    sendMxCmd( MX_DIG_BOTH + offset, genChar);
}

void displayString( char *toDisplay, BYTE offset)

{
    BYTE    i;

    for (i = 0; i<strlen(toDisplay); i++)
    {
        displayChar( toDisplay[i], offset+i);
    }
}


// For first pass test 7 seg display board, fix segment value due to error in board
// Map an LED from row and column to digit number and segment bit map
// Row is 0-8
// Column is 0-7 (for A-H)

Segment mapLED( BYTE row, BYTE column )

{
    Segment segment;

    segment.dig = row;
    if (column > row)
        column--;

    segment.seg = (column == 0 ? 0x80 : 1<<(column-1) );

    return( segment );
}


void displayMessage( char *message, BYTE offset, BYTE digits, BOOL scroll )

{

}

void scrollDisplay( BOOL direction, BYTE limit )
{

}

void sayHello( void )

{
    char    message[9] = {'H','E','L','L','O',' ',' ',' ',0};

    displayString( (char *) &message, 0);
}

void displayVersion( void )

{
    char    message[9] = {'V',' ',' ',' ',' ',' ',' ',' ',0};
    
    message [2] = MAJOR_VER + 0x30;
    message [3] = MINOR_VER;
            
    if (WIP > 0)
    {
        message [4] = 'W';
        message [5] = 'I';
        message [6] = 'P';
        message [7] = WIP + 0x30;
    }
    else if (BETA > 0)
    {
        message[4] = 'B';
        message[5] = 'E';
        message[6] = 'T';
        message[7] = BETA+ 0x30;
    }    
    displayString( (char *) &message, 0);
}

void doSwDelay( WORD milliseconds )

// ?? This routine will be replaced once the ISR with heartbeat support is done
{
    WORD  msCounter;
    WORD  loopCounter;
    WORD  loopsPerMs;

    loopsPerMs = GetInstructionMHz();
    loopsPerMs  *= 50L;

    for (msCounter = 0; msCounter < milliseconds; msCounter++)
    // Divide instruction clock by 1000 to get instruction cycles per mS, then divide again for number of instructions per loop
        for (loopCounter = 0; loopCounter < loopsPerMs; loopCounter++);
 
        
}

void sendMxCmd( BYTE mxRegister, BYTE mxValue )

{
    BOOL    intState;
    BYTE    dummy;


    intState = INTCONbits.GIEL;
    INTCONbits.GIEL = 0;             // Disable low priority interrupts whilst using SPI, as common I/O pins may be used by ISR

    SSPCON1bits.SSPEN = 1;          // Enable SPI
    MX_CS_IO = 0;                   // Enable MAX chip
    SSPBUF = mxRegister;            // Send register address
    WaitForDataByte();              // Wait for transfer to complete
    SSPBUF = mxValue;               // Send data value
    WaitForDataByte();
    MX_CS_IO = 1;                   // Transfers sent data into register
    MX_CS_IO = 0;                   // Next command
    SSPBUF = MX_NOP;                // Finish with a nop so subsequent transitions on CS cause no problem
    WaitForDataByte();
    SSPBUF = 0;
    WaitForDataByte();
    MX_CS_IO = 1;                   // Latch NOP command into MAX chip

    SSPCON1bits.SSPEN = 0;          // Disable SPI so pins can be used for other things

    INTCONbits.GIEL = intState;
}