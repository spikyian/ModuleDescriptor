/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - I/O routines specific to the CANPanel

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

#include "canpanel.h"

void initIO(void) {
    
    INTCON = 0;     // Disable all interrupts to start with
    INTCON2 = 0;
    INTCON3 - 0;

    EECON1 = 0;     // EEPROM control to known start state

    // Initialise all interrupt priorities to low

    IPR1 = 0;
    IPR2 = 0;
    IPR2 = 0;
    IPR3 = 0;

    // Initiallise all peripheral interrupt enables to off

    PIE1 = 0;
    PIE2 = 0;
    PIE3 = 0;

    // Clear all interrupt flag bits

    PIR1 = 0;
    PIR2 = 0;
    PIR3 = 0;

    // Additional registers in k series

#ifdef CPUF18K
    IPR4 = 0;
    IPR5 = 0xFF;
    
    PIE4 = 0;
    PIE5 = 0;

    PIR4 = 0;
    PIR5 = 0;
#endif

    RCONbits.IPEN = 1;  // Enable interrupt priorities (when interrupts enabled)

    // All digital I/O, so disable A to D

    ADCON0 = 0x00;

#ifdef CPUF18K
    ADCON1 = 0;
    ANCON0 = 0;
    ANCON1 = 0;
#endif

#ifdef CPUF18F
    ADCON1 = 0x0F;
#endif

    ADCON2 = 0;

// Set data direction registers

    TRISA = PORTA_DDR;
    TRISB = PORTB_DDR;
    TRISC = PORTC_DDR;

// Enable internal pull-ups.

    INTCON2bits.RBPU = 0;   // Enable pull up feature on Port B
    
#ifdef CPUF18K
    WPUB = PORTB_DDR;       // Data direction bits correspond to the need for pullups
    ODCON = 0;              // Open drains all off
#endif
    
    LED1Y = LED_OFF;
    LED2G = LED_OFF;
    initStatusLeds();
}

void writeOutput(int idx, unsigned char val)
{
}

unsigned char readInput(int idx)
{
  unsigned char val = 0;
  return !val;
}


// Called every 3ms.

//void doLEDTimers(void) {
//  if (led1timer > 0) {
//    led1timer--;
//    if (led1timer == 0) {
//      LED1 = LED_OFF;
//    }
//  }
//
//  if (led2timer > 0) {
//    led2timer--;
//    if (led2timer == 0) {
//      LED2 = LED_OFF;
//    }
//  }
//
//  if (led3timer > 0) {
//    led3timer--;
//    if (led3timer == 0) {
//      LED3 = LED_ON;
//    }
//  }
//
//  if (Wait4NN) {
//    return;
//  }
//}
//
//void doIOTimers(void) {
//  int i = 0;
//}
//
//void doTimedOff(int i) {
//}


//unsigned char checkInput(unsigned char idx) {
//  unsigned char ok = 1;
//  return ok;
//}
//
//void saveOutputStates(void) {
//  int idx = 0;
//  byte o1 = 0;
//  byte o2 = 0;
//
//  //eeWrite(EE_PORTSTAT + 1, o2);
//
//
//}
//
//
//
//static unsigned char __LED3 = 0;
//
//void doLEDs(void) {
//  if (Wait4NN || isLearning) {
//    LED3 = __LED3;
//    FLIM_LEDY = __LED3;
//
//    __LED3 ^= 1;
//    led3timer = 20;
//  }
//}
//
//void setOutput(ushort nn, ushort addr, byte on) {
//}
