/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - I/O definitions specific to the CANPanel

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

#ifndef __IO_H
#define __IO_H

// Define I/O bit usage for the CANPanel

// RA4 not available on K series processor
// RA6,RA7 not available as they are used for oscillator pins

// Control the LED driver chip, MAX6951
// These  pins are configured as SPI port when talking to the MAX chip

#define LEDS_CS         PORTCbits.RC6   // Chip select
#define LEDS_CLK        PORTCbits.RC3   // SPI clock
#define LEDS_DATA       PORTCbits.RC5   // SPI data


// Button/Switch matrix inputs

#define BUTTON_ROWS     PORTC           // Row outputs, also shares with MAX chip control lines
#define BUTTON_COLS0    PORTB           // Column inputs, will define mapping scheme
#define BUTTON_COLS1    PORTA

// Data direction register initialisation values
// 1=input   0=output

#define PORTA_DDR   0b00111111     // All available port A pins are inputs, FLiM switch and cols
#define PORTB_DDR   0b00110011     // Available B port pins are inputs, the ICSP pins double as status LED outputs
#define PORTC_DDR   0b00000000     // All outputs - switch rows (Some pins reconfigured as SPI when writing to MAX chip)



void initIO(void);


//void writeOutput(int port, unsigned char val);
//unsigned char readInput(int port);
//void doIOTimers(void);
//void doLEDTimers(void);
//void doLEDs(void);
//void setOutput(ushort nn, ushort addr, byte on);


#endif	// __IO_H
