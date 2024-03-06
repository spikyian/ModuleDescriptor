/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - Button/Switch matrix scanning

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

#include "buttonscan.h"

#pragma romdata

// Keypad lookup table for single key presses - note that the order of this table depends on the PCB layout

BYTE    singleKeyLookupTable[] = { 0,0 }; // KEY_LOOKUPS;

KeyCombinationLookupEntry   keyCombinationLookupTable[] = { 0,0 }; //  COMBINATION_LOOKUPS;

#pragma code APP

KeypadStatus    keyStatus;

void initKeyscan(void)
{
    BYTE i;
    
    keyStatus.status.statusByte = 0;
    
    for (i=0; i<COLUMN_OUTPUTS; i++)
        keyStatus.buttonState.stateArray[i] = COLUMN_MASK;  // No buttons presseed (button inputs are active low)  ??? NEED TO ALLOW FOR MULTIPLE MASKS
    
    // Keypad strobe  output pins - intialise all high
    
    COL_LAT |= COLUMN_MASK;  
            
    #ifdef COL_LAT2
        COL_LAT2 |= COLUMN_MASK2
    #endif
            
   
    
    // Keypad strobe  input pins
    KBD_INP0_TRIS = 1;
    KBD_INP1_TRIS = 1;
    KBD_INP2_TRIS = 1;
    KBD_INP3_TRIS = 1;

    #if ROW_INPUTS >3
        KBD_INP3_TRIS = 1;
        KBD_INP4_TRIS = 1;
        KBD_INP5_TRIS = 1;
        KBD_INP6_TRIS = 1;
        KBD_INP7_TRIS = 1;
    #endif

}

BYTE keyScan( void )

// This routine does one scan of the keypad, updating any debounce counts. It returns the value
// of the currently pressed key, when the press has been debounced.
// IF no valid key (or combination) is  pressed, or there has been no chnage, then 0xFF is returned.

// NOTE: this code is optmiased by using specific binary patterns for the bit masks.
// Thse are defined in matrix.h which is application hardware specific.
// You need to generate a matrix.h file specific to your hardware
// If there are paticular hardware differenes, conditional code changes may also be required

// The keypad control pins are shared with the LCD output and the SPI clock.
// The LCD routines must leave LCD CS disabled
// The SPI routines must leave all SPI device CS signals disabled and the SPI module disabled to enable normal I/O operation
// Interrupts are disabled during the strobe out and read in sequence in case LCD or SPI operations happen in the ISR

// This routine does NOT block during the debounce, and should be called repeatadly from the main loop,
// It will return the new keycode when the debounce period has completed.

{
    BYTE        strobeCount;
    BYTE        strobeMask;
    BYTE_BITS   strobedValue;
    MatrixState newButtonState;
    BYTE        returnCode;
    BOOL        oneButtonPressed;
    BOOL        multiButtonsPressed;
    BYTE        buttonNum, colNum, rowBits, rowNum;
    BOOL        intState;
    


    newButtonState.stateVal[0] = 0;
    newButtonState.stateVal[1] = 0;
    oneButtonPressed = FALSE;
    multiButtonsPressed = FALSE;
    buttonNum = colNum = 0;
    rowBits = 0xFF;

    for ( strobeCount = 0; strobeCount < COLUMN_OUTPUTS; strobeCount++)
    {
       
        strobeMask = ~((0b00000001 << strobeCount) & COLUMN_MASK);    // Shifting column from bit 0

        // disable interrupts during the keypad strobe because one strobe line is
        // shared with SPI and we might get the wrong answer if it is waggled by
        // the ISR reading a radio message via SPI.

        intState = INTEN;
        INTEN = 0;

        COL_LAT |= COLUMN_MASK;                                 // Set all strobe column bits
        COL_LAT &= strobeMask;                                  // Clear strobe bit to active low for this column

        #ifdef  COL_LAT2
            KBD_STROBE6 = (strobeCount != (COLUMN_OUTPUTS-1));  // Column 6 on a different port NOTE: this is not generic at the moment, just defined to work on wicab
        #endif

  
        // Read in the value strobed by the outputs

        strobedValue.Val = 0;
        strobedValue.bits.b0 = KBD_INP0;
        strobedValue.bits.b1 = KBD_INP1;
        strobedValue.bits.b2 = KBD_INP2;

        #if ROW_INPUTS > 3
            strobedValue.bits.b3 = KBD_INP3;
            strobedValue.bits.b4 = KBD_INP4;
            strobedValue.bits.b5 = KBD_INP5;
        #endif
                
        #if ROW_INPUTS > 6
            strobedValue.bits.b6 = KBD_INP6;
            strobedValue.bits.b7 = KBD_INP7;
        #endif       
                

        INTEN = intState; // Put interrupts back how they were

        #ifdef RETURN_LOOKUP
            if (strobedValue  != 0x07)     // 7 = no buts   3,5,6 = 1 but  1,2,4 = 2 buts   0 = 3 buts
            {
                multiButtonsPressed = oneButtonPressed;
                oneButtonPressed = TRUE;

                if ((strobedValue == 3) || (strobedValue == 5) || (strobedValue == 6) ) // single button pressed on this column
                {
                    buttonNum = !strobedValue;
                    if (buttonNum == 4)
                        buttonNum = 3;
                    buttonNum *= strobeCount;
                }
                else
                    multiButtonsPressed = TRUE;
            }
        #endif
        #ifdef RETURN_ROW_COLUMN
            if (strobedValue.Val != ROW_MASK )
            {
                colNum = strobeCount;
                rowBits = ~strobedValue.Val;
            }
        #endif

    
     //   ??? need to think about option of delayed setup time

        newButtonState.stateArray[strobeCount] = strobedValue.Val;     // Build up a complete map of the current state of the keypad buttons
    }  // for each column strobe group

    returnCode = 0xFF; // assume no change to start with

    // if (multiButtonsPressed)
    //    newButtonState |= 0x8000000;    // msbit set flags multiple buttons  ???

    if (keyStatus.status.deBouncing)
    {    
        if (matrixEquals(newButtonState, keyStatus.pendingState))       // Still the same as when we started debouncing?
        {
            if (tickTimeSince( keyStatus.debounceStart) > KEY_DEBOUNCE_TIME)
            {
                keyStatus.buttonState.stateVal[0] = keyStatus.pendingState.stateVal[0];
                keyStatus.buttonState.stateVal[1] = keyStatus.pendingState.stateVal[1];
                keyStatus.status.deBouncing = FALSE;
                
                #ifdef RETURN_LOOKUP
                    returnCode = keyLookup( buttonNum, newButtonState ); 
                #endif

                #ifdef RETURN_ROW_COLUMN
                   if (rowBits != 0 && rowBits != 0xFF) // 0xFF means this is a button release ??? will need option of off events later
                   {
                        for (rowNum = 0; (rowBits & 0x01) != 0x01; rowNum++)
                            rowBits >>= 1;  // Count the bit number that is set
                        returnCode  = (rowNum << 4) + colNum;
                   }     
                #endif
            }
        }
        else
        {
            if (matrixEquals(newButtonState, keyStatus.buttonState))    // value reverted, so cancel debounce
                keyStatus.status.deBouncing = FALSE;
            else                                            // value changed during debounce, so restart debounce
            {
                keyStatus.pendingState.stateVal[0] = newButtonState.stateVal[0];
                keyStatus.pendingState.stateVal[1] = newButtonState.stateVal[1];
                keyStatus.debounceStart.Val = tickGet();
            }
        }
    }
    else // no debounce in progress - see if anything has changed
    {
        if (matrixChanged(newButtonState, keyStatus.buttonState))
        {
            keyStatus.pendingState.stateVal[0] = newButtonState.stateVal[0];
            keyStatus.pendingState.stateVal[1] = newButtonState.stateVal[1];
            keyStatus.status.deBouncing = TRUE;
            keyStatus.debounceStart.Val = tickGet();
        }
    }
    return ( returnCode );

}   // keyscan



#ifdef RETURN_LOOKUP

BYTE keyLookup( BYTE buttonNum, MatrixState buttonState )

// Return lookup keycode value as a BYTE from the strobe results array

{
    BYTE keyCode;
    BYTE i;

    if ((buttonState & 0x8000000) == 0)  // Single or no key pressed
    {
        // Look up single key press
        return( singleKeyLookupTable[buttonNum] );
    }
    else // key combination
    {
        for (i = 0; (i< KEY_COMBINATIONS) && (buttonState != keyCombinationLookupTable[i].buttonState) ; i++);
        return(keyCombinationLookupTable[i].keycode);
    }
}

#endif










