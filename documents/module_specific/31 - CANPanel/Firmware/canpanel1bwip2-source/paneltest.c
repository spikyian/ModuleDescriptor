/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - test routines

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

	
 Revision History

    27/11/15 Ver 1b - these routines moved here from main CANPanel source file
    30/12/16    PNB - Add single button presses for each test mode, add random number gen test
    25/01/17    PNB - Update header, update docs below

 To put CANPanel into test mode, hold down FLiM button until it flashes (about 5 seconds), keep it down until fast flashes (about 10 seconds) then release.
 To take out of test mode, repeat the above.
 
 There are several test modes, which are cycled through by pressing the FLiM pushbutton for about 1 second. Within each test, a short press will do an
 action which depends on the test in use:

 Test number:
 
  1. Exercises LEDs with a sequence of
          ALL ON (Maxim test mode)
          Form large X (if using TM6 test baord)
          Display "Hello"
          Display 01234567
          Sequence through each LED (or segment) in turn, two cycles
    A short press will restart the sequence.
 
  2. CBUS message display, shows first 4 hex bytes of most recent CBUS message received, in real time. 
 
  3. Receive CAN status: Real time display of number of messages received, max software RX FIFO usage, current software RX FIFO usage and RX FIFO overflow count.
     A short press resets the counters.
 
  4. Transmit test mode: Sends a block of CBUS messages (number and speed defined in canpanel.h - later will make a Node variable),
     real time display of number of messages sent, max software TX FIFO usage, current software TX FIFO usage and TX FIFO overflow count. 
    A short press repeats the transmission. OPtionally increases count and/or decreases delay between messages according to canpanel.h
 
  5. Random number generator - generates 1 to 8 in a random order. Seeds from free running tick timer.

 The various messages will be displayed when 8 digits of 7 segment display are connected. If you have LEDs connected, such as the TM6 test board, then
 a pattern of LEDs will be displayed corresponding to those display segments.

   Note:   This source code has been written using a tab stop and indentation setting
          of 4 characters. To see everything lined up correctly, please set your
          IDE or text editor to the same settings.

*/

#include "paneltest.h"       
#include "stdlib.h"         // For random number generation


#pragma code APP
 
TickValue   startTime, stateTime; // , timeNow;
WORD_VAL    testStatus;
WORD        txBlockSize;

  
void panelTestInit(  )

{
    mainStatus.panelMode = testOff;
    mainStatus.testCount = 0;
    mainStatus.passCount = 0;
    mainStatus.eventCount = 0;
}
               
void panelTest()
{
    
    BYTE    dispCount, checkCount;
    char    displayContents[9];
    BOOL    unique;
    
    switch (mainStatus.panelMode)
    {
        case displayTest:
            if (mainStatus.testInput)
            {
                mainStatus.testCount = 0;
                mainStatus.testInput = FALSE;
            }    
            
            switch (mainStatus.testCount)
            {
                case 0:
                    clearAllLeds();
                    setLedTestMode( TRUE );
                    stateTime.Val = tickGet();
                    mainStatus.testCount++;
                    break;

                case 1:
                    if (tickTimeSince(stateTime) > TEST_MODE_TIME)
                    {
                        setLedTestMode( FALSE );
                        clearAllLeds();
                        sayHello();             // Message on 7 seg displays
                        stateTime.Val = tickGet();
                        mainStatus.testCount++;
                    }
                    break;
                    
                case 2:
                    if (tickTimeSince(stateTime) > HELLO_TIME)
                    {
                        displayVersion();            // Version no. on 7 seg displays
                        stateTime.Val = tickGet();
                        mainStatus.testCount++;
                    }
                    break;
                case 3:
                    if (tickTimeSince(stateTime) > HELLO_TIME)
                    {
                        clearAllLeds();
                        displayDigit( 0 , 0 );
                        displayDigit( 1 , 1 );
                        displayByte( 0x23, 2 );
                        displayNumber( 0x4567, 4, 4, 0 );
                        stateTime.Val = tickGet();
                        mainStatus.testCount++;
                    }
                    break;

                case 4:
                    if (tickTimeSince(stateTime) > TEST_NUM_TIME)
                    {
                        clearAllLeds();
                        showTestX();
                        stateTime.Val = tickGet();
                        mainStatus.testCount++;
                    }
                    break;

                case 5:
                    if (tickTimeSince(stateTime) > TEST_X_TIME)
                    {
                        testStatus.Val = 0xFFFFFFFF;    // Set start of test
                        mainStatus.passCount = 0;
                        testStatus = ledTestCycle( testStatus);        // Start test of each LED in turn
                        stateTime.Val = tickGet();
                        mainStatus.testCount++;                    }
                    break;

               case 6:
                    if (tickTimeSince(stateTime) > TEST_LED_TIME)
                    {
                        if ((testStatus.Val == 0) && (++(mainStatus.passCount) >= TEST_LED_PASSES))  // WARNING!! This line depends on ++ only being executed if first test passes, so may be compiler or optimisation dependant
                            mainStatus.testCount = 0;
                        else
                        {
                            testStatus = ledTestCycle( testStatus);        // test of next LED in turn
                            stateTime.Val = tickGet();
                        }
                    }
                    break;
            } // switch
            break;

        case displayFIFO:
            
            if (mainStatus.testInput)
            {
                mainStatus.testInput = FALSE;
                mainStatus.eventCount = 0;
                maxCanRxFifo = 0;
                rxFifoUsage = 0;
                rxOflowCount = 0;
            }    
                
            if (mainStatus.msgReceived)
            {
                mainStatus.eventCount++;
                mainStatus.msgReceived = FALSE;
            }


            // This part is CAN specific - displays CAN RX FIFO information on the 7 segment display
            displayByte(mainStatus.eventCount,0);
            displayByte(maxCanRxFifo,2);
            displayByte(rxFifoUsage,4);
            displayByte(rxOflowCount,6);

            break;

        case transmitTest:
            // Transmit a block of CBUS event messages
 
            if (mainStatus.eventCount == 0)
            {
                mainStatus.testCount = 0;
                txBlockSize = NV->testBlockSize;
                mainStatus.eventCount++;
            }    
            
            if (mainStatus.testInput) 
            {
                mainStatus.testInput = FALSE;
                mainStatus.testCount = 0;
                mainStatus.eventCount++;
 
                if (NV->testFlags.incTestBlock)
                    txBlockSize *= 2;
            }    

            if ((mainStatus.testCount < txBlockSize) && (tickTimeSince(stateTime) > (NV->testFrameDelay*HUNDRED_MICRO_SECOND)))
            {
                cbusSendEvent(0,-1,mainStatus.testCount++,TRUE);
                stateTime.Val = tickGet();
            }


            // This part is CAN specific - displays CAN TX FIFO information on the 7 segment display

            displayByte(mainStatus.testCount,0);
            displayByte(maxCanTxFifo,2);
            displayByte(txFifoUsage,4);
            displayByte(txOflowCount,6);
            break;
        
        case randomTest:
            // Generate 8 random digits (eg: shunting puzzle)
            
            if (mainStatus.testInput || (mainStatus.eventCount == 0 )) 
            {
                mainStatus.eventCount = 1;
                mainStatus.testInput = FALSE;
                srand((TMR_H << 8) + TMR_L );   // Seed the random number generator from the free running timer
                
                // Loop for each digit - hard code 8 digits for now
                
                for (dispCount = 0; dispCount <8; dispCount++)
                {   
                    do // Check generated digit is different to all previous digits
                    {    
                        displayContents[dispCount] = (rand() & 0x07) + 0x31;   // Generate a random Ascii digit between 1 and 8)

                        unique = TRUE;

                        for (checkCount = 0; ((checkCount < dispCount) & unique); checkCount++)
                            unique = displayContents[dispCount] != displayContents[checkCount];
                    } while (!unique);
                }
                displayContents[8] = 0; // Terminate the string (put this at an earlier char position to only show the first N digits)
                displayString(displayContents,0);
            }   
            break;
    } // switch panel mode
} // panelTest

