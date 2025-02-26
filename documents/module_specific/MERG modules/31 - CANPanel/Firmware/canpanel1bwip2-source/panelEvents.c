/*

 Copyright (C) Pete Brownlow 2014-2015   software@upsys.co.uk

 	Event processing specific to the CANPanel

	These routines implement the CANPanel behaviour in response to events

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

   ** For commercial use, please contact the orignal copyright holder(s) to agree licensing terms

**************************************************************************************************************
  Note:   This source code has been written using a tab stop and indentation setting
          of 4 characters. To see everyting lined up correctly, please set your
          IDE or text editor to the same settings.
******************************************************************************************************

 Revision History

	11/6/16	Pete Brownlow	- Initial outline

*/

#include "canpanel.h"
#include "ticktime.h"



#pragma code APP



// Blink green LED on to show busy doing commands, but overidden by flash if in FLiM setup mode

//BYTE showActivity( BOOL blinkstatus )
//
//
//{
//	BYTE LEDstatus;
//
//	if (FLiMFlash)
//	{
//		LEDstatus = FlashStatus;
//		// FlashStatus = !FlashStatus;
//	}
//	else
//		LEDstatus = blinkstatus;
//
//	return( LEDstatus ? 1 : 0 );
//	
//} // BlinkLED

// process an incoming CBUS event. The eventIndex gives access to the event entry with Node Variables and Event Variables.
// The msg parameter gives access to the original CBUS message

// This is where we join up all the generic message processing with the specific CANPanel routines to light LEDs

BOOL processEvent( BYTE eventIndex, BYTE *msg )

{
    BYTE                    ledIndex;
    rom ModuleEventEntry    *EVMPtr;
    EventFlags              ledFlags;
    BOOL                    ledState;
    BYTE                    ledNumber;
    BOOL                    turnLedOn, turnLedOff, makeLedFlash, thisOnEvent, processAsOn, moreEntries;
    
    
    
    EVMPtr = (ModuleEventEntry*)EVTPtr;
    
    // No action here if it is a producer event
    if (!(EVTPtr[eventIndex].event.evtFlags.producerEvent))
    {
        
    }
    else
    {
        ledIndex = 0;
        moreEntries = TRUE;
         
        while (moreEntries)
        {
            ledFlags = EVMPtr[eventIndex].modEvs[ledIndex].panelEventFlags;
            thisOnEvent = ((msg[d0] & EVENT_ON_MASK)  == EVENT_ON_MASK); // check opcode for on event
            
            if ((thisOnEvent && ledFlags.onEvent) || (!thisOnEvent && ledFlags.offEvent))
            {    
                processAsOn = (thisOnEvent != ledFlags.polarity);

                makeLedFlash = ledFlags.ledOn && ledFlags.ledOff && processAsOn;
                turnLedOn = ledFlags.ledOn && !ledFlags.ledOff && processAsOn;
                turnLedOff = ledFlags.ledOff && !processAsOn;
                ledNumber = EVMPtr[eventIndex].modEvs[ledIndex].ledNumber;
                
                if (makeLedFlash)
                    flashLed( ledNumber );
                else if (turnLedOn)
                    setLed( ledNumber, TRUE );
                else if (turnLedOff)
                    setLed( ledNumber, FALSE );
            }
               
            if (moreEntries = !ledFlags.endOfList)
            {    
                ledIndex += 2;

                if (ledIndex >= EVperEVT / sizeof(ModuleEvsEntry))  
                {
                    // Link to next continuation entry if one exists
                    if (moreEntries = EVTPtr[eventIndex].event.evtFlags.conctinues)
                    {
                        // step to next continuation entry
                        eventIndex = findEventContinuation(eventIndex);
                        ledIndex = 0;        
                        moreEntries = (eventIndex != 0) && (EVTPtr[eventIndex].event.evtFlags.continuation);   
                    } 
                }
            } // if not end of list   
        } // while more entries   
    }

}


