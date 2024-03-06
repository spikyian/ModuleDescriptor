#ifndef __CANPANEL_H
#define __CANPANEL_H

/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS firmware for CANPanel module - project definitions

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
	
CANPanel Version History

    07/01/17 Ver 1b WIP 2 - Keyscan works for pushbuttons (make = event on) - test routines moved to separate source file and various tests implemented
                            Put into FLiM and allocate a node number working, read NVs ok (although they don't do anything yet)
    21/02/15 Ver 1a WIP 1 - initial testing
    21/12/13 PNB Coding started

*/

#define MAJOR_VER 	1
#define MINOR_VER 	'b'        // Minor version character
#define BETA        2
#define WIP         2

#include "hwsettings.h"
#include <GenericTypeDefs.h>
#include <cbusdefs8m.h>
#include "panelFliM.h"
#include "StatusLeds.h"

#include "io-canpanel.h"
#include <TickTime.h>

#ifdef CANPanel
    #include "max6951.h"
#elif defined CANLED

#endif


#define MANU_ID         MANU_MERG
#define MODULE_ID       MTYP_CANPanel
#define MODULE_TYPE     "Panel"

#define MODULE_FLAGS    PF_COMBI+PF_BOOT  // Producer, consumer, boot
#define BUS_TYPE        PB_CAN
#define LOAD_ADDRESS    0x0800      // Need to put in parameter block at compile time, only known to linker so hard code here
#define MNAME_ADDRESS   LOAD_ADDRESS + 0x20 + sizeof(ParamBlock)   // Put module type string above params so checksum can be calculated at compile time

#define DEFAULT_NODE_ID 129

// Debug events sent

#define START_SOD_EVENT      0x81
// DEFINE DEFAULT VALUES

// Time delays 

#define CBUS_START_DELAY    TWO_SECOND




typedef struct
{
    BOOL    started:1;
    ModuleTestMode  panelMode;
    WORD    testCount;
    BYTE    passCount;
    BYTE    eventCount;
    BOOL    testInput;
    BOOL    msgReceived;
} PanelStatus;

extern PanelStatus     mainStatus;

void canPanelInit(PanelStatus mainStatus);


#endif	// __CANPANEL_H
