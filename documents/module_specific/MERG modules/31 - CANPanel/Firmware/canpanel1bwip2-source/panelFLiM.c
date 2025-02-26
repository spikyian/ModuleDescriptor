/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - FLiM related routines specific to the CANPanel

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
#include "ticktime.h"


#pragma romdata PARAMETERS

#define PRM_CKSUM MANU_ID+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+MODULE_FLAGS+CPU+PB_CAN +(LOAD_ADDRESS>>8)+(LOAD_ADDRESS&0xFF)+CPUM_MICROCHIP+BETA+sizeof(ParamVals)+(MNAME_ADDRESS>>8)+(MNAME_ADDRESS&0xFF)


const rom ParamVals     FLiMparams = { MANU_ID, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM, MAJOR_VER, MODULE_FLAGS, CPU,PB_CAN,LOAD_ADDRESS,0,CPUM_MICROCHIP,BETA  };
const rom SpareParams   spareparams;
const rom FCUParams     FCUparams   = { sizeof(ParamVals),(DWORD)&module_type,(WORD)PRM_CKSUM };
const rom char          module_type[] = MODULE_TYPE;


// Event numbers for transmitted CBUS events

#define AMPERAGE_EVENT  1

// Node and event variables at a fixed place in ROM, starting on a segment boundary
// so they can be written to as required 

#pragma romdata	flimdata	// Node and event variables

const rom NodevarTable	nodeVarTable = { 0,0,0,0,8,0,8,8,10,0,0,0,0,0,0,0,0xFF,0xFF };           
const rom EventTableEntry  eventTable[EVT_NUM];

#pragma romdata


// Static RAM variables 


#pragma udata MAIN_VARS

rom     ModuleNvDefs    *NV;          // Pointer to node variables structure
BOOL	NV_changed;
BOOL	FLiMFlash;			// LED is flashing
BOOL	FlashStatus;			// Control flash on/off of LED during FLiM setup etc


#pragma udata 


#pragma code APP


// cmdFLimInit called during initialisation 

void	panelFlimInit(void)

{
	BYTE        i;
	
  	FLiMInit(DEFAULT_NODE_ID);

        // Initialise node variables

	NVPtr = &(nodeVarTable.nodevars[0]);         // Node Variables table
    NV = (rom ModuleNvDefs*) NVPtr;
	EVTPtr = &(eventTable[0]);           // Event table
	NV_changed = FALSE;
        FlashStatus = FALSE;

 
} // PanelFlimInit


//void SaveNodeDetails(WORD Node_id, BOOL	FLiMmode)
//
//{
//    ee_write_short(EE_Node_id, Node_id);
//    ee_write(EE_FlimMode, FLiMmode);
//} // SaveNodeDetails




// Blink green LED on to show busy doing commands, but overidden by flash if in FLiM setup mode

BYTE BlinkLED( BOOL blinkstatus )


{
	BYTE LEDstatus;

	if (FLiMFlash)
	{
		LEDstatus = FlashStatus;
		// FlashStatus = !FlashStatus;
	}
	else
		LEDstatus = blinkstatus;

	return( LEDstatus ? 1 : 0 );
	
} // BlinkLED




