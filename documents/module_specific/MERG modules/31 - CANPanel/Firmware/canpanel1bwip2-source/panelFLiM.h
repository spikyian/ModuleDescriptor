/*

 Copyright (C) Pete Brownlow 2014-2017   software@upsys.co.uk

  CBUS CANPanel - Definitions for FLiM operations on CANPanel

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

// Created on 24 August 2012, 19:51


#ifndef PANELFLIM_H
#define	PANELFLIM_H

#include "FliM.h"

#define NV_NUM		sizeof(ModuleNvDefs)     // Number of node variables
#define EVT_NUM		255                     // Number of events

#define DEFAULT_NN 	130

#define NUM_PBS         64
#define NUM_LEDS        64


// Node Variable definitions

typedef enum
{
    testOff = 0,
    displayTest,
    displayMSG,
    displayFIFO,
    transmitTest,
    randomTest,        
    backToStart
} ModuleTestMode;

typedef struct
{

    ModuleTestMode  panelTestMode:3;
    BOOL            startInTest:1;
    BOOL            sequenceTest:1;
    BOOL            incTestBlock:1;     // Set true to increase (double) block size each test output
    BOOL            halveTestIval:1;    // Set true to halve the packet interval time each test output

} TestFlags;

typedef union
{
    struct
    {
        BOOL    sayHello:1;         // Display Hello (if 7 segment displays selected)
        BOOL    flashAtStart:1;     // Flash all LEDs at startup
        BOOL    syncFlipFlops:1;    // Monitor events to sync flip flop states
        BOOL    unitialised:1;      // Set to 1 by default memory, so knows to initialise NVs
        BYTE    scanDelay:4;        // Delay on each button scan in mS 1-15, set to zero for default scanning
    };
    BYTE    flagByte;
} PanelFlags;



// Structure to define which outputs are driving 7 segment display digits as opposed to discrete LEDs

typedef union
{
    struct
    {
        BOOL    segDisplay0:1;     // This block is 7 segment display
        BOOL    segDisplay1:1;     // This block is 7 segment display
        BOOL    segDisplay2:1;     // This block is 7 segment display
        BOOL    segDisplay3:1;     // This block is 7 segment display
        BOOL    segDisplay4:1;     // This block is 7 segment display
        BOOL    segDisplay5:1;     // This block is 7 segment display
        BOOL    segDisplay6:1;     // This block is 7 segment display
        BOOL    segDisplay7:1;     // This block is 7 segment display
    };
    BYTE    segByte;
} SegOutputs;


typedef union
{
    struct
    {
        BOOL    sendOn:1;       // Send on events on make
        BOOL    sendOff:1;      // Send off events on break
        BOOL    polarity:1;     // Inverts make/break event polarity
        BOOL    flipflop:1;     // Send alternately on/off events on make (flip-flop mode)
        BOOL    incInSod:1;     // Include button state in start of day response
        BOOL    uninitialised:1; // Default virgin state is FF, so if this bit set will initialise button array
    };
    BYTE    flagByte;
} PbFlags;

typedef PbFlags PbSettings[NUM_PBS];

typedef struct
{
        BYTE            sendSodDelay;                   // Time after start in 100mS (plus 2 seconds) to send an automatic SoD. Set to zero for no auto SoD
        BYTE            hbDelay;                        // Interval in 100mS for automatic heartbeat. Set to zero for no heartbeat.
        PanelFlags      panelFlags;                     // Global operation flags
        SegOutputs      segDisplays;                    // Which groups of 8 are 7 segment displays
        BYTE            brightness;                     // LED brightness setting - 16 levels in 4 bits
        TestFlags       testFlags;                      // Test mode settings
        BYTE            testFrameSize;                  // Size of each test frame to send, from 1 to 8 data bytes
        BYTE            testBlockSize;                  // Initial test block size, will be doubled each time if incTestBlock is set
        BYTE            testFrameDelay;                 // Delay, in 100uS units, between frame transmissions (set 0 for max rate continuous)
        BYTE            spareNV[7];
        PbSettings      pbSettings;                     // Array of settings one byte per pushbutton
} ModuleNvDefs;


typedef union
{
        NodeBytes   	nodevars[NV_NUM];
        ModuleNvDefs    panelNVs;
} NodevarTable;


// Event table definitions


typedef union
{
    struct  // Consumer flags
    {
        BOOL    ledOn:1;        //__ Both on indicates flash
        BOOL    ledOff:1;       //
        BOOL    polarity:1;
        BOOL    onEvent:1;
        BOOL    offEvent:1;
        BOOL    routeEntry:1;
        BOOL    routeExit:1;
        BOOL    endOfList:1;    // Set on last entry in EV list
    };
    struct  // Producer flags
    {
        BYTE    buttonNum:6;
        BOOL    spare:1;
        BOOL    endOfList:1;    // Set on last entry in EV list
    };
    BYTE flagByte; // or access as Byte
    
} EventFlags;

typedef struct
{
    EventFlags  panelEventFlags;
    BYTE        ledNumber;
} ModuleEvsEntry;    

typedef ModuleEvsEntry  ModuleEvs[EVperEVT / sizeof(ModuleEvsEntry)];

typedef struct
{
    EventEntry  event;
    ModuleEvs   modEvs;
} ModuleEventEntry;


extern rom     ModuleNvDefs    *NV;          // Pointer to node variables structure


void	panelFlimInit(void);



#ifdef	__cplusplus
}
#endif

#endif	/* PANELFLIM_H */

