----------------------------------------------------------------------------------------------
CANACE8C/CANTOTI/CANACE8MIO v2qBeta3 Notes

Integrated CANMIO changes from v2pBeta3 and v2qBeta2 versions.

Phil Wheeler, 14-Dec-16

----------------------------------------------------------------------------------------------
CANACE8C/CANTOTI/CANACE8MIO v2pBeta3 Notes

Changes from v2pBeta1:

Fixed an interrupt race condition with some fast changing inputs.
	
Phil Wheeler, 9-Dec-16

----------------------------------------------------------------------------------------------
CANACE8C/CANTOTI/CANACE8MIO v2pBeta1 Notes

Changes from v2n:

Fixed a few minor bugs with Route logic

Rewritten output routines to correct ONONLY oddities

----------------------------------------------------------------------------------------------
CANACE8C/CANTOTI/CANACE8MIO v2n Notes

Changes from v2k:

Added "Inhibit SOD" for each input. Requires FlimConfig_1_4_7_19

If an input is configured as ON ONLY, a SOD won't be sent if the input is OFF.

Input changes will not produce any events until 2000mS after power up

Updated parameter block and include files to latest standards.

Added CANACE8MIO variant.

Bus events are now monitored to keep push button toggle inputs in step.

Corrects spurious events that could be sent with v2k at powerup.

The Self Enumeration on Conflict code introduced in v2m has been made optional.
 Until some issues with this and CANCANare resolved, this is not enabled by default.
 If it is required, set the AUTOID equate at the top of the file.
 (Bob V. M660)

One extra NV has been added, which controls the behaviour of the 'Route Event on input change'
 Value of zero module behaviour unchanged.
 Value of 0xFF (255) generates an ON event on every change <0x90><NNHi><NNLo><0x02><Input code>
 in addition to any actual change event.
 It is expected that any other value consists of a single bit set but no check is made to see 
 that this is the case.
 When a change occurs on an input who's bit is set within NV7 the 'Route Event' generated
 will be On or Off according to the change that has taken place, since the trigger input value
 is a part of the <Input code> the On and Off events will be unique.
 This will prevent the inverse trigger event from undoing the wanted event actions.
 (Bob V. M660)

Revisions v2l and v2m were never formally released.

There are now 3 variants of the hex file:

canace8c_xx.hex   -  Standard CANACE8c firmware
canatoti_xx.hex   -  CANTOTI firmware which is a variant of canace8c for use with train on track indicators
canace8mio_xx.hex -  CANACE8c firmware to run on the CANMIO board

Where xx is the version number.

NOTES FOR DEVELOPERS:

CANACE8C_v2n.asm is used to compile the firmware for CANACE8C
CANACE8MIO_v2n.asm is used to for ACE8C functionality using CANMIO hardware, and uses CANACE8C_v2n.asm
CANTOTI_v2n.asm is used to compile the firmware for CANTOTI, and uses CANACE8C_v2n.asm

You can also build individual variants by setting macro definitions in MPLAB as follows:

1. Select menu option Project->Build Options->Project
2. Select the MPASM assembler tab
3. Click the ADD button to add a macro definition.
4. For the variants, add macros as follows:
	canace8c    - none
	cantoti     - CANTOTI
	canace8mio  - CANMIO
	cantotimio  - both CANTOTI and CANMIO (add as two separate macros)
	
Note that when building in the IDE, the hex file will always be called the same name as the source file.	
	
Phil Wheeler, 17-Jun-14

----------------------------------------------------------------------------------------------
CANACE8C/CANTOTI v2k Notes

The CANACE8C/CANTOTI firmware has been extended to optionally allow a push button to be wired
to an input. Each time this is pressed, it will alternately send an ON or OFF event.

FCU V1.4.7.9 or later is needed to configure the firmware.

Push Button mode is processed after any input inversion or input delays.

If a SOD is received, inputs set to push button mode will NOT send their current status as
an event, but an internal flag will be reset so the next push will send an ON event.

CANACE8C_v2k.asm is used to compile the firmware for CANACE8C
CANTOTI_v2k.asm is used to compile the firmware for CANTOTI, and uses CANACE8C_v2k.asm

Phil Wheeler, 28-Jul-13

----------------------------------------------------------------------------------------------
CANTOTI Notes

The CANTOTI firmware is an extension of the CANACE8C firmware and is designed to run
on CANACE8C hardware. FlimUtility V1.4.5.10 or later is required to configure the CANTOTI.

Unlike previous versions of CANACE8C firmware, this samples each input every 10mS, and
will, by default, generate an event when the input is the same for more than
two 10mS samples.

The inputs can be optionally inverted to allow for "Active High" inputs.

To allow for noisy signals being connected to the CANTOTI, optional delays can be enabled.

If "Delayed input" is enabled for an input, the input must be continously ON for
at least the "On time" before an ON event is generated, and must be continously OFF
for at least the "Off time" before an OFF event is generated.

The "On time" and "Off time" are set in units of 10mS, with a minimum time of 10mS and
a maximum time of 2540mS.

The "Extended Configuration" value is not currently used; ensure this is set to 0 for
future upgrade compatibility.

Phil Wheeler, 14-Oct-12

----------------------------------------------------------------------------------------------
