CANACE3C Readme

CANACE3C_v3a

Formal Release.

Now using "cbusdefs8h.inc". Returns physical PIC in node properties.

PJW 16-Jun-14

----------------------------------------------------------------------------------------------
CANACE3C_v3a8 Beta

This fixes a bug with monitoring events for Pushbutton Toggle mode.
An auto-generated OFF SOD event will be sent after the same delay for an ON SOD event.

PJW 23-Feb-14

----------------------------------------------------------------------------------------------
CANACE3C_v3a7 Beta

This fixes a couple of bugs with processing a SOD.
It also has revisions to the AUTOID logic; this is still not enabled by default

PJW 20-Feb-14

----------------------------------------------------------------------------------------------
CANACE3C_v3a3 Beta

This firmware is a rewrite of CANACE3_v2f code, providing enhanced
facilities for 'next generation' control panels.

Existing CANACE3_v2f  do not need to be upgraded to this firmware as
this will involve reconfiguring the firmware and re-entering all events.

However, with suitable reconfiguration, this firmware can be used with
any existing control panel hardware.

With this firmware, the 128 potential switches are divided into 8 blocks
of 16 switches, each of which can be independently configured for
different types of switches and operating modes.

See the .pdf for detailed information.

Note that this release requires FCU version 1.4.7.16 or later for configuration.


PJW 10-Feb-14

----------------------------------------------------------------------------------------------
