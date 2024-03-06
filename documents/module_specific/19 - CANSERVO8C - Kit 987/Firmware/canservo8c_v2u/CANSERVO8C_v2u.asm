;   	TITLE		"Code for a 8 channel servo driver  FLiM node for CBUS"
; filename CANSERVO8C_v2u.asm	 	03/11/13

;adapted from CANACC8_u and Servo4e.

; started 15/05/11. No bounce yet


; Uses 4 MHz resonator and PLL for 16 MHz clock
; The setup timer is TMR3. Used during self enumeration.
; CAN bit rate of 125 Kbits/sec
; Standard frame only except for bootloader



;Servo timer is TMR0.  Servo timing uses HPINT
;CAN input uses LPINT

;node number release frame <0x51><NN hi><NN lo>
;keep alive frame  <0x52><NN hi><NN lo>
;set learn mode <0x53><NN hi><NN lo>
;out of learn mode <0x54><NN hi><NN lo>
;clear all events <0x55><NN hi><NN lo>  Valid only if in learn mode
;read no. of events left <0x56><NN hi><NN lo>
;set event in learn mode  <0xD2><EN1><EN2><EN3><EN4><EVI><EV>  uses EV indexing
;The EV sent will overwrite the existing one
;read event in learn mode <0xB2><EN1><EN2><EN3><EN4><EVI>
;unset event in learn mode <0x95><EN1><EN2><EN3><EN4>
;reply to 0xB2. <0xD3><EN1><EN2><EN3><EN4><EVI><EV>
;Also sent if attempt to read / write too many EVs. Returns with EVI = 0 in that case

;read node parameters <0x10> Only works in setup mode. Sends string of 7 bytes as 
;<0xEF><para1><para2><para3><para4><para5><para6><para7>
;Learn NV (NVLRN) <0x96><NNhi><NNlo><NN#><NNval>
;Read back NVs by index  (NVRD) <0x71><NNhi><NNlo><NN#>
;Response (NVANS)  <0x97><NNhi><NNlo><NN#><NNval>
;Implements ENUM and CANID OpCodes

;this code assumes a one byte EV giving the servo numbers and a second EV for polarity. One bit per servo.
;EV1 = 	servo number.  Can have several servos per event.
;EV2 = 	polarity for that event
;EV3 = 	Used to teach the feedback responses. See below.
;EV4  	Available but mot used yet.
;
;		EV3.  Bit format is  ABCNNNDD
;				A must be set for a response event.
;				B If set, reverses polarity of end events
;				C If set, reverses polarity of mid point event
;				NNN is the servo number (000 to 111) for servos 1 to 8)
;				DD.  00 is event sent at ON end
;					 01 is event sent when at OFF end
;					 10 is event sent at mid travel
;					 11 used to flag a SoD, bit 7 must be 0.

;Allows for 4 NVs for module control. NV# is 1 to 4.
 
;NV1  	this sets whether the servo cuts off at end or not. A bit set is cutoff enabled.
;		default is all cutoffs enabled	
;NV2 	Sets end position at power on. Bit set is OFF end. default is all to OFF
;NV3	Sets whether a servo moves on power up.  Bit set is move. Default is all move.
;NV4	Sets whether servos operate sequentially or simultaneously.
;		A bit set is sequential for that servo. Default is all sequential.
;
;Servo positions and speeds are set in NVs 5 to 36.
;Each servo has four NVs. first is the ON end position, second is OFF end position. Values 0 to 255. 
;mid point is 127
;an ON event drives a servo to the first of the two NV settings
;an OFF event drives a servo to the second of the two NV settings
;unless the pol bit is set in the second EV
;default servo positions are all centred (127)
;Third servo NV is the speed in the ON direction
;Fourth servo NV is the speed in the OFF direction
;Speed values 0 to 7. 0 is fastest. 





;Rev 2 A   initial attempt  15 / 05 / 11
;Rev 2 B	Full version with 8 channels  20/05/11
;Rev 2 C	Changes for learn mode and testing.
;Rev 2 D	Speed lookup table moved to Flash
;Rev 2 E	Mods to ev_set for multiple events.  30/05/11
;Rev 2 F	Read NVs now checks node number
;Rev 2 G	Change to 'test' routine so only one servo affected (28/07/11  MB)
;Rev 2 H    Add WRACK to EVULN

;Rev 103a 	First version wrt CBUS Developers Guide
;			Add code to support 0x11 (RQMN) 0x0d and QNN)
;			Add code to return 8th parameter by index - Flags
;Rev 103b	Ignore extended frames in receive routine
;Rev 103c	remove 102b fix, change RXB1CON ititialistaion in setup routine
;Rev 103d	move Spdtbl to 0x2800

;Rev 104 is development version for 128 events in Flash

;Rev 104a	initial version
;Rev 104b	use FSR2 in hpint, change servo to use FSR1 for Sn_now variables
;			move Sn_now to upper half of CBLOCK 0
;Rev 104c	Checked still works OK
;Rev 104d	Removed old event code, added evhndlr_c.asm
;Rev 104e
;Rev 104f	add logging
;Rev 104g	Test build, to find bug
;Rev 104h	Save INTCON when erasing and writing Flash

;Rev v3a		First release build
;Rev v3b		Use evehndlr_c.asm include file
;Rev v3c		add call to rdfbev in rdbak
;Rev v3d		change reply to QNN to OPC_PNN
;Rev v3e		Add check for zero index in read params and corect error code
;Rev v3f		Include file now evhndlr_d.asm
;Rev v3g		Include file now evhndlr_e.asm, remove NEVER compile code
;Rev v3h		Change parameters to new format
;Rev v3j		Mods by MB for sending events when ends reached / left and event when passing centre
;		On when end reached, off when leaving. Centre is ON / OFF
;		Events user settable.  
;		Uses evhndlr_f.asm to remove error messages. Must be in project directory.
;Now changed to CANSERVO8C v1a for combi node. 
;Rev v1b		Added event for frog switching. (not a centre one)
;				Sends OFF / ON when arriving at opposite end
;Rev v2a		Sends mid point event.  Frame rate now constant. NV5 added. Follows the SVs
;Rev v2b		As 2a but with self enum as separate subroutine and added 0x5D OpCode
;				Also forced enum when button pressed in FLiM.
;				Added OpCode 0x75 for forced CAN_ID (0x75, NNhi, NNlo, CAN_ID) Limited to .99 or 0x63
;Rev v2c		Bug fixes and text update. 
;Rev v2d		More bug fixes.
;Rev v2e		Startup problem fixed.	09/08/12. Not fully tested.
;Rev v2f		More bug fixes. 14/08/12
;Rev v2g		Mod for deleting response events
;Rev v2h		Changes to startup and wait sequences 29/11/12  MB
;				Now uses ECAN and polling for CAN receive
;				ECAN RX buffer is 8 deep.
;				Has SoD capability. Teach SoD with EV3 = 00000011  (3)
;				Automatically sends a SoD on POR.
;				State saved in EEPROM after each servo change.
;				On POR goes to last saved state if NV2 bits are 0. (default)
;Rev v2j		Improved version of rev2h.  18/12/1012  MB
;				Cutoff time reduced
;				Bug fix for PB in SLiM
;				Note from testing:  If using feedback events, it needs a separate power
;				supply for the servos, otherwise it causes resets due to drop in 5V line.
;Rev v2k		Fixes for feedback event changes
;				Add midpoints on startup
;Rev v2m		No rev 2l.
;				Changed unlearn all to also erase all EEPROM
;Rev v2n		Add ORG 0xF000BE before hashnum to force EEPROM alignment (RH)
;Rev v2p		Added clear of NV5 on setup (MB) 'Test' works OK.
;Rev v2q		Fixed fault with self enumeration resulting from change to ECAN
;				Fixed problem with PB press if no NN allocated
;Rev v2r		clear EVtemp3 in test routine (RH)
;Rev v2s		Changed short event handling. Problem caused by use of ECAN (MB)
;				Now handled in 'copyev'
;Rev v2t		Mod to prevent incorrect FLiM mode if power removed during NN allocation.
;				Moved A/D setting for digital I/O to before port setup. 03/11/12 (MB) 
;Rev v2u		Changes to startup. Make the same as the K version. 06/06/15 (MB)

;end of comments for CANSERVO8C. This is FLiM only at present but still needs putting into FLiM and giving a NN.

; This is the bootloader section

;*	Filename Boot2.asm  30/10/09

;*************************************************************** * * * * * * * * * * * * * * ;*
;*	CBUS bootloader

;*	Based on the Microchip botloader 'canio.asm' tho which full acknowledgement is made.
;*	Relevant information is contained in the Microchip Application note AN247

;*
;* Basic Operation:
;* The following is a CAN bootloader designed for PIC18F microcontrollers
;* with built-in CAN such as the PIC18F458. The bootloader is designed to
;* be simple, small, flexible, and portable.
;*
;
;
;*
;* Commands:
;* Put commands received from source (Master --> Slave)
;* The count (DLC) can vary.
;* XXXXXXXXXXX 0 0 8 XXXXXXXX XXXXXX00 ADDRL ADDRH ADDRU RESVD CTLBT SPCMD CPDTL CPDTH
;* XXXXXXXXXXX 0 0 8 XXXXXXXX XXXXXX01 DATA0 DATA1 DATA2 DATA3 DATA4 DATA5 DATA6 DATA7
;*


;*
;* ADDRL - Bits 0 to 7 of the memory pointer.
;* ADDRH - Bits 8 - 15 of the memory pointer.
;* ADDRU - Bits 16 - 23 of the memory pointer.
;* RESVD - Reserved for future use.
;* CTLBT - Control bits.
;* SPCMD - Special command.
;* CPDTL - Bits 0 - 7 of 2s complement checksum
;* CPDTH - Bits 8 - 15 of 2s complement checksum
;* DATAX - General data.
;*
;* Control bits:
;* MODE_WRT_UNLCK-Set this to allow write and erase operations to memory.
;* MODE_ERASE_ONLY-Set this to only erase Program Memory on a put command. Must be on 64-byte
;*	boundary.
;* MODE_AUTO_ERASE-Set this to automatically erase Program Memory while writing data.
;* MODE_AUTO_INC-Set this to automatically increment the pointer after writing.
;* MODE_ACK-Set this to generate an acknowledge after a 'put' (PG Mode only)
;*
;* Special Commands:
;* CMD_NOP			0x00	Do nothing
;* CMD_RESET		0x01	Issue a soft reset after setting last EEPROM data to 0x00
;* CMD_RST_CHKSM 	0x02	Reset the checksum counter and verify
;* CMD_CHK_RUN		0x03	Add checksum to special data, if verify and zero checksum
;* CMD_BOOT_TEST 	0x04	Just sends a message frame back to verify boot mode.

;*	Modified version of the Microchip code by M Bolton  30/10/09
;
;	The user program must have the folowing vectors

;	User code reset vector  0x0800
;	User code HPINT vector	0x0808
;	user code LPINT vector	0x0818

;	Checksum is 16 bit addition of all programmable bytes.
;	User sends 2s complement of addition at end of program in command 0x03 (16 bits only)

;**********************************************************************************



;	
; Assembly options
	LIST	P=18F2480,r=hex,N=75,C=120,T=ON

	include		"p18f2480.inc"
	include		"cbusdefs8h.inc"
	
; Must define FLIM_ONLY for evhndlr_g.asm	
#define FLIM_ONLY

S_PORT 	equ	PORTA	;setup switch  Change as needed
S_BIT	equ	2


LED_PORT equ	PORTB  ;change as needed
Outport  equ	PORTC ;servo drives
LED1	equ		7	;PB7 is the green LED on the PCB
LED2	equ		6	;PB6 is the yellow LED on the PCB


CMD_ON		equ	0x90	;on event
CMD_OFF	equ	0x91	;off event
CMD_REQ	equ	0x92	;request event

SCMD_ON	equ	0x98		;short events
SCMD_OFF	equ	0x99
SCMD_REQ	equ	0x9A

OPC_PNN	equ	0xB6

OLD_EN_NUM  equ	.32		;old number of allowed events
EN_NUM	equ	.128
EV_NUM  equ 	3		;number of allowed EVs per event
;SERVO_MAJVER equ	.1		; release version

HASH_SZ	equ	8

Modstat equ 1		;address in EEPROM

MAN_NO      equ MANU_MERG    ;manufacturer number
MAJOR_VER   equ 2
MINOR_VER   equ "U"
MODULE_ID   equ	MTYP_CANSERVO8C 	; id to identify this type of module
EVT_NUM     equ EN_NUM           ; Number of events
EVperEVT    equ EV_NUM           ; Event variables per event
NV_NUM      equ .37          ; Number of node variables
NODEFLGS    equ PF_CONSUMER + PF_PRODUCER + PF_BOOT
CPU_TYPE    equ P18F2480
BETA		equ 0


Cutval	equ 	.30			;change for cutoff delay

;table positions for SVs.

S1_hi	equ	0
S1_low	equ	1
S1_rat1	equ 2
S1_rat2 equ 3

S2_hi	equ	4
S2_low	equ	5
S2_rat1	equ 6
S2_rat2 equ 7

S3_hi	equ	8
S3_low	equ	9
S3_rat1	equ .10
S3_rat2 equ .11

S4_hi	equ	.12
S4_low	equ	.13
S4_rat1	equ .14
S4_rat2 equ .15

S5_hi	equ	.16
S5_low	equ	.17
S5_rat1	equ .18
S5_rat2 equ .19

S6_hi	equ	.20
S6_low	equ	.21
S6_rat1	equ .22
S6_rat2 equ .23

S7_hi	equ	.24
S7_low	equ	.25
S7_rat1	equ .26
S7_rat2 equ .27

S8_hi	equ	.28
S8_low	equ	.29
S8_rat1	equ .30
S8_rat2 equ .31


; definitions used by bootloader

#define	MODE_SELF_VERIFY	;Enable self verification of written data (undefine if not wanted)

#define	HIGH_INT_VECT	0x0808	;HP interrupt vector redirect. Change if target is different
#define	LOW_INT_VECT	0x0818	;LP interrupt vector redirect. Change if target is different.
#define	RESET_VECT	0x0800	;start of target
#define	CAN_CD_BIT	RXB0EIDL,0	;Received control / data select bit
#define	CAN_PG_BIT	RXB0EIDL,1	;Received PUT / GET bit
#define	CANTX_CD_BIT	TXB0EIDL,0	;Transmit control/data select bit
#define	CAN_TXB0SIDH	B'10000000'	;Transmitted ID for target node
#define	CAN_TXB0SIDL	B'00001000'
#define	CAN_TXB0EIDH	B'00000000'	;
#define	CAN_TXB0EIDL	B'00000100'
#define	CAN_RXF0SIDH	B'00000000'	;Receive filter for target node
#define	CAN_RXF0SIDL	B'00001000'
#define	CAN_RXF0EIDH	B'00000000'
#define	CAN_RXF0EIDL	B'00000111'
#define	CAN_RXM0SIDH	B'11111111'	;Receive masks for target node
#define	CAN_RXM0SIDL	B'11101011'
#define	CAN_RXM0EIDH	B'11111111'
#define	CAN_RXM0EIDL	B'11111000'
#define	CAN_BRGCON1		B'00000011'	;CAN bit rate controls. As for other CBUS modules
#define	CAN_BRGCON2		B'10011110'
#define	CAN_BRGCON3		B'00000011'
#define	CAN_CIOCON		B'00100000'	;CAN I/O control	
;	************************************************************ ** * * * * * * * * * * * * * * *
;	************************************************************ ** * * * * * * * * * * * * * * *
#ifndef	EEADRH		
#define	EEADRH	EEADR+ 1	
#endif			
#define	TRUE	1	
#define	FALSE	0	
#define	WREG1	PRODH	; Alternate working register
#define	WREG2	PRODL	
#define	MODE_WRT_UNLCK	_bootCtlBits, 0	; Unlock write and erase
#define	MODE_ERASE_ONLY	_bootCtlBits, 1	; Erase without write
#define	MODE_AUTO_ERASE	_bootCtlBits, 2	; Enable auto erase before write
#define	MODE_AUTO_INC	_bootCtlBits, 3	; Enable auto inc the address
#define	MODE_ACK		_bootCtlBits, 4	; Acknowledge mode
#define	ERR_VERIFY		_bootErrStat, 0	; Failed to verify if set
#define	CMD_NOP			0x00	
#define	CMD_RESET		0x01	
#define	CMD_RST_CHKSM	0x02	
#define	CMD_CHK_RUN		0x03
#define CMD_BOOT_TEST 	0x04	



;set config registers. These are common to bootloader and ACC8. 

; note. there seem to be differences in the naming of the CONFIG parameters between
; versions of the p18F2480.inf files

	CONFIG	FCMEN = OFF, OSC = HSPLL, IESO = OFF
	CONFIG	PWRT = ON,BOREN = BOHW, BORV=0
	CONFIG	WDT=OFF
	CONFIG	MCLRE = ON
	CONFIG	LPT1OSC = OFF, PBADEN = OFF
	CONFIG	DEBUG = OFF
	CONFIG	XINST = OFF,LVP = OFF,STVREN = ON,CP0 = OFF
	CONFIG	CP1 = OFF, CPB = OFF, CPD = OFF,WRT0 = OFF,WRT1 = OFF, WRTB = OFF
	CONFIG 	WRTC = OFF,WRTD = OFF, EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF
	
;original CONFIG settings left here for reference
	
;	__CONFIG	_CONFIG1H,	B'00100110'	;oscillator HS with PLL
;	__CONFIG	_CONFIG2L,	B'00001110'	;brown out voltage and PWT	
;	__CONFIG	_CONFIG2H,	B'00000000'	;watchdog time and enable (disabled for now)
;	__CONFIG	_CONFIG3H,	B'10000000'	;MCLR enable	
;	__CONFIG	_CONFIG4L,	B'10000001'	;B'10000001'  for 	no debug
;	__CONFIG	_CONFIG5L,	B'00001111'	;code protection (off)	
;	__CONFIG	_CONFIG5H,	B'11000000'	;code protection (off)	
;	__CONFIG	_CONFIG6L,	B'00001111'	;write protection (off)	
;	__CONFIG	_CONFIG6H,	B'11100000'	;write protection (off)	
;	__CONFIG	_CONFIG7L,	B'00001111'	;table read protection (off)	
;	__CONFIG	_CONFIG7H,	B'01000000'	;boot block protection (off)

;	processor uses  4 MHz. Resonator with HSPLL to give a clock of 16MHz

;********************************************************************************
;	RAM addresses used by boot. can also be used by application.

	CBLOCK 0
	_bootCtlMem
	_bootAddrL		; Address info
	_bootAddrH		
	_bootAddrU		
	_unused0		;(Reserved)
	_bootCtlBits	; Boot Mode Control bits
	_bootSpcCmd		; Special boot commands
	_bootChkL		; Chksum low byte fromPC
	_bootChkH		; Chksum hi byte from PC		
	_bootCount		
	_bootChksmL		; 16 bit checksum
	_bootChksmH		
	_bootErrStat	;Error Status flags
	ENDC
	
	; end of bootloader RAM


;****************************************************************
;	define RAM storage for CANSERVO
	
	CBLOCK	0		;file registers - access bank
					;interrupt stack for low priority
					;hpint uses fast stack
	W_tempL
	St_tempL
	Bsr_tempL
	PCH_tempH		;save PCH in hpint
	PCH_tempL		;save PCH in lpint (if used)
	Fsr_temp0L
	Fsr_temp0H 
	Fsr_temp1L
	Fsr_temp1H 
	Fsr_temp2L
	Fsr_temp2H
	
	TempCANCON
	TempCANSTAT
	TempINTCON
	TempECAN
	CanID_tmp	;temp for CAN Node ID
	IDtemph		;used in ID shuffle
	IDtempl
	NN_temph		;node number in RAM
	NN_templ
	ENtemp		;number of events
	
	IDcount		;used in self allocation of CAN ID.
	
	Mode		;for FLiM / SLiM etc
	Count		;counter for loading
	Count1
	Count2
	Count3
	St_temp		;temp for SoD states
	T1count		;flash rate counter
	Keepcnt		;keep alive counter
	Latcount	;latency counter
	Datmode		;flag for data waiting and other states
	Temp		;temps
	Temp1
	Shift
	Dlc			;data length
	
	State				;state for interrupt routine

	Intcnt				;count	interrupts
	Index1				;for NV table pointer
	Input				;input setting
	Intemp				;temporary storage for input

	Inchange			;servo bit has changed
	No_flag				;flags for no change but still send
	Wait_flg			;flag for servo to wait
	Set_flg				;used for initial setup
	Start_flg			;used on first setting

	W_temp
	S_temp
	PCH_tmp

	Off1				;time steps till servo turned off
	Off2
	Off3
	Off4
	Off5
	Off6
	Off7
	Off8
	Outmsk				;output mask for servo off state

	Delflg				;flags turnoff delay per output
	Gotflg				;flag for end of delay  (got there)
	Dir_flg				;flag for direction for DNs
	Cutmode				;cutoff mode  (copy of NV1)
	ON_end				;bits for servos at ON end
	OFF_end				;bits for servos at OFF end	

	SV_flg				;servo flag for learning
	T_roll				;servo for testing
	S_num				;servo number
				
	NV1
	NV2
	NV3
	NV4
	NV5
	Lastpos				;byte for last position, 1 = off, 0 = on
	Oldpos				;byte for saved position
	Ischange			;used to flag if EEPROM update needed
	
	

	Match		;match flag
	ENcount		;which EN matched
	ENcount1		;temp for count offset
	EVtemp		;holds current EV pointer
	EVtemp1		;EV for which servo
	EVtemp2		;holds current EV qualifier (polarity)
	EVtemp3		;EV for device numbering
	EVtemp4		;for extra EV 
	
	

	Roll		;rolling bit for enum
	
	Fsr_tmp1Le	;temp store for FSR1
	Fsr_tmp1He 

	;variables used by Flash Ram event handling

	evaddrh			; event data ptr
	evaddrl
	prevadrh		; previous event data ptr
	prevadrl
	nextadrh		; next event data ptr
	nextadrl
	htaddrh			; current hash table ptr
	htaddrl
	htidx			; index of current hash table entry in EEPROM
	hnum			; actual hash number
	freadrh			; current free chain address
	freadrl
	initFlags		; used in intialising Flash from EEPROM events
	Saved_Fsr0L		; used in rdfbev routine
	Saved_Fsr0H
	
	ev_opc			;temp storage for critical buffer values
	ev0
	ev1
	ev2
	ev3
	
	EVidx		; EV index from learn cmd
	EVdata		; EV data from learn cmd
	ENidx		; event index from commands which access events by index
	CountFb0	; counters used by Flash handling
	CountFb1
		
	ENDC
	
	CBLOCK	0x80
	Enum0		;bits for new enum scheme.
	Enum1
	Enum2
	Enum3
	Enum4
	Enum5
	Enum6
	Enum7
	Enum8
	Enum9
	Enum10
	Enum11
	Enum12
	Enum13

;Settings	
	S1_now				;servo1	current position
	S2_now
	S3_now
	S4_now
	S5_now
	S6_now
	S7_now
	S8_now

	S1_mid			;half way points for frog relay event
	S2_mid
	S3_mid
	S4_mid
	S5_mid
	S6_mid
	S7_mid
	S8_mid

	Tx1con			;start of transmit frame  1
	Tx1sidh
	Tx1sidl
	Tx1eidh
	Tx1eidl
	Tx1dlc
	Tx1d0
	Tx1d1
	Tx1d2
	Tx1d3
	Tx1d4
	Tx1d5
	Tx1d6
	Tx1d7

	Svc1			;counters for servo speeds per servo
	Svc2
	Svc3
	Svc4
	Svc5
	Svc6
	Svc7
	Svc8



	;add variables to suit
	
	ENDC

	CBLOCK 0x100		;bank 1
	; 64 bytes of event data - the quanta size for updating Flash
	evt00				; Event number - 4 bytes
	evt01
	evt02
	evt03
	next0h				; next entry in list
	next0l
	prev0h				; previous entry in list
	prev0l
	ev00				; event variables - upto 8
	ev01
	ev02
	ev03
	ev04
	ev05
	ev06
	ev07
	
	evt10				; Event number - 4 bytes
	evt11
	evt12
	evt13
	next1h				; next entry in list
	next1l
	prev1h				; previous entry in list
	prev1l
	ev10				; event variables - upto 8
	ev11
	ev12
	ev13
	ev14
	ev15
	ev16
	ev17
	
	evt20				; Event number - 4 bytes
	evt21
	evt22
	evt23
	next2h				; next entry in list
	next2l
	prev2h				; previous entry in list
	prev2l
	ev20				; event variables - upto 8
	ev21
	ev22
	ev23
	ev24
	ev25
	ev26
	ev27
	
	evt30				; Event number - 4 bytes
	evt31
	evt32
	evt33
	next3h				; next entry in list
	next3l
	prev3h				; previous entry in list
	prev3l
	ev30				; event variables - upto 8
	ev31
	ev32
	ev33
	ev34
	ev35
	ev36
	ev37
	
	ENDC
	
	CBLOCK	0x180
						;holds SVs. 0 to 31
	SV1
	SV2
	SV3
	SV4
		
	ENDC
			
	CBLOCK	0x200		;bank 2 lower half
	EN1					;only used during copying to flash
						
	ENDC
	
	CBLOCK	0x280		;bank 2	upper half	
						;holds EVs 
	EV1					;only used during copying to flash
						
	ENDC
;****************************************************************
;	This is the bootloader
; ***************************************************************************** 
;_STARTUPCODE	0x00
	ORG 0x0000
; *****************************************************************************
	bra	_CANInit
	bra	_StartWrite
; ***************************************************************************** 
;_INTV_H CODE	0x08
	ORG 0x0008
; *****************************************************************************

	goto	HIGH_INT_VECT

; ***************************************************************************** 
;_INTV_L CODE	0x18
	ORG 0x0018
; *****************************************************************************

	goto	LOW_INT_VECT 

; ************************************************************** 
;	Code start
; **************************************************************
	ORG 0x0020
;_CAN_IO_MODULE CODE
; ************************************************************ ** * * * * * * * * * * * * * * * 
; Function: VOID _StartWrite(WREG _eecon_data)
;PreCondition: Nothing
;Input: _eecon_data
;Output: Nothing. Self write timing started.
;Side Effects: EECON1 is corrupted; WREG is corrupted.
;Stack Requirements: 1 level.
;Overview: Unlock and start the write or erase sequence to protected
;	memory. Function will wait until write is finished.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_StartWrite
	movwf 	EECON1
	btfss 	MODE_WRT_UNLCK	; Stop if write locked
	return
	movlw 	0x55	; Unlock
	movwf 	 EECON2 
	movlw	 0xAA 
	movwf 	 EECON2
	bsf	 EECON1, WR	; Start the write
	nop
	btfsc 	EECON1, WR	; Wait (depends on mem type)
	bra	$ - 2
 	return
; ************************************************************ ** * * * * * * * * * * * * * * *

; Function: _bootChksm _UpdateChksum(WREG _bootChksmL)
;
; PreCondition: Nothing
; Input: _bootChksmL
; Output: _bootChksm. This is a static 16 bit value stored in the Access Bank.
; Side Effects: STATUS register is corrupted.
; Stack Requirements: 1 level.
; Overview: This function adds a byte to the current 16 bit checksum
;	count. WREG should contain the byte before being called.
;
;	The _bootChksm value is considered a part of the special
;	register set for bootloading. Thus it is not visible. ;
;*************************************************************** * * * * * * * * * * * *
_UpdateChksum:
	addwf	_bootChksmL,	F ; Keep a checksum
	btfsc	STATUS,	C
	incf	_bootChksmH,	F
	return
;************************************************************ ** * * * * * * * * * * * * * * *
;
;	Function:	VOID _CANInit(CAN,	BOOT)
;
;	PreCondition: Enter only after a reset has occurred.
; Input: CAN control information, bootloader control information ; Output: None.
; Side Effects: N/A. Only run immediately after reset.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
;	return when called. It has been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied. ;
;	This routine tests the boot flags to determine if boot mode is
;	desired or normal operation is desired. If boot mode then the
;	routine initializes the CAN module defined by user input. It
;	also resets some registers associated to bootloading.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_CANInit:
	clrf	EECON1
	setf	EEADR	; Point to last location of EEDATA
	setf	EEADRH
	bsf	EECON1, RD	; Read the control code
	incfsz EEDATA, W

	goto	RESET_VECT


	clrf	_bootSpcCmd 	; Reset the special command register
	movlw 	0x1C		; Reset the boot control bits
	movwf 	_bootCtlBits 
	movlb	d'15'		; Set Bank 15
	bcf 	TRISB, CANTX 	; Set the TX pin to output 
	movlw 	CAN_RXF0SIDH 	; Set filter 0
	movwf 	RXF0SIDH
	movlw 	CAN_RXF0SIDL 
	movwf 	RXF0SIDL
	comf	WREG		; Prevent filter 1 from causing a receive event





	movwf	RXF1SIDL	;		
	movlw	CAN_RXF0EIDH	
	movwf	RXF0EIDH	
	movlw	CAN_RXF0EIDL	
	movwf	RXF0EIDL	
	movlw	CAN_RXM0SIDH	;	Set mask
	movwf	RXM0SIDH	
	movlw	CAN_RXM0SIDL	
	movwf	RXM0SIDL	
	movlw	CAN_RXM0EIDH	
	movwf	RXM0EIDH	
	movlw	CAN_RXM0EIDL	
	movwf	RXM0EIDL	
	movlw	CAN_BRGCON1	;	Set bit rate
	movwf	BRGCON1	
	movlw	CAN_BRGCON2	
	movwf	BRGCON2	
	movlw	CAN_BRGCON3	
	movwf	BRGCON3	
	movlw	CAN_CIOCON	;	Set IO
	movwf	CIOCON	
	
	clrf	CANCON	; Enter Normal mode
	movlw	B'00001110'
	movwf	ADCON1
	bcf	TRISB,7
	bcf	TRISB,6
	bsf	PORTB,7		;green LED on
	bsf	PORTB,6		;yellow LED on


; ************************************************************ ** * * * * * * * * * * * * * * * 
; This routine is essentially a polling loop that waits for a
; receive event from RXB0 of the CAN module. When data is
; received, FSR0 is set to point to the TX or RX buffer depending
; upon whether the request was a 'put' or a 'get'.
; ************************************************************ ** * * * * * * * * * * * * * * * 
_CANMain
	
	bcf	RXB0CON, RXFUL	; Clear the receive flag
_wait	clrwdt			; Clear WDT while waiting		
	btfss 	RXB0CON, RXFUL	; Wait for a message	
	bra	_wait



_CANMainJp1
	lfsr	0, RXB0D0
	movf	RXB0DLC, W 
	andlw 	0x0F
	movwf 	_bootCount 
	movwf 	WREG1
	bz	_CANMain 
_CANMainJp2				;?
	


; ************************************************************** * * * * * * * * * * * * * * * 
; Function: VOID _ReadWriteMemory()
;
; PreCondition:Enter only after _CANMain().
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: This routine is technically not a function since it will not
;	return when called. It has been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;This is the memory I/O engine. A total of eight data bytes are received and decoded. In addition two control bits are received, put/get and control/data.
;A pointer to the buffer is passed via FSR0 for reading or writing. 
;The control register set contains a pointer, some control bits and special command registers.
;Control
;<PG><CD><ADDRL><ADDRH><ADDRU><_RES_><CTLBT>< SPCMD><CPDTL><CPDTH>
;Data
;<PG>< CD>< DATA0>< DATA1>< DATA2>< DATA3>< DATA4>< DATA5>< DATA6>< DATA7>
;PG bit:	Put = 0, Get = 1
;CD bit:	Control = 0, Data = 1

; ************************************************************ ** * * * * * * * * * * * * * * *
_ReadWriteMemory:
	btfsc	CAN_CD_BIT	; Write/read data or control registers
	bra	_DataReg
; ************************************************************ ** * * * * * * * * * * * * * * * ; This routine reads or writes the bootloader control registers,
; then executes any immediate command received.
_ControlReg
	lfsr	1, _bootAddrL		;_bootCtlMem
_ControlRegLp1

	movff 	POSTINC0, POSTINC1 
	decfsz 	WREG1, F
	bra	_ControlRegLp1

; ********************************************************* 
; This is a no operation command.
	movf	_bootSpcCmd, W		; NOP Command
	bz	_CANMain
;	bz	_SpecialCmdJp2		; or send an acknowledge

; ********************************************************* 
; This is the reset command.
	xorlw 	CMD_RESET		; RESET Command 
	btfss 	STATUS, Z
	bra		_SpecialCmdJp4
	setf	EEADR		; Point to last location of EEDATA
	setf	EEADRH
	clrf	EEDATA		; and clear the data (FF for now)
	movlw 	b'00000100'	; Setup for EEData
	rcall 	_StartWrite
	bcf		PORTB,6		;yellow LED off
	reset
; *********************************************************
; This is the Selfcheck reset command. This routine 
; resets the internal check registers, i.e. checksum and 
; self verify.
_SpecialCmdJp4
	movf	_bootSpcCmd, W 
	xorlw 	CMD_RST_CHKSM
	bnz		_SpecialCmdJp1
	clrf	_bootChksmH
	clrf	_bootChksmL
	bcf		ERR_VERIFY		
	clrf	_bootErrStat
	bra		_CANMain
; RESET_CHKSM Command
; Reset chksum
; Clear the error verify flag

;This is the Test and Run command. The checksum is
; verified, and the self-write verification bit is checked. 
; If both pass, then the boot flag is cleared.
_SpecialCmdJp1
	movf	_bootSpcCmd, W		; RUN_CHKSM Command
	xorlw 	CMD_CHK_RUN 
	bnz	_SpecialCmdJp3
	movf	_bootChkL, W	; Add the control byte
	addwf	 _bootChksmL, F
	bnz	_SpecialCmdJp2
	movf	_bootChkH, W 
	addwfc	_bootChksmH, F
	bnz	_SpecialCmdJp2
	btfsc 	ERR_VERIFY		; Look for verify errors
	bra	_SpecialCmdJp2

	bra		_CANSendOK	;send OK message


_SpecialCmdJp2

	bra	_CANSendNOK	; or send an error acknowledge


_SpecialCmdJp3
	movf	_bootSpcCmd, W		; RUN_CHKSM Command
	xorlw 	CMD_BOOT_TEST 
	bnz	_CANMain
	bra	_CANSendBoot

; ************************************************************** * * * * * * * * * * * * * * * 
; This is a jump routine to branch to the appropriate memory access function.
; The high byte of the 24-bit pointer is used to determine which memory to access. 
; All program memories (including Config and User IDs) are directly mapped. 
; EEDATA is remapped.
_DataReg
; *********************************************************
_SetPointers
	movf	_bootAddrU, W	; Copy upper pointer
	movwf 	TBLPTRU
	andlw 	0xF0	; Filter
	movwf 	WREG2
	movf	_bootAddrH, W	; Copy the high pointer
	movwf 	TBLPTRH
	movwf 	EEADRH
	movf	_bootAddrL, W	; Copy the low pointer
	movwf 	TBLPTRL
	movwf	 EEADR
	btfss 	MODE_AUTO_INC	; Adjust the pointer if auto inc is enabled
	bra	_SetPointersJp1
	movf	_bootCount, W	; add the count to the pointer
	addwf	 _bootAddrL, F 
	clrf	WREG
	addwfc	 _bootAddrH, F 
	addwfc	 _bootAddrU, F 

_SetPointersJp1			;?

_Decode
	movlw 	0x30
	cpfslt 	WREG2
	bra	_DecodeJp1



	bra	_PMEraseWrite

_DecodeJp1
	movf	WREG2,W
	xorlw 	0x30
	bnz	_DecodeJp2



	bra	_CFGWrite 
_DecodeJp2
	movf	WREG2,W 
	xorlw 0xF0
	bnz	_CANMain
	bra	_EEWrite

	

; Program memory < 0x300000
; Config memory = 0x300000
; EEPROM data = 0xF00000
	
; ************************************************************ ** * 
; ************************************************************** * 
; Function: VOID _PMRead()
;	VOID _PMEraseWrite ()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of
; the source data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space.Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;These are the program memory read/write functions. Erase is available through control flags. An automatic erase option is also available.
; A write lock indicator is in place to ensure intentional write operations.
;Note: write operations must be on 8-byte boundaries and must be 8 bytes long. Also erase operations can only occur on 64-byte boundaries.
; ************************************************************ ** * * * * * * * * * * * * * * *



_PMEraseWrite:
	btfss 	MODE_AUTO_ERASE
	bra	_PMWrite
_PMErase:
	movf	TBLPTRL, W
	andlw	b'00111111'
	bnz	_PMWrite
_PMEraseJp1
	movlw	b'10010100' 
	rcall 	_StartWrite 
_PMWrite:
	btfsc 	MODE_ERASE_ONLY


	bra	_CANMain 

	movf	TBLPTRL, W
	andlw	b'00000111'
	bnz	_CANMain 
	movlw 	0x08
	movwf WREG1

_PMWriteLp1					; Load the holding registers
	movf	POSTINC0, W 
	movwf 	TABLAT
	rcall	 _UpdateChksum 	; Adjust the checksum
	tblwt*+
	decfsz	 WREG1, F
	bra	_PMWriteLp1

#ifdef MODE_SELF_VERIFY 
	movlw	 0x08
	movwf 	WREG1 
_PMWriteLp2
	tblrd*-			; Point back into the block
	movf	POSTDEC0, W 
	decfsz	 WREG1, F
	bra	_PMWriteLp2
	movlw	 b'10000100' 	; Setup writes
	rcall	_StartWrite 	; Write the data
	movlw 	0x08
	movwf 	WREG1
_PMReadBackLp1
	tblrd*+			; Test the data
	movf	TABLAT, W 
	xorwf 	POSTINC0, W
	btfss	STATUS, Z
	bsf	ERR_VERIFY 
	decfsz 	WREG1, F
	bra	_PMReadBackLp1	; Not finished then repeat
#else
	tblrd*-			; Point back into the block
				 ; Setup writes
	movlw 	b'10000100' 	; Write the data
	rcall 	_StartWrite 	; Return the pointer position
	tblrd*+
#endif

	bra	_CANMain


; ************************************************************** * * * * * * * * * * * * * * *
 ; Function: VOID _CFGWrite()
;	VOID _CFGRead()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of the source data. 
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;
;	These are the Config memory read/write functions. Read is
;	actually the same for standard program memory, so any read
;	request is passed directly to _PMRead.
;
; ************************************************************ ** * * * * * * * * * * * * * * *
_CFGWrite

#ifdef MODE_SELF_VERIFY		; Write to config area
	movf	INDF0, W		; Load data
#else
	movf	POSTINC0, W
#endif
	movwf 	TABLAT
	rcall 	_UpdateChksum	; Adjust the checksum
	tblwt*			; Write the data
	movlw	b'11000100' 
	rcall 	_StartWrite
	tblrd*+			; Move the pointers and verify
#ifdef MODE_SELF_VERIFY 
	movf	TABLAT, W 
	xorwf 	POSTINC0, W

#endif
	decfsz 	WREG1, F
	bra	_CFGWrite	; Not finished then repeat

	bra	_CANMain 



; ************************************************************** * * * * * * * * * * * * * * * 
; Function: VOID _EERead()
;	VOID _EEWrite()
;
; PreCondition:WREG1 and FSR0 must be loaded with the count and address of
 ;	the source data.
; Input:	None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied.
;
;	This is the EEDATA memory read/write functions.
;
; ************************************************************ ** * * * * * * * * * * * * * * *


_EEWrite:

#ifdef MODE_SELF_VERIFY
	movf	INDF0, W
#else
	movf	POSTINC0, W 
#endif

	movwf 	EEDATA
	rcall 	_UpdateChksum 
	movlw	b'00000100' 
	rcall	 _StartWrite

#ifdef MODE_SELF_VERIFY 
	clrf	EECON1
	bsf	EECON1, RD
	movf	EEDATA, W 
	xorwf 	POSTINC0, W
	btfss	STATUS, Z
	bsf	ERR_VERIFY
#endif

	infsnz	 EEADR, F 
	incf 	EEADRH, F 
	decfsz 	WREG1, F
	bra	_EEWrite


	bra	_CANMain 
	

; Read the data

; Adjust EEDATA pointer
; Not finished then repeat
; Load data
; Adjust the checksum 
; Setup for EEData
; and write
; Read back the data ; verify the data ; and adjust pointer
; Adjust EEDATA pointer
; Not finished then repeat

; ************************************************************** * * * * * * * * * * * * * * *
; Function: VOID _CANSendAck()
;	VOID _CANSendResponce ()
;
; PreCondition:TXB0 must be preloaded with the data.
; Input: None.
; Output: None.
; Side Effects: N/A.
; Stack Requirements: N/A
; Overview: These routines are technically not functions since they will not
;	return when called. They have been written in a linear form to
;	save space. Thus 'call' and 'return' instructions are not
;	included, but rather they are implied. ;
;	These routines are used for 'talking back' to the source. The
;	_CANSendAck routine sends an empty message to indicate
;	acknowledgement of a memory write operation. The
;	_CANSendResponce is used to send data back to the source. ;
; ************************************************************ ** * * * * * * * * * * * * * * *



_CANSendMessage
	btfsc 	TXB0CON,TXREQ 
	bra	$ - 2
	movlw 	CAN_TXB0SIDH 
	movwf 	TXB0SIDH
	movlw 	CAN_TXB0SIDL 
	movwf 	TXB0SIDL
	movlw 	CAN_TXB0EIDH 
	movwf 	TXB0EIDH	

	movlw	CAN_TXB0EIDL
	movwf	TXB0EIDL
	bsf	CANTX_CD_BIT
	btfss	CAN_CD_BIT 
	bcf	CANTX_CD_BIT
	bsf	TXB0CON, TXREQ
    	bra	 _CANMain	; Setup the command bit

_CANSendOK				;send OK message 
	movlw	1			;a 1 is OK
	movwf	TXB0D0
	movwf	TXB0DLC
	bra		_CANSendMessage
	
_CANSendNOK				;send not OK message
	clrf	TXB0D0		;a 0 is not OK
	movlw	1
	movwf	TXB0DLC
	bra		_CANSendMessage

_CANSendBoot
	movlw	2			;2 is confirm boot mode
	movwf	TXB0D0
	movlw	1
	movwf	TXB0DLC
	bra		_CANSendMessage
    
; Start the transmission

; 	End of bootloader

;************************************************************************************************************
;
;		start of canservo program code

		ORG		0800h
loadadr
		nop						;for debug
		goto	setup

		ORG		0808h
		goto	hpint			;high priority interrupt
		
		ORG		0810h			;node type parameters
myName	db	"SERVO8C"

		ORG		0818h	
		goto	lpint			;low priority interrupt
		
		org		0820h

nodeprm     db  MAN_NO, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM 
			db	MAJOR_VER,NODEFLGS,CPU_TYPE,PB_CAN    ; Main parameters
            dw  RESET_VECT     ; Load address for module code above bootloader
            dw  0           ; Top 2 bytes of 32 bit address not used
			db  0,0,0,0,CPUM_MICROCHIP,BETA

sparprm     fill 0,prmcnt-$ ; Unused parameter space set to zero

PRMCOUNT    equ sparprm-nodeprm ; Number of parameter bytes implemented

             ORG 0838h

prmcnt      dw  PRMCOUNT    ; Number of parameters implemented
nodenam     dw  myName      ; Pointer to module type name
            dw  0 ; Top 2 bytes of 32 bit address not used


PRCKSUM     equ MAN_NO+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+NODEFLGS+CPU_TYPE+PB_CAN+HIGH myName+LOW myName+HIGH loadadr+LOW loadadr+PRMCOUNT+CPUM_MICROCHIP+BETA

cksum       dw  PRCKSUM     ; Checksum of parameters


;*******************************************************************

		ORG		0840h			;start of program
;	
;
;	
hpint	movf	PCLATH,W
		movwf	PCH_tmp
		movlw	8
		movwf	PCLATH
		
		movff	FSR2H, Fsr_temp2H
		movff	FSR2L, Fsr_temp2L
		
		bcf	INTCON,TMR0IF
	
nomask		rlncf	State,W
		addwf	PCL,F
		bra	state0
		bra	state1
		bra	state2
		bra	state3
		bra	state4
		bra	state5
		bra	state6
		bra	state7
		bra	state8
		bra	state9
		bra	state10
		bra	state11
		bra	state12
		bra	state13
		bra	state14
		bra	state15
		bra	state16
		bra	state17
		bra	state18

state0		goto	sback

state1	movlw	B'11000011'		;timer 0 used for servo
		movwf	T0CON

		movlw	.10
		movwf	TMR0L	
		incf	State,F
		goto	sback
		
state2		btfsc	Outmsk,0		;servo off?
		bsf	Outport,0		;servo 1 up
		movlw	.10
		movwf	TMR0L
		incf	State,F
		goto	sback
		
state3	lfsr	FSR2,S1_now
		movf	INDF2,W
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state4	btfsc	Outmsk,0		;leave 1 alone
		bcf		Outport,0		;servo 1 down
		btfsc	Outmsk,1		;servo off?
		bsf		Outport,1		;servo 2 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state5	lfsr	FSR2,S2_now
		movf	INDF2,W
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state6	btfsc	Outmsk,1		;leave 2 alone
		bcf		Outport,1		;servo 2 down
		btfsc	Outmsk,2		;servo off?
		bsf		Outport,2		;servo 3 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state7	lfsr	FSR2,S3_now
		movf	INDF2,W
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state8	btfsc	Outmsk,2		;leave 3 alone
		bcf		Outport,2		;servo 3 down
		btfsc	Outmsk,3		;servo off?
		bsf		Outport,3		;servo 4 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback
	
state9	lfsr	FSR2,S4_now
		movf	INDF2,W
		movwf	TMR0L
		incf	State,F
		goto	sback

state10	btfsc	Outmsk,3		;leave 4 alone
		bcf		Outport,3		;servo 4 down
		btfsc	Outmsk,4		;servo off?
		bsf		Outport,4		;servo 5 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state11	lfsr	FSR2,S5_now
		movf	INDF2,W
		movwf	TMR0L
		incf	State,F
		goto	sback

state12	btfsc	Outmsk,4		;leave 5 alone
		bcf		Outport,4		;servo 5 down
		btfsc	Outmsk,5		;servo off?
		bsf		Outport,5		;servo 6 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state13	lfsr	FSR2,S6_now
		movf	INDF2,W
		movwf	TMR0L
		incf	State,F
		goto	sback

state14	btfsc	Outmsk,5		;leave 6 alone
		bcf		Outport,5		;servo 6 down
		btfsc	Outmsk,6		;servo off?
		bsf		Outport,6		;servo 7 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state15	lfsr	FSR2,S7_now
		movf	INDF2,W
		movwf	TMR0L
		incf	State,F
		goto	sback

state16	btfsc	Outmsk,6		;leave 7 alone
		bcf		Outport,6		;servo 7 down
		btfsc	Outmsk,7		;servo off?
		bsf		Outport,7		;servo 8 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state17	lfsr	FSR2,S8_now
		movf	INDF2,W
		movwf	TMR0L
		incf	State,F
		goto	sback	


state18 btfsc	Outmsk,7		;leave 8 alone
		bcf		Outport,7		;servo 8 down
		clrf	State
		goto	sback



sback
		movff	Fsr_temp2H,FSR2H
		movff	Fsr_temp2L,FSR2L 
		movf	PCH_tmp,W
		movwf	PCLATH
	
		retfie	1


;**************************************************************
;
;		low priority interrupt. Used for CAN transmit error / latency.
;
		ORG 0A00h		

lpint		movwf	W_tempL				
		movff	STATUS,St_tempL
		movff	CANCON,TempCANCON
		movff	CANSTAT,TempCANSTAT
	
		movff	PCLATH,PCH_tempH		;save PCLATH
		clrf	PCLATH
	
		movff	FSR0L,Fsr_temp0L		;save FSR0
		movff	FSR0H,Fsr_temp0H
		movff	FSR1L,Fsr_temp1L		;save FSR1
		movff	FSR1H,Fsr_temp1H
		
		

		movlw	0x0A			;for relocated code
		movwf	PCLATH
		movf	TempCANSTAT,W			;Jump table
		andlw	B'00001110'
		addwf	PCL,F			;jump
		bra		back
		bra		errint			;error interrupt
		bra		back
		bra		back
		bra		back
		bra		back			;only receive interrupts used (now in ECAN)
		bra		back
		bra		back
		

		

		
		;error routine here. Only acts on lost arbitration	
errint	movlb	.15					;change bank			
		btfss	TXB1CON,TXLARB
		bra		errbak				;not lost arb.
	
		movf	Latcount,F			;is it already at zero?
		bz		errbak
		decfsz	Latcount,F
		bra		errbak
		bcf		TXB1CON,TXREQ
		movlw	B'00111111'
		andwf	TXB1SIDH,F			;change priority
txagain bsf		TXB1CON,TXREQ		;try again
					
errbak		bcf		RXB1CON,RXFUL
		movlb	0
		bcf		RXB0CON,RXFUL		;ready for next
		
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL		
		bra		back1


		
back	bcf		RXB0CON,RXFUL	;ready for next
	
	
back1	clrf	PIR3			;clear all flags
		movf	CANCON,W
		andlw	B'11110001'
		iorwf	TempCANCON,W
		
		movwf	CANCON
		movff	PCH_tempH,PCLATH
		movff	Fsr_temp0L,FSR0L		;recover FSR0
		movff	Fsr_temp0H,FSR0H

		movff	Fsr_temp1L,FSR1L		;recover FSR1
		movff	Fsr_temp1H,FSR1H
		movf	W_tempL,W
		movff	St_tempL,STATUS	
		
		retfie	

;now handled as a subroutine			
		
isRTR	btfsc	Datmode,1		;setup mode?
		return					;back
		btfss	Mode,1			;FLiM?
		return
		movlb	.15
isRTR1	btfsc	TXB2CON,TXREQ	
		bra		isRTR1		
		bsf		TXB2CON,TXREQ	;send ID frame - preloaded in TXB2

		movlb	0
		return


;	
;**************************************************************************************************

;		Note. SLiM mode only used for initial setup. This is not a SLiM module

main	btfsc	Datmode,0		;busy?
		bra		main_OK
		btfsc	COMSTAT,7		;look for CAN input. 
		bra		getcan

		bra		main_OK
	

getcan	movf	CANCON,W
		andlw	B'00001111'
		movwf	TempCANCON
		movf	ECANCON,W
		andlw	B'11110000'
		iorwf	TempCANCON,W
		movwf	ECANCON
		btfsc	RXB0SIDL,EXID		;ignore extended frames here
		bra		no_can
		btfss	RXB0DLC,RXRTR		;is it RTR?
		bra		get_1
		call	isRTR
		bra		no_can
		
get_1				;setup mode?
		
get_3	movf	RXB0DLC,F
		bnz		get_2			;ignore zero length frames
		bra		no_can 
get_2	bsf		Datmode,0		;valid message frame	
		bra		main_OK

no_can	bcf		RXB0CON,RXFUL

main_OK	btfsc	Mode,1			;is it SLiM?
		bra		mainf

mains	;SLiM not used in this module
	
nofl_s	bra		noflash				;main1
		
; here if FLiM mde

mainf	
		btfss	PIR1,TMR1IF		;is it flash?
		bra		noflash
		bcf		PIR1,TMR1IF
		btfss	Datmode,2
		bra		nofl1
		decfsz	T1count
		bra		noflash
		btg		PORTB,6			;flash yellow LED
		movlw	4
		movwf	T1count
	

	
		
nofl1	btfss	Datmode,3		;running mode
		bra		noflash
		decfsz	Keepcnt			;send keep alive?
		bra		noflash
		movlw	.10
		movwf	Keepcnt
		movlw	0x52
;		call	nnrel			;send keep alive frame (works OK, turn off for now)

noflash	btfsc	S_PORT,S_BIT	;setup button?
		bra		main3			;main3
		movlw	.100
		movwf	Count
		clrf	Count1
		clrf	Count2
		movlw	4
		movwf	T1count

wait	decfsz	Count2
		goto	wait
		btfss	Datmode,2
		bra		wait2
wait3	btfss	PIR1,TMR1IF		;is it flash?
		bra		wait2
		decfsz	T1count
		bra		wait2
		btg		PORTB,6			;flash LED
		movlw	4
		movwf	T1count
		bcf		PIR1,TMR1IF

wait2	decfsz	Count1
		goto	wait
		btfsc	S_PORT,S_BIT
		bra		main4			;not held long enough
		decfsz	Count
		goto	wait
		btfss	Mode,1			;is it in FLiM?
		bra		go_FLiM
		clrf	Datmode			;back to virgin

		bcf		PORTB,6			;yellow off
		
		bsf		PORTB,7			;Green LED on
		clrf	INTCON			;interrupts off
		movlw	1
		movwf	IDcount			;back to start
		movlw	Modstat
		movwf	EEADR
		movlw 	0
		call	eewrite			;status to reset
		movlw	0x51			;send node release frame
		call	nnrel
		clrf	NN_temph
		clrf	NN_templ
wait1	btfss	S_PORT,S_BIT
		bra		wait1			;wait till release
		call	ldely
		btfss	S_PORT,S_BIT
		bra		wait1
	
		
		movlw	LOW NodeID			;put NN back to 0000
		movwf	EEADR
		movlw	0
		call	eewrite
		incf	EEADR
		movlw	0
		call	eewrite	
		btfss	Mode,1
		bra		main5				;FLiM setup
		movlw	Modstat
		movwf	EEADR
		movlw	0
		call	eewrite				;mode back to SLiM
		clrf	Datmode
		bcf		Mode,1
		bcf		PORTB,6
		bsf		PORTB,7				;green LED on
	
		movlw	B'11100000'
		movwf	INTCON
		goto	main				;

main5	movlw	Modstat
		movwf	EEADR
		movlw	1
		call	eewrite				;mode to setup in EEPROM

		bsf		Mode,1				;to FLiM
		movlw	B'11100000'
		movwf	INTCON
		
		call	self_en				;self enumerate routine
		bcf		Datmode,1
		call	nnack				;send request for NN
		bsf		Datmode,2
		movlw	Modstat
		movwf	EEADR
		movlw	B'00000100'
		call	eewrite				;mode to wait for NN in EEPROM
		bra		main1

main4	btfss	Mode,1				;is it in FLiM?
		bra		main				;no so try again
		btfss	Datmode,3			;has NN?
		bra		main
		btfss	Datmode,2
		bra		mset2
		bcf		Datmode,2
		bsf		PORTB,6			;LED on

		movlw	0x52
		call	nnrel
		movlw	Modstat
		movwf	EEADR
		movlw	B'00001000'
		movwf	Datmode			;normal
		call	eewrite
		bra		main3
mset2	bsf		Datmode,2
		call	self_en			;re_enumerate
		bcf		Datmode,1
		call	nnack
		bra		main1

main3	btfss	Datmode,1		;setup mode ?
		bra		main1
		bsf		Datmode,2		;wait for NN
		bcf		Datmode,1
		movlw	Modstat
		movwf	EEADR
		movlw	B'00000100'
		call	eewrite		
	
		bra		main1			;continue normally

go_FLiM	bsf		Datmode,1		;FLiM setup mode
		bcf		PORTB,7			;green off
		bra		wait1
		
		

; 	
	
	
main1	btfss	Datmode,0		;any new CAN frame received?
		bra		gap0
		bra		packet			;yes
gap0	btfss	Datmode,3		;running mode?
		bra		main			;no
		bsf		T0CON,TMR0ON	;start servo timer
		bsf		INTCON2,TMR0IP	;high priority for T0 interrupt
		movlw	B'11100000'
		movwf	INTCON			;start interrupts
		movf	State,F			;is state 0?
		bz		gap				;gap between servo cycles

		
		
		bra		main
		
		
gap		btfss	PIR2,TMR3IF
		bra		gap2
		bcf		PIR2,TMR3IF
		movlw	0x61			;reset T3 to 20 mSec for frame time
		movwf	TMR3H
		movlw	0xDF
		movwf	TMR3L
		movlw	B'10010001'
		movwf	T3CON			;enable timer 3
		movf	Oldpos,W		;has position changed?
		subwf	Lastpos,W
		bz		gap3			;no change
		tstfsz	Wait_flg		;still any moving?
		bra		gap3
		movlw	LOW NVnow
		movwf	EEADR
		movf	Lastpos,W
		call	eewrite			;update if any change and finished moving
		movff	Lastpos,Oldpos
gap3	
gap4	btfsc	Datmode,7		;in servo setting mode?
		call	test
	
	;	
		call	servo			;update servo settings

gap1	incf	State
	
gap2	goto	main		
;********************************************************************

;		These are here as branch was too long

unset	
		btfss	Datmode,4
		bra		main2			;prevent error message
		bsf		Datmode,5
		call	copyev
		bra		learn2
		
readEV	btfss	Datmode,4
		bra		main2			;prevent error message
		call	copyev
		movf	EVidx,w			;check EV index
		bz		rdev1
		decf	EVidx
		movlw	EV_NUM
		cpfslt	EVidx
rdev1	bra		noEV1
		bsf		Datmode,6
		bra		learn2

evns1	call	thisNN				;read event numbers
		sublw	0
		bnz		notNNx
		call	evnsend
		bra		main2

reval	call	thisNN				;read event numbers
		sublw	0
		bnz		notNNx
		movff	RXB0D3, ENidx
		movff	RXB0D4, EVidx
		call	evsend
		bra		main2
notNNx	goto	notNN

go_on_x goto	go_on

params	btfss	Datmode,2		;only in setup mode
		bra		main2
		call	parasend
		bra		main2

name
 		btfss	Datmode,2		;only in setup mode
		bra		main2
		call	namesend
		bra		main2
			
doQnn
		movf	NN_temph,w		;respond if NN is not zero
		addwf	NN_templ,w
		btfss	STATUS,Z
		call	whoami
		bra		main2
		

		
		
;********************************************************************
								;main packet handling is here
								;add more commands for incoming frames as needed
		
packet	movlw	CMD_ON  ;only ON, OFF  events supported
		subwf	RXB0D0,W	
		bz		go_on_x
		movlw	CMD_OFF
		subwf	RXB0D0,W
		bz		go_on_x
		movlw	CMD_REQ
		subwf	RXB0D0,W
		bz		go_on_x
		movlw	SCMD_ON
		subwf	RXB0D0,W
		bz		go_on_x
		movlw	SCMD_OFF
		subwf	RXB0D0,W
		bz		go_on_x
		movlw	SCMD_REQ
		subwf	RXB0D0,W
		bz		go_on_x
		movlw	0x96			;learn NV by index
		subwf	RXB0D0,W
		bz		lrn_NV
		
		movlw	0x5C			;reboot
		subwf	RXB0D0,W
		bz		reboot
		movlw	0x5D			;re-enumerate
		subwf	RXB0D0,W
		bz		enum
		movlw	0x73
		subwf	RXB0D0,W
		bz		para1a			;read individual parameters

		movlw	0x75			;force new CAN_ID
		subwf	RXB0D0,W
		bz		new_ID1
		movlw	0x42			;set NN on 0x42
		subwf	RXB0D0,W
		bz		setNNx
		movlw	0x0d			; QNN
		subwf	RXB0D0,w
		bz		doQnn
		movlw	0x10			
		subwf	RXB0D0,W
		bz		params			;read node parameters
		movlw	0x11
		subwf	RXB0D0,w
		bz		name			;read module name		
		movlw	0x53			;set to learn mode on 0x53
		subwf	RXB0D0,W
		bz		setlrn		
		movlw	0x54			;clear learn mode on 0x54
		subwf	RXB0D0,W
		bz		notlrn
		movlw	0x55			;clear all events on 0x55
		subwf	RXB0D0,W
		bz		clren1
		movlw	0x56			;read number of events left
		subwf	RXB0D0,W
		bz		rden_x
		movlw	0xD2			;is it set event?
		subwf	RXB0D0,W
		bz		chk1			;do learn
		movlw	0x95			;is it unset event
		subwf	RXB0D0,W			
		bz		unset
		movlw	0xB2			;read event variables
		subwf	RXB0D0,W
		bz		readEV
	
		movlw	0x57			;is it read events
		subwf	RXB0D0,W
		bz		readEN1
		movlw	0x72
		subwf	RXB0D0,W
		bz		rdENi1			;read event by index
		movlw	0x58
		subwf	RXB0D0,W
		bz		evns
		movlw	0x9C				;read event variables by EN#
		subwf	RXB0D0,W
		bz		reval
		movlw	0x71				;read NVs by index
		subwf	RXB0D0,W
		bz		readNVx
		bra		main2

setNNx	goto	setNN
rden_x	goto	rden
chk1	goto	chklrn
rdENi1 	goto	readENi		
evns	goto	evns1
clren1	goto	clrens
enum	call	thisNN
		sublw	0
		bnz		notNN1
		call	self_en
		movlw	0x52
		call	nnrel			;send confirm frame
		bcf		RXB0CON,RXFUL
		movlw	B'00001000'		;back to normal running
		movwf	Datmode
		goto	main2
new_ID1	goto	new_ID
notNN1	goto	notNN
		
lrn_NV	goto	lrn_NV1

readEN1 goto	readEN

readNVx	goto	read_NV

reboot	btfss	Mode,1			;FLiM?
		bra		reboots
		call	thisNN
		sublw	0
		bnz		notNN
		
reboot1	movlw	0xFF
		movwf	EEADR
		movlw	0xFF
		call	eewrite			;set last EEPROM byte to 0xFF
		reset					;software reset to bootloader

reboots
		movf	RXB0D1,w
		addwf	RXB0D2,w
		bnz		notNN
		bra		reboot1	
	
para1a	btfss	Mode, 1
		bra		para1s
		call	thisNN			;read parameter by index
		sublw	0
		bnz		notNN
		call	para1rd
		bra		main2
		
para1s
		movf	RXB0D1,w
		addwf	RXB0D2,w
		bnz		notNN
		call	para1rd
		bra		main2

notlrn	call	thisNN
		sublw	0
		bnz		notNN
		bcf		Datmode,4
notln1									;leave in learn mode
		bcf		Datmode,5
		btfsc	Datmode,7				;SV learn?
		call	storeSV
		bra		main2

setlrn	call	thisNN
		sublw	0
		bnz		notNN
		bsf		Datmode,4
		bsf		LED_PORT,LED2			;LED on
		bra		main2
			
main2	bcf		RXB0CON,RXFUL		;finished with this frame
		bcf		Datmode,0
		goto	main			;loop
		
setNN	btfss	Datmode,2		;in NN set mode?
		bra		main2			;no
		call	putNN			;put in NN
		movlw	Modstat
		movwf	EEADR
		movlw	B'00001000'
		call	eewrite			;set to normal status
		bcf		Datmode,1		;out of setup
		bcf		Datmode,2
		bsf		Datmode,3		;run mode
	
		movlw	.10
		movwf	Keepcnt			;for keep alive
		movlw	0x52
		call	nnrel			;confirm NN set
startNN	bsf		LED_PORT,LED2	;LED ON
		bcf		LED_PORT,LED1
		movlw	0x61			;reset T3 to 20 mSec for frame time
		movwf	TMR3H
		movlw	0xDF
		movwf	TMR3L
		movlw	B'10010001'
		movwf	T3CON			;enable timer 3
		goto	start
		;bra		main2

new_ID	call	thisNN
		sublw	0
		bnz		notNN
		movff	RXB0D3,IDcount
;		bsf		Datmode,1			;?
		call	here2				;put in as if it was enumerated
		movlw	0x52
		call	nnrel				;acknowledge new CAN_ID
		goto	main2
		
sendNN	btfss	Datmode,2		;in NN set mode?
		bra		main2			;no
		movlw	0x50			;send back NN
		movwf	Tx1d0
		movlw	3
		movwf	Dlc
		call	sendTX
		bra		main2

rden	goto	rden1




		
clrens	call	thisNN
		sublw	0
		bnz		notNN
		btfss	Datmode,4
		bra		clrerr
		call	initevdata
		call	clreepr		;clear all EEPROM responses
		movlw	0x59
		call	nnrel		;send WRACK
		bra		notln1
notNN	bra		main2

clrerr	movlw	2			;not in learn mode
		goto	errmsg

chklrn	btfss	Datmode,4		;is in learn mode?
		bra		main2			;jump if not
		call	copyev
		movf	EVidx,w			;check EV index
		bz		noEV1
		decf	EVidx
		movlw	EV_NUM
		cpfslt	EVidx
		bra		noEV1
		bra		learn2
		
noEV1
		movlw	6
		goto	errmsg
		
readENi	call	thisNN			;read event by index
		sublw	0
		bnz		notNN
		movff	RXB0D3,ENidx
		call	enrdi
		bra		main2


copyev		; copy event data to safe buffer
		movff	RXB0D0, ev_opc
		movff	RXB0D1, ev0
		movff	RXB0D2, ev1
		movff	RXB0D3, ev2
		movff	RXB0D4, ev3
		movff	RXB0D5, EVidx		; only used by learn and some read cmds
		movff	RXB0D6, EVdata		; only used by learn cmd
		movlw	SCMD_ON
		subwf	RXB0D0,W
		bz		short
		movlw	SCMD_OFF
		subwf	RXB0D0,W
		bz		short
		movlw	SCMD_REQ
		subwf	RXB0D0,W
		bz		short
		return
short	clrf	ev0					;for short events, clear ev0 and ev1
		clrf	ev1
		return
		
go_on	call	copyev
		btfss	Mode,1			;FLiM?
		bra		main2
		
go_on1	call	enmatch
		sublw	0
		bz		do_it
		bra		main2			;not here

read_NV	call 	thisNN
		sublw	0
		bnz		notNN
		call	readNVs			;read NV by index
		goto	main2


paraerr	movlw	3				;error not in setup mode
		goto	errmsg

readEN	call	thisNN
		sublw	0
		bnz		notNN
		call	enread
		bra		main2
		
lrn_NV1	call	thisNN			;is it this NN?
		sublw	0
		bnz		lrn_not			;not this NN
		call	lrnNVs
lrn_not	goto	main2	
		
do_it
		call	rdfbev
		movff	POSTINC0, EVtemp
		movff	POSTINC0, EVtemp2
		movff	POSTINC0, EVtemp3
		movff	POSTINC0, EVtemp4

		call	ev_set			;do it -  for consumer action
		bra		main2
		
rden1	call	thisNN
		sublw	0
		bnz		notNN
		call	rdFreeSp
		bra		main2		
		
learn1
		bra		learn2

learn2	btfss	Mode,1			;FLiM?
		goto	main2
		call	enmatch			;is it there already?
		sublw 	0
		bz		isthere

learn3	btfsc	Datmode,6		;read EV?
		bra		rdbak1			;not here
		btfsc	Datmode,5		;if unset and not here
		bra		l_out1			;do nothing else 
		
learn4	movlw	2
		subwf	EVidx,W
		bnz		learn5
		call	dn_teach
learn5		call	learnin			;put EN into flash
		sublw	0
		bz		lrnend
		
		movlw	4
		goto	errmsg2	
		
rdbak1	movlw	5				;no match
		goto	errmsg2	

lrnend
		bra		l_out1
					
isthere	
		btfsc	Datmode, 6		;is it read back
		bra		rdbak					
		btfss	Datmode,5		;FLiM unlearn?
		bra		dolrn
		call	rdfbev			;get EVs
		movlw	2
		movff	PLUSW0,EVtemp3
		call	ev_del			;delete feedback event from EEPROM
		call	unlearn
		movlw	0x59
		call	nnrel
		bra		l_out1

dolrn	movlw	2
		subwf	EVidx,W
		bnz		do_lrn1
		call	dn_teach
do_lrn1		call 	learnin
		bra		l_out1	
		
rdbak
		call	rdfbev			; read event info
		movff	EVidx,Tx1d5		;Index for readout	
		incf	Tx1d5,F			;add one back
		movf	EVidx,w
		movff	PLUSW0,Tx1d6
		movlw	0xD3				;readback of EVs
		movwf	Tx1d0
		movff	ev0,Tx1d1
		movff	ev1,Tx1d2
		movff	ev2,Tx1d3
		movff	ev3,Tx1d4
		movlw	7
		movwf	Dlc
		call	sendTXa	
		bra		l_out1



l_out	bcf		Datmode,4

l_out1	bcf		Datmode,6
l_out2	bcf		RXB0CON,RXFUL
		bcf		Datmode,0
		
		clrf	PCLATH
		goto	main2
		
noEV	movlw	6				;invalid EV#
		goto	errmsg2




;***************************************************************************
;		main setup routine
;*************************************************************************

setup	clrf	ADCON0			;turn off A/D, all digital I/O
		movlw	B'00001111'
		movwf	ADCON1
		setf	LATC			;outputs higt at start
		movlw	B'00000000'		;Port C  set to outputs.
		movwf	TRISC
		lfsr	FSR0, 0			; clear page 1
		
nextram	clrf	POSTINC0
		tstfsz	FSR0L
		bra		nextram	
		
		clrf	INTCON			;no interrupts yet
	
		
		;port settings will be hardware dependent. RB2 and RB3 are for CAN.
		;set S_PORT and S_BIT to correspond to port used for setup.
		;rest are hardware options
		
	
		movlw	B'00000111'		;Port A  PA0 and PA1 inputs for SLiM compatibility. PA2 is setup PB
		movwf	TRISA			;
		movlw	B'00111011'		;RB2 = CANTX, RB3 = CANRX, 	
								;RB6,7 for debug and ICSP and LEDs
								;PORTB has pullups enabled on inputs
		movwf	TRISB
		bcf		LED_PORT,LED2
		bcf		LED_PORT,LED1
		bsf		PORTB,2			;CAN recessive
	

	
		
;	next segment is essential.
		
		bsf		RCON,IPEN		;enable interrupt priority levels
		clrf	BSR				;set to bank 0
		clrf	EECON1			;no accesses to program memory	
		clrf	Datmode
		clrf	Latcount
		bsf		CANCON,7		;CAN to config mode
		movlw	B'10110000'
		movwf	ECANCON	
		bsf		ECANCON,5		;CAN mode 2 
		movf	ECANCON,W
		movwf	TempECAN 

		movlb	.15				;change bank
		clrf	BSEL0			;8 frame FIFO
		clrf	RXB0CON
		clrf	RXB1CON
		clrf	B0CON
		clrf	B1CON
		clrf	B2CON
		clrf	B3CON
		clrf	B4CON
		clrf	B5CON
		movlb	0

		movlw	B'00000011'		;set CAN bit rate at 125000 for now
		movwf	BRGCON1
		movlw	B'10011110'		;set phase 1 etc
		movwf	BRGCON2
		movlw	B'00000011'		;set phase 2 etc
		movwf	BRGCON3
		movlw	B'00100000'
		movwf	CIOCON			;CAN to high when off
		movlw	B'00100100'		;B'00100100'
		movwf	RXB0CON			;enable double buffer of RX0
	
		


		
mskload	lfsr	0,RXM0SIDH		;Clear masks, point to start
mskloop	clrf	POSTINC0		
		movlw	LOW RXM1EIDL+1		;end of masks
		cpfseq	FSR0L
		bra		mskloop
		
		clrf	CANCON			;out of CAN setup mode
		clrf	CCP1CON
			
		movlw	B'10110001'		;Timer 1 set Timer 1 for LED flash
		movwf	T1CON
		movlw	0x00
		movwf	TMR1H			;Timer 1 is a 16 bit timer
		movlw	B'01000011'		;Timer 0 is an 8 bit timer
		movwf	T0CON
		
		clrf	Tx1con
		movlw	B'00000000'
		movwf	IPR3			;low priority CAN RX and Tx error interrupts(for now)
		clrf	IPR1			;all peripheral interrupts are low priority
		clrf	IPR2
		clrf	PIE2
;		clrf	PORTC			;all low
		

		clrf	Off1
		clrf	Off2
		clrf	Off3
		clrf	Off4

		clrf	ON_end			;indeterminate at start
		clrf	OFF_end
		clrf	No_flag
		clrf	Wait_flg
		clrf	Delflg
		clrf	Gotflg
		clrf	Start_flg


;next segment required
		
		movlw	B'00000001'
		movwf	IDcount			;set at lowest value for starters
		
		clrf	INTCON2			;
		clrf	INTCON3			;
		

		movlw	B'00100000'		;B'00100011'   Tx error only
								
		movwf	PIE3
	
		clrf	PIR1
		clrf	PIR2
		
		
		bcf		RXB0CON,RXFUL		;ready for next
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL
		clrf	PIR3			;clear all flags

		clrf	State			;servo state to 0
		call	nv_ram			;put NVs in ram
		call	mid_pt			;work out mid points
	
		
		
		
		;		test for setup mode
		call	copyEVs			;set up flash ram if not already done
		clrf	NV5
		clrf	Mode
		movlw	Modstat			;get setup status
		movwf	EEADR
		call	eeread
		movwf	Datmode		
		sublw	8				;is it in run mode?
		bz		setid
		movlw	0
		movwf	Datmode
		call	eewrite			;clear back to SLiM
	
		bra		slimset			;wait for setup PB
	
		
setid	
		bsf		Mode,1			;flag FLiM
		call	newid_f			;put ID into Tx1buf, TXB2 and ID number store
		btfsc	Datmode,2
		bra		seten_f
		movlw	0x61			;reset T3 to 20 mSec for frame time
		movwf	TMR3H
		movlw	0xDF
		movwf	TMR3L
		movlw	B'10010001'
		movwf	T3CON			;enable timer 3
		
seten_f	
		bcf		LED_PORT,LED1
		btfss	Datmode,2		;flashing?
		
		bsf		LED_PORT,LED2			;Yellow LED on.
		bcf		RXB0CON,RXFUL
		bcf		Datmode,0
		goto	start

slimset	bcf		Mode,1
		clrf	NN_temph
		clrf	NN_templ
	
		bcf		PORTB,6
		bsf		PORTB,7			;RUN LED on. Green for SLiM
		goto	main

;	new startup routine

		
start	call	ser_start		;set servo starting settings
		movlw	B'11111111'
		movwf	Set_flg			;flag setup 
		
		goto	main



		

		
;****************************************************************************
;		start of subroutines

;		Do an event.  arrives with EVs in EVtemp, EVtemp2, EVtemp3 
ev_set	
		movlw	0x92
		subwf	ev_opc,W
		bz		ev_reqx
		movlw	0x9A
		subwf	ev_opc,W
		bz		ev_reqx

	;	movff	Intemp,Input	;recover last inputs
		movlw	B'10000011'
		andwf	EVtemp3,W
		btfss	WREG,7
		bra		sod_test
		return					;not a action event

ev_reqx	goto	ev_req

sod_test
		sublw	B'00000011'
		bnz		ev_set2
		bra		sod_send
ev_set2	movf	EVtemp,W		;carry on with event
		btfss	ev_opc,0			;direction
		comf	WREG			;
		xorwf	EVtemp2,W		;polarity change?
		movwf	Temp
		movf	EVtemp,W
		andwf	Temp,F			;mask							;

		comf	EVtemp,W		;for masking
		andwf	Intemp,W		;leave irrelevant bits alone
		iorwf	Temp,W
		movwf	Input			;has new input


		
		xorwf	Intemp,W		;compare with last input
		iorwf	Inchange		;for servo loop
		movff	Input,Intemp
		movf	Inchange,W
		comf	WREG
		andwf	EVtemp,W		;have the selected servos changed?
		bnz		ev_set1			;no
		return					;has change so do normally
ev_set1	movwf	No_flag			;no change so flag for acknowledge
		
		return	



sod_send						;send a SoD sequence
		
sod1	tstfsz	Wait_flg		;any moving?
		return					;can't do if moving
sod2	
		movlw	LOW NVnow		;get current state
		movwf	EEADR
		call	eeread
		movwf	St_temp
		bcf		Dir_flg,0
		btfsc	St_temp,0		;get end
		bsf		Dir_flg,0
		movlw	1
		movwf	S_num
		call	snd_mid			;mid point
		call	sdely
		movlw	1
		call	dn_end
		call	sdely
		bcf		Dir_flg,0
		btfsc	St_temp,1		;get end
		bsf		Dir_flg,0
		movlw	2
		movwf	S_num
		call	snd_mid			;mid point
		call	sdely
		movlw	2
		call	dn_end
		call	sdely
		bcf		Dir_flg,0
		btfsc	St_temp,2		;get end
		bsf		Dir_flg,0
		movlw	3
		movwf	S_num
		call	snd_mid			;mid point
		call	sdely
		movlw	3
		call	dn_end
		call	sdely
		bcf		Dir_flg,0
		btfsc	St_temp,3		;get end
		bsf		Dir_flg,0
		movlw	4
		movwf	S_num
		call	snd_mid			;mid point
		call	sdely
		movlw	4
		call	dn_end
		call	sdely
		bcf		Dir_flg,0
		btfsc	St_temp,4		;get end
		bsf		Dir_flg,0
		movlw	5
		movwf	S_num
		call	snd_mid			;mid point
		call	sdely
		movlw	5
		call	dn_end
		call	sdely
		bcf		Dir_flg,0
		btfsc	St_temp,5		;get end
		bsf		Dir_flg,0
		movlw	6
		movwf	S_num
		call	snd_mid			;mid point
		call	sdely
		movlw	6
		call	dn_end
		call	sdely
		bcf		Dir_flg,0
		btfsc	St_temp,6		;get end
		bsf		Dir_flg,0
		movlw	7
		movwf	S_num
		call	snd_mid			;mid point
		call	sdely
		movlw	7
		call	dn_end
		call	sdely
		bcf		Dir_flg,0
		btfsc	St_temp,7		;get end
		bsf		Dir_flg,0
		movlw	8
		movwf	S_num
		call	snd_mid			;mid point
		call	sdely
		movlw	8
		call	dn_end
		call	sdely
		return


;		request status of any servo.	EVtemp3 has servo number and end to test
			
ev_req	rrncf	EVtemp3,W
		rrncf	WREG
		andlw	B'00000111'		;mask except servo number
		movwf	Count
		clrf	Count1
ev_req4		movf	Count,F
		bz		ev_req5
		movlw	.16
		addwf	Count1,F
		decf	Count
		bra		ev_req4
ev_req5	call	ev_get			;get the event 4 bytes into Tx1 buffer
		
ev_req3		clrf	Count		;set for bit test
		bsf		Count,0
ev_req2		movf	WREG,W
		bz		ev_req1			;got there
		rlncf	Count,F
		decf	WREG,W
		bra		ev_req2
ev_req1	movlw	B'00000001'
		andwf	EVtemp3,W
		bz		on_test			;is ON end
		movf	Count,W
		andwf	OFF_end,W
		bz		is_on
	
is_off	movlw	0x92			;this is for the OFF end of travel
		subwf	ev_opc,W			;is it long?
		bnz		sev_off			;short event off
		movlw	0x94
		movwf	Tx1d0			;set for response
		call	sendTXa
		return

sev_off	movlw	0x9E
		movwf	Tx1d0
		call	sendTX
		return

is_on	movlw	0x92
		subwf	ev_opc,W			;is it long?
		bnz		sev_on			;short event off
		movlw	0x93
		movwf	Tx1d0			;set for response
		call	sendTXa
		return

sev_on	movlw	0x9D
		movwf	Tx1d0
		call	sendTX
		return


on_test	movf	Count,W			;this for the ON end of travel
		andwf	ON_end,W
		bz		is_on1
	
is_off1	movlw	0x92
		subwf	ev_opc,W			;is it long?
		bnz		sev_off1			;short event off
		movlw	0x94
		movwf	Tx1d0			;set for response
		call	sendTXa
		return

sev_off1	movlw	0x9E
		movwf	Tx1d0
		call	sendTX
		return

is_on1	movlw	0x92
		subwf	ev_opc,W			;is it long?
		bnz		sev_on1			;short event off
		movlw	0x93
		movwf	Tx1d0			;set for response
		call	sendTXa
		return

sev_on1	movlw	0x9D
		movwf	Tx1d0
		call	sendTX
		return
		

;**************************************************************
;
;		gets event for response
;		start address in Count1 in event table					

ev_get	movlw 	LOW ENstart		;get response event
		addwf	Count1,W		;get offset
		movwf	EEADR
		movf	EVtemp3,W
		andlw	B'00000011'
		movwf	Count
		clrf	WREG
ev_get1	movf	Count,F			;which EN of the servo is it?
		bz		ev_get2
		addlw	4
		decf	Count,F
		bra		ev_get1
		
ev_get2	addwf	EEADR	
		call	eeread
		movwf	Tx1d1
		incf	EEADR
		call	eeread
		movwf	Tx1d2
		incf	EEADR
		call	eeread
		movwf	Tx1d3
		incf	EEADR
		call	eeread
		movwf	Tx1d4
		movlw	5
		movwf	Dlc
		return

;		Send contents of Tx1 buffer via CAN TXB1

sendTX1	movff	FSR1L,Fsr_temp1L		;save FSRs
		movff	FSR1H,Fsr_temp1H
		movff	FSR0L,Fsr_temp0L
		movff	FSR0H,Fsr_temp0H


		lfsr	FSR0,Tx1con
		lfsr	FSR1,TXB1CON
		
		movlb	.15				;check for buffer access
ldTX2	btfsc	TXB1CON,TXREQ	; Tx buffer available...?
		bra		ldTX2			;... not yet
		movlb	0
		
ldTX1	movf	POSTINC0,W
		movwf	POSTINC1	;load TXB1
		movlw	Tx1d7+1
		cpfseq	FSR0L
		bra		ldTX1

		
		movlb	.15				;bank 15
tx1test	btfsc	TXB1CON,TXREQ	;test if clear to send
		bra		tx1test
		bsf		TXB1CON,TXREQ	;OK so send
		
tx1done	movlb	0				;bank 0
		movff	Fsr_temp1L,FSR1L		;recover FSRs
		movff	Fsr_temp1H,FSR1H
		movff	Fsr_temp0L,FSR0L
		movff	Fsr_temp0H,FSR0H
		return					;successful send

		
;*********************************************************************
;		put in NN from command

putNN	movff	RXB0D1,NN_temph
		movff	RXB0D2,NN_templ
		movlw	LOW NodeID
		movwf	EEADR
		movf	RXB0D1,W
		call	eewrite
		incf	EEADR
		movf	RXB0D2,W
		call	eewrite
		movlw	Modstat
		movwf	EEADR
		movlw	B'00001000'		;Module status has NN set
		call	eewrite
		return

;***************************************************************************	

newid_f	movlw	LOW CANid			;put in stored ID. FLiM mode
		movwf	EEADR
		bsf		EECON1,RD
		movf	EEDATA,W
		movwf	CanID_tmp			
		call	shuffle
		movlw	B'11110000'
		andwf	Tx1sidh
		movf	IDtemph,W		;set current ID into CAN buffer
		iorwf	Tx1sidh			;leave priority bits alone
		movf	IDtempl,W
		movwf	Tx1sidl			;only top three bits used
		movlw	LOW NodeID
		movwf	EEADR
		call	eeread
		movwf	NN_temph			;get stored NN
		incf	EEADR
		call	eeread
		movwf	NN_templ	
		
		movlb	.15				;put ID into TXB2 for enumeration response to RTR
new_1	btfsc	TXB2CON,TXREQ
		bra		new_1
		clrf	TXB2SIDH
		movf	IDtemph,W
		movwf	TXB2SIDH
		movf	IDtempl,W
		movwf	TXB2SIDL
		movlw	0xB0
		iorwf	TXB2SIDH		;set priority
		clrf	TXB2DLC			;no data, no RTR
		movlb	0
;		btfsc	Datmode,3		;already set up?
		return

;*********************************************************************		


		
nnack	movlw	0x50			;request frame for new NN or ack if not virgin
nnrel	movwf	Tx1d0
		movff	NN_temph,Tx1d1
		movff	NN_templ,Tx1d2
		movlw	3
		movwf	Dlc
		call	sendTX
		return



		
;*****************************************************************************
;
;		shuffle for standard ID. Puts 7 bit ID into IDtemph and IDtempl for CAN frame
shuffle	movff	CanID_tmp,IDtempl		;get 7 bit ID
		swapf	IDtempl,F
		rlncf	IDtempl,W
		andlw	B'11100000'
		movwf	IDtempl					;has sidl
		movff	CanID_tmp,IDtemph
		rrncf	IDtemph,F
		rrncf	IDtemph,F
		rrncf	IDtemph,W
		andlw	B'00001111'
		movwf	IDtemph					;has sidh
		return

;*********************************************************************************

;		reverse shuffle for incoming ID. sidh and sidl into one byte.

shuffin	movff	RXB0SIDL,IDtempl
		swapf	IDtempl,F
		rrncf	IDtempl,W
		andlw	B'00000111'
		movwf	IDtempl
		movff	RXB0SIDH,IDtemph
		rlncf	IDtemph,F
		rlncf	IDtemph,F
		rlncf	IDtemph,W
		andlw	B'01111000'
		iorwf	IDtempl,W			;returns with ID in W
		return
;************************************************************************************
;		
eeread	bcf		EECON1,EEPGD	;read a EEPROM byte, EEADR must be set before this sub.
		bcf		EECON1,CFGS		;returns with data in W
		bsf		EECON1,RD
		movf	EEDATA,W
		return

;**************************************************************************
eewrite	movwf	EEDATA			;write to EEPROM, EEADR must be set before this sub.
		bcf		EECON1,EEPGD	;data to write in W
		bcf		EECON1,CFGS
		bsf		EECON1,WREN
		movff	INTCON,TempINTCON
		
		clrf	INTCON	;disable interrupts
		movlw	0x55
		movwf	EECON2
		movlw	0xAA
		movwf	EECON2
		bsf		EECON1,WR
eetest	btfsc	EECON1,WR
		bra		eetest
		bcf		PIR2,EEIF
		bcf		EECON1,WREN
		movff	TempINTCON,INTCON		;reenable interrupts
		
		return	
		

;***********************************************************************

;#include "..\evhndlr_e.asm"
#include "evhndlr_g.asm"


;*********************************************************************
;		send a CAN frame
;		entry at sendTX puts the current NN in the frame - for producer events
;		entry at sendTXa neeeds Tx1d1 and Tx1d2 setting first
;		Latcount is the number of CAN send retries before priority is increased
;		the CAN-ID is pre-loaded in the Tx1 buffer 
;		Dlc must be loaded by calling source to the data length value
		
sendTX	movff	NN_temph,Tx1d1
		movff	NN_templ,Tx1d2

sendTXa	movf	Dlc,W				;get data length
		movwf	Tx1dlc
		movlw	B'00001111'		;clear old priority
		andwf	Tx1sidh,F
		movlw	B'10110000'
		iorwf	Tx1sidh			;low priority
		movlw	.10
		movwf	Latcount
		call	sendTX1			;send frame
		return			

;**************************************************************************

;		check if command is for this node

thisNN	movf	NN_temph,W
		subwf	RXB0D1,W
		bnz		not_NN
		movf	NN_templ,W
		subwf	RXB0D2,W
		bnz		not_NN
		retlw 	0			;returns 0 if match
not_NN	retlw	1
							
;**********************************************************************
;		loads ENs from EEPROM to RAM for fast access
;		shifts all 32 even if less are used

en_ram	movlw	OLD_EN_NUM
		movwf	Count			;number of ENs allowed 
		
		bcf		STATUS,C		;clear carry
		rlncf	Count,F			;double it
		rlncf	Count,F			;double again
		lfsr	FSR0,EN1		;set FSR0 to start of ram buffer
		movlw	LOW ENstart			;load ENs from EEPROM to RAM
		movwf	EEADR
enload	bsf		EECON1,RD		;get first byte
		movf	EEDATA,W
		movwf	POSTINC0
		incf	EEADR
		decfsz	Count,F
		bra		enload

ev_ram	movlw	OLD_EN_NUM		;now copy original EVs to RAM
		movwf	Count			;number of ENs allowed 
		bcf		STATUS,C
;		rlncf	Count			;1 EV per event
		lfsr	FSR0, EV1
		movlw	LOW EVstart
		movwf	EEADR
ev_load
		bsf		EECON1,RD		;get first byte
		movf	EEDATA,W
		movwf	POSTINC0
		incf	EEADR
		decfsz	Count,F
		bra		ev_load
		return

nv_ram	movlw	NV_NUM-5			;shift variables to ram
		movwf	Count
		lfsr	FSR0,SV1
		movlw	LOW SVstart
		movwf	EEADR
svload	bsf		EECON1,RD		;get first byte
		movf	EEDATA,W
		movwf	POSTINC0
		incf	EEADR
		decfsz	Count,F
		bra		svload
	
		bsf		EECON1,RD		;get last pos byte
		movf	EEDATA,W
		movwf	Oldpos			;previous saved position
		movlw	4
		movwf	Count
		lfsr	FSR0,NV1		;shift the 4 NVs
		movlw	LOW NVstart
		movwf	EEADR
nvload	bsf		EECON1,RD		;get first byte
		movf	EEDATA,W
		movwf	POSTINC0
		incf	EEADR
		decfsz	Count,F
		bra		nvload
		movff	NV1,Cutmode
		return	
		
;**************************************************************************
;		send node parameter bytes (7 maximum)

parasend	
		movlw	0xEF
		movwf	Tx1d0
		movlw	LOW nodeprm
		movwf	TBLPTRL
		movlw	8
		movwf	TBLPTRH		;relocated code
		lfsr	FSR0,Tx1d1
		movlw	7
		movwf	Count
		bsf		EECON1,EEPGD
		
para1	tblrd*+
		movff	TABLAT,POSTINC0
		decfsz	Count
		bra		para1
		bcf		EECON1,EEPGD	
		movlw	8
		movwf	Dlc
		call	sendTXa
		return

;**************************************************************************
;		send module name - 7 bytes

namesend	
		movlw	0xE2
		movwf	Tx1d0
		movlw	LOW myName
		movwf	TBLPTRL
		movlw	HIGH myName
		movwf	TBLPTRH		;relocated code
		lfsr	FSR0,Tx1d1
		movlw	7
		movwf	Count
		bsf		EECON1,EEPGD
		
name1	tblrd*+
		movff	TABLAT,POSTINC0
		decfsz	Count
		bra		name1
		bcf		EECON1,EEPGD	
		movlw	8
		movwf	Dlc
		call	sendTXa
		return
		

;**********************************************************

;		send individual parameter

;		Index 0 sends no of parameters

para1rd	movf	RXB0D3,w
		sublw	0
		bz		numParams
		movlw	PRMCOUNT
		movff	RXB0D3, Temp
		decf	Temp
		cpfslt	Temp
		bra		pidxerr
		movlw	0x9B
		movwf	Tx1d0
		movlw	7		;FLAGS index in nodeprm
		cpfseq	Temp
		bra		notFlags			
		call	getflags
		movwf	Tx1d4
		bra		addflags
notFlags		
		movlw	LOW nodeprm
		movwf	TBLPTRL
		movlw	HIGH nodeprm
		movwf	TBLPTRH		;relocated code
		clrf	TBLPTRU
		decf	RXB0D3,W
		addwf	TBLPTRL
		bsf		EECON1,EEPGD
		tblrd*
		movff	TABLAT,Tx1d4
addflags						
		movff	RXB0D3,Tx1d3
		movlw	5
		movwf	Dlc
		call	sendTX
		return	
		
numParams
		movlw	0x9B
		movwf	Tx1d0
		movlw	PRMCOUNT
		movwf	Tx1d4
		movff	RXB0D3,Tx1d3
		movlw	5
		movwf	Dlc
		call	sendTX
		return
		
pidxerr
		movlw	.10
		call	errsub
		return
		
getflags		; create flags byte
		movlw	PF_CONSUMER
		btfsc	Mode,1
		iorlw	4		; set bit 2
		movwf	Temp
		bsf		Temp,3		;set bit 3, we are bootable
		movf	Temp,w
		return
		
		
;**********************************************************

; returns Node Number, Manufacturer Id, Module Id and Flags

whoami
		call	ldely		;wait for other nodes
		movlw	OPC_PNN
		movwf	Tx1d0
		movlw	MAN_NO		;Manufacturer Id
		movwf	Tx1d3
		movlw	MODULE_ID		; Module Id
		movwf	Tx1d4
		call	getflags
		movwf	Tx1d5
		movlw	6
		movwf	Dlc
		call	sendTX
		return

;***********************************************************

;	error message send

errmsg	call	errsub
		goto	main2 
errmsg1	call	errsub
		goto	l_out2
errmsg2	call	errsub
		goto	l_out1

errsub	movwf	Tx1d3		;main error message send. Error no. in WREG
		movlw	0x6F
		movwf	Tx1d0
		movlw	4
		movwf	Dlc
		call	sendTX
		return

	
;**************************************************************
;
;		store all SVs after a learn process

storeSV	lfsr	FSR2,SV1
		movlw	.32
		movwf	Count
		movlw	LOW	SVstart
		movwf	EEADR
sto_1	movf	POSTINC2,W
		call	eewrite
		incf	EEADR
		decfsz	Count
		bra		sto_1
		lfsr	FSR2,NV1
		movlw	4
		movwf	Count
		movlw	LOW	NVstart
		movwf	EEADR
sto_2	movf	POSTINC2,W
		call	eewrite
		incf	EEADR
		decfsz	Count
		bra		sto_2
		bcf		Datmode,7
		movlw	0x59
		call	nnrel		;send WRACK
		return

;********************************************************
;		converts speed value to correct value for Count
;		arrives with programmed value in w
;		leaves with new value in w

spdconv	addlw	LOW	Spdtbl	
		movwf	TBLPTRL
		movlw	HIGH Spdtbl
		movwf	TBLPTRH
		movlw	0
		addwfc	TBLPTRH
;		bsf		EECON1,EEPGD
		tblrd*+
		movf	TABLAT,W
;		bcf		EECON1,EEPGD	
		return

;*****************************************************	
;
;	main servo scanning routine

servo		clrf	Dir_flg
		
		lfsr	FSR2,SV1		;set to SV table in FSR2

t1		lfsr	FSR1, S1_now
		btfsc	Start_flg,0		;is it first time?
		bra		t1s
		btfsc	NV3,0			;is it no move?
		bra		t1s1
		bcf		Inchange,0
		bsf		Gotflg,0
t1s1	bsf		Start_flg,0

t1s		btfss	Inchange,0		;has this input changed?
		goto	t1_noch			;no
		btfss	NV4,0			;is it a wait?
		bra		t1a				;no
		btfsc	Wait_flg,0		;this one moving?
		bra		t1a				;start again
		tstfsz	Wait_flg		;all clear to go?
		bra		t2
t1a		bcf		Inchange,0
		bsf		Outmsk,0		;enable output on servo 1
		bcf		Delflg,0		;clear delay mode on 1
		bcf		Gotflg,0		;not got there on 1
		bsf		Wait_flg,0		;set for running
		
		btfsc	Intemp,0		;on or off event?
		bsf		Dir_flg,0
		bcf		ON_end,0		;position indeterminate
		bcf		OFF_end,0
		movlw	1				;set for sending DN start event
		movwf	S_num			;for mid point
		call	dn_start
	
		movlw	Cutval
		movwf	Off1			;set delay timer for 1

t1_noch	btfss	No_flag,0
		bra		t1_no1
		bcf		No_flag,0
		bra		t1_done1
t1_no1	btfsc	Gotflg,0		;already there?
		goto	t2				;goto	t1_done1				;next servo
	
		btfss	Intemp,0		;what is input direction?
		goto	t1_low			;input low
		bsf		Dir_flg,0		;input is high (off)
		movlw	S1_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc1			;speed counter
	


t1_1h	movlw	1
		movwf	S_num
		movlw	S1_low			;loop for speed / position test
		movff	PLUSW2,S_temp	;recover last position
		btfsc	Set_flg,0
		call	snd_mid
		bcf		Set_flg,0
		movf	INDF1,W
		subwf	S1_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t1_done			;is there at set point
		
		btfss	STATUS,C		;which way?
		goto	t1_dh
		tstfsz	Svc1
		goto	t1_ih
		incf	INDF1,F
		goto	t2				;rate countdown
	
t1_ih	decf	Svc1
		incf	INDF1,F			;increment position
		goto	t1_1h

t1_dh	tstfsz	Svc1			;rate countdown
		goto	t1_dh2
		decf	INDF1,F
		goto	t2				;finished t1
t1_dh2	decf	Svc1
		decf	INDF1,F			;decrement position
		goto	t1_1h



t1_low	bcf		Dir_flg,0		;input is low (on)
		movlw	S1_rat1			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc1


t1_1l	movlw	1
		movwf	S_num
		movlw	S1_hi
		movff	PLUSW2,S_temp
		btfsc	Set_flg,0
		call	snd_mid
		bcf		Set_flg,0
		movf	INDF1,W

		subwf	S1_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t1_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t1_dl
		tstfsz	Svc1			;rate countdown
		goto	t1_il
		incf	INDF1,F
		goto	t2				;finished t1
t1_il	decf	Svc1
		incf	INDF1,F
		goto	t1_1l

t1_dl	tstfsz	Svc1			;rate countdown
		goto	t1_dl2
		decf	INDF1,F
		goto	t2
t1_dl2	decf	Svc1
		decf	INDF1,F
		goto	t1_1l

t1_done	btfsc	Delflg,0
		bra		t2
		bsf		Delflg,0		;set flag for delay cutoff
		bcf		Wait_flg,0		;finished
t1_done1	clrf	Dir_flg
		bcf		Lastpos,0		;save position
		btfsc	Intemp,0
		bsf		Lastpos,0
		btfsc	Intemp,0
		bsf		Dir_flg,0			;for DN output
		btfss	Intemp,0			;set position flag
		bra		t1_pos
		bsf		OFF_end,0
		bra		t1_pos1
t1_pos	bsf		ON_end,0
t1_pos1	movlw	1		
		call	dn_end			;send dn event at end


; servo	2	

t2		lfsr	FSR1, S2_now
		btfsc	Start_flg,1		;is it first time?
		bra		t2s
		btfsc	NV3,1			;is it no move?
		bra		t2s1
		bcf		Inchange,1
		bsf		Gotflg,1
t2s1	bsf		Start_flg,1

t2s		btfss	Inchange,1		;has this input changed?
		goto	t2_noch			;no
		btfss	NV4,1			;is it a wait?
		bra		t2a				;no
		btfsc	Wait_flg,1		;this one moving?
		bra		t2a				;start again
		tstfsz	Wait_flg		;all clear to go?
		bra		t3
t2a		bcf		Inchange,1
		bsf		Outmsk,1		;enable output on servo 2
		bcf		Delflg,1		;clear delay mode on 2
		bcf		Gotflg,1		;not got there on 2
		bsf		Wait_flg,1		;set for running
		btfsc	Intemp,1
		bsf		Dir_flg,0
		bcf		ON_end,1			;position indeterminate
		bcf		OFF_end,1
		movlw	2			;set for sending DN start event
		movwf	S_num
		call	dn_start
		movlw	Cutval
		movwf	Off2			;set delay timer for 2

t2_noch	btfss	No_flag,1
		bra		t2_no1
		bcf		No_flag,1
		bra		t2_done1
t2_no1	btfsc	Gotflg,1		;already there?
		goto	t3				;next servo
		btfss	Intemp,1		;what is input direction?
		goto	t2_low			;input low
		bsf		Dir_flg,0
		movlw	S2_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc2


t2_1h	movlw	2
		movwf	S_num
		movlw	S2_low
		movff	PLUSW2,S_temp
		btfsc	Set_flg,1
		call	snd_mid
		bcf		Set_flg,1
		movf	INDF1,W
		subwf	S2_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t2_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t2_dh
		tstfsz	Svc2			;rate countdown
		goto	t2_ih
		incf	INDF1,F
		goto	t3				;finished t2
t2_ih	decf	Svc2
		incf	INDF1,F
		goto	t2_1h

t2_dh	tstfsz	Svc2			;rate countdown
		goto	t2_dh2
		decf	INDF1,F
		goto	t3				;finished t2
t2_dh2	decf	Svc2
		decf	INDF1,F
		goto	t2_1h



t2_low	bcf		Dir_flg,0
		movlw	S2_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv	
		movwf	Svc2


t2_1l	movlw	2
		movwf	S_num
		movlw	S2_hi
		movff	PLUSW2,S_temp
		btfsc	Set_flg,1
		call	snd_mid
		bcf		Set_flg,1
		movf	INDF1,W
		subwf	S2_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t2_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t2_dl
		tstfsz	Svc2			;rate countdown
		goto	t2_il
		incf	INDF1,F
		goto	t3				;finished t2
t2_il	decf	Svc2
		incf	INDF1,F
		goto	t2_1l

t2_dl	tstfsz	Svc2			;rate countdown
		goto	t2_dl2
		decf	INDF1,F
		goto	t3
t2_dl2	decf	Svc2
		decf	INDF1,F
		goto	t2_1l

t2_done	btfsc	Delflg,1
		bra		t3
		bsf		Delflg,1		;set flag for delay cutoff
		bcf		Wait_flg,1		;finished
t2_done1	clrf	Dir_flg
		bcf		Lastpos,1		;save position
		btfsc	Intemp,1
		bsf		Lastpos,1
		btfsc	Intemp,1
		bsf		Dir_flg,0
		btfss	Intemp,1			;set position flag
		bra		t2_pos
		bsf		OFF_end,1
		bra		t2_pos1
t2_pos	bsf		ON_end,1
t2_pos1	movlw	2				;send dn event at end
		call	dn_end		

;	servo 3
		
t3		lfsr	FSR1, S3_now
		btfsc	Start_flg,2		;is it first time?
		bra		t3s
		btfsc	NV3,2			;is it no move?
		bra		t3s1
		bcf		Inchange,2
		bsf		Gotflg,2
t3s1	bsf		Start_flg,2

t3s		btfss	Inchange,2		;has this input changed?
		goto	t3_noch			;no
		btfss	NV4,2			;is it a wait?
		bra		t3a				;no
		btfsc	Wait_flg,2		;this one moving?
		bra		t3a				;start again
		tstfsz	Wait_flg		;all clear to go?
		bra		t4
t3a		bcf		Inchange,2
		bsf		Outmsk,2		;enable output on servo 3
		bcf		Delflg,2		;clear delay mode on 3
		bcf		Gotflg,2		;not got there on 3
		bsf		Wait_flg,2		;running
		btfsc	Intemp,2
		bsf		Dir_flg,0
		bcf		ON_end,2			;position indeterminate
		bcf		OFF_end,2
		movlw	3			;set for sending DN start event
		movwf	S_num
		call	dn_start
		movlw	Cutval
		movwf	Off3			;set delay timer for 3

t3_noch	btfss	No_flag,2
		bra		t3_no1
		bcf		No_flag,2
		bra		t3_done1
t3_no1	btfsc	Gotflg,2		;already there?
		goto	t4				;next servo
		btfss	Intemp,2		;what is input direction?
		goto	t3_low			;input low
		bsf		Dir_flg,0
		movlw	S3_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc3


t3_1h	movlw	3
		movwf	S_num
		movlw	S3_low
		movff	PLUSW2,S_temp
		btfsc	Set_flg,2
		call	snd_mid
		bcf		Set_flg,2
		movf	INDF1,W
		subwf	S3_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t3_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t3_dh
		tstfsz	Svc3			;rate countdown
		goto	t3_ih
		incf	INDF1,F
		goto	t4				;finished t3
t3_ih	decf	Svc3
		incf	INDF1,F
		goto	t3_1h

t3_dh	tstfsz	Svc3			;rate countdown
		goto	t3_dh2
		decf	INDF1,F
		goto	t4				;finished t3
t3_dh2	decf	Svc3
		decf	INDF1,F
		goto	t3_1h



t3_low	bcf		Dir_flg,0
		movlw	S3_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc3


t3_1l	movlw	3
		movwf	S_num
		movlw	S3_hi
		movff	PLUSW2,S_temp
		btfsc	Set_flg,2
		call	snd_mid
		bcf		Set_flg,2
		movf	INDF1,W
		subwf	S3_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t3_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t3_dl
		tstfsz	Svc3			;rate countdown
		goto	t3_il
		incf	INDF1,F
		goto	t4				;finished t3
t3_il	decf	Svc3
		incf	INDF1,F
		goto	t3_1l

t3_dl	tstfsz	Svc3			;rate countdown
		goto	t3_dl2
		decf	INDF1,F
		goto	t4
t3_dl2	decf	Svc3
		decf	INDF1,F
		goto	t3_1l

t3_done	btfsc	Delflg,2
		bra		t4
		bsf		Delflg,2		;set flag for delay cutoff
		bcf		Wait_flg,2		;finished
t3_done1	clrf	Dir_flg
		bcf		Lastpos,2		;save position
		btfsc	Intemp,2
		bsf		Lastpos,2
		btfsc	Intemp,2
		bsf		Dir_flg,0	
		btfss	Intemp,2			;set position flag
		bra		t3_pos
		bsf		OFF_end,2
		bra		t3_pos1
t3_pos	bsf		ON_end,2
t3_pos1	movlw	3			;send dn event at end
		call	dn_end		

;	servo 4

		
t4		lfsr	FSR1, S4_now
		btfsc	Start_flg,3		;is it first time?
		bra		t4s
		btfsc	NV3,3			;is it no move?
		bra		t4s1
		bcf		Inchange,3
		bsf		Gotflg,3
t4s1	bsf		Start_flg,3

t4s		btfss	Inchange,3		;has this input changed?
		goto	t4_noch			;no
		btfss	NV4,3			;is it a wait?
		bra		t4a				;no
		btfsc	Wait_flg,3		;this one moving?
		bra		t4a				;start again
		tstfsz	Wait_flg		;all clear to go?
		bra		t5
t4a		bcf		Inchange,3
		bsf		Outmsk,3		;enable output on servo 4
		bcf		Delflg,3		;clear delay mode on 4
		bcf		Gotflg,3		;not got there on 4
		bsf		Wait_flg,3		;running
		btfsc	Intemp,3
		bsf		Dir_flg,0
		bcf		ON_end,3			;position indeterminate
		bcf		OFF_end,3
		movlw	4			;set for sending DN start event
		movwf	S_num
		call	dn_start
		movlw	Cutval
		movwf	Off4			;set delay timer for 4

t4_noch	btfss	No_flag,3
		bra		t4_no1
		bcf		No_flag,3
		bra		t4_done1
t4_no1	btfsc	Gotflg,3		;already there?
		goto	t5				;next servo
	
		btfss	Intemp,3		;what is input direction?
		goto	t4_low			;input low
		bsf		Dir_flg,0
		movlw	S4_rat2		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc4


t4_1h	movlw	4
		movwf	S_num
		movlw	S4_low
		movff	PLUSW2,S_temp
		btfsc	Set_flg,3
		call	snd_mid
		bcf		Set_flg,3
		movf	INDF1,W
		subwf	S4_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t4_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t4_dh
		tstfsz	Svc4			;rate countdown
		goto	t4_ih
		incf	INDF1,F
		goto	t5				;finished t4
t4_ih	decf	Svc4
		incf	INDF1,F
		goto	t4_1h

t4_dh	tstfsz	Svc4			;rate countdown
		goto	t4_dh2
		decf	INDF1,F
		goto	t5				;finished t4
t4_dh2	decf	Svc4
		decf	INDF1,F
		goto	t4_1h



t4_low	bcf		Dir_flg,0
		movlw	S4_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc4


t4_1l	movlw	4
		movwf	S_num
		movlw	S4_hi
		movff	PLUSW2,S_temp
		btfsc	Set_flg,3
		call	snd_mid
		bcf		Set_flg,3
		movf	INDF1,W
		subwf	S4_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t4_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t4_dl
		tstfsz	Svc4			;rate countdown
		goto	t4_il
		incf	INDF1,F
		goto	t5				;finished t4
t4_il	decf	Svc4
		incf	INDF1,F
		goto	t4_1l

t4_dl	tstfsz	Svc4			;rate countdown
		goto	t4_dl2
		decf	INDF1,F
		goto	t5
t4_dl2	decf	Svc4
		decf	INDF1,F
		goto	t4_1l

t4_done	btfsc	Delflg,3
		bra		t5
		bsf		Delflg,3		;set flag for delay cutoff
		bcf		Wait_flg,3		;finished
t4_done1	clrf	Dir_flg
		bcf		Lastpos,3		;save position
		btfsc	Intemp,3
		bsf		Lastpos,3
		btfsc	Intemp,3
		bsf		Dir_flg,0
		btfss	Intemp,3			;set position flag
		bra		t4_pos
		bsf		OFF_end,3
		bra		t4_pos1
t4_pos	bsf		ON_end,3
t4_pos1	movlw	4				;send dn event at end
		call	dn_end		

;**********************************

;		servo 5

t5		lfsr	FSR1, S5_now
		btfsc	Start_flg,4		;is it first time?
		bra		t5s
		btfsc	NV3,4			;is it no move?
		bra		t5s1
		bcf		Inchange,4
		bsf		Gotflg,4
t5s1	bsf		Start_flg,4

t5s		btfss	Inchange,4		;has this input changed?
		goto	t5_noch			;no
		btfss	NV4,4			;is it a wait?
		bra		t5a				;no
		btfsc	Wait_flg,4		;this one moving?
		bra		t5a	
		tstfsz	Wait_flg		;all clear to go?
		bra		t6
t5a		bcf		Inchange,4
		bsf		Outmsk,4		;enable output on servo 5
		bcf		Delflg,4		;clear delay mode on 5
		bcf		Gotflg,4		;not got there on 5
		bsf		Wait_flg,4		;running
		btfsc	Intemp,4
		bsf		Dir_flg,0
		bcf		ON_end,4			;position indeterminate
		bcf		OFF_end,4
		movlw	5			;set for sending DN start event
		movwf	S_num
		call	dn_start
		movlw	Cutval
		movwf	Off5			;set delay timer for 5

t5_noch	btfss	No_flag,4
		bra		t5_no1
		bcf		No_flag,4
		bra		t5_done1
t5_no1	btfsc	Gotflg,4		;already there?
		goto	t6				;next servo
		btfss	Intemp,4		;what is input direction?
		goto	t5_low			;input low
		bsf		Dir_flg,0
		movlw	S5_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc5


t5_1h	movlw	5
		movwf	S_num
		movlw	S5_low
		movff	PLUSW2,S_temp
		btfsc	Set_flg,4
		call	snd_mid
		bcf		Set_flg,4
		movf	INDF1,W
		subwf	S5_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t5_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t5_dh
		tstfsz	Svc5			;rate countdown
		goto	t5_ih
		incf	INDF1,F
		goto	t6				;finished t5
t5_ih	decf	Svc5
		incf	INDF1,F
		goto	t5_1h

t5_dh	tstfsz	Svc5			;rate countdown
		goto	t5_dh2
		decf	INDF1,F
		goto	t6				;finished t5
t5_dh2	decf	Svc5
		decf	INDF1,F
		goto	t5_1h



t5_low	
		bcf		Dir_flg,0
		movlw	S5_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc5


t5_1l	movlw	5
		movwf	S_num
		movlw	S5_hi
		movff	PLUSW2,S_temp
		btfsc	Set_flg,4
		call	snd_mid
		bcf		Set_flg,4
		movf	INDF1,W
		subwf	S5_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t5_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t5_dl
		tstfsz	Svc5			;rate countdown
		goto	t5_il
		incf	INDF1,F
		goto	t6				;finished t5
t5_il	decf	Svc5
		incf	INDF1,F
		goto	t5_1l

t5_dl	tstfsz	Svc5			;rate countdown
		goto	t5_dl2
		decf	INDF1,F
		goto	t6
t5_dl2	decf	Svc5
		decf	INDF1,F
		goto	t5_1l

t5_done	btfsc	Delflg,4
		bra		t6
		bsf		Delflg,4		;set flag for delay cutoff
		bcf		Wait_flg,4		;finished
t5_done1	clrf	Dir_flg
		bcf		Lastpos,4		;save position
		btfsc	Intemp,4
		bsf		Lastpos,4
		btfsc	Intemp,4
		bsf		Dir_flg,0
		btfss	Intemp,4			;set position flag
		bra		t5_pos
		bsf		OFF_end,4
		bra		t5_pos1
t5_pos	bsf		ON_end,4
t5_pos1	movlw	5			
		call	dn_end			;send dn event at end	

;**************************************************

;		servo 6

t6		lfsr	FSR1, S6_now
		btfsc	Start_flg,5		;is it first time?
		bra		t6s
		btfsc	NV3,5			;is it no move?
		bra		t6s1
		bcf		Inchange,5
		bsf		Gotflg,5
t6s1	bsf		Start_flg,5

t6s		btfss	Inchange,5		;has this input changed?
		goto	t6_noch			;no
		btfss	NV4,5			;is it a wait?
		bra		t6a				;no
		btfsc	Wait_flg,5		;this one moving?
		bra		t6a				;start again
		tstfsz	Wait_flg		;all clear to go?
		bra		t7
t6a		bcf		Inchange,5
		bsf		Outmsk,5		;enable output on servo 6
		bcf		Delflg,5		;clear delay mode on 6
		bcf		Gotflg,5		;not got there on 6
		bsf		Wait_flg,5		;running
		btfsc	Intemp,5
		bsf		Dir_flg,0
		bcf		ON_end,5			;position indeterminate
		bcf		OFF_end,5
		movlw	6			;set for sending DN start event
		movwf	S_num
		call	dn_start
		movlw	Cutval
		movwf	Off6			;set delay timer for 6

t6_noch	btfss	No_flag,5
		bra		t6_no1
		bcf		No_flag,5
		bra		t6_done1
t6_no1	btfsc	Gotflg,5		;already there?
		goto	t7				;next servo
		btfss	Intemp,5		;what is input direction?
		goto	t6_low			;input low
		bsf		Dir_flg,0
		movlw	S6_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc6


t6_1h	movlw	6
		movwf	S_num
		movlw	S6_low
		movff	PLUSW2,S_temp
		btfsc	Set_flg,5
		call	snd_mid
		bcf		Set_flg,5
		movf	INDF1,W
		subwf	S6_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t6_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t6_dh
		tstfsz	Svc6			;rate countdown
		goto	t6_ih
		incf	INDF1,F
		goto	t7				;finished t6
t6_ih	decf	Svc6
		incf	INDF1,F
		goto	t6_1h

t6_dh	tstfsz	Svc6			;rate countdown
		goto	t6_dh2
		decf	INDF1,F
		goto	t7				;finished t6
t6_dh2	decf	Svc6
		decf	INDF1,F
		goto	t6_1h



t6_low	bcf		Dir_flg,0
		movlw	S6_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc6


t6_1l	movlw	6
		movwf	S_num
		movlw	S6_hi
		movff	PLUSW2,S_temp
		btfsc	Set_flg,5
		call	snd_mid
		bcf		Set_flg,5
		movf	INDF1,W
		subwf	S6_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t6_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t6_dl
		tstfsz	Svc6			;rate countdown
		goto	t6_il
		incf	INDF1,F
		goto	t7				;finished t6
t6_il	decf	Svc6
		incf	INDF1,F
		goto	t6_1l

t6_dl	tstfsz	Svc6			;rate countdown
		goto	t6_dl2
		decf	INDF1,F
		goto	t7
t6_dl2	decf	Svc6
		decf	INDF1,F
		goto	t6_1l

t6_done	btfsc	Delflg,5
		bra		t7
		bsf		Delflg,5		;set flag for delay cutoff
		bcf		Wait_flg,5		;finished
t6_done1	clrf	Dir_flg
		bcf		Lastpos,5		;save position
		btfsc	Intemp,5
		bsf		Lastpos,5
		btfsc	Intemp,5
		bsf		Dir_flg,0
		btfss	Intemp,5			;set position flag
		bra		t6_pos
		bsf		OFF_end,5
		bra		t6_pos1
t6_pos	bsf		ON_end,5
t6_pos1	movlw	6	
		call	dn_end		

;********************************************************

;		servo 7

t7		lfsr	FSR1, S7_now
		btfsc	Start_flg,6		;is it first time?
		bra		t7s
		btfsc	NV3,6			;is it no move?
		bra		t7s1
		bcf		Inchange,6
		bsf		Gotflg,6
t7s1	bsf		Start_flg,6

t7s		btfss	Inchange,6		;has this input changed?
		goto	t7_noch			;no
		btfss	NV4,6			;is it a wait?
		bra		t7a				;no
		btfsc	Wait_flg,6		;this one moving?
		bra		t7a				;start again
		tstfsz	Wait_flg		;all clear to go?
		bra		t8
t7a		bcf		Inchange,6
		bsf		Outmsk,6		;enable output on servo 7
		bcf		Delflg,6		;clear delay mode on 7
		bcf		Gotflg,6		;not got there on 7
		bsf		Wait_flg,6		;running
		btfsc	Intemp,6
		bsf		Dir_flg,0
		bcf		ON_end,6			;position indeterminate
		bcf		OFF_end,6
		movlw	7			;set for sending DN start event
		movwf	S_num
		call	dn_start
		movlw	Cutval
		movwf	Off7			;set delay timer for 7

t7_noch	btfss	No_flag,6
		bra		t7_no1
		bcf		No_flag,6
		bra		t7_done1
t7_no1	btfsc	Gotflg,6		;already there?
		goto	t8				;next servo
		btfss	Intemp,6		;what is input direction?
		goto	t7_low			;input low
		bsf		Dir_flg,0
		movlw	S7_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc7


t7_1h	movlw	7
		movwf	S_num
		movlw	S7_low
		movff	PLUSW2,S_temp
		btfsc	Set_flg,6
		call	snd_mid
		bcf		Set_flg,6
		movf	INDF1,W
		subwf	S7_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t7_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t7_dh
		tstfsz	Svc7			;rate countdown
		goto	t7_ih
		incf	INDF1,F
		goto	t8				;finished t7
t7_ih	decf	Svc7
		incf	INDF1,F
		goto	t7_1h

t7_dh	tstfsz	Svc7			;rate countdown
		goto	t7_dh2
		decf	INDF1,F
		goto	t8				;finished t7
t7_dh2	decf	Svc7
		decf	INDF1,F
		goto	t7_1h



t7_low	bcf		Dir_flg,0
		movlw	S7_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc7


t7_1l	movlw	7
		movwf	S_num
		movlw	S7_hi
		movff	PLUSW2,S_temp
		btfsc	Set_flg,6
		call	snd_mid
		bcf		Set_flg,6
		movf	INDF1,W
		subwf	S7_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t7_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t7_dl
		tstfsz	Svc7			;rate countdown
		goto	t7_il
		incf	INDF1,F
		goto	t8				;finished t7
t7_il	decf	Svc7
		incf	INDF1,F
		goto	t7_1l

t7_dl	tstfsz	Svc7			;rate countdown
		goto	t7_dl2
		decf	INDF1,F
		goto	t8
t7_dl2	decf	Svc7
		decf	INDF1,F
		goto	t7_1l

t7_done	btfsc	Delflg,6
		bra		t8
		bsf		Delflg,6		;set flag for delay cutoff
		bcf		Wait_flg,6		;finished
t7_done1	clrf	Dir_flg
		bcf		Lastpos,6		;save position
		btfsc	Intemp,6
		bsf		Lastpos,6
		btfsc	Intemp,6
		bsf		Dir_flg,0
		btfss	Intemp,6			;set position flag
		bra		t7_pos
		bsf		OFF_end,6
		bra		t7_pos1
t7_pos	bsf		ON_end,6
t7_pos1	movlw	7				;send dn event at end
		call	dn_end		

;*****************************************************

;		servo 8

t8		lfsr	FSR1, S8_now
		btfsc	Start_flg,7		;is it first time?
		bra		t8s
		btfsc	NV3,7			;is it no move?
		bra		t8s1
		bcf		Inchange,7
		bsf		Gotflg,7
t8s1	bsf		Start_flg,7

t8s		btfss	Inchange,7		;has this input changed?
		goto	t8_noch			;no
		btfss	NV4,7			;is it a wait?
		bra		t8a				;no
		btfsc	Wait_flg,7		;this one moving?
		bra		t8a				;start again
		tstfsz	Wait_flg		;all clear to go?
		bra		t9
t8a		bcf		Inchange,7
		bsf		Outmsk,7		;enable output on servo 8
		bcf		Delflg,7		;clear delay mode on 8
		bcf		Gotflg,7		;not got there on 8
		bsf		Wait_flg,7		;running
		btfsc	Intemp,7
		bsf		Dir_flg,0
		bcf		ON_end,7			;position indeterminate
		bcf		OFF_end,7
		movlw	8			;set for sending DN start event
		movwf	S_num
		call	dn_start
		movlw	Cutval
		movwf	Off8			;set delay timer for 8

t8_noch	btfss	No_flag,7
		bra		t8_no1
		bcf		No_flag,7
		bra		t8_done1
t8_no1	btfsc	Gotflg,7		;already there?
		goto	t9				;next servo
		btfss	Intemp,7		;what is input direction?
		goto	t8_low			;input low
		bsf		Dir_flg,0
		movlw	S8_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc8


t8_1h	movlw	8
		movwf	S_num
		movlw	S8_low
		movff	PLUSW2,S_temp
		btfsc	Set_flg,7
		call	snd_mid
		bcf		Set_flg,7
		movf	INDF1,W
		subwf	S8_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t8_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t8_dh
		tstfsz	Svc8			;rate countdown
		goto	t8_ih
		incf	INDF1,F
		goto	t9				;finished t8
t8_ih	decf	Svc8
		incf	INDF1,F
		goto	t8_1h

t8_dh	tstfsz	Svc8			;rate countdown
		goto	t8_dh2
		decf	INDF1,F
		goto	t9				;finished t8
t8_dh2	decf	Svc8
		decf	INDF1,F
		goto	t8_1h



t8_low	bcf		Dir_flg,0
		movlw	S8_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Svc8


t8_1l	movlw	8
		movwf	S_num
		movlw	S8_hi
		movff	PLUSW2,S_temp
		btfsc	Set_flg,7
		call	snd_mid
		bcf		Set_flg,7
		movf	INDF1,W
		subwf	S8_mid,W
		btfsc	STATUS,Z
		call	snd_mid			;at mid point	
		movf	INDF1,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t8_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t8_dl
		tstfsz	Svc8			;rate countdown
		goto	t8_il
		incf	INDF1,F
		goto	t9				;finished t8
t8_il	decf	Svc8
		incf	INDF1,F
		goto	t8_1l

t8_dl	tstfsz	Svc8			;rate countdown
		goto	t8_dl2
		decf	INDF1,F
		goto	t9
t8_dl2	decf	Svc8
		decf	INDF1,F
		goto	t8_1l

t8_done	btfsc	Delflg,7
		bra		t9
		bsf		Delflg,7		;set flag for delay cutoff
		bcf		Wait_flg,7
t8_done1	clrf	Dir_flg
		bcf		Lastpos,7		;save position
		btfsc	Intemp,7
		bsf		Lastpos,7
		btfsc	Intemp,7
		bsf		Dir_flg,0
		btfss	Intemp,7			;set position flag
		bra		t8_pos
		bsf		OFF_end,7
		bra		t8_pos1
t8_pos	bsf		ON_end,7
t8_pos1	movlw	8			;send dn event at end
		call	dn_end	





;*************************************************************************
;		here when all servos set

t9		btfss	Delflg,0		;set up for delays
		goto	t10
		bsf		Outmsk,0		;enable servo 1
		decfsz	Off1,F			;decrement delay 1
		goto	t10
		bcf		Delflg,0		;delay out
		btfss	Cutmode,0		;no cutoff?
		bra		t9a
		bcf		Outmsk,0		;stop servo 1
		bsf		Outport,0		;leave output high
t9a		bsf		Gotflg,0		;no futher action till change

t10		btfss	Delflg,1	
		goto	t11
		bsf		Outmsk,1		;enable servo 2
		decfsz	Off2,F			;decrement delay 2
		goto	t11
		bcf		Delflg,1		;delay out
		btfss	Cutmode,1		;no cutoff?
		bra		t10a
		bcf		Outmsk,1		;stop servo 2
		bsf		Outport,1		;leave output high
t10a	bsf		Gotflg,1		;no futher action till change

t11		btfss	Delflg,2		;set up for delays
		goto	t12
		bsf		Outmsk,2		;enable servo 3
		decfsz	Off3,F			;decrement delay 3
		goto	t12
		bcf		Delflg,2		;delay out
		btfss	Cutmode,2		;no cutoff?
		bra		t11a
		bcf		Outmsk,2		;stop servo 3
		bsf		Outport,2		;leave output high
t11a	bsf		Gotflg,2		;no futher action till change

t12		btfss	Delflg,3		;set up for delays
		goto	t13
		bsf		Outmsk,3		;enable servo 4
		decfsz	Off4,F			;decrement delay 4
		goto	t13
		bcf		Delflg,3		;delay out
		btfss	Cutmode,3		;no cutoff?
		bra		t12a
		bcf		Outmsk,3		;stop servo 4
		bsf		Outport,3		;leave output high
t12a	bsf		Gotflg,3		;no futher action till change


	

t13		btfss	Delflg,4		;set up for delays
		goto	t14
		bsf		Outmsk,4		;enable servo 5
		decfsz	Off5,F			;decrement delay 5
		goto	t14
		bcf		Delflg,4		;delay out
		btfss	Cutmode,4		;no cutoff?
		bra		t13a
		bcf		Outmsk,4		;stop servo 5
		bsf		Outport,4		;leave output high
t13a	bsf		Gotflg,4		;no futher action till change

t14		btfss	Delflg,5		;set up for delays
		goto	t15
		bsf		Outmsk,5		;enable servo 6
		decfsz	Off6,F			;decrement delay 6
		goto	t15
		bcf		Delflg,5		;delay out
		btfss	Cutmode,5		;no cutoff?
		bra		t14a
		bcf		Outmsk,5		;stop servo 6
		bsf		Outport,5		;leave output high
t14a	bsf		Gotflg,5		;no futher action till change

t15		btfss	Delflg,6		;set up for delays
		goto	t16
		bsf		Outmsk,6		;enable servo 7
		decfsz	Off7,F			;decrement delay 7
		goto	t16
		bcf		Delflg,6		;delay out
		btfss	Cutmode,6		;no cutoff?
		bra		t15a
		bcf		Outmsk,6		;stop servo 7
		bsf		Outport,6		;leave output high
t15a	bsf		Gotflg,6		;no futher action till change

t16		btfss	Delflg,7		;set up for delays
		goto	t17
		bsf		Outmsk,7		;enable servo 8
		decfsz	Off8,F			;decrement delay 8
		goto	t17
		bcf		Delflg,7		;delay out
		btfss	Cutmode,7		;no cutoff?
		bra		t16a
		bcf		Outmsk,7		;stop servo 8
		bsf		Outport,7		;leave output high
t16a	bsf		Gotflg,7		;no futher action till change


t17		return
;


;**********************************************

;	separate servo routine for startup

ser_start
		;clrf	Input
		movlw	LOW	NVnow
		movwf	EEADR
		call	eeread
		movwf	Oldpos
		iorwf	NV2,W
		movwf	Input
		movwf	Intemp
;		movff	NV3,Inchange
		lfsr	FSR2,S1_now
		movlw	LOW SVstart
		movwf	EEADR
		btfsc	Intemp,0		;set start for servo 1
		incf	EEADR,F
		call	eeread
		movwf	POSTINC2
		movlw	LOW SVstart+4
		movwf	EEADR
		btfsc	Intemp,1		;set start for servo 2
		incf	EEADR,F
		call	eeread
		movwf	POSTINC2
		movlw	LOW SVstart+8
		movwf	EEADR
		btfsc	Intemp,2		;set start for servo 3
		incf	EEADR,F
		call	eeread
		movwf	POSTINC2
		movlw	LOW SVstart+.12
		movwf	EEADR
		btfsc	Intemp,3		;set start for servo 4
		incf	EEADR,F
		call	eeread
		movwf	POSTINC2
		movlw	LOW SVstart+.16
		movwf	EEADR
		btfsc	Intemp,4		;set start for servo 5
		incf	EEADR,F
		call	eeread
		movwf	POSTINC2
		movlw	LOW SVstart+.20
		movwf	EEADR
		btfsc	Intemp,5		;set start for servo 6
		incf	EEADR,F
		call	eeread
		movwf	POSTINC2
		movlw	LOW SVstart+.24
		movwf	EEADR
		btfsc	Intemp,6		;set start for servo 7
		incf	EEADR,F
		call	eeread
		movwf	POSTINC2
		movlw	LOW SVstart+.28
		movwf	EEADR
		btfsc	Intemp,7		;set start for servo 8
		incf	EEADR,F
		call	eeread
		movwf	POSTINC2
		
		return
		



;*******************************************
;	Routine to send an event when movement starts or ends if the DN bit is set for that servo
;	Comes with servo number in W
;	Looks to see if it is a DN servo
;	Gets event for that DN, checks ON or OFF
;	Checks for event polarity
;	Sends as either short or long depending on Event hi bytes

dn_start	
		decf  	WREG		;servo number - 1
		movwf	Count
		clrf	WREG
dn_s1	movf	Count,F		;is it zero?
		bz		dn_s2
		addlw	4
		decf	Count,F
		bra		dn_s1
dn_s2	movwf	ENtemp

		addlw	LOW EVstart	;point to EV table
		movwf	EEADR
		btfss	Dir_flg,0		;is it ON or OFF 
		incf	EEADR
		call 	eeread		;get EV
		movwf	EVtemp3
		btfss	WREG,7		;is it a DN?
		return
		
		movf	ENtemp,W	;Events stored as 4 bytes
		rlncf	WREG
		rlncf	WREG
		btfss	Dir_flg,0		;on or off end
		addlw	4			
		call	load_dn
		btfsc	WREG,0
		bra		long_st		;is long start event
		movlw	0x99		;off
		btfsc	EVtemp3,6		;polarity reverse?
		decf	WREG
		movwf	Tx1d0
		call	dely		;pause between incoming and outgoing frames
		call	sendTX
		return
long_st	movlw	0x91		;off
		btfsc	EVtemp3,6		;polarity reverse?
		decf	WREG
		movwf	Tx1d0
		call	dely		;pause between incoming and outgoing frames
		call	sendTXa
		return

dn_end	decf  	WREG		;servo number - 1
		movwf	Count
		clrf	WREG
dn_e1	movf	Count,F		;is it zero?
		bz		dn_e2
		addlw	4
		decf	Count,F
		bra		dn_e1
dn_e2	movwf	ENtemp


		addlw	LOW EVstart	;point to EV table
		movwf	EEADR
		btfsc	Dir_flg,0		;is it ON or OFF end
		incf	EEADR
		call 	eeread		;get EV
		movwf	EVtemp
		btfss	WREG,7		;is it a DN?
		return
		rlncf	ENtemp,F		;Events stored as 4 bytes
		rlncf	ENtemp,W
		btfsc	Dir_flg,0
		addlw	4	
		call	load_dn
		btfsc	WREG,0
		bra		long_end
		movlw	0x98		;on
		btfsc	EVtemp,6		;polarity reverse?
		incf	WREG
		movwf	Tx1d0
		call	sendTX	
		return

long_end	movlw	0x90		;on
		btfsc	EVtemp,6		;polarity reverse?
		incf	WREG
		movwf	Tx1d0
		call	sendTXa
		return


load_dn	
		addlw	LOW ENstart		;get events
		movwf	EEADR
		call	eeread
		movwf	Tx1d1
		incf	EEADR
		call	eeread
		movwf	Tx1d2
		incf	EEADR
		call	eeread
		movwf	Tx1d3
		incf	EEADR
		call	eeread
		movwf	Tx1d4
		movlw	5
		movwf	Dlc
		movlw 	0		;clear flag
		movf	Tx1d1,F
		bnz		long_ev
		movf	Tx1d2,F
		bnz		long_ev
		return
long_ev	bsf	WREG,0		;flag long event
	
		return

;*************************************************************************
;
;	calculate mid point value for servo travel

mid_pt	lfsr	FSR1,SV1		;set pointers
		lfsr	FSR2,S1_mid
		movlw	8
		movwf	Count
mid_pt1	movf	POSTINC1,W
		movwf	Temp			;save 1st end
		subwf	POSTINC1,W
		btfsc	STATUS,C		;cary clear if negative
		bra		mid_pt2
		comf	WREG
		incf	WREG			;2s complement if negative
		rrncf	WREG
		bcf		WREG,7
		subwf	Temp,W
		movwf	POSTINC2
		bra		mid_pt3
mid_pt2	rrncf	WREG
		bcf		WREG,7
		addwf	Temp,W
		movwf	POSTINC2	

	
mid_pt3	incf	FSR1L
		incf	FSR1L
		decfsz	Count
		bra		mid_pt1
		return

;****************************************************************
;
;		test routine when setting

test	tstfsz	NV5				;any test message?
		bra		test1			;no
		return
test1	movff	NV5,Temp		;save NV5
		bcf		Temp,7			;clear direction bit
		clrf	T_roll			;which servo?
		bsf		T_roll,0
test2	dcfsnz	Temp,f
		bra		test3
		rlncf	T_roll,F
		bra		test2
test3	movff	T_roll,EVtemp	;simulate real event
		clrf	EVtemp2
		clrf	EVtemp3
		clrf	ev_opc
		btfss	NV5,7			;direction
		bsf		ev_opc,0
		clrf	NV5
		goto	ev_set


;*******************************************************************************
;		a delay routine
			
dely	movlw	.10
		movwf	Count1
dely2	clrf	Count3
dely1	decfsz	Count3,F
		goto	dely1
		decfsz	Count1
		bra		dely2
		return		
		
;****************************************************************

;		a bit longer delay routine
			
sdely	movlw	.200
		movwf	Count1
sdely2	clrf	Count3
sdely1	decfsz	Count3,F
		goto	sdely1
		decfsz	Count1
		bra		sdely2
		return		
;****************************************************************

;		longer delay

ldely	movlw	.100
		movwf	Count2
ldely1	call	dely
		decfsz	Count2
		bra		ldely1
		
		return


;***********************************************************************

readNVs		btfss	Mode,1		;in FLiM?
			return
			movlw	LOW	NVstart
			addwf	RXB0D3,W		;add index
			movwf	EEADR
			decf	EEADR,F		;index starts at 1, buffer at 0
			call	eeread
			movwf	Tx1d4		;NV val to transmit buffer
			movff	RXB0D3,Tx1d3	;transfer index
			movlw	0x97		;NVANS
			movwf	Tx1d0
			movlw	5
			movwf	Dlc
			call	sendTX
			return

;**********************************************************************
;			learns NVs by index. Used for setting servo positions.

lrnNVs		btfsc	Datmode,4		;is it in learn mode?
			bra		lrnNVs_1
			movlw	2				;error not in learn mode
			call	errsub
			return	
lrnNVs_1	bsf		Datmode,7		;flag learning servo state
			movf	RXB0D3,W
			sublw	4				;1 to 4 are not SVs
			bc		notSV

			movlw	.37
			cpfseq	RXB0D3
			bra		doSVs
			movff	RXB0D4,NV5
			return
			
doSVs		lfsr	FSR2,SV1
			movlw	5
			subwf	RXB0D3,W
			movff	RXB0D4,PLUSW2	;put in ram
			movwf	Index1
			rrncf	WREG			;get servo number into WREG  (0 to 7)
			rrncf	WREG
			andlw	B'00000111'
		
			clrf	SV_flg
			bsf		SV_flg,0		;set rolling bit
sv_roll		movf	WREG,W
			bnz		sv_roll2
			bra		sv_roll1
sv_roll2	rlncf	SV_flg,F
			decf	WREG
			bra		sv_roll

sv_roll1	btfsc	Index1,1		;is it a speed value?
			return
				
			movf	SV_flg,W
			movwf	Inchange		;set servo to run
			movwf	Intemp
			movwf	Gotflg			
			comf	Gotflg,F		;so it runs
			btfsc	RXB0D3,0			;on or off state
			clrf	Intemp
			call	mid_pt			;recalculate mid points
			return
notSV		lfsr	FSR2,NV1
			decf	RXB0D3,W
			movff	RXB0D4,PLUSW2
			movff	NV1,Cutmode		;cutoff mode. Cutoff if bit set.
			return


sendlog
		movlw	0xF7
		movwf	Tx1d0
		movff	FSR0H, Tx1d3
		movff	FSR0L, Tx1d4
		movff	EVtemp, Tx1d5
		movff	EVtemp2, Tx1d6
		clrf	Tx1d7
		movlw	8
		movwf	Dlc
		call	sendTX
		call	ldely
		return

;**************************************************************
;	teach position DNs. Takes settings in EV3, held in EVdata

dn_teach		btfss	EVdata,7		;is it a DN setting?
		return			;no
		
		

dn_store		movff	EVdata,EVtemp
		rlncf	EVdata,F
		rlncf	EVdata,W
		andlw	B'01111100'	;mask all except address
		addlw	LOW ENstart
		movwf	EEADR
		movf	ev0,W
		call	eewrite
		incf	EEADR
		movf	ev1,W
		call	eewrite
		incf	EEADR
		movf	ev2,W
		call	eewrite
		incf	EEADR
		movf	ev3,W
		call	eewrite

		movf	EVtemp,W
		andlw	B'00011111'	;mask top two bits
		addlw	LOW EVstart	;now put in EVs 
		
		movwf	EEADR		;set address in EEPROM
		movf	EVtemp,W
		call	eewrite		;write EV value
		movff	EVtemp,EVdata		;restore EVdata
		return

;************************************************************
;
;			Sends mid way event for frog switching.
;			ON or OFF depending on pol bit in EV3 (bit 5)
;			S_num comes with servo number 1 to 8
;			Can be long or short event depending on what was taught

		
snd_mid	
		decf  	S_num,W		;servo number - 1
		movwf	Count
		clrf	WREG
mid_1	movf	Count,F
		bz		mid_2	
		addlw	4
		decf	Count,F
		bra		mid_1
mid_2	movwf	ENtemp

		addlw	LOW EVstart	;point to EV table
		movwf	EEADR
		incf	EEADR		;3rd EV of set
		incf	EEADR
		call 	eeread		;get EV
		movwf	EVtemp
		btfss	WREG,7		;is it a DN?
		return
		
		movf	ENtemp,W	;Events stored as 4 bytes
		rlncf	WREG
		rlncf	WREG
		
		addlw	.8			
		call	load_dn
		btfsc	WREG,0
		bra		long_fg
		btfss	Dir_flg,0
		bra		frog_on
		movlw	0x99		;off
		btfsc	EVtemp,5		;polarity reverse?
		decf	WREG
		
frog_sh	movwf	Tx1d0
		call	sendTX
		return

frog_on	movlw	0x98
		btfsc	EVtemp,5		;polarity reverse?
		incf	WREG
		bra		frog_sh

long_fg	btfss	Dir_flg,0
		bra		lngf_on
		movlw	0x91		;off
		btfsc	EVtemp,5		;polarity reverse?
		decf	WREG
	
		
frog_ln	movwf	Tx1d0
		call	sendTXa
		return

lngf_on	movlw	0x90
		btfsc	EVtemp,5
		incf	WREG
		bra		frog_ln

;***************************************************************

;		self enumeration as separate subroutine

self_en	movff	FSR1L,Fsr_tmp1Le	;save FSR1 just in case
		movff	FSR1H,Fsr_tmp1He 
;		movlw	B'11000000'
;		movwf	INTCON			;start interrupts if not already started
		bsf		Datmode,1		;set to 'setup' mode
		movlw	.14
		movwf	Count
		lfsr	FSR0, Enum0
clr_en
		clrf	POSTINC0
		decfsz	Count
		bra		clr_en

		movlw	0x3C			;set T3 to 1 mSec (may need more?)
		movwf	TMR3H
		movlw	0xAF
		movwf	TMR3L
		movlw	B'10110001'
		movwf	T3CON			;enable timer 3
		bcf		PIR2,TMR3IF
		
		movlb	.15
		movlw	B'10111111'		;fixed node, default ID  
		movwf	TXB1SIDH
		movlw	B'11100000'
		movwf	TXB1SIDL
		movlw	B'01000000'		;RTR frame
		movwf	TXB1DLC
rtr_snd	btfsc	TXB1CON,TXREQ
		bra		rtr_snd
		bsf		TXB1CON,TXREQ
rtr_go	btfsc	TXB1CON,TXREQ		;wait till sent
		bra		rtr_go
		clrf	TXB1DLC				;no more RTR frames
		movlb	0
		
	

	
		
	

self_en1	btfsc	PIR2,TMR3IF		;setup timer out?
		bra		en_done
		btfsc	COMSTAT,7		;look for CAN input. 
		bra		getcan1
		bra		self_en1		;no CAN
	

getcan1	movf	CANCON,W
		andlw	B'00001111'
		movwf	TempCANCON
		movf	ECANCON,W
		andlw	B'11110000'
		iorwf	TempCANCON,W
		movwf	ECANCON
		btfsc	RXB0SIDL,EXID		;ignore extended frames here
		bra		no_can1
		
		
en_1	btfss	Datmode,1			;setup mode?
		bra		no_can1
		movf	RXB0DLC,F
		bnz		no_can1				;only zero length frames
		call	setmode
		bra		no_can1	

no_can1	bcf		RXB0CON,RXFUL
		bra		self_en1			;loop till timer out 

en_done	bcf		T3CON,TMR3ON	;timer off
		bcf		PIR2,TMR3IF		;clear flag


		clrf	IDcount
		incf	IDcount,F			;ID starts at 1
		clrf	Roll
		bsf		Roll,0
		lfsr	FSR1,Enum0			;set FSR to start
here1	incf	INDF1,W				;find a space
		bnz		here
		movlw	8
		addwf	IDcount,F
		incf	FSR1L
		bra		here1
here	movf	Roll,W
		andwf	INDF1,W
		bz		here2
		rlcf	Roll,F
		incf	IDcount,F
		bra		here
here2	movlw	.100				;limit to ID
		cpfslt	IDcount
		bra		segful				;segment full
		
here3	movlw	LOW CANid		;put new ID in EEPROM
		movwf	EEADR
		movf	IDcount,W
		call	eewrite
		movf	IDcount,W
		call	newid_f			;put new ID in various buffers

			
		movff	Fsr_tmp1Le,FSR1L	;
		movff	Fsr_tmp1He,FSR1H 
		return	0					

segful	movlw	7		;segment full, no CAN_ID allocated
		call	errsub
		setf	IDcount
		bcf		IDcount,7
		bra		here3

;****************************************************************
;
setmode	tstfsz	RXB0DLC
		return				;only zero length frames for setup
		
		swapf	RXB0SIDH,W			;get ID into one byte
		rrcf	WREG
		andlw	B'01111000'			;mask
		movwf	Temp
		swapf	RXB0SIDL,W
		rrncf	WREG
		andlw	B'00000111'
		iorwf	Temp,W
		movwf	IDcount				;has current incoming CAN_ID

		lfsr	FSR1,Enum0			;set enum to table
enum_st	clrf	Roll				;start of enum sequence
		bsf		Roll,0
		movlw	8
enum_1	cpfsgt	IDcount
		bra		enum_2
		subwf	IDcount,F			;subtract 8
		incf	FSR1L				;next table byte
		bra		enum_1
enum_2	dcfsnz	IDcount,F
		bra		enum_3
		rlncf	Roll,F
		bra		enum_2
enum_3	movf	Roll,W
		iorwf	INDF1,F
		bcf		RXB0CON,RXFUL		;clear read


		return

;**************************************************************
;
;	deletes a feedback event by clearing  EV3 in EEPROM
;	arrives with EV3 in EVtemp3 

ev_del	movf	EVtemp3,W
		btfss	WREG,7		;is it a response event?
		return
		andlw	B'00011111'	;mask top two bits
		movwf	EVtemp
		addlw	LOW EVstart	;now put in EVs 
		
		movwf	EEADR		;set address in EEPROM
		clrf	WREG
		call	eewrite		;write EV value
		rlncf	EVtemp,F	;4 bytes per event
		rlncf	EVtemp,W
		addlw	LOW ENstart
		movwf	EEADR
		clrf	WREG
		call	eewrite
		clrf	WREG
		incf	EEADR,F
		call	eewrite
		clrf	WREG
		incf	EEADR,F
		call	eewrite
		clrf	WREG
		incf	EEADR,F
		call	eewrite


		return

;*************************************************************
;		clear all EEPROM if delete all events

clreepr movlw	LOW ENstart		;clear response events
		movwf	EEADR
		movlw	.128
		movwf	Count
clree1	clrf	WREG
		call	eewrite
		incf	EEADR
		decfsz	Count
		bra		clree1

		movlw	LOW EVstart		;clear EV3 set
		movwf	EEADR
		movlw	.32
		movwf	Count
clree2	clrf	WREG
		call	eewrite
		incf	EEADR
		decfsz	Count
		bra		clree2
		return
		
	
	
		
;***************************************************************

;			ORG 0x2800		;speed table
	
Spdtbl		db	.20,.10		;fastest. These values could be modified if wanted
			db	.7,.5
			db	.3,.2
			db	.1,.0		;slowest. Value of 0 is  1/256 per 20 mSec cycle

; Events data, room for 128 events at 16 bytes/event = 2kb			
	ORG	0x3000
evdata

;************************************************************************		
	ORG 0xF00000			;EEPROM data. Defaults
	
CANid	de	B'01111111',0	;CAN id default and module status
NodeID	de	0,0			;Node ID
ENindex	de	0,0		; hi byte contains free space
					; lo byte contains number of events
					; hi byte + lo byte = EN_NUM

	ORG 0xF00006

ENstart	;event numbers stored here. Room for 32 four byte events.
	;these are now indexed by servo number. 4 events per servo but only three used so far.
	;used for output events only.
		
	ORG 0xF00086
	
		;response event variables stored here. set to defaults initially (no responses)
		
EVstart	de		0,0,0,0,0,0,0,0	;allows for 1 EV per event (EV3) 
	de		0,0,0,0,0,0,0,0
	de		0,0,0,0,0,0,0,0
	de		0,0,0,0,0,0,0,0

	ORG 0xF000BE
hashnum
	de		0,0,0,0,0,0,0,0


		

NVstart	de 0xFF,0x00,0xFF,0xFF			;4 NVs for node variables
										;NV1 is cutoff mode. Bit set is cutoff
										;NV2 startup position. Bit set is OFF end. 
										;bit clear is now go to last saved position
										;NV3 is move on startup. Bit set is move.
										;NV4 is wait to start. Bit set is wait.

;allows for 4 SVs per servo

SVstart	de	.127,.127,0,0				;defaults 
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0
NVnow	de	0xFF,0							;holds current or last servo positions
											;default is all off

hashtab	de		0,0,0,0,0,0,0,0
		de		0,0,0,0,0,0,0,0
FreeCh	de		0,0

		
	

		ORG	0xF000FE
		de		0,0		;for boot load
		end


