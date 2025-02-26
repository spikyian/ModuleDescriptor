;   	TITLE		"Code for combined panel module  FLiM node for CBUS"
; filename CANPAN_v1y.asm	 	30/05/16
; This code is for a PIC  18F25K80
; Uses  "newevhndler_v1b.asm"
; uses ccbusdefh.inc

;adapted from CANLED64 and CANACE3.

; Beta 104 24/03/14 RH Uses newevhndlr_1b.asm, fixed LED Flash and set NV_NUM to 0
; Beta 105 25/03/14 RH Added REVAL so FCU can read events

; Uses 4 MHz resonator and PLL for 16 MHz clock
; The setup timer is TMR0. Used during self enumeration.
; CAN bit rate of 125 Kbits/sec
; Standard frame only except for bootloader





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

;Implements ENUM and CANID OpCodes

;Teaches both switch events and LED events

;EV1	If 0 is a LED event, if a 1 is a switch event. Can also have LEDs set for switch feedback
;		If 2 it is a SoD event only. 3 if self SoD.
;EV2	Switch number if used. Base 1.Top bit is set by firmware if a taught switch event.
;EV3	Switch action byte (SV)

;  EV3  Switch states for toggles etc  (32 switches). 
;		bit 0 is set for on/off  Default is set
;		bit 1 is polarity
;		bit 2 is on only (off only if pol set)
;		bit 3 is toggle on / off mode
;		bit 4 is send to self as well. (set LEDs)
;		bit 5 is short event
;		bit 6 is toggle state (used in RAM only)
;		bit 7 set for a learned event. Used in startup routine.
;		all zero is do nothing



;EV4	not used at present. May need it for startup options


;this code uses EV13 to determine what the LED event is.
;EV13 must be written before teaching the rest of the EVs

;EV13 = 0XFF		All LEDs are on / off
;EV13 = 0xFE		All LEDs ON only
;EV13 = 0xFD		All LEDs OFF only
;EV13 = 0xF8		LEDs flash

;EV5 = LEDs 1 to 8		bit set is LED active
;EV6 = LEDs 9 to 15
;EV7 = LEDs 16 to 23
;EV8 = LEDs 24 to 32
;EV9 = POL 1 to 8
;EV10 = POL 9 to 15
;EV11 = POL 16 to 23
;EV12 = POL 24 to 32
;EV13 = LED control byte.






;Rev v1a		Starting version
;Rev 1b			LEDs working
;Rev 1c			Added switches
;Rev 1d			Made 'do_it' a subroutine for self action of lEDs
;Rev 1e			Enabled bootloader on PORTC
;Rev 1e2		Rev 1e but port changes for 74HC238. Reversed switch scan polarity
;Rev 1f2		As 1e2 but switch table moved to 0x2900 from 0x5000
;Rev 1g2		Correction in putsw_ev for FLASH address. Correction to module name and NN0
;				Indexed mode for switches and non-indexed for LEDs  17/02/14
;Rev 1h			Changed way switch events are taught  12/03/14
;				No use of NV0. Uses non-indexed mode for all events.Indexed mode not used at all.
;				Sequence of event EVs now changed. LED scheme updated for diffent EV positions.
;Rev 1k			As 1h but added polling of on/off inputs and toggle states.
;Rev1m			Roger's mods to v1k (n0 v1l) 25/03/14
;Rev 1n			same as 1m but green and yellow LEDs swapped to save chamging PCB
;Rev 1p			No rev o. Added save of state for switches and POR sequence to set outputs.
;Rev 1q			As rev p but default SVs now set in code, not defined in FLASH - for bootloading
;				Also used NV1 to say what it does on a POR. 0 = last state, 1= nothing, 2 = all on.
;Rev 1r			Rearrange the layout setup so it will respond to incoming events from other CANPANs.
;Rev 1s			Change for startup error. Added SoD  30/10/14
;Rev 1t			Correction for SoD response 02/03/15. Will not do a self SoD from its own button.
;				OK for external SoD, as from a PC etc.
;Rev 1u			Correction for CAN_ID. 24/05/15 
;				Change so external switch events set the layout status flags in EEPROM  
;Rev 1u Beta102 Fix parameters, add MCPU_MICROCHIP
;Rev 1y Beta101 A new version of Rev 1u Beta102 to incorporate Self-S0D from panel switch
;				Rev v, w and x had problems so went back to 1u Beta102 as the basis 
;Rev 1y			Release version of 1y. Beta removed.

;end of comments for CANPAN. This is FLiM only at present but still needs putting into FLiM and giving a NN.


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
	LIST	P=18F25K80,r=hex,N=75,C=120,T=ON

	include		"p18f25K80.inc"
	include		"cbusdefs8h.inc"
	
; Must define FLIM_ONLY for "newevhndler_v1a.asm"


#define FLIM_ONLY
#define EVBLK_SZ	.32

BLK_SZ 		equ	EVBLK_SZ
EN_NUM		equ .128
EVENT_SZ	equ	BLK_SZ-8
ERASE_SZ	equ	.64
HASH_SZ 	equ .32
HASH_MASK	equ	HASH_SZ-1
EVSPER_BLK	equ ERASE_SZ/BLK_SZ

S_PORT 	equ	PORTA	;setup switch  Change as needed
S_BIT	equ	3


LED_PORT equ	PORTC  ;change as needed

;	Values for CANPAN32 Rev D PCB

LED1	equ		1	;RC1 is the yellow LED on the PCB
LED2	equ		0	;RC0 is the green LED on the PCB




CMD_ON		equ	0x90	;on event
CMD_OFF	equ	0x91	;off event
CMD_REQ	equ	0x92	;request event

SCMD_ON	equ	0x98
SCMD_OFF	equ	0x99
SCMD_REQ	equ	0x9A
OPC_PNN	equ	0xB6

OLD_EN_NUM  equ	.32		;old number of allowed events
EN_NUM	equ	.128
EV_NUM  equ 	.13		;number of allowed EVs per event



Modstat equ 1		;address in EEPROM

MAN_NO      equ MANU_MERG    ;manufacturer number
MAJOR_VER   equ 1
MINOR_VER   equ "Y"
MODULE_ID   equ MTYP_CANPAN 		;for CANPAN  id to identify this type of module
EVT_NUM     equ EN_NUM           ; Number of events
EVperEVT    equ EV_NUM           ; Event variables per event
NV_NUM      equ 1          		; Number of node variables
NODEFLGS    equ PF_CONSUMER + PF_PRODUCER + PF_BOOT
CPU_TYPE    equ P18F25K80
BETA_VER	equ	0				;Release version





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
#define	CANTX			0x06			;port C RC6	



;set config registers. 



	CONFIG	FCMEN = OFF, FOSC = HS1, IESO = OFF, PLLCFG = ON
	CONFIG	PWRTEN = ON, BOREN = SBORDIS, BORV=0, SOSCSEL = DIG
	CONFIG	WDTEN=OFF
	CONFIG	MCLRE = ON, CANMX = PORTC
	CONFIG	BBSIZ = BB1K 
	
	CONFIG	XINST = OFF,STVREN = ON,CP0 = OFF
	CONFIG	CP1 = OFF, CPB = OFF, CPD = OFF,WRT0 = OFF,WRT1 = OFF, WRTB = OFF
	CONFIG 	WRTC = OFF,WRTD = OFF, EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF
	


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
;	define RAM storage for CANPAN
	
	CBLOCK	0		;file registers - access bank
					;interrupt stack for low priority
					;hpint uses fast stack
	W_tempL
	St_tempL
	Bsr_tempL
	PCH_tempH		;save PCH in hpint
	PCH_tempL		;save PCH in lpint
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
	Saved_Fsr0H
	Saved_Fsr0L
	Saved_Fsr1H
	Saved_Fsr1L
	CanID_tmp		;temp for CAN Node ID
	IDtemph			;used in ID shuffle
	IDtempl
	NN_temph		;node number in RAM
	NN_templ
	ENtemp1			;number of events
	Nvtemp

	
	IDcount			;used in self allocation of CAN ID.
	
	Mode			;for FLiM / SLiM etc
	Datmode			;flag for data waiting 
	Count			;counter for loading
	Count1
	Count2
	Count3
	CountFb0	; used when reading/writing fb data from/to flash
	CountFb1	;  ditto
	Lcount2		;used in Lpint
	Lcount3
	Scount		;counter for startup sequence
	Keepcnt		;keep alive counter
	Latcount	;latency counter

	Number			;used to set matrix
	Numtemp
	Roll
	Row				;LED scan row
	Rowsel			;Rolling bit for row drivers
	Colout			;LED scan column data
	Rowout			;final row value
	Temp			;temps
	Temp1
	Temp_er			;error temp
	Dlc				;data length
	
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
	initFlags
	Set_flg


	Switch1			;holds curent switches 1 to 8
	Switch2			;switches 9 to 16
	Switch3			;switches 17 to 24
	Switch4			;switches 25 to 32
	Swtemp1			;temps for switches
	Swtemp2
	Swtemp3
	Swtemp4
	Swtemp5
	Switchno		;switch number
	Swbit			;rolling bit for switch test
	Swtemp			;temp for switch scan
	Colcnt			;column count for switches
	Colsave
	Sflag			;flag byte for switches
	Scancnt			;counter for scan interval
	Svtemp			;temp for Sv. Used in set_sw.
	


	Tx1con
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
	
	Match		;match flag
	ENcount		;which EN matched
	ENcount1	;temp for count offset

	EVtemp		;holds current EV
	EVtemp1
	EVtemp2
	
	EVidx		; EV index from learn cmd
	EVdata		; EV data from cmd
	ENidx		; event index from commands which access events by index
	
	EVflags

	
	
	Matrix		;8 bytes for scan array
	Matrix1
	Matrix2
	Matrix3

	
	FlMatOn0	; LEDs which flash On/Off
	FlMatOn1
	FlMatOn2
	FlMatOn3

	
	FlMatOff0	; LEDs which flash Off/On
	FlMatOff1
	FlMatOff2
	FlMatOff3

	
	FlCtrl		; LED flash contol
	FlCount		; flash rate counter
	TkCount		; counts number of timer ticks
	T1count		; used in setup
	
	Rolle		;rolling bit for enum
	
	Fsr_tmp1Le	;temp store for FSR1
	Fsr_tmp1He 
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
	
	;add variables to suit
;data area to store data for event learning and event matching

	ev_opc
	ev0		;event number from learn command and from received event
	ev1
	ev2
	ev3
	Ss0		; storage for event used in self send
	Ss1
	Ss2
	Ss3
	Ss4



	flags
	
	setevt	; temp variable. holds next event number
	
	endc

; data area for storing event data while flash is being updated	
; Structure of each 32 bytes is as follows
; Bytes 0-3 - the event number
; Bytes 4-5 - the next event pointer
; Bytes 6-7 - the previous event pointer
; Bytes 8-11- the switch control bits
; Bytes 12-19 - the LED control bits
; Byte  20	- the LED flags byte
; Bytes 21-31 - spare (unused)

	CBLOCK 0x100		;bank 1
	; 64 bytes of event data - the quanta size for updating Flash
	; uses generic event handler
	; allows for 32 bytes total per event (24 EVs)


	evt00
	evt01
	evt02
	evt03
	next0h
	next0l
	prev0h
	prev0l
	ev00
	ev01
	ev02
	ev03
	ev04
	ev05
	ev06
	ev07
	ev08
	ev09
	ev0a
	ev0b
	ev0c
	ev0d
	ev0e
	ev0f
	ev10
	ev11
	ev12
	ev13
	ev14
	ev15
	ev16
	ev17
	; second event data block
	evt10
	evt11
	evt12
	evt13
	next1h
	next1l
	prev1h
	prev1l
	ev20
	ev21
	ev22
	ev23
	ev24
	ev25
	ev26
	ev27
	ev28
	ev29
	ev2a
	ev2b
	ev2c
	ev2d
	ev2e
	ev2f
	ev30
	ev31
	ev32
	ev33
	ev34
	ev35
	ev36
	ev37
	
	ENDC


	CBLOCK	0x180		;bank 1 upper half. Used to hold switch parameters
	
	sv0
	sv1
	sv2
	sv3
	sv4
	sv5
	sv6
	sv7
	sv8
	sv9
	sv10
	sv11
	sv12
	sv13
	sv14
	sv15
	sv16
	sv17
	sv18
	sv19
	sv20
	sv21
	sv22
	sv23
	sv24
	sv25
	sv26
	sv27
	sv28
	sv29
	sv30
	sv31

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
	nop
	incfsz EEDATA, W

	goto	RESET_VECT


	clrf	_bootSpcCmd 	; Reset the special command register
	movlw 	0x1C		; Reset the boot control bits
	movwf 	_bootCtlBits 
	movlb	d'14'		; Set Bank 14 for K series
	bcf 	TRISC, CANTX 	; Set the TX pin to output 
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
	movlb	.15
	clrf	ANCON0
	clrf	ANCON1
;	movlb	0
	movlw	CAN_CIOCON	;	Set IO
	movwf	CIOCON	
	
	clrf	CANCON	; Enter Normal mode

	bcf	TRISC,0
	bcf	TRISC,1
	bsf	LED_PORT,LED2		;green LED on
	bsf	LED_PORT,LED1		;yellow LED on


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
	clrf	EEDATA		; and clear the data (at 0x3FF for now)
	movlw 	b'00000100'	; Setup for EEData
	rcall 	_StartWrite
	bcf		LED_PORT,LED1		;yellow LED off
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
	movf	_bootAddrL, W	;   the low pointer
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
	nop
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
;		start of  program code

		ORG		0800h
loadadr
		nop						;for debug
		goto	setup

		ORG		0808h
		goto	hpint			;high priority interrupt
		
		ORG		0810h			;node type parameters
myName	db	"PAN    "

		ORG		0818h	
		goto	lpint			;low priority interrupt
		
		org		0820h

nodeprm     db  MAN_NO, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM 
			db	MAJOR_VER,NODEFLGS,CPU_TYPE,PB_CAN    ; Main parameters
            dw  RESET_VECT     ; Load address for module code above bootloader
            dw  0           ; Top 2 bytes of 32 bit address not used
			dw	0			;CPU Maufacturers ID low
			dw	0			;CPU manufacturers ID hi
			db	CPUM_MICROCHIP,BETA_VER

sparprm     fill 0,prmcnt-$ ; Unused parameter space set to zero

PRMCOUNT    equ sparprm-nodeprm ; Number of parameter bytes implemented

             ORG 0838h

prmcnt      dw  PRMCOUNT    ; Number of parameters implemented
nodenam     dw  myName      ; Pointer to module type name
            dw  0 ; Top 2 bytes of 32 bit address not used


PRCKSUM     equ MAN_NO+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+NODEFLGS+CPU_TYPE+PB_CAN+HIGH myName+LOW myName+HIGH loadadr+LOW loadadr+PRMCOUNT+CPUM_MICROCHIP+BETA_VER

cksum       dw  PRCKSUM     ; Checksum of parameters


;*******************************************************************

		ORG	0x0840
;	

;	
;		high priority interrupt. Used by  timer 3 overflow. Every 2 millisecs.
;		used for LED scan routine
;	
hpint	


		movff	FSR2L,Fsr_temp2L
		movff	FSR2H,Fsr_temp2H
		movff	FSR1L,Fsr_temp1L
		movff	FSR1H,Fsr_temp1H
		movff	FSR0L,Fsr_temp0L
		movff	FSR0H,Fsr_temp0H

		bcf		PIR2,TMR3IF		;clear all timer flags
					
		clrf	TMR3L				;reset timer 3
		
		btfss	Mode,1				; FLiM mode?
		bra		doMatrix
		decfsz	TkCount				; tick counter, wraps giving 1/2 sec count
		bra		doMatrix			; j if not expired
		bcf		FlCount,7			; clear state flag
		btfss	FlCtrl, 7			; check for On or Off control
		bra		flashOff

		; clear Off LEDs from matrix and add On Leds
		
		lfsr	FSR2, Matrix
		lfsr	FSR1, FlMatOff0
		movlw	4
		movwf	Lcount3
clrOff
		comf	POSTINC1, w
		andwf	POSTINC2
		decfsz	Lcount3
		bra		clrOff
		
		lfsr	FSR2, Matrix
		lfsr	FSR1, FlMatOn0
		movlw	4
		movwf	Lcount3
setOn
		movf	POSTINC1,w
		iorwf	POSTINC2
		decfsz	Lcount3
		bra		setOn	
		btg		FlCtrl, 7			; toggle state contol bit
		bra		doMatrix
		
flashOff
		lfsr	FSR2, Matrix
		lfsr	FSR1, FlMatOn0
		movlw	4
		movwf	Lcount3
clrOn
		comf	POSTINC1, w
		andwf	POSTINC2
		decfsz	Lcount3
		bra		clrOn
		
		lfsr	FSR2, Matrix
		lfsr	FSR1, FlMatOff0
		movlw	4
		movwf	Lcount3
setOff
		movf	POSTINC1,w
		iorwf	POSTINC2
		decfsz	Lcount3
		bra		setOff	
		btg		FlCtrl, 7			; toggle state contol bit
		
doMatrix		
		
		incf	Row,W
		andlw	B'00000011'			;count 0 to 3
		movwf	Row
		lfsr	FSR2,Matrix
		addwf	FSR2L
		
nxt_byt	bsf		PORTC,2				;row disable
		movf	SSPBUF,W	;dummy read
		movf	INDF2,W		;ready to send byte of column
		movwf	SSPBUF
nxt_by1 btfss	PIR1,SSPIF	;has it been sent
		bra		nxt_by1
		bcf		PIR1,SSPIF
		bsf		PORTC,4				;latch serial data
		bcf		PORTC,4	
		

row		rlncf	Rowsel,F		;rolling zero bit for row driver
		movf	Rowsel,W
		andlw	B'11110000'			;mask
		movwf	PORTB				;put in new row
		nop
		bcf		PORTC,2				;turn back on
		
		
hpend
		movff	Fsr_temp2L,FSR2L
		movff	Fsr_temp2H,FSR2H
		movff	Fsr_temp1L,FSR1L
		movff	Fsr_temp1H,FSR1H
		movff	Fsr_temp0L,FSR0L
		movff	Fsr_temp0H,FSR0H
		

		retfie	1	
	


	
	


;**************************************************************
;
;		low priority interrupt. Used for CAN transmit error / latency.
;
	
				ORG 0A00h

lpint	retfie


;	
;**************************************************************************************************

	

;		Note. SLiM mode only used for initial setup. This is not a SLiM module

main	btfsc	Mode,5			;is it a SoD
		bra		sod1
		btfsc	Mode,6			;is it layout setting?
		bra		set_del
		btfss	Mode,7
		bra		main0
		btfss	INTCON,TMR0IF	;65 mSec delay out?
		bra		main0
		call	set_loop		;send setup info
		movf	WREG,W
		bnz		main0			;not finished
		bcf		Mode,7			;clear flag
		bcf		Mode,5			;clear SoD if set
		bcf		T0CON,TMR0ON	;stop timer
		bcf		Datmode,0		;done

main0	btfsc	Datmode,0		;busy?
		bra		main_OK
		btfsc	COMSTAT,7		;look for CAN input. 
		bra		getcan
		bra		main_OK

set_del btfss	INTCON,TMR0IF	;initial delay of one sec.
		bra		main0
		bcf		INTCON,TMR0IF
		bcf		T0CON,TMR0ON	;stop it
		bcf		Mode,6
sod2	bsf		Mode,7			;if SoD,do as if a startup.
		bcf		Mode,5			;clear flag
		clrf	TMR0H
		clrf	TMR0L
		movlw	B'10000001'		;loop of 65 mSec
		movwf	T0CON
		
		bra		main

sod1	movlw	.32				;set up for SoD
		movwf	Scount
		clrf	Switchno
		bra		sod2	

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
		call	copyev			;save to buffer	
		bra		main_OK

no_can	bcf		RXB0CON,RXFUL

main_OK	btfsc	Mode,1			;is it SLiM?
		bra		mainf

mains	;is FLiM only
	
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
		btg		LED_PORT,LED1			;flash yellow LED
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
		btg		LED_PORT,LED1			;flash LED
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

		bcf		LED_PORT,LED1			;yellow off
		
		bsf		LED_PORT,LED2			;Green LED on
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
		bcf		LED_PORT,LED1
		bsf		LED_PORT,LED2				;green LED on
	
		movlw	B'11000000'
		movwf	INTCON 
		goto	main				;

main5	movlw	Modstat
		movwf	EEADR
		movlw	1
		call	eewrite				;mode to setup in EEPROM

		bsf		Mode,1				;to FLiM
		movlw	B'11000000'
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
		bsf		LED_PORT,LED1			;LED on

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
		bcf		LED_PORT,LED2			;green off
		bra		wait1
		
		

; 	
	
	
main1	btfss	Datmode,0		;any new CAN frame received?
		bra		no_pkt
		bra		packet			;yes
no_pkt	btfss	Datmode,3		;running mode?
		bra		main			;no
		
		movlw	B'11000000'
		movwf	INTCON			;start interrupts
	

		
do_keys	decfsz	Scancnt 		;only scan every 256 loops
		bra		main
		call	keyscan	
		tstfsz	WREG			;WREG set if a switch change while learning
		bra		is_sw
		bra		main
is_sw	btfss	Datmode,4		;in learn mode?
		bra		main			;no
		movlw	0xB3			;ARON1 response. Sends switch number to FCU
								;without risking doing anything
		movwf	Tx1d0
		movf	Switchno,W
		bcf		WREG,7			;clear pol bit if set
		movwf	Tx1d5
		incf	Tx1d5			;switch numbers start at 1
		clrf	Tx1d4
		movlw	6
		movwf	Dlc
		call	sendTX			;send event with NN

		bra		main2
	
	
;********************************************************************

;		These are here as branch was too long

unset	
		btfss	Datmode,4
		bra		main2			;prevent error message
		bsf		Datmode,5
		bra		learn2
		
readEV	btfss	Datmode,4
		bra		main2			;prevent error message
		movf	EVidx,w			;check EV index
		bz		rdev1
		movlw	EV_NUM+1
		cpfslt	EVidx

rdev1	bra		noEV1
		bsf		Datmode,6
		bra		learn2

evns1	call	thisNN				;read event numbers
		sublw	0
		bnz		notNNx
		call	evnsend
		bra		main2x
main2x	goto	main2


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
			
doQnn	movf	NN_temph,w		;respond if NN is not zero
		addwf	NN_templ,w
		btfss	STATUS,Z
		call	whoami
		bra		main2
		
reval	call	thisNN
		sublw	0
		bnz		notNNx
		movff	RXB0D3, ENidx
		movff	RXB0D4, EVidx
		call	evsend
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
		
		movlw	SCMD_ON			;short commands handled in copyev
		subwf	RXB0D0,W
		bz		go_on_x
		movlw	SCMD_OFF
		subwf	RXB0D0,W
		bz		go_on_x
		movlw	SCMD_REQ
		subwf	RXB0D0,W
		bz		go_on_x

		
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
		movlw	0x0D			; QNN
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
		movlw	0x9C
		subwf	RXB0D0,W
		bz		reval1
		movlw	0x95			;is it unset event
		subwf	RXB0D0,W			
		bz		unset
		movlw	0xB2			;read event variables
		subwf	RXB0D0,W
		bz		readEV
		movlw	0x96			;set NV
		subwf	ev_opc,W
		bz		setNVx
		movlw	0x71			;read NVs
		subwf	ev_opc,W
		bz		readNVx
	
		movlw	0x57			;is it read events
		subwf	RXB0D0,W
		bz		readEN1

		movlw	0x58
		subwf	RXB0D0,W
		bz		evns

		bra		main2

setNNx	goto	setNN
rden_x	goto	rden
chk1	goto	chklrn
idxlrn1	goto	main2
reval1	goto	reval
evns	goto	evns1
clren1	goto	clrens
setNVx	goto 	setNV
readNVx	goto	readNV
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
		


readEN1 goto	readEN



reboot	btfss	Mode,1			;FLiM?
		bra		reboots
		call	thisNN
		sublw	0
		bnz		notNN1
		
reboot1	movlw	0xFF
		movwf	EEADR
		movlw	0x3F
		movwf	EEADRH
		movlw	0xFF
		call	eewrite			;set last EEPROM byte to 0xFF
		reset					;software reset to bootloader

reboots
		movf	RXB0D1,w
		addwf	RXB0D2,w
		bnz		notNN1
		bra		reboot1	
	
para1a	btfss	Mode, 1
		bra		para1s
		call	thisNN			;read parameter by index
		sublw	0
		bnz		notNN1
		call	para1rd
		bra		main2
		
para1s
		movf	RXB0D1,w
		addwf	RXB0D2,w
		bnz		notNN1
		call	para1rd
		bra		main2

readNV	call	thisNN
		sublw	0
		bnz		notNN1			;not this node
		call	readNVs
		bra		main2

setNV	call	thisNN
		sublw	0
		bnz		notNN1			;not this node
		call	putNV
		bra		main2

notlrn	call	thisNN
		sublw	0
		bnz		notNN1
		bcf		Datmode,4
		
notln1									;leave in learn mode
		bcf		Datmode,5
	
		bra		main2

setlrn	call	thisNN
		sublw	0
		bnz		notNN
		bsf		Datmode,4
;		bsf		LED_PORT,LED2			;LED on
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
startNN	bsf		LED_PORT,LED1	;LED ON
		bcf		LED_PORT,LED2
		bra		main2

new_ID	call	thisNN
		sublw	0
		bnz		notNN
		movff	RXB0D3,IDcount
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
		call	clr_sub
		
		movlw	0x59
		call	nnrel		;send WRACK
		bra		notln1
notNN	bra		main2

clrerr	movlw	2			;not in learn mode
		goto	errmsg

chklrn	btfss	Datmode,4		;is in learn mode?
		bra		main2			;jump if not
		movf	EVidx,w			;check EV index
		bz		noEV1
		movlw	EV_NUM+1
		cpfslt	EVidx
		bra		noEV1

		bra		learn2
	
			
sw_lrn	call	putsw_ev		;put switch event into FLASH

		movlw	0x59			; send WRACK
		call	nnrel
		bra		main2			;
				
		
noEV1	movlw	6
		goto	errmsg
		
		
go_on	btfss	Mode,1			;FLiM?
		bra		main2
		
go_on1	
		call	do_it
		bra		main2			;not here


		

paraerr	movlw	3				;error not in setup mode
		goto	errmsg

readEN	call	thisNN
		sublw	0
		bnz		notNN
		call	enread
		bra		main2

		

		


rden1	call	thisNN
		sublw	0
		bnz		notNN2
		call	rdFreeSp
		bra		main2

notNN2	goto	notNN			;branch too long

sw_lrna	movlw	1
		call	errsub
		goto	main2			;don't do if a switch	

learn2	btfss	Mode,1			;FLiM?
		goto	main2
		
		call	enmatch			;is it there already?
		sublw 	0
		bz		isthere

learn3	btfsc	Datmode,6		;read EV?
		bra		rdbak1			;not here
		btfsc	Datmode,5		;if unset and not here
		bra		l_out1			;do nothing else 
		
	
learn5	decf	EVidx,F			;base 0
		call	learn			;put EN into flash
		sublw	0
		bz		lrnend
		
		movlw	4
		goto	errmsg2	
		
rdbak1	movlw	5				;no match
		goto	errmsg2	
		bra		l_out1	

isthere	
		btfsc	Datmode, 6		;is it read back
		bra		rdbak					
		btfss	Datmode,5		;FLiM unlearn?
		bra		do_learn
		call	rdfbev			;get EVs
		movf	INDF0,F			;FSR0 points to start of EVs in RAM
		bz		uln1			;is it switch event?
		call	unset_sw		;put switch info back to default
uln1	call	unlearn
		movlw	0x59
		call	nnrel
		bra		l_out1
		
		
do_learn
		movf	EVidx,W			; indexes start at 1
		bnz		do_lrn1
		bra		rdeverr			;error
do_lrn1
		decf	EVidx,F			;base 0
		call	learn
		sublw	0				; check if no space left
		bz		do_lrn2
		movlw	4
		goto	errmsg2	
		
do_lrn2	call	enmatch
		call	rdfbev
		movf	INDF0,F			;FSR0 points to start of EVs in RAM
		bz		lrnend			;not a switch event
		movlw	2
		subwf	EVidx,W			;is it SV event?
		bnz		lrnend			;no
		call	set_sw			;sort out switch event
;		
	
				
lrnend	;call	do_it
		bra		l_out2
		
do_unlearn
		call	unlearn
		movlw	0x59			; send WRACK
		call	nnrel
;		call	do_it
		bra		main2
		
do_rdev
		tstfsz	EVidx
		bra		do_rdev1
rdeverr

		movlw	6
		call	errsub
		goto	main2
		
do_rdev1

		movlw	EV_NUM+1
		cpfslt	EVidx
		bra		rdeverr
		call	readev
		bra		main2
		
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

setup	movlb	.15
	
		clrf	ANCON0			;disable A/D
		clrf	ANCON1
		clrf	CM1CON			;disable comparator
		clrf	CM2CON
		clrf	INTCON2			
		bsf		INTCON2,7		;weak pullups off
		clrf	WPUB			;pullups clear
		movlb	0	

	;port settings will be hardware dependent. RC6 and RC7 are for CAN.
		;set S_PORT and S_BIT to correspond to port used for setup.
		;rest are hardware options

		setf	LATC			;outputs high at start
		movlw	B'10000000'		;Port C  set to outputs except RC7 which is CANRX.
								;RC0 drived LD1, RC1 drives LD2
								;RC2 is U5 output enable
								;RC3 is SPI clock
								;RC4 U5 latch enable
								;RC5 is Serial data to U5
								;RC6 is CANTX
		movwf	TRISC
		lfsr	FSR0, 0			; clear page 1
		
nextram	clrf	POSTINC0
		tstfsz	FSR0L
		bra		nextram	
		
		clrf	INTCON			;no interrupts yet
		
	
		
	
		
	
		movlw	B'00001000'		;Port A   PA3 is setup PB
		movwf	TRISA			;RA0,1 and 2 are switch column select
								;RA5 not used	


		movlw	B'11110000'
		movwf	LATB			;set LED row drivers off
		movlw	B'00001111'		;RB0,1,2,3  are switch row inputs
								;RB4,5 LED row drivers
								;RB6,7 for debug and ICSP and LED row drivers
								;PORTB has pullups disabled on inputs
								;
		movwf	TRISB
		bcf		LED_PORT,LED2
		bcf		LED_PORT,LED1
		bsf		PORTC,6			;CAN recessive

;	set SPI

		movlw	B'00101010'		;master SPI
		movwf	SSPCON1
		bsf		SSPSTAT,CKE
		clrf	Row				;sync counter for LEDs
		movlw	0xEE
		movwf	Rowsel
	

	
		
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

		movlb	.14
		clrf	BSEL0			;8 frame FIFO
		clrf	RXB0CON
		clrf	RXB1CON
		clrf	B0CON
		clrf	B1CON
		clrf	B2CON
		clrf	B3CON
		clrf	B4CON
		clrf	B5CON
		
			
		movlw	B'00000011'		;set CAN bit rate at 125000 for now
		movwf	BRGCON1
		movlw	B'10011110'		;set phase 1 etc
		movwf	BRGCON2
		movlw	B'00000011'		;set phase 2 etc
		movwf	BRGCON3
		movlb	0

		movlw	B'00100000'
		movwf	CIOCON			;CAN to high when off


		
mskload	lfsr	FSR0,RXM0SIDH		;Clear masks, point to start
mskloop	clrf	POSTINC0		

		cpfseq	FSR0L			;0xEFF is last mask address
		bra		mskloop
		
		clrf	CANCON			;out of CAN setup mode
		clrf	CCP1CON

		movlw	B'00000000'
		movwf	IPR5			;low priority CAN RX and Tx error interrupts(for now)
		clrf	IPR1			;all peripheral interrupts are low priority
		clrf	IPR2
		clrf	PIE2
			
		movlw	B'00110001'		;Timer 1 set Timer 1 for LED flash
		movwf	T1CON
		movlw	0x00
		movwf	TMR1H			;Timer 1 is a 16 bit timer
	
		bsf		PIE2,TMR3IE 	;high priority for T3 interrupt
		bsf		IPR2,TMR3IP
		
		movlw	B'00100011'		;Timer 3 is LED mux, start it
		movwf	T3CON
		movlw	0xF8
		movwf	TMR3H
		clrf	TMR3L			;2.048 mSec period
		clrf	Tx1con
	

;		for switch scan
		setf	Switch1
		setf	Switch2
		setf	Switch3
		setf	Switch4
		clrf	Colcnt
		clrf	Sflag
		movff	Colcnt,LATA		;set first scan column
		clrf	Scancnt
	


;next segment required
		
		movlw	B'00000001'
		movwf	IDcount			;set at lowest value for starters
		
	
		clrf	INTCON3			;
		clrf	T3GCON
			
		clrf	PIR1
;		clrf	PIR2
		
		
		bcf		RXB0CON,RXFUL		;ready for next
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL
		clrf	PIR5			;clear all ECAN flags

		clrf	EEADRH			;upper EEPROM page to 0
		
;		setf	EVflags			;no flash till taught
		
		
		
		;		test for setup mode


		clrf	Mode
		movlw	Modstat			;get setup status
		movwf	EEADR
		call	eeread
		movwf	Datmode
		sublw	8
		bz		setid	
		movlw	0
		movwf	Datmode				;not set yet
		call	eewrite
		bra		slimset			;wait for setup PB
	
		
setid	call	chkevdata		;set up flash ram if not already done
		call	readsw			;switch parameters to RAM
		bsf		Mode,1			;flag FLiM
		clrf	EEADR
		call	newid_f			;put ID into Tx1buf, TXB2 and ID number store
		btfsc	Datmode,2
		bra		seten_f
		bsf		T3CON,TMR3ON	;start scan timer
		movlw	1				; set up flash control
		movwf	FlCtrl
		movwf	FlCount
	


		
seten_f	
		bcf		LED_PORT,LED2
		btfss	Datmode,2		;flashing?
		
		bsf		LED_PORT,LED1			;Yellow LED on.
		bcf		RXB0CON,RXFUL
		bcf		Datmode,0
		
		movlw	LOW	NVstart			;layout setup routine 
		movwf	EEADR
		call	eeread				;get NV
		movwf	Nvtemp				;save it
		
	

;		call	set_lay				;configure layout and LEDs to last saved state	

		movf	Nvtemp,W
		bz		lay0		;set to last (default)
		btfsc	WREG,0		;is it do nothing?  NV1 = 1
		bra		set_end
		btfss	WREG,1		;is it all ON?  NV1 = 2
		bra		set_end		;invalid NV
	
lay_on 	movlw	.32			;sets all taught events to on.
		movwf	Scount
		clrf	Switchno
on_loop	movlw	LOW swstate		;get saved SVs
		addwf	Switchno,W
		movwf	EEADR
		call	eeread
		btfss	WREG,7			;is it a taught event?
		bra		nxt_on
		bcf		WREG,6			;set to on.
		call	eewrite
;		call	dely
nxt_on	incf	Switchno	;next switch
		decfsz	Scount
		bra		on_loop
		bra		setlay		;flag for layout setup

;events are the last saved

lay0	movlw	.32		;32 possible switch events
		movwf	Scount
		clrf	Switchno	;start at first switch

setlay	bsf		Mode,6		;use as layout setup flag for initial delay
		clrf	TMR0H
		clrf	TMR0L		;set for one second
		movlw	B'10000101'
		movwf	T0CON		;start it

;		call	ldely		;allow nodes to settle

set_end		goto	main

slimset	bcf		Mode,1
		clrf	NN_temph
		clrf	NN_templ
		call	clr_sub				;set defaults for events and SVs
		setf	EVflags				;no flash till taught
		bcf		LED_PORT,LED1
		bsf		LED_PORT,LED2			;RUN LED on. Green for SLiM
		goto	main



;***************************************************************************		


#include "newevhndler_v1b.asm"

		

		
;****************************************************************************
;		start of subroutines


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

newid_f	
		call	eeread
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
eeread	bcf		EECON1,EEPGD	;read a EEPROM byte,    must be set before this sub.
		bcf		EECON1,CFGS		;returns with data in W
		bsf		EECON1,RD
		nop
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
		movff	TempINTCON,INTCON		;reenable interrupts
eetest	btfsc	EECON1,WR
		bra		eetest
		bcf		PIR2,EEIF
		bcf		EECON1,WREN
;		movff	TempINTCON,INTCON		;reenable interrupts
		
		return	
		

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

		; copy event data to safe buffer

copyev	movff	RXB0D0, ev_opc
		movff	RXB0D1, ev0
		movff	RXB0D2, ev1
		movff	RXB0D3, ev2
		movff	RXB0D4, ev3
		movff	RXB0D5, EVidx      	; only used by learn and some read cmds
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

short	clrf	ev0
		clrf	ev1
		return

		
;****************************************************************

;		longer delay

ldely	movlw	.100
		movwf	Count2
ldely1	call	dely
		decfsz	Count2
		bra		ldely1
		
		return

;******************************************************************

;		set switch info in FLASH, EEPROM and RAM

set_sw	movff	INDF0,Svtemp	;save EV1 for SoD
		movf	PREINC0,W		;Get switch number
		decf	WREG			;switch numbers start at 1 so decrement
		bcf		WREG,7			;pol bit
		movwf	Switchno
		
		movlw	LOW swstate		;put in EEPROM
		movwf	EEADR
		movf	Switchno,W
		addwf	EEADR

		movf	PREINC0,W		;get SV
		btfss	Svtemp,1		;is it a SoD? Don't set as learned.
		bsf		WREG,7			;flag for taught switch event
		call	eewrite
		call	readsw			;put back in RAM
		
		call	putsw_ev		;put event in FLASH

		btfss	Svtemp,1		;is it a SoD switch?
		return
								;need to know if a SoD switch
		movlw	LOW sod_sw		;put SoD switch in EEPROM
		movwf	EEADR
		movf	Switchno,W
		bsf		WREG,7			;flag valid SoD switch
		call	eewrite

		

		return
;***********************************************************************

;	unset switch info in FLASH, EEPROM and RAM. restore to default

unset_sw	movf	PREINC0,W		;Get switch number
		decf	WREG			;switch numbers start at 1 so decrement
		bcf		WREG,7			;pol bit
		movwf	Switchno
		movlw	LOW swstate		;put in EEPROM
		movwf	EEADR
		movf	Switchno,W
		addwf	EEADR

		movlw	1				;get default SV
		call	eewrite
		call	readsw			;put back in RAM
		call	defsw_ev		;put in FLASH

		movlw	LOW sod_sw		;release if SoD switch
		movwf	EEADR
		call	eeread
		btfss	WREG,7			;a valid SoD?
		return
		bcf		WREG,7
		subwf	Switchno,W		;is this a SoD switch?
		bz		unsetSoD
		return
unsetSoD clrf WREG
		call	eewrite			;back to zero
		return

		

;**********************************************************************

;			read switch variables to RAM
;
readsw		lfsr	FSR0,sv0
			movlw	.32
			movwf	Count
			movlw	LOW swstate
			movwf	EEADR

readsw1		call	eeread
			movwf	POSTINC0
			incf	EEADR
			decfsz	Count
			bra		readsw1
			
			return
			
;*********************************************************


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
;
;		main key scan routine

keyscan	movlw	B'11000000'		;don't scan while sending layout setup info
		andwf	Mode,W
		bz		key1
		retlw	0
key1	btfsc	Sflag,1			;has changed so do debounce
		bra		debnce
		btfsc	Colcnt,0		;is it first of a pair?
		bra		second
		movf	PORTB,W			;get rows
		comf	WREG			;for 74HC238
		andlw	B'00001111'		;mask
		movwf	Swtemp			;hold value
		incf	Colcnt			;next
		movff	Colcnt,LATA	
		retlw	0

second	movf	PORTB,W			;get row
		comf	WREG
		andlw	B'00001111'
		swapf	WREG			;high 4 bits
		iorwf	Swtemp			;now has all 8 bits
		lfsr	FSR1,Switch1	;load FSRs
		lfsr	FSR2,Swtemp1
		rrncf	Colcnt,W		;get which of 4
		andlw	B'00000011'
		addwf	FSR1L			;offset
		addwf	FSR2L
		movf	Swtemp,W		;get switch byte
		xorwf	INDF1,W			;has it changed?
		bz		sec1			;no
		bsf		Sflag,0			;set for debounce
sec1	movff	Swtemp,INDF2	;save current input
		incf	Colcnt
		movff	Colcnt,LATA		;next column
		movlw	B'00000111'
		andwf	Colcnt,W		;end of scan?
		bnz		sec2
		btfsc	Sflag,0			;any change?
		bsf		Sflag,1			;do debounce
sec2	retlw	0

debnce	bcf		Sflag,0			;clear match
		btfsc	Colcnt,0		;is it first of a pair?
		bra		sec_b
		movf	PORTB,W			;get rows
		comf	WREG
		andlw	B'00001111'		;mask
		movwf	Swtemp			;hold value
		incf	Colcnt			;next
		movff	Colcnt,LATA	
		retlw	0

sec_b	movf	PORTB,W			;get row
		comf	WREG
		andlw	B'00001111'
		swapf	WREG			;high 4 bits
		iorwf	Swtemp			;now has all 8 bits
		lfsr	FSR2,Swtemp1
		rrncf	Colcnt,W		;get which of 4
		andlw	B'00000011'
		addwf	FSR2L
		movf	Swtemp,W		;get switch byte
		xorwf	INDF2,W			;has it changed again?
		bz		sec1_b			;no
		bsf		Sflag,0			;set for change
sec1_b	
		incf	Colcnt
		movff	Colcnt,LATA		;next column
		movlw	B'00000111'
		andwf	Colcnt,W		;end of scan?
		bz		swchng
sec2_b	retlw	0

swchng	btfss	Sflag,0			;change so is a bounce
		bra		deb1
		bcf		Sflag,1			;clear debounce flag
		bcf		Sflag,0			;clear change flag
		retlw	0				;scan again

;	now sort out which has changed

deb1	bcf		Sflag,1			;clear debounce
		lfsr	FSR1,Switch1	;here on valid change
		lfsr	FSR2,Swtemp1
		clrf	Switchno		;switch number counter
bits_1	movlw	B'00000001'		;set rolling bit
		movwf	Swbit
		movlw	8
		movwf	Count			;8 bits per byte
		movf	INDF1,W
		xorwf	INDF2,W			;has this byte changed?
		bz		nxt_sw			;no
		movwf	Swtemp			;save bit that has changed
bit_tst	movf	Swbit,W			;get rolling bit
		andwf	Swtemp,W		;this switch?
		bnz		this_sw			;yes
		rlncf	Swbit,F			;no, so roll one
		incf	Switchno,F
		decfsz	Count			;last bit?
		bra		bit_tst
nxt_sw	incf	FSR1L
		incf	FSR2L
		movf	FSR2L,W			;last of set?
		sublw	Swtemp+4
		bz		sw_done			;finished
		movlw	8
		addwf	Switchno
		bra		bits_1			;next comparison
sw_done	retlw	0					;should never get here

this_sw	movff	INDF2,INDF1		;new pattern to old
		btfss	Datmode,4		;is it switch learn?
		bra		chng2			;no
		retlw	1				;return for switch learn
chng2	andwf	INDF2,W			;which polarity?
		bz		sw_off
		bsf		Switchno,7		;bit 7 used as pol flag
		bra		chngsw			;do switch change
sw_off	bcf		Switchno,7
chngsw	lfsr	FSR1,sv0		;look up switch variable
		movf	Switchno,W		;get switch number
		bcf		WREG,7			;clear POL bit if set
		addwf	FSR1L
		movf	INDF1,W			;get variable
		bnz		chng1	
		retlw	0				;abort
chng1	movwf	Swtemp			;save sw variable
		btfsc	WREG,0			;is it on / off
		bra		on_off
		btfsc	WREG,2			;is it ON only?
		bra		on_only
		btfsc	WREG,3			;is it toggle?
		bra		tog_sw
		retlw	0				;must be one of these
on_off	btfsc	Swtemp,5		;is it a short event
		bra		on_off1
		movlw	0x90
		movwf	Tx1d0			;set OPC
		bra		on_off2
on_off1	movlw	0x98
		movwf	Tx1d0
on_off2	btfsc	Switchno,7		;on or off
		bsf		Tx1d0,0
		btfsc	Swtemp,1
		btg		Tx1d0,0			;reverse polarity if pol bit set in sv.
		bra		snd_sw

on_only	btfsc	Switchno,7		;ignore an off
		retlw	0
		btfsc	Swtemp,5
		bra		only1
		movlw	0x90
		movwf	Tx1d0
		bra		only2
only1	movlw	0x98
		movwf	Tx1d0
only2	btfsc	Swtemp,1		;pol bit
		bsf		Tx1d0,0
		bra		snd_sw

tog_sw	btfsc	Switchno,7		;ignore off state
		retlw	0
		lfsr	FSR1,sv0		;ready for bit change
		btfsc	Swtemp,6		;old toggle bit
		bra		tog_on
		movf	Switchno,W
		addwf	FSR1L
		bsf		INDF1,6			;set toggle bit	
		bsf		Switchno,7		;set for off
		bra		on_off			;send OFF
tog_on  movf	Switchno,W
		addwf	FSR1L
		bcf		INDF1,6			;clear toggle
		bra		on_off			;send ON
		


snd_sw	call	load_sw			;get event for that switch into Tx buffer
		movff	Tx1d0,Ss0		;put OPC for self send
		movlw	5
		movwf	Dlc
		btfsc	Swtemp,5		;is	it short?
		bra		snd_sw1
		movf	Tx1d1,F			;check for initial switch long events
		bnz		snd_sw3			;if NN = 00 00, then add current NN
		movf	Tx1d2,F			;else leave alone
		bnz		snd_sw3
		bra		snd_sw1			;add NN
snd_sw3	call	sendTXa			;Don't change N bytes
		bra		snd_sw2
snd_sw1	call	sendTX			;Add NN bytes
snd_sw2	bcf		Swtemp,6		;clear polarity bit
		btfsc	Tx1d0,0			;what polarity was sent?
		bsf		Swtemp,6		;was off so set bit 6
		movlw	LOW swstate		;now save in EEPROM
		movwf	EEADR
		movf	Switchno,W		;get switch number
		bcf		WREG,7
		addwf	EEADR
		movf	Swtemp,W		;save new SV
		call	eewrite
		movlw	LOW sod_sw		;is this a SoD switch?
		movwf	EEADR
		call	eeread
		movwf	Svtemp			;save SoD value
		btfss	Svtemp,7		;a valid SoD?
		bra		snd_sw4
		bcf		Svtemp,7
		movf	Switchno,W
		bcf		Switchno,7		;pol bit
		movf	Switchno,W		;get switch
		subwf	Svtemp,W		;is it the sams as the SoD number?	
		bnz		snd_sw4
		bsf		Mode,5			;for self SoD

snd_sw4	btfsc	Swtemp,4		;send to self?
		call	self_snd		
					
		retlw	0

load_sw movlw	0x29			;get switch event to send
		movwf	TBLPTRH
		clrf	TBLPTRL
		movf	Switchno,W
		bcf		WREG,7			;mask POL bit
		rlncf	WREG			;four bytes per entry
		rlncf	WREG
		addwf	TBLPTRL
		tblrd*+
		movf	TABLAT,W
		movwf	Tx1d1
		movwf	Ss1
		tblrd*+
		movf	TABLAT,W
		movwf	Tx1d2
		movwf	Ss2
		tblrd*+
		movf	TABLAT,W
		movwf	Tx1d3
		movwf	Ss3
		tblrd*+
		movf	TABLAT,W
		movwf	Tx1d4
		movwf	Ss4
		retlw	0

self_snd lfsr	FSR1,Ss0				;move switch event to incoming RAM buffer
		lfsr	FSR2,ev_opc
		movlw	5
		movwf	Count
ss_1	movff	POSTINC1,POSTINC2
		decfsz	Count
		bra		ss_1
		btfsc	Swtemp,3				;is it toggle?
		bra		is_tog	
ss_3	call	do_it					;do it is if it were an event
		retlw	0

is_tog	bra		ss_3



;***********************************************************
;
;		put switch event into flash

putsw_ev	movlw	0x29
		movwf	evaddrh	
		movf	Switchno,W
		bcf		WREG,7			;clear pol bit
		sublw	.15			;which bank is switch in?
		
		bn		bank2		
		clrf	evaddrl
		call	rdfbev			;read first block
		lfsr	FSR0,evt00
		lfsr	FSR1,ev0
		movf	Switchno,W
		rlncf	WREG
		rlncf	WREG
		addwf	FSR0L			;4 bytes per event
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0
		clrf	evaddrl	
		call	wrfbev
		return	

bank2	movlw	.64
		movwf	evaddrl
		call	rdfbev			;read second block
		lfsr	FSR0,evt00
		lfsr	FSR1,ev0
		movlw	0x10			;reset switch no to base 0
		subwf	Switchno,W
		
		rlncf	WREG
		rlncf	WREG
		addwf	FSR0L			;4 bytes per event
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0
		movff	POSTINC1,POSTINC0	
		movlw	.64
		movwf	evaddrl
		call	wrfbev
		return	

;*********************************************************************

;		put default switch event into flash

defsw_ev	movlw	0x29
		movwf	evaddrh	
		movf	Switchno,W
		bcf		WREG,7			;clear pol bit
		sublw	.15			;which bank is switch in?
		
		bn		bank2a		
		clrf	evaddrl
		call	rdfbev			;read first block
		lfsr	FSR0,evt00
		movf	Switchno,W
		rlncf	WREG
		rlncf	WREG
		addwf	FSR0L			;4 bytes per event
		clrf	POSTINC0
		clrf	POSTINC0
		clrf	POSTINC0
		incf	Switchno,W
		movwf	POSTINC0
		clrf	evaddrl	
		call	wrfbev
		return	

bank2a	movlw	.64
		movwf	evaddrl
		call	rdfbev			;read second block
		lfsr	FSR0,evt00
		movlw	0x10			;reset switch no to base 0
		subwf	 Switchno,W
		
		rlncf	WREG
		rlncf	WREG
		addwf	FSR0L			;4 bytes per event
		clrf	POSTINC0
		clrf	POSTINC0
		clrf	POSTINC0
		incf	Switchno,W
		movwf	POSTINC0
		
		movlw	.64
		movwf	evaddrl
		call	wrfbev
		return	
;********************************************************************
;		do_it as a subroutine
;
do_it	movff	FSR1H,Saved_Fsr1H
		movff	FSR1L,Saved_Fsr1L
		call	enmatch
		sublw	0
		bz		do_it1
		return

sod		bsf		Mode,5		;flag for a SoD
		return

do_rsp1	goto	do_rsp		;branch too long

do_it1	call	rdfbev		; read relevant event data
		movff	FSR0L,Saved_Fsr0L ;save pointer
		movff	FSR0H,Saved_Fsr0H
		movff	FSR0H,FSR1H	;point to correct set of 32
		movlw	0x92		;is it a request?
		subwf	ev_opc,W
		bz		do_rsp1		;do a response
		movlw	0x9A
		subwf	ev_opc,W
		bz		do_rsp1
		btfsc	INDF0,1		;for  SoD
		
		bra		sod
	
		
		
do_it2	movlw	.12				;LED EVflags now at 12
		addwf	FSR0L
		movff	INDF0, EVflags
		btfss	EVflags,2	; test flash control bit
		bra		setupfl
		movlw	8
		subwf	FSR0L		;point to start of LED Evs
		movff	FSR0L,FSR1L 
		movlw	4
		addwf	FSR1L		;?
		lfsr	FSR2,Matrix
		movlw	4
		movwf	Count
		btfsc	ev_opc,0		; is it ON cmnd
		bra		offcmnd		; j if OFF
		btfss	EVflags, 1
		bra		switch		; j if On event not actioned

	
		
nxtled
		movff	POSTINC1, Temp1	; POL bits
		comf	Temp1,F			; invert
		movf	Temp1,W
		andwf	INDF2			; turn off POL bits in matrix
		btfss	EVflags,2		; flash off
		bra		nxtld1
		movff	INDF0,Temp		;get LED bits
		comf	Temp,F
		movlw	4				;if not flash, clear flash matrix bits
		addwf	FSR2L
		movf	Temp,W
		andwf	INDF2
		movlw	4
		addwf	FSR2L
		movf	Temp,W
		andwf	INDF2
		movlw	8
		subwf	FSR2L
nxtld1	movf	Temp1,W
		andwf	POSTINC0,W		; remove POL bits from data
		iorwf	POSTINC2		; and 'or' into matrix
		decfsz	Count
		bra		nxtled

switch	btfsc	Datmode,0		;is it an incoming event?
		bra		sw_in			;yes
		movff	Saved_Fsr0L,FSR0L
		movlw	2
		addwf	FSR0L			;get switch control byte
		btfss	INDF0,3			;not a toggle event?
		return
		decf	FSR0L			;
		movf	INDF0,W			;get switch number
		lfsr	FSR1,sv0		;switch status table
		addwf	FSR1L			;point to switch
		decf	FSR1L			;switchno to base 0
		btfsc	ev_opc,0		;on or off
		bra		sw_off1
		btfss	INDF1,6
		bra		sw_ret			;do nothing
		bcf		INDF1,6
		bra		sw_ret
sw_off1	btfsc  	INDF1,6
		bra		sw_ret
		bsf		INDF1,6	
sw_ret	
		movff	Saved_Fsr1H,FSR1H
		movff	Saved_Fsr1L,FSR1L	
		return	

 

sw_in	movff	Saved_Fsr0L,FSR0L
		incf	FSR0L,F			;get switch number
		movf	INDF0,W			;get switch number
		movwf	Switchno
		lfsr	FSR1,sv0		;switch status table
		addwf	FSR1L			;point to switch
		decf	FSR1L			;switchno to base 0
		btfsc	ev_opc,0		;on or off
		bra		sw_off2
		bcf		INDF1,6			;set on/off bit
		bra		sw_ret1			
		
sw_off2	bsf  	INDF1,6
		bra		sw_ret1

sw_ret1 
		movlw	LOW swstate		;now save in EEPROM
		movwf	EEADR
		movf	Switchno,W		;get switch number
		decf	WREG			;base 0
		bcf		WREG,7
		addwf	EEADR
		movf	INDF1,W			;get new state
		
		call	eewrite

		bcf		Datmode,0		;done incoming event
		bra		sw_ret
			
		
offcmnd	btfss	EVflags, 0
		bra		switch			; j if Off event not actioned
offcmnd1
		movff	POSTINC0, Temp	; leds which may be ON
		comf	Temp,F			; invert
		movf	Temp,W
		andwf	INDF2			; turn them off in matrix
		btfss	EVflags,2		; flash off
		bra		offcmd1
		movlw	4
		addwf	FSR2L
		movf	Temp,W
		andwf	INDF2
		movlw	4
		addwf	FSR2L
		movf	Temp,W
		andwf	INDF2
		movlw	8
		subwf	FSR2L
offcmd1		movf	POSTINC1,w		; get POL bits
		iorwf	POSTINC2		; or into matrix
		decfsz	Count
		bra		offcmnd1
		bra		switch
		
setupfl	
		movff	Saved_Fsr0L,FSR0L  		; event LEDs
		movlw	4
		addwf	FSR0L		; point at LED EVs
		lfsr	FSR1, FlMatOn0	; On flash matrix
		movlw	4
		movwf	Count
		btfsc	ev_opc,0		; is it ON cmnd
		bra		unsetfl	
			
		movff	Saved_Fsr0L,FSR2L
		movff	Saved_Fsr0H,FSR2H 
		movlw	8
		addwf	FSR2L
nxtonfl
		comf	POSTINC2,w	; comp of pol bits
		andwf	POSTINC0,w	; remove pol bits from LEDs
		iorwf	POSTINC1	; or On LEDS into On matrix
		decfsz	Count
		bra		nxtonfl
		
		movff	Saved_Fsr0L,FSR0L  		; event LEDs
		movlw	8
		addwf	FSR0L					; pol bits
		lfsr	FSR1, FlMatOff0
		movlw	4
		movwf	Count
nxtoffl
		movf	POSTINC0,W	; get pol bits
		iorwf	POSTINC1	; or into Off matrix
		decfsz	Count
		bra		nxtoffl
		bra 	switch
		
unsetfl
		lfsr	FSR2, FlMatOff0	;Off flash matrix
nxtunset
		comf	POSTINC0,w		; comp of all LEDs
		andwf	POSTINC1		; remove LEDs from On matrix
		andwf	POSTINC2		; remove LEDs from Off matrix
		decfsz	Count
		bra		nxtunset
		
		; now remove all LEDs from main matrix
		movff	Saved_Fsr0L,FSR0L
		movlw	4
		addwf	FSR0L
		lfsr	FSR1, Matrix
		movlw	4
		movwf	Count
unsetmx
		comf 	POSTINC0, W	; comp of event LEDs
		andwf	POSTINC1	; remove from matrix
		decfsz	Count
		bra		unsetmx
		
		bra		switch

do_rsp	movff	Saved_Fsr0L,FSR1L
		btfss	INDF1,0		;is it a switch event?
		return				;no
		
resp1	movlw 	1
		movf	PLUSW1,W		;get switch number
		decf	WREG
		movwf	Swtemp5		;base 0
	
read	movlw	LOW swstate		;get switch state
		addwf	Swtemp5,W		;add switch number
		movwf	EEADR
		call	eeread
		movwf	Swtemp5		;save SV
		movlw	0x92		;is it a long?
		subwf	ev_opc,W
		bnz		shrt_on
resp3	movlw	0x93
		movwf	Tx1d0
		btfsc	Swtemp5,6	;on or off?
		bra		off
snd_lng		movff	ev0,Tx1d1
		movff	ev1,Tx1d2
		movff	ev2,Tx1d3
		movff	ev3,Tx1d4
		movlw	5
		movwf	Dlc
		call	sendTXa
		bra		sw_ret

shrt_on	movlw	0x9A
		subwf	ev_opc,W
		bnz		sw_retz		;not a short request
		movlw	0x9D
		movwf	Tx1d0
		btfsc	Swtemp5,6
		bra		shrt_of
snd_sh	movff	ev2,Tx1d3
		movff	ev3,Tx1d4
		movlw	5
		movwf	Dlc
		call	sendTX
	
		bra		sw_retz

off		movlw	0x92		;is it a long?
		subwf	ev_opc,W
		bnz		shrt_of
		movlw	0x94		;off
		movwf	Tx1d0
		bra		snd_lng		;send it

shrt_of	movlw	0x9E
		movwf	Tx1d0
		bra		snd_sh

sw_retz	goto	sw_ret		;too long branch

;******************************************************************

;		routine to set layout to last state on startup
;		called if NV1 = 0 (default)or on SoD



set_loop bcf	INTCON,TMR0IF
		clrf	TMR0L			;reset timer
set_nxt movlw	LOW swstate		;get saved SVs
		addwf	Switchno,W
		movwf	EEADR
		call	eeread
		btfss	WREG,7			;is it a taught event?
		bra		nxt_lay
		movwf	Swtemp
		movlw	0x90		;set OPC
		movwf	Tx1d0
		btfsc	Swtemp,5	;is it a short?
		bsf		Tx1d0,3		;yes
		btfsc	Swtemp,6	;is it on or off?
		bsf		Tx1d0,0		;off
		call	snd_sw		;send event

		incf	Switchno	;next switch
		decf	Scount,F
		movf	Scount,W
		return
nxt_lay incf	Switchno
		decf	Scount,F
		movf	Scount,W
		bnz		set_nxt
		return

;**********************************************************************	
;		restores FLASH and EEPROM to default state. Includes SVs		

clr_sub	call	initevdata	;clear event FLASH
		movlw	.32
		movwf	Count
		movlw	LOW swstate	;reset EEPROM
		movwf	EEADR
clr1	movlw	1			;default SV state
		movwf	EEDATA
		call	eewrite
		incf	EEADR
		decfsz	Count
		bra		clr1
		call	readsw		;restore RAM table
		lfsr	FSR0,evt00	;set default event table
		movlw	.64
		movwf	Count
clr2	clrf	POSTINC0
		decfsz	Count
		bra		clr2
		movlw	.16
		movwf	Count	
		lfsr	FSR0,evt00
		movlw	1			;start switch number
		movwf	Temp
		decf	FSR0L		;first sw no at evt3
clr3	movlw	4
		addwf	FSR0L		;only fourth byte to change
		movff	Temp,INDF0
		incf	Temp
		decfsz	Count
		bra		clr3
		clrf	evaddrl
		movlw	HIGH swdata
		movwf	evaddrh
		call	wrfbev		;write 64 byte block

		lfsr	FSR0,evt00	;set default event table
		movlw	.64
		movwf	Count
clr4	clrf	POSTINC0
		decfsz	Count
		bra		clr4
		movlw	.16
		movwf	Count	
		lfsr	FSR0,evt00
		decf	FSR0L

clr5	movlw	4
		addwf	FSR0L		;only fourth byte to change
		movff	Temp,INDF0
		incf	Temp
		decfsz	Count
		bra		clr5
		movlw	.64
		movwf	evaddrl
		movlw	HIGH swdata
		movwf	evaddrh
		call	wrfbev		;write 64 byte block
		return

;*************************************************************	

readNVs		movlw	LOW	NVstart
			addwf	ev2,W		;add index
			movwf	EEADR
			decf	EEADR,F		;index starts at 1, buffer at 0
			call	eeread
			movwf	Tx1d4		;NV val to transmit buffer
			movff	ev2,Tx1d3	;transfer index
			movlw	0x97		;NVANS
			movwf	Tx1d0
			movlw	5
			movwf	Dlc
			call	sendTX	
			return

;***********************************************************

putNV	movlw	NV_NUM + 1		;put new NV in EEPROM 
		cpfslt	ev2
		return
		movf	ev2,W
		bz		no_NV
		decf	WREG			;NVI starts at 1
		addlw	LOW NVstart
		movwf	EEADR
		movf	ev3,W
	
		call	eewrite	
		
no_NV	return
		

;******************************************************************
;		self enumeration as separate subroutine

self_en	movff	FSR1L,Fsr_tmp1Le	;save FSR1 just in case
		movff	FSR1H,Fsr_tmp1He 
		bsf		Datmode,1		;set to 'setup' mode
		movlw	.14
		movwf	Count
		lfsr	FSR0, Enum0
clr_en
		clrf	POSTINC0
		decfsz	Count
		bra		clr_en
;		clrf	T3GCON			;disable gating
		movlw	0x00			;set T0 to 128 mSec (may need more?)
		movwf	TMR0H
		movlw	0x00
		movwf	TMR0L
		movlw	B'10000010'		;clock div  8 (	2uSec clock)									
		movwf	T0CON			;enable timer 0
		bcf		INTCON,TMR0IF
		
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
				
	

self_en1	btfsc	INTCON,TMR0IF		;setup timer out?
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

en_done	bcf		T0CON,TMR0ON	;timer off
		bcf		INTCON,TMR0IF		;clear flag


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

;********************************************************

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

;
;*********************************************************************
	
;		a delay routine
			
dely	movlw	.10
		movwf	Count1
dely2	clrf	Count
dely1	decfsz	Count,F
		goto	dely1
		decfsz	Count1
		bra		dely2
		return	
		
	
; Switch event. Indexed by switch number. 32 switches of 4 bytes / switch = 128 Bytes

	ORG 0x2900

;default table

swdata
	
		


; Events data, room for 128 events at 32 bytes/event = 4KB	
		
	ORG	0x3000

evdata



	

;************************************************************************		
	ORG 0xF00000			;EEPROM data. Defaults
	
CANid	de	B'01111111',0	;CAN id default and module status
NodeID	de	0,0			;Node ID
ENindex	de	0,0		; hi byte contains free space
					; lo byte contains number of events
					; hi byte + lo byte = EN_NUM
FreeCh	de 0,0

	ORG 0xF00010

hashtab	de 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
		de 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
		de 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0		
		de 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
	
		
; number of events in each hash table entry


hashnum	de	0,0,0,0,0,0,0,0
		de	0,0,0,0,0,0,0,0
		de	0,0,0,0,0,0,0,0
		de	0,0,0,0,0,0,0,0


swstate	de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1
		de	1,1,1,1,1,1,1,1

sod_sw	de	0,0
NVstart	de	0,0							;one NV, initialised to 0

		ORG	0xF003FE
		de		0,0		;for boot load
		end


