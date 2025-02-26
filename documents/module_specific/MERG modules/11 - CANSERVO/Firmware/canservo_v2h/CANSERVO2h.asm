;   	TITLE		"Code for a 8 channel servo driver  FLiM node for CBUS"
; filename CANSERVO2h.asm  	Now incorporates Bootloader

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

;this code assumes one byte EV giving the servo numbers and a second EV for polarity. One bit per servo.
;EV1 = servo number.  Can have several servos per event.
;EV2 = polarity for that event

;allows for 4 NVs but only NV1 used
;this sets whether the servo cuts off at end or not. A bit set is cutoff enabled
;default is all cutoffs enabled
;default servo position is centred
;an ON event drives a servo to the first of the two NV settings
;an OFF event drives a servo to the second of the two NV settings
;unless the pol bit is set in the second EV

;Rev 2 A   initial attempt  15 / 05 / 11
;Rev 2 B	Full version with 8 channels  20/05/11
;Rev 2 C	Changes for learn mode and testing.
;Rev 2 D	Speed lookup table moved to Flash
;Rev 2 E	Mods to ev_set for multiple events.  30/05/11
;Rev 2 F	Read NVs now checks node number
;Rev 2 G	Change to 'test' routine so only one servo affected (28/07/11  MB)
;Rev 2 H    Add WRACK to EVULN


;end of comments for CANSERVO2. This is FLiM only at present but still needs putting into FLiM and giving a NN.

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
;	include		"FlimIds.inc"
	
	;definitions  for CANLED   Change these to suit hardware.
	
S_PORT 	equ	PORTA	;setup switch  Change as needed
S_BIT	equ	2


LED_PORT equ	PORTB  ;change as needed
Outport  equ	PORTC ;servo drives
LED1	equ		7	;PB7 is the green LED on the PCB
LED2	equ		6	;PB6 is the yellow LED on the PCB


CMD_ON		equ	0x90	;on event
CMD_OFF	equ	0x91	;off event

SCMD_ON	equ	0x98
SCMD_OFF	equ	0x99

EN_NUM  equ	.32		;number of allowed events
EV_NUM  equ 	2		;number of allowed EVs per event
NV_NUM	equ	.36		;number of allowed NVs for node (provisional)
SERVO_ID equ 	.11		;temp for now
SERVO_ID1 equ	2

Modstat equ 1		;address in EEPROM

;module parameters  change as required

Para1	equ	.165	;manufacturer number
Para2	equ	 "H"	;for now
Para3	equ	SERVO_ID
Para4	equ 	EN_NUM		;node descriptors (temp values)
Para5	equ 	EV_NUM
Para6	equ 	NV_NUM
Para7	equ 	SERVO_ID1

Cutval	equ 	.60			;change for cutoff delay

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
	
	TempCANCON
	TempCANSTAT
	TempINTCON
	CanID_tmp	;temp for CAN Node ID
	IDtemph		;used in ID shuffle
	IDtempl
	NN_temph		;node number in RAM
	NN_templ
	ENtemp1		;number of events
	
	IDcount		;used in self allocation of CAN ID.
	
	Mode		;for FLiM / SLiM etc
	Count		;counter for loading
	Count1
	Count2
	Count3
	T1count		;flash rate counter
	Keepcnt		;keep alive counter
	Latcount	;latency counter
	Datmode		;flag for data waiting and other states
	Temp		;temps
	Temp1
	Shift
	Dlc			;data length
	

	State				;state for interrupt routine
;Settings	
	S1_now				;servo1	current position
	S2_now
	S3_now
	S4_now
	S5_now
	S6_now
	S7_now
	S8_now

	
	
	Intcnt				;count	interrupts
	Index1				;for NV table pointer
	
	
	
	Intemp				;temporary storage for input

	Inchange			;servo bit has changed

	EEtemp				;temp for EE write data
	W_temp
	S_temp
	PCH_tmp
	Save
	Reset1
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
	Cutmode				;cutoff mode  (copy of NV1)	

	SV_flg				;servo flag for learning
	T_roll				;servo for testing
				
	NV1
	NV2
	NV3
	NV4

	
	Rx0con			;start of receive packet from RXB0
	Rx0sidh
	Rx0sidl
	Rx0eidh
	Rx0eidl
	Rx0dlc
	Rx0d0
	Rx0d1
	Rx0d2
	Rx0d3
	Rx0d4
	Rx0d5
	Rx0d6
	Rx0d7
	
	Cmdtmp		;command temp for number of bytes in frame jump table
	

	Match		;match flag
	ENcount		;which EN matched
	ENcount1	;temp for count offset
	ENend		;last  EN number
	ENtemp		;holds current EN pointer
	EVtemp		;holds current EV pointer
	EVtemp1		;EV for which servo
	EVtemp2		;holds current EV qualifier (polarity)
	
	LogFlag		;added to byte 6 of log reply
	
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

	Roll		;rolling bit for enum
	
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

		
	ENDC
	
	CBLOCK	0x100		;bank 1
	EN1					;start of EN ram
	
	
	ENDC
	
	CBLOCK	0x200		;bank 2		;holds SVs
	
	SV1
	SV2
	SV3
	SV4
	

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
		nop						;for debug
		goto	setup

		ORG		0808h
		goto	hpint			;high priority interrupt
		
		ORG		0810h			;node type parameters
node_ID	db		Para1,Para2,Para3,Para4,Para5,Para6,Para7
								;change these 7 bytes as required

		ORG		0818h	
		goto	lpint			;low priority interrupt


;*******************************************************************

		ORG		0820h			;start of program
;	
;
;	
hpint	movf	PCLATH,W
		movwf	PCH_tmp
		movlw	8
		movwf	PCLATH
		
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
;		movlw	0xFF
;		movwf	TMR0H
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
state3	movf	S1_now,W
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state4	bcf		Outport,0		;servo 1 down
		btfsc	Outmsk,1		;servo off?
		bsf		Outport,1		;servo 2 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state5	movf	S2_now,W
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state6	bcf		Outport,1		;servo 2 down
		btfsc	Outmsk,2		;servo off?
		bsf		Outport,2		;servo 3 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state7	movf	S3_now,W
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state8	bcf		Outport,2		;servo 3 down
		btfsc	Outmsk,3		;servo off?
		bsf		Outport,3		;servo 4 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback
	
state9	movf	S4_now,W
		movwf	TMR0L
		incf	State,F
		goto	sback

state10	bcf		Outport,3		;servo 4 down
		btfsc	Outmsk,4		;servo off?
		bsf		Outport,4		;servo 5 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state11	movf	S5_now,W
		movwf	TMR0L
		incf	State,F
		goto	sback

state12	bcf		Outport,4		;servo 5 down
		btfsc	Outmsk,5		;servo off?
		bsf		Outport,5		;servo 6 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state13	movf	S6_now,W
		movwf	TMR0L
		incf	State,F
		goto	sback

state14	bcf		Outport,5		;servo 6 down
		btfsc	Outmsk,6		;servo off?
		bsf		Outport,6		;servo 7 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state15	movf	S7_now,W
		movwf	TMR0L
		incf	State,F
		goto	sback

state16	bcf		Outport,6		;servo 7 down
		btfsc	Outmsk,7		;servo off?
		bsf		Outport,7		;servo 8 up
		movlw	.10
		movwf	TMR0L			;reload
		incf	State,F
		goto	sback

state17	movf	S8_now,W
		movwf	TMR0L
		incf	State,F
		goto	sback	


state18 bcf		Outport,7		;servo 8 down
		clrf	State
		goto	sback



sback	movf	PCH_tmp,W
		movwf	PCLATH
	
		retfie	1


;**************************************************************
;
;		low priority interrupt. Used for CAN receive and transmit error / latency.
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
		bra		rxb1int			;only receive interrupts used
		bra		rxb0int
		bra		back
		
rxb1int	bcf		PIR3,RXB1IF		;uses RB0 to RB1 rollover so may never use this
		
		lfsr	FSR0,Rx0con		;
		
		goto	access
		
rxb0int	bcf		PIR3,RXB0IF
		btfsc	Datmode,1			;setup mode?
		bra		setmode	
		lfsr	FSR0,Rx0con
		
		goto	access
		
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

access	movf	CANCON,W				;switch buffers
		andlw	B'11110001'
		movwf	CANCON
		movf	TempCANSTAT,W
		andlw	B'00001110'
		iorwf	CANCON
		lfsr	FSR1,RXB0CON	;this is switched bank
load	movf	POSTINC1,W
		movwf	POSTINC0
		movlw	0x6E			;end of access buffer lo byte
		cpfseq	FSR1L
		bra		load
		bcf		RXB0CON,RXFUL
		
		btfsc	Rx0dlc,RXRTR		;is it RTR?
		bra		isRTR
;		btfsc	Datmode,1			;setup mode?
;		bra		setmode	
		movf	Rx0dlc,F
		bz		back				;ignore zero length frames 
		bsf		Datmode,0		;valid message frame	
		
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
		
isRTR	btfsc	Datmode,1		;setup mode?
		bra		back			;back
		btfss	Mode,1			;FLiM?
		bra		back
		movlb	.15
isRTR1	btfsc	TXB2CON,TXREQ	
		bra		isRTR1		
		bsf		TXB2CON,TXREQ	;send ID frame - preloaded in TXB2

		movlb	0
		bra		back

setmode	tstfsz	RXB0DLC
		bra		back				;only zero length frames for setup
		
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

		bra		back
;	
;**************************************************************************************************



main	bsf		T0CON,TMR0ON	;start servo timer
		bsf		INTCON2,TMR0IP	;high priority for T0 interrupt
		btfsc	Mode,1			;is it SLiM?
		bra		mainf

mains	;btfss	PORTA,LEARN		;ignore NN switches if in learn mode
		;bra		main1
		btfss	PIR2,TMR3IF		;flash timer overflow?
		bra		nofl_s			;no SLiM flash
		btg		PORTB,7			;toggle green LED
		bcf		PIR2,TMR3IF
		movlw	4
		movwf	T1count
nofl_s	bra		noflash				;main1
		
; here if FLiM mde

mainf	btfss	PIR1,TMR1IF		;is it flash?
		bra		noflash
		bcf		PIR1,TMR1IF
		btfss	Datmode,2
		bra		nofl1
		decfsz	T1count
		bra		noflash
		btg		PORTB,6			;flash yellow LED
		movlw	4
		movwf	T1count
	
;		clrf	TMR1L			;reset timer
	
		
nofl1	btfss	Datmode,3		;running mode
		bra		noflash
		decfsz	Keepcnt			;send keep alive?
		bra		noflash
		movlw	.10
		movwf	Keepcnt
		movlw	0x52
;		call	nnrel			;send keep alive frame (works OK, turn off for now)

noflash	btfsc	S_PORT,S_BIT	;setup button?
		bra		main3
		movlw	.100
		movwf	Count
		clrf	Count1
		clrf	Count2
		movlw	4
		movwf	T1count
;		clrf	TMR1L
wait	decfsz	Count2
		goto	wait
		btfss	Datmode,2
		bra		wait2
		btfss	PIR1,TMR1IF		;is it flash?
		bra		wait2
		decfsz	T1count
		bra		wait2
		btg		PORTB,6			;flash LED
		movlw	4
		movwf	T1count
		bcf		PIR1,TMR1IF
;		clrf	TMR1L
wait2	decfsz	Count1
		goto	wait
		btfsc	S_PORT,S_BIT
		bra		main4			;not held long enough
		decfsz	Count
		goto	wait
		btfss	Mode,1			;is it in FLiM?
		bra		go_FLiM
		clrf	Datmode			;back to virgin
;		bcf		Mode,1			;SLiM mode
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
		goto	main				;setloop

main5	movlw	Modstat
		movwf	EEADR
		movlw	1
		call	eewrite				;mode to FLiM in EEPROM
;		bsf		PORTB,7				;yellow will flash
;		bsf		PORTB,7				;green LED on still
		bsf		Mode,1				;to FLiM
		movlw	B'11100000'
		movwf	INTCON
		goto	setloop				;setloop

main4	btfss	Datmode,3		
		bra		main3
		btfss	Datmode,2
		bra		set2
		bcf		Datmode,2
		bsf		PORTB,6			;LED on
		bra		main3
set2	bsf		Datmode,2
		call	nnack

main3	btfss	Datmode,1		;setup mode ?
		bra		main1
		btfss	PIR2,TMR3IF		;setup timer out?
		bra		main3			;fast loop till timer out (main3?)
		bcf		T3CON,TMR3ON	;timer off
		bcf		PIR2,TMR3IF		;clear flag
		call	new_enum		;enum routine
		movlw	LOW CANid		;put new ID in EEPROM
		movwf	EEADR
		movf	IDcount,W
		call	eewrite
		call	newid_f			;put new ID in various buffers
		movlw	Modstat
		movwf	EEADR
		movlw	1
		call	eewrite			;set to normal status
		bcf		Datmode,1		;out of setup
		bsf		Datmode,2		;wait for NN
;		call	nnack			;send blank NN for config
;		bsf		PORTB,7			;on light
		
		bra		main			;continue normally

go_FLiM	bsf		Datmode,1		;FLiM setup mode
		bcf		PORTB,7			;green off
		bra		wait1
		
		

; common to FLiM and SLiM		
	
	
main1	btfss	Datmode,0		;any new CAN frame received?
		bra		gap0
		bra		packet			;yes
gap0	movf	State,F			;is state 0?
		bz		gap				;gap between servo cycles

		
		
		bra		main
		
		
gap		bcf		T0CON,T08BIT
		bcf		INTCON,TMR0IF
		movlw	0xFF
		movwf	TMR0H			;change speed for delay
		movlw	B'00000111'
		iorwf	T0CON,F
		movlw	.1
		movwf	TMR0L
		btfsc	Datmode,7		;in servo setting mode?
		call	test
		call	servo			;update servo settings
gap1	incf	State
	
		goto	main		
;********************************************************************

;		These are here as branch was too long

unset	;bsf	Datmode,5		;unlearn this event
		;bra	go_on
		btfss	Datmode,4
		bra		main2			;prevent error message
		bsf		Datmode,5
		bra		learn1
		
readEV	btfss	Datmode,4
		bra		main2			;prevent error message
		bsf		Datmode,6			;read back an EV
		bra		learn1

evns1	call	thisNN				;read event numbers
		sublw	0
		bnz		evns3
		call	evns2
		bra		main2
evns3	goto	notNN

reval	call	thisNN				;read event numbers
		sublw	0
		bnz		notNNx
		call	evsend
		bra		main2
notNNx	goto	notNN

go_on_x goto	go_on

params	btfss	Datmode,2		;only in setup mode
		bra		main2
		call	parasend
		bra		main2
		
short	clrf	Rx0d1
		clrf	Rx0d2
		bra		go_on	
		
		
;********************************************************************
								;main packet handling is here
								;add more commands for incoming frames as needed
		
packet	movlw	CMD_ON  ;only ON, OFF  events supported
		subwf	Rx0d0,W	
		bz		go_on_x
		movlw	CMD_OFF
		subwf	Rx0d0,W
		bz		go_on_x
		
		movlw	SCMD_ON
		subwf	Rx0d0,W
		bz		short
		movlw	SCMD_OFF
		subwf	Rx0d0,W
		bz		short
		movlw	0x96			;learn NV by index
		subwf	Rx0d0,W
		bz		lrn_NV
		
		movlw	0x5C			;reboot
		subwf	Rx0d0,W
		bz		reboot
		movlw	0x73
		subwf	Rx0d0,W
		bz		para1a			;read individual parameters
		btfss	Mode,1			;FLiM?
		bra		main2
		movlw	0x42			;set NN on 0x42
		subwf	Rx0d0,W
		bz		setNN
		movlw	0x10			
		subwf	Rx0d0,W
		bz		params			;read node parameters
		
		movlw	0x53			;set to learn mode on 0x53
		subwf	Rx0d0,W
		bz		setlrn		
		movlw	0x54			;clear learn mode on 0x54
		subwf	Rx0d0,W
		bz		notlrn
		movlw	0x55			;clear all events on 0x55
		subwf	Rx0d0,W
		bz		clrens
		movlw	0x56			;read number of events left
		subwf	Rx0d0,W
		bz		rden
		movlw	0xD2			;is it set event?
		subwf	Rx0d0,W
		bz		chklrn			;do learn
		movlw	0x95			;is it unset event
		subwf	Rx0d0,W			
		bz		unset
		movlw	0xB2			;read event variables
		subwf	Rx0d0,W
		bz		readEV
	
		movlw	0x57			;is it read events
		subwf	Rx0d0,W
		bz		readEN1
		movlw	0x72
		subwf	Rx0d0,W
		bz		readENi			;read event by index
		movlw	0x58
		subwf	Rx0d0,W
		bz		evns
		movlw	0x9C				;read event variables by EN#
		subwf	Rx0d0,W
		bz		reval
		movlw	0x71				;read NVs by index
		subwf	Rx0d0,W
		bz		read_NV
;		call	thisNN
;		bnz		main2			;not this node
;		movlw	1				;error 1 not supported by this node
;		goto	errmsg
		bra	main2	
evns	goto	evns1
		bra		main2
lrn_NV	goto	lrn_NV1
readEN1 goto	readEN

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
		movf	Rx0d1,w
		addwf	Rx0d2,w
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
		movf	Rx0d1,w
		addwf	Rx0d2,w
		bnz		notNN
		call	para1rd
		bra		main2
			
main2	bcf		Datmode,0
		goto	main			;loop
		
setNN	btfss	Datmode,2		;in NN set mode?
		bra		main2			;no
		call	putNN			;put in NN
		bcf		Datmode,2
		bsf		Datmode,3
		movlw	.10
		movwf	Keepcnt			;for keep alive
		movlw	0x52
		call	nnrel			;confirm NN set
		bsf		LED_PORT,LED2	;LED ON
		bcf		LED_PORT,LED1
		bra		main2
		
sendNN	btfss	Datmode,2		;in NN set mode?
		bra		main2			;no
		movlw	0x50			;send back NN
		movwf	Tx1d0
		movlw	3
		movwf	Dlc
		call	sendTX
		bra		main2

rden	goto	rden1


		
setlrn	call	thisNN
		sublw	0
		bnz		notNN
		bsf		Datmode,4
		bsf		LED_PORT,LED2			;LED on
		bra		main2

notlrn	call	thisNN
		sublw	0
		bnz		notNN
		bcf		Datmode,4
notln1		;leave in learn mode
		bcf		Datmode,5
;		bcf		LED_PORT,LED2
		btfsc	Datmode,7				;SV learn?
		call	storeSV
		bra		main2
clrens	call	thisNN
		sublw	0
		bnz		notNN
		btfss	Datmode,4
		bra		clrerr
		call	enclear
		movlw	0x59
		call	nnrel		;send WRACK
		bra		notln1
notNN	bra		main2
clrerr	movlw	2			;not in learn mode
		goto	errmsg

		
chklrn	btfsc	Datmode,4
		bra		learn1			;is in learn mode
		bra		main2


go_on	btfss	Mode,1			;FLiM?
		

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

readENi	call	thisNN			;read event by index
		sublw	0
		bnz		notNN
		call	enrdi
		bra		main2


paraerr	movlw	3				;error not in setup mode
		goto	errmsg


readEN	call	thisNN
		sublw	0
		bnz		notNN
		call	enread
		bra		main2
		
do_it	
		call	ev_set			;do it -  for consumer action
		bra		main2
		

		
rden1	call	thisNN
		sublw	0
		bnz		notNN
		movlw	LOW ENindex+1		;read number of events available
		movwf	EEADR
		call	eeread
		sublw	EN_NUM
		movwf	Tx1d3
		movlw	0x70
		movwf	Tx1d0
		movlw	4
		movwf	Dlc
		call	sendTX
		bra		main2		
		
learn1	btfss	Mode,1			;FLiM?
		bra		learn2
		movlw	0xD2
		subwf	Rx0d0,W			;is it a learn command
		bz		learn2			;OK
		movlw	0x95			;is it unlearn
		subwf	Rx0d0,W
		bz		learn2
		movlw	0xB2
		subwf	Rx0d0
		bz		learn2
		movlw	1				;cmd not supported
		goto	errmsg1
;		bra		l_out2

learn2	call	enmatch			;is it there already?
		movwf	LogFlag			;for logging
		sublw 	0
		bz		isthere
		btfss	Mode,1			;FLiM?
		goto	main2
	
learn3	btfsc	Datmode,6		;read EV?
		bra		rdbak1			;not here
		btfsc	Datmode,5		;if unset and not here
		bra		l_out1			;do nothing else 
learn4	call	learnin			;put EN into stack and RAM
		sublw	0
		bz		new_EV
		movlw	4
		goto	errmsg2			;bra		l_out1			;too many
isthere	btfss	Mode,1
		goto	main2
		
isth1	btfss	Datmode,5		;FLiM unlearn?
		bra		mod_EV
		bra		unlearn
	


rdbak	movff	EVtemp,Tx1d5		;Index for readout	
		incf	Tx1d5,F				;add one back	
		bsf		EECON1,RD			;address set already
		movff	EEDATA,Tx1d6
		bra		shift4
rdbak1	movlw	5				;no match
		goto	errmsg2			
	

shift4	movlw	0xD3				;readback of EVs
		movwf	Tx1d0
		movff	Rx0d1,Tx1d1
		movff	Rx0d2,Tx1d2
		movff	Rx0d3,Tx1d3
		movff	Rx0d4,Tx1d4
		movlw	7
		movwf	Dlc
		call	sendTXa	
		bra		l_out1

new_EV	btfsc	Mode,1				;FLiM?
		bra		new_EVf				;not relevant if FLiM
		movlw	LOW ENindex+1		;here if a new event
		movwf	EEADR
		bsf		EECON1,RD
		decf	EEDATA,W
		movwf	ENcount				;recover EN counter

mod_EV	btfsc	Mode,1				;FLiM?
		bra		mod_EVf				;not relevant if FLiM
		goto	main1

new_EVf	movlw	LOW ENindex+1		;here if a new event in FLiM mode
		movwf	EEADR
		call	eeread
;		bsf		EECON1,RD
;		decf	EEDATA,W
		decf	WREG
		movwf	ENcount				;recover EN counter
mod_EVf	movff	Rx0d5,EVtemp	;store EV index
		movf	EVtemp,F		;is it zero?
		bz		noEV
		decf	EVtemp,F		;decrement. EVs start at 1
		movlw	EV_NUM
		cpfslt	EVtemp
		bra		noEV		
;		btfsc	Datmode,5		;is it here and unlearn,goto unlearn
;		bra		unlearn			;else modify EVs
		
		movff	Rx0d6,EVtemp2	;store EV
		
		
		rlncf	ENcount,W			;two byte values
		addlw	LOW EVstart			;point to EV
		movwf	EEADR
		movf	EVtemp,W			;add index to EEPROM value
		addwf	EEADR,F
		btfsc	Datmode,6			;is it readback
		bra		rdbak
		movf	EVtemp2,W
		call	eewrite				;put in
		movlw	0x59
		call	nnrel				;send WRACK
;		movlw	0x52
;		call	nnrel
		bra		l_out2


			

l_out	bcf		Datmode,4
;		bcf		LED_PORT,LED2
l_out1	bcf		Datmode,6
l_out2	bcf		Datmode,0
		

		clrf	PCLATH
		goto	main2
		
noEV	movlw	6				;invalid EV#
		goto	errmsg2
								;unlearn an EN. 
unlearn	movlw	LOW ENindex+1		;get number of events in stack
		movwf	EEADR
		bsf		EECON1,RD
		
		movff	EEDATA,ENend
		movff	EEDATA,ENtemp
		rlncf	ENend,F			;ready for end value
		rlncf	ENend,F
		movlw	LOW ENstart
		addwf	ENend,F			;end now points to next past end in EEPROM
		movlw	4
		addwf	ENend,F
		rlncf	ENcount,F		;Double the counter for two bytes
		rlncf	ENcount,F		;Double the counter for two bytes
		movlw	LOW ENstart + 4
		addwf	ENcount,W
		movwf	EEADR
un1		bsf		EECON1,RD
		movf	EEDATA,W		;get byte
		decf	EEADR,F
		decf	EEADR,F
		decf	EEADR,F
		decf	EEADR,F
		call	eewrite			;put back in
		movlw	5
		addwf	EEADR,F
		movf	ENend,W
		cpfseq	EEADR
		bra		un1
		
		rrncf	ENcount,F		;back to double bytes
		rlncf	ENtemp,F
		movlw	LOW EVstart
		addwf	ENtemp,F
		movlw	2
		addwf	ENtemp,F
		movlw	LOW EVstart + 2
		addwf	ENcount,W
		movwf	EEADR
un2		bsf		EECON1,RD
		movf	EEDATA,W		;get byte
		decf	EEADR,F
		decf	EEADR,F
		call	eewrite			;put back in
		movlw	3
		addwf	EEADR,F
		movf	ENtemp,W
		cpfseq	EEADR
		bra		un2
		movlw	LOW ENindex+1
		movwf	EEADR
		bsf		EECON1,RD
		movf	EEDATA,W
		movwf	Temp
		decf	Temp,W
		call	eewrite			;put back number in stack less 1
		call	en_ram			;rewrite RAM stack
		bcf		Datmode,5
		movlw	0x59
		call	nnrel			;send WRACK
		
		bra		l_out1
				

lrn_NV1	call	thisNN			;is it this NN?
		sublw	0
		bnz		lrn_NV2			;not this NN
		call	lrnNVs
lrn_NV2	goto	main2	

	
	
	
		

				
		
		
		
		
;***************************************************************************
;		main setup routine
;*************************************************************************

setup	clrf	INTCON			;no interrupts yet
		clrf	ADCON0			;turn off A/D, all digital I/O
		movlw	B'00001111'
		movwf	ADCON1
		
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
		movlw	B'00000000'		;Port C  set to outputs.
		movwf	TRISC
		clrf	PORTC
	
		
;	next segment is essential.
		
		bsf		RCON,IPEN		;enable interrupt priority levels
		clrf	BSR				;set to bank 0
		clrf	EECON1			;no accesses to program memory	
		clrf	Datmode
		clrf	Latcount
		clrf	ECANCON			;CAN mode 0 for now. 
		 
		bsf		CANCON,7		;CAN to config mode
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
		movlb	.15
		movlw	B'00100100'		;reject extended frames
		movwf	RXB1CON
		clrf	RXF0SIDL
		clrf	RXF1SIDL
		movlb	0
		


		
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
		clrf	PORTC			;all low
		
		clrf	Save
		clrf	Reset1
		clrf	Off1
		clrf	Off2
		clrf	Off3
		clrf	Off4
		clrf	Intemp
		clrf	Inchange
	
		clrf	Delflg
		clrf	Gotflg



;next segment required
		
		movlw	B'00000001'
		movwf	IDcount			;set at lowest value for starters
		
		clrf	INTCON2			;
		clrf	INTCON3			;
		

		movlw	B'00100011'		;B'00100011'  Rx0 and RX1 interrupt and Tx error
								
		movwf	PIE3
	
		clrf	PIR1
		clrf	PIR2
		movlb	.15
		bcf		RXB1CON,RXFUL
		movlb	0
		bcf		RXB0CON,RXFUL		;ready for next
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL
		clrf	PIR3			;clear all flags
;		bsf		PIE1,0			;timer 1
;		bsf		IPR1,0
		clrf	State			;servo state to 0
		call	nv_ram			;put NVs in ram
		call	en_ram			;put events in RAM
		movlw	LOW NVstart
		movwf	EEADR
		call	eeread
		movwf	Cutmode
		
		
		;		test for setup mode
		clrf	Mode
		movlw	Modstat			;get setup status
		movwf	EEADR
		call	eeread
		movwf	Datmode
		sublw	0				;not set yet
		bnz		setid
		bra		slimset			;wait for setup PB
	
		
setid	bsf		Mode,1			;flag FLiM
		call	newid_f			;put ID into Tx1buf, TXB2 and ID number store
		
	
seten_f	call	en_ram			;put events in RAM
	
		movlw	B'11100000'
		movwf	INTCON			;enable interrupts
		bcf		LED_PORT,LED1
		bsf		LED_PORT,LED2			;Yellow LED on.
		bcf		Datmode,0
		goto	main

slimset	bcf		Mode,1
		clrf	NN_temph
		clrf	NN_templ
		;test for clear all events
;		btfss	PORTA,LEARN		;ignore the clear if learn is set
;		goto	seten
;		btfss	PORTA,UNLEARN
;		call	enclear			;clear all events if unlearn is set during power up
seten	call	en_ram			;put events in RAM
		
	
		movlw	B'11100000'
		movwf	INTCON			;enable interrupts
		bcf		PORTB,6
		bsf		PORTB,7			;RUN LED on. Green for SLiM
		goto	main

setloop	
		movlw	B'11100000'
		movwf	INTCON			;enable interrupts
		bsf		Datmode,1		;setup mode

		call	enum			;sends RTR frame
		bra		main		


		
;****************************************************************************
;		start of subroutines

;		Do an event.  arrives with EVs in EVtemp and EVtemp2

ev_set	comf	EVtemp,W
		andwf	Gotflg,F		;for run	
		movwf	EVtemp1			;for bit mask
		movf	EVtemp,W
		iorwf	Inchange,F		;set which has changed
		andwf	EVtemp2,W		;for polarity
		xorwf	EVtemp,F
		btfsc	Rx0d0,0			;ON or OFF ?
		bra		ev_off
		comf	EVtemp,F		;reverse
		
			
ev_off	movf	EVtemp1,W		;get mask
		andwf	Intemp,F		;clear relevant servo bits only
		comf	EVtemp1,W
		andwf	EVtemp,W
		iorwf	Intemp,F
		xorwf	EVtemp2,F
		movf	EVtemp,W
		andwf	EVtemp2,W
		iorwf	Intemp,F			;set direction 	
		return					



;		Send contents of Tx1 buffer via CAN TXB1

sendTX1	lfsr	FSR0,Tx1con
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
		return					;successful send

		
;*********************************************************************
;		put in NN from command

putNN	movff	Rx0d1,NN_temph
		movff	Rx0d2,NN_templ
		movlw	LOW NodeID
		movwf	EEADR
		movf	Rx0d1,W
		call	eewrite
		incf	EEADR
		movf	Rx0d2,W
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
		btfsc	Datmode,3		;already set up?
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

shuffin	movff	Rx0sidl,IDtempl
		swapf	IDtempl,F
		rrncf	IDtempl,W
		andlw	B'00000111'
		movwf	IDtempl
		movff	Rx0sidh,IDtemph
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
		
;***************************************************************
enum	clrf	Tx1con			;CAN ID enumeration. Send RTR frame, start timer
		clrf	Enum0
		clrf	Enum1
		clrf	Enum2
		clrf	Enum3
		clrf	Enum4
		clrf	Enum5
		clrf	Enum6
		clrf	Enum7
		clrf	Enum8
		clrf	Enum9
		clrf	Enum10
		clrf	Enum11
		clrf	Enum12
		clrf	Enum13

		
		movlw	B'10111111'		;fixed node, default ID  
		movwf	Tx1sidh
		movlw	B'11100000'
		movwf	Tx1sidl
		movlw	B'01000000'		;RTR frame
		movwf	Dlc
		
		movlw	0x3C			;set T3 to 100 mSec (may need more?)
		movwf	TMR3H
		movlw	0xAF
		movwf	TMR3L
		movlw	B'10110001'
		movwf	T3CON			;enable timer 3
;		bsf		Datmode,1		;used to flag setup state
		movlw	.10
		movwf	Latcount
		
		call	sendTXa			;send RTR frame
		clrf	Tx1dlc			;prevent more RTR frames
		return


;*********************************************************

;		learn input of EN

learnin	btfss	Mode,1
		goto	main2
		


lrnin1	btfsc	Datmode,5	;don't do if unlearn
		return
		movlw	LOW ENindex+1
		movwf	EEADR
		call	eeread
;		bsf		EECON1,RD
;		movf	EEDATA,W
		movwf	ENcount		;hold pointer
		movlw	EN_NUM
		cpfslt	ENcount
		retlw	1					;too many
		lfsr	FSR0,EN1			;point to EN stack in RAM
		
		rlncf	ENcount,F			;double it
		rlncf	ENcount,F			;double again
		movf	ENcount,W
		movff	Rx0d1,PLUSW0		;put in RAM stack
		addlw	1
		movff	Rx0d2,PLUSW0
		addlw	1
		movff	Rx0d3,PLUSW0
		addlw	1
		movff	Rx0d4,PLUSW0
		movlw	LOW ENstart
		addwf	ENcount,W
		movwf	EEADR
		movf	Rx0d1,W				;get EN hi byte
		call	eewrite
		incf	EEADR
		movf	Rx0d2,W
		call	eewrite
		incf	EEADR
		movf	Rx0d3,W
		call	eewrite
		incf	EEADR
		movf	Rx0d4,W
		call	eewrite
		
		
		movlw	LOW ENindex+1
		movwf	EEADR
;		bsf		EECON1,RD
;		movf	EEDATA,W
		call	eeread
		addlw	1					;increment for next
		movwf	Temp
		call	eewrite				;put back
		btfsc	Mode,1
		bra		notful
		movlw	EN_NUM				;is it full now?
		subwf	Temp,W
		bnz		notful
		bsf		T1CON,TMR1ON		;set for flash
		retlw	1
notful	retlw	0

		
		
;**************************************************************************
;
;		EN match.	Compares EN (in Rx0d1, Rx0d2, Rx0d3 and Rx0d4) with stored ENs
;		If match, returns with W = 0
;		The matching number is in ENcount. 
;
enmatch	lfsr	FSR0,EN1	;EN ram image
		movlw	LOW ENindex+1	;
		movwf	EEADR
;		bsf		EECON1,RD
;		movf	EEDATA,W
		call	eeread
		movwf	Count
		movf	Count,F
	
		bz		en_out		;if no events set, do nothing
		clrf	ENcount
	
		
ennext	clrf	Match
		movf	POSTINC0,W
		cpfseq	Rx0d1
		incf	Match
		movf	POSTINC0,W
		cpfseq	Rx0d2
		incf	Match
		movf	POSTINC0,W
		cpfseq	Rx0d3
		incf	Match
		movf	POSTINC0,W
		cpfseq	Rx0d4
		incf	Match
		tstfsz	Match
		bra		en_match
		rlncf	ENcount,W		;get EVs
		addlw	LOW EVstart		
		movwf	EEADR
		bcf	EEADR,0		;multiple of 2

		call	eeread
	
		movwf	EVtemp		;EV  (EV1)
		incf	EEADR
		call	eeread
		movf	EEDATA,W
		movwf	EVtemp2		;EV   (EV2)
		retlw	0			;is a match
en_match	
		movf	Count,F
		bz		en_out
		decf	Count,F
		incf	ENcount,F
		bra		ennext
en_out	retlw	1		
		


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
		subwf	Rx0d1,W
		bnz		not_NN
		movf	NN_templ,W
		subwf	Rx0d2,W
		bnz		not_NN
		retlw 	0			;returns 0 if match
not_NN	retlw	1
							
;**********************************************************************
;		loads ENs from EEPROM to RAM for fast access
;		shifts all 32 even if less are used

en_ram	movlw	EN_NUM
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

nv_ram	movlw	NV_NUM-4			;shift variables to ram
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
		return	
		
		
;		clears all stored events

enclear	movlw	EN_NUM * 4 + 2		;number of locations in EEPROM
		movwf	Count
		movlw	LOW ENindex
		movwf	EEADR
enloop	movlw	0
		call	eewrite
		incf	EEADR
		decfsz	Count
		bra		enloop
		;now clear the ram
		movlw	EN_NUM * 4
		movwf	Count
		lfsr	FSR0, EN1
ramloop	clrf	POSTINC0
		decfsz	Count
		bra		ramloop
		return	
;************************************************************




;		read back all events in sequence

enread	clrf	Temp
		movlw	LOW ENindex + 1
		movwf	EEADR
		call	eeread
		movwf	ENtemp1
		bz		noens		;no events set
		
		movlw	1
		movwf	Tx1d7		;first event
		movlw	LOW	ENstart
		movwf	EEADR
		
	
enloop1	
;		movff	NN_temph,Tx1d1
;		movff	NN_templ,Tx1d2
		call	eeread
		movwf	Tx1d3
		incf	EEADR,F
		call	eeread
		movwf	Tx1d4
		incf	EEADR,F
		call	eeread
		movwf	Tx1d5
		incf	EEADR,F
		call	eeread
		movwf	Tx1d6
		incf	EEADR,F
		
ensend	movlw	0xF2
		movwf	Tx1d0		;OPC
		movlw	8
		movwf	Dlc
		call	sendTX			;send event back
		call	dely
		movf	Tx1d7,F
		bz		lasten
		incf	Temp,F
		movf	ENtemp1,W
		subwf	Temp,W
		bz		lasten
		incf	Tx1d7
		bra		enloop1			;next one
noens	clrf	Rx0d3
		bra		noens1
	
	
lasten	return	

;*************************************************************************

;	send individual event by index

enrdi	movlw	LOW ENindex + 1
		movwf	EEADR
		call	eeread
		movwf	ENtemp1
		bz		noens1		;no events set
		movf	Rx0d3,W	
		decf	WREG
		cpfsgt	ENtemp1
		bra		noens1		;too many
		rlncf	WREG
		rlncf	WREG
		addlw	LOW	ENstart
		movwf	EEADR
		call	eeread
		movwf	Tx1d3
		incf	EEADR
		call	eeread
		movwf	Tx1d4
		incf	EEADR
		call	eeread
		movwf	Tx1d5
		incf	EEADR
		call	eeread
		movwf	Tx1d6
enrdi1	movff	Rx0d3,Tx1d7
		movlw	0xF2
		movwf	Tx1d0
		movlw	8
		movwf	Dlc
		call	sendTX
		return
noens1	movlw	7				;no events set
		call	errsub
		return

;************************************************************************
		
;		send number of events

evns2	movlw	LOW	ENindex+1
		movwf	EEADR
		call	eeread
		movwf	Tx1d3
		movlw	0x74
		movwf	Tx1d0
		movlw	4
		movwf	Dlc
		call	sendTX
		return

;***********************************************************

;		send EVs by reference to EN index

evsend	movf	Rx0d3,W		;get event index
		bz		notEN		;can,t be zero
		decf	WREG
		movwf	Temp
		movlw	LOW ENindex+1	;get number of stored events
		movwf	EEADR
		call	eeread
		cpfslt	Temp
		bra		notEN		;too many events in index
		
		movf	Temp,W
		mullw	EV_NUM		;PRODL has start of EVs
		movf	Rx0d4,W		;get EV index
		bz		notEN		;
		decf	WREG
		movwf	Temp1
		movlw	EV_NUM
		cpfslt	Temp1
		bra		notEN		;too many EVs in index
		movf	Temp1,W
		addwf	PRODL,W		;get EV adress
		addlw	LOW EVstart
		movwf	EEADR
		call	eeread
		movwf	Tx1d5		;put in EV value
		movlw	0xB5
		movwf	Tx1d0
		movff	Rx0d3,Tx1d3
		movff	Rx0d4,Tx1d4
		movlw	6
		movwf	Dlc
		call	sendTX	
		return
notEN	movlw	8		;invalid EN#
		call	errsub
		return
		

;**************************************************************************
;		send node parameter bytes (7 maximum)

parasend	
		movlw	0xEF
		movwf	Tx1d0
		movlw	LOW node_ID
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

;**********************************************************

;		send individual parameter

para1rd	movlw	0x9B
		movwf	Tx1d0
		movlw	LOW node_ID
		movwf	TBLPTRL
		movlw	8
		movwf	TBLPTRH		;relocated code
		decf	Rx0d3,W
		addwf	TBLPTRL
		bsf		EECON1,EEPGD
		tblrd*
		movff	TABLAT,Tx1d4
		bcf		EECON1,EEPGD
		movff	Rx0d3,Tx1d3
		movlw	5
		movwf	Dlc
		call	sendTX
		return	


;sendlog
;		movlw	0xF7
;		movwf	Tx1d0
;		movff	Rx0d5, Tx1d3
;		movff	Rx0d6, Tx1d4
;		movff	ENcount, Tx1d5
;		movff	LogFlag, Tx1d6
;		clrf	Tx1d7
;		movlw	8
;		movwf	Dlc
;		call	sendTX
;		call	ldely
;		return
		
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

;*********************************************************
;		loads current off position on reset

load_init		movlw	8
		movwf	Count
		lfsr	FSR0,SV1+1			;initialise starting positions on reset
		lfsr	FSR1,S1_now
load_loop		movff	POSTINC0,POSTINC1
		movlw	3
		addwf	FSR0L				;pos goes every 4th NV
		decfsz	Count,F
		bra	load_loop
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
		movlw	0x30
		movwf	TBLPTRH
		bsf		EECON1,EEPGD
		
		tblrd*+
		movf	TABLAT,W
		
		bcf		EECON1,EEPGD	
	
		return


;addlw	LOW Spdtbl
	;	movwf	EEADR
	;	call	eeread
	;	return	

;*****************************************************	

;
;	main scanning routine

servo	lfsr	FSR2,SV1		;set to SV table in FSR2

t1		btfss	Inchange,0		;has this input changed?
		goto	t1_noch			;no
		bcf		Inchange,0
		bsf		Outmsk,0		;enable output on servo 1
		bcf		Delflg,0		;clear delay mode on 1
		bcf		Gotflg,0		;not got there on 1
	
		movlw	Cutval
		movwf	Off1			;set delay timer for 1

t1_noch	btfsc	Gotflg,0		;already there?
		goto	t2				;next servo
		btfss	Intemp,0		;what is input direction?
		goto	t1_low			;input low
		
		movlw	S1_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
	
		btfss	STATUS,Z		;is rate zero?
		goto	t1_1h
		movlw	S1_low			;speed is max so update S1_now to limit
		movf	PLUSW2,W
		movwf	S1_now

t1_1h	movlw	S1_low
		movff	PLUSW2,S_temp
		movf	S1_now,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t1_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t1_dh
		decfsz	Count
		goto	t1_ih
		goto	t2				;rate countdown
	
t1_ih	incf	S1_now,F
		goto	t1_1h

t1_dh	decfsz	Count,F			;rate countdown
		goto	t1_dh2
		goto	t2				;finished t1
t1_dh2	decf	S1_now,F
		goto	t1_1h



t1_low	movlw	S1_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t1_1l
		movlw	S1_hi		;speed is max so update S1_now to limit
		movff	PLUSW2,S1_now

t1_1l	movlw	S1_hi
		movff	PLUSW2,S_temp
		movf	S1_now,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t1_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t1_dl
		decfsz	Count,F			;rate countdown
		goto	t1_il
		goto	t2				;finished t1
t1_il	incf	S1_now,F
		goto	t1_1l

t1_dl	decfsz	Count,F			;rate countdown
		goto	t1_dl2
		goto	t2
t1_dl2	decf	S1_now,F
		goto	t1_1l

t1_done	bsf		Delflg,0		;set flag for delay cutoff	


; servo	2	

t2		btfss	Inchange,1		;has this input changed?
		goto	t2_noch			;no
		bcf		Inchange,1
		bsf		Outmsk,1		;enable output on servo 2
		bcf		Delflg,1		;clear delay mode on 2
		bcf		Gotflg,1		;not got there on 2
		movlw	Cutval
		movwf	Off2			;set delay timer for 2

t2_noch	btfsc	Gotflg,1		;already there?
		goto	t3				;next servo
		btfss	Intemp,1		;what is input direction?
		goto	t2_low			;input low
		movlw	S2_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t2_1h
		movlw	S2_low		;speed is max so update S2_now to limit
		movff	PLUSW2,S2_now

t2_1h	movlw	S2_low
		movff	PLUSW2,S_temp
		movf	S2_now,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t2_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t2_dh
		decfsz	Count,F			;rate countdown
		goto	t2_ih
		goto	t3				;finished t2
t2_ih	incf	S2_now,F
		goto	t2_1h

t2_dh	decfsz	Count,F			;rate countdown
		goto	t2_dh2
		goto	t3				;finished t2
t2_dh2	decf	S2_now,F
		goto	t2_1h



t2_low	movlw	S2_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv	
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t2_1l
		movlw	S2_hi		;speed is max so update S2_now to limit
		movff	PLUSW2,S2_now

t2_1l	movlw	S2_hi
		movff	PLUSW2,S_temp
		movf	S2_now,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t2_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t2_dl
		decfsz	Count,F			;rate countdown
		goto	t2_il
		goto	t3				;finished t2
t2_il	incf	S2_now,F
		goto	t2_1l

t2_dl	decfsz	Count,F			;rate countdown
		goto	t2_dl2
		goto	t3
t2_dl2	decf	S2_now,F
		goto	t2_1l

t2_done	bsf		Delflg,1		;set flag for delay cutoff	

;	servo 3
		
t3		btfss	Inchange,2		;has this input changed?
		goto	t3_noch			;no
		bcf		Inchange,2
		bsf		Outmsk,2		;enable output on servo 3
		bcf		Delflg,2		;clear delay mode on 3
		bcf		Gotflg,2		;not got there on 3
		movlw	Cutval
		movwf	Off3			;set delay timer for 3

t3_noch	btfsc	Gotflg,2		;already there?
		goto	t4				;next servo
		btfss	Intemp,2		;what is input direction?
		goto	t3_low			;input low
		movlw	S3_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t3_1h
		movlw	S3_low		;speed is max so update S3_now to limit
		movff	PLUSW2,S3_now

t3_1h	movlw	S3_low
		movff	PLUSW2,S_temp
		movf	S3_now,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t3_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t3_dh
		decfsz	Count,F			;rate countdown
		goto	t3_ih
		goto	t4				;finished t3
t3_ih	incf	S3_now,F
		goto	t3_1h

t3_dh	decfsz	Count,F			;rate countdown
		goto	t3_dh2
		goto	t4				;finished t3
t3_dh2	decf	S3_now,F
		goto	t3_1h



t3_low	movlw	S3_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t3_1l
		movlw	S3_hi		;speed is max so update S3_now to limit
		movff	PLUSW2,S3_now

t3_1l	movlw	S3_hi
		movff	PLUSW2,S_temp
		movf	S3_now,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t3_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t3_dl
		decfsz	Count,F			;rate countdown
		goto	t3_il
		goto	t4				;finished t3
t3_il	incf	S3_now,F
		goto	t3_1l

t3_dl	decfsz	Count,F			;rate countdown
		goto	t3_dl2
		goto	t4
t3_dl2	decf	S3_now,F
		goto	t3_1l

t3_done	bsf		Delflg,2		;set flag for delay cutoff	

;	servo 4

		
t4		
		btfss	Inchange,3		;has this input changed?
		goto	t4_noch			;no
		bcf		Inchange,3
		bsf		Outmsk,3		;enable output on servo 4
		bcf		Delflg,3		;clear delay mode on 4
		bcf		Gotflg,3		;not got there on 4
		movlw	Cutval
		movwf	Off4			;set delay timer for 4

t4_noch	btfsc	Gotflg,3		;already there?
		goto	t5				;next servo
	
		btfss	Intemp,3		;what is input direction?
		goto	t4_low			;input low
		movlw	S4_rat2		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t4_1h
		movlw	S4_low		;speed is max so update S4_now to limit
		movff	PLUSW2,S4_now

t4_1h	movlw	S4_low
		movff	PLUSW2,S_temp
		movf	S4_now,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t4_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t4_dh
		decfsz	Count,F			;rate countdown
		goto	t4_ih
		goto	t5				;finished t4
t4_ih	incf	S4_now,F
		goto	t4_1h

t4_dh	decfsz	Count,F			;rate countdown
		goto	t4_dh2
		goto	t5				;finished t4
t4_dh2	decf	S4_now,F
		goto	t4_1h



t4_low	movlw	S4_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t4_1l
		movlw	S4_hi		;speed is max so update S4_now to limit
		movff	PLUSW2,S4_now

t4_1l	movlw	S4_hi
		movff	PLUSW2,S_temp
		movf	S4_now,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t4_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t4_dl
		decfsz	Count,F			;rate countdown
		goto	t4_il
		goto	t5				;finished t4
t4_il	incf	S4_now,F
		goto	t4_1l

t4_dl	decfsz	Count,F			;rate countdown
		goto	t4_dl2
		goto	t5
t4_dl2	decf	S4_now,F
		goto	t4_1l

t4_done	bsf		Delflg,3		;set flag for delay cutoff	

;**********************************

;		servo 5

t5		btfss	Inchange,4		;has this input changed?
		goto	t5_noch			;no
		bcf		Inchange,4
		bsf		Outmsk,4		;enable output on servo 5
		bcf		Delflg,4		;clear delay mode on 5
		bcf		Gotflg,4		;not got there on 5
		movlw	Cutval
		movwf	Off5			;set delay timer for 5

t5_noch	btfsc	Gotflg,4		;already there?
		goto	t6				;next servo
		btfss	Intemp,4		;what is input direction?
		goto	t5_low			;input low
		movlw	S5_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t5_1h
		movlw	S5_low			;speed is max so update S5_now to limit
		movf	PLUSW2,W
		movwf	S5_now

t5_1h	movlw	S5_low
		movff	PLUSW2,S_temp
		movf	S5_now,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t5_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t5_dh
		decfsz	Count,F			;rate countdown
		goto	t5_ih
		goto	t6				;finished t5
t5_ih	incf	S5_now,F
		goto	t5_1h

t5_dh	decfsz	Count,F			;rate countdown
		goto	t5_dh2
		goto	t6				;finished t5
t5_dh2	decf	S5_now,F
		goto	t5_1h



t5_low	movlw	S5_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t5_1l
		movlw	S5_hi		;speed is max so update S1_now to limit
		movff	PLUSW2,S5_now

t5_1l	movlw	S5_hi
		movff	PLUSW2,S_temp
		movf	S5_now,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t5_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t5_dl
		decfsz	Count,F			;rate countdown
		goto	t5_il
		goto	t6				;finished t5
t5_il	incf	S5_now,F
		goto	t5_1l

t5_dl	decfsz	Count,F			;rate countdown
		goto	t5_dl2
		goto	t6
t5_dl2	decf	S5_now,F
		goto	t5_1l

t5_done	bsf		Delflg,4		;set flag for delay cutoff	

;**************************************************

;		servo 6

t6		btfss	Inchange,5		;has this input changed?
		goto	t6_noch			;no
		bcf		Inchange,5
		bsf		Outmsk,5		;enable output on servo 6
		bcf		Delflg,5		;clear delay mode on 6
		bcf		Gotflg,5		;not got there on 6
		movlw	Cutval
		movwf	Off6			;set delay timer for 6

t6_noch	btfsc	Gotflg,5		;already there?
		goto	t7				;next servo
		btfss	Intemp,5		;what is input direction?
		goto	t6_low			;input low
		movlw	S6_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t6_1h
		movlw	S6_low			;speed is max so update S6_now to limit
		movf	PLUSW2,W
		movwf	S6_now

t6_1h	movlw	S6_low
		movff	PLUSW2,S_temp
		movf	S6_now,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t6_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t6_dh
		decfsz	Count,F			;rate countdown
		goto	t6_ih
		goto	t7				;finished t6
t6_ih	incf	S6_now,F
		goto	t6_1h

t6_dh	decfsz	Count,F			;rate countdown
		goto	t6_dh2
		goto	t7				;finished t6
t6_dh2	decf	S6_now,F
		goto	t6_1h



t6_low	movlw	S6_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t6_1l
		movlw	S6_hi		;speed is max so update S6_now to limit
		movff	PLUSW2,S6_now

t6_1l	movlw	S6_hi
		movff	PLUSW2,S_temp
		movf	S6_now,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t6_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t6_dl
		decfsz	Count,F			;rate countdown
		goto	t6_il
		goto	t7				;finished t6
t6_il	incf	S6_now,F
		goto	t6_1l

t6_dl	decfsz	Count,F			;rate countdown
		goto	t6_dl2
		goto	t7
t6_dl2	decf	S6_now,F
		goto	t6_1l

t6_done	bsf		Delflg,5		;set flag for delay cutoff	

;********************************************************

;		servo 7

t7		btfss	Inchange,6		;has this input changed?
		goto	t7_noch			;no
		bcf		Inchange,6
		bsf		Outmsk,6		;enable output on servo 7
		bcf		Delflg,6		;clear delay mode on 7
		bcf		Gotflg,6		;not got there on 7
		movlw	Cutval
		movwf	Off7			;set delay timer for 7

t7_noch	btfsc	Gotflg,6		;already there?
		goto	t8				;next servo
		btfss	Intemp,6		;what is input direction?
		goto	t7_low			;input low
		movlw	S7_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t7_1h
		movlw	S7_low			;speed is max so update S7_now to limit
		movf	PLUSW2,W
		movwf	S7_now

t7_1h	movlw	S7_low
		movff	PLUSW2,S_temp
		movf	S7_now,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t7_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t7_dh
		decfsz	Count,F			;rate countdown
		goto	t7_ih
		goto	t8				;finished t7
t7_ih	incf	S7_now,F
		goto	t7_1h

t7_dh	decfsz	Count,F			;rate countdown
		goto	t7_dh2
		goto	t8				;finished t7
t7_dh2	decf	S7_now,F
		goto	t7_1h



t7_low	movlw	S7_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t7_1l
		movlw	S7_hi		;speed is max so update S7_now to limit
		movff	PLUSW2,S7_now

t7_1l	movlw	S7_hi
		movff	PLUSW2,S_temp
		movf	S7_now,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t7_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t7_dl
		decfsz	Count,F			;rate countdown
		goto	t7_il
		goto	t8				;finished t7
t7_il	incf	S7_now,F
		goto	t7_1l

t7_dl	decfsz	Count,F			;rate countdown
		goto	t7_dl2
		goto	t8
t7_dl2	decf	S7_now,F
		goto	t7_1l

t7_done	bsf		Delflg,6		;set flag for delay cutoff	

;*****************************************************

;		servo 8

t8		btfss	Inchange,7		;has this input changed?
		goto	t8_noch			;no
		bcf		Inchange,7
		bsf		Outmsk,7		;enable output on servo 8
		bcf		Delflg,7		;clear delay mode on 8
		bcf		Gotflg,7		;not got there on 8
		movlw	Cutval
		movwf	Off8			;set delay timer for 8

t8_noch	btfsc	Gotflg,7		;already there?
		goto	t9				;next servo
		btfss	Intemp,7		;what is input direction?
		goto	t8_low			;input low
		movlw	S8_rat2			;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t8_1h
		movlw	S8_low			;speed is max so update S8_now to limit
		movf	PLUSW2,W
		movwf	S8_now

t8_1h	movlw	S8_low
		movff	PLUSW2,S_temp
		movf	S8_now,W
		subwf	S_temp,W		;got there?
		btfsc	STATUS,Z
		goto	t8_done			;is there at set point
		btfss	STATUS,C		;which way?
		goto	t8_dh
		decfsz	Count,F			;rate countdown
		goto	t8_ih
		goto	t9				;finished t8
t8_ih	incf	S8_now,F
		goto	t8_1h

t8_dh	decfsz	Count,F			;rate countdown
		goto	t8_dh2
		goto	t9				;finished t8
t8_dh2	decf	S8_now,F
		goto	t8_1h



t8_low	movlw	S8_rat1		;get speed
		movf	PLUSW2,W
		call	spdconv
		movwf	Count
		btfss	STATUS,Z		;is rate zero?
		goto	t8_1l
		movlw	S8_hi		;speed is max so update S8_now to limit
		movff	PLUSW2,S8_now

t8_1l	movlw	S8_hi
		movff	PLUSW2,S_temp
		movf	S8_now,W
		subwf	S_temp,W			;got there?
		btfsc	STATUS,Z
		goto	t8_done			;is there at set point
		btfss	STATUS,C		;which way
		goto	t8_dl
		decfsz	Count,F			;rate countdown
		goto	t8_il
		goto	t9				;finished t8
t8_il	incf	S8_now,F
		goto	t8_1l

t8_dl	decfsz	Count,F			;rate countdown
		goto	t8_dl2
		goto	t9
t8_dl2	decf	S8_now,F
		goto	t8_1l

t8_done	bsf		Delflg,7		;set flag for delay cutoff	

;*******************************************



;*************************************************************************
;		here when all servos set

t9		btfss	Delflg,0		;set up for delays
		goto	t10
		bsf		Outmsk,0		;enable servo 1
		decfsz	Off1,F			;decrement delay 1
		goto	t10
		bcf		Delflg,0		;delay out
		btfsc	Cutmode,0		;no cutoff?
		bcf		Outmsk,0		;stop servo 1
		bsf		Gotflg,0		;no futher action till change

t10		btfss	Delflg,1	
		goto	t11
		bsf		Outmsk,1		;enable servo 2
		decfsz	Off2,F			;decrement delay 2
		goto	t11
		bcf		Delflg,1		;delay out
		btfsc	Cutmode,1		;no cutoff?
		bcf		Outmsk,1		;stop servo 2
		bsf		Gotflg,1		;no futher action till change

t11		btfss	Delflg,2		;set up for delays
		goto	t12
		bsf		Outmsk,2		;enable servo 3
		decfsz	Off3,F			;decrement delay 3
		goto	t12
		bcf		Delflg,2		;delay out
		btfsc	Cutmode,2		;no cutoff?
		bcf		Outmsk,2		;stop servo 3
		bsf		Gotflg,2		;no futher action till change

t12		btfss	Delflg,3		;set up for delays
		goto	t13
		bsf		Outmsk,3		;enable servo 4
		decfsz	Off4,F			;decrement delay 4
		goto	t13
		bcf		Delflg,3		;delay out
		btfsc	Cutmode,3		;no cutoff?
		bcf		Outmsk,3		;stop servo 4
		bsf		Gotflg,3		;no futher action till change


	

t13		btfss	Delflg,4		;set up for delays
		goto	t14
		bsf		Outmsk,4		;enable servo 5
		decfsz	Off5,F			;decrement delay 5
		goto	t14
		bcf		Delflg,4		;delay out
		btfsc	Cutmode,4		;no cutoff?
		bcf		Outmsk,4		;stop servo 5
		bsf		Gotflg,4		;no futher action till change

t14		btfss	Delflg,5		;set up for delays
		goto	t15
		bsf		Outmsk,5		;enable servo 6
		decfsz	Off6,F			;decrement delay 6
		goto	t15
		bcf		Delflg,5		;delay out
		btfsc	Cutmode,5		;no cutoff?
		bcf		Outmsk,5		;stop servo 6
		bsf		Gotflg,5		;no futher action till change

t15		btfss	Delflg,6		;set up for delays
		goto	t16
		bsf		Outmsk,6		;enable servo 7
		decfsz	Off7,F			;decrement delay 7
		goto	t16
		bcf		Delflg,6		;delay out
		btfsc	Cutmode,6		;no cutoff?
		bcf		Outmsk,6		;stop servo 7
		bsf		Gotflg,6		;no futher action till change

t16		btfss	Delflg,7		;set up for delays
		goto	t17
		bsf		Outmsk,7		;enable servo 8
		decfsz	Off8,F			;decrement delay 8
		goto	t17
		bcf		Delflg,7		;delay out
		btfsc	Cutmode,7		;no cutoff?
		bcf		Outmsk,7		;stop servo 7
		bsf		Gotflg,7		;no futher action till change


t17		return

;******************************************************************************
;
;		test routine when setting

test	tstfsz	NV2				;any test message?
		bra		test1			;no
		return
test1	movff	NV2,Temp		;save NV2
		bcf		Temp,7			;clear direction bit
		clrf	T_roll			;which servo?
		bsf		T_roll,0
test2	dcfsnz	Temp,f
		bra		test3
		rlncf	T_roll,F
		bra		test2
test3	movff	T_roll,EVtemp	;simulate real event
		clrf	EVtemp2
		clrf	Rx0d0
		btfss	NV2,7			;direction
		bsf		Rx0d0,0
		clrf	NV2
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

;		longer delay

ldely	movlw	.100
		movwf	Count2
ldely1	call	dely
		decfsz	Count2
		bra		ldely1
		
		return

;**********************************************************************

;		new enumeration scheme
;		here with enum array set
;
new_enum	movff	FSR1L,Fsr_tmp1Le	;save FSR1 just in case
			movff	FSR1H,Fsr_tmp1He 
			clrf	IDcount
			incf	IDcount,F			;ID starts at 1
			clrf	Roll
			bsf		Roll,0
			lfsr	FSR1,Enum0			;set FSR to start
here1		incf	INDF1,W				;find a space
			bnz		here
			movlw	8
			addwf	IDcount,F
			incf	FSR1L
			bra		here1
here		movf	Roll,W
			andwf	INDF1,W
			bz		here2
			rlcf	Roll,F
			incf	IDcount,F
			bra		here
here2		movlw	.99					;limit to ID
			cpfslt	IDcount
			call	segful				;segment full
			movff	Fsr_tmp1Le,FSR1L	;
			movff	Fsr_tmp1He,FSR1H 
			return

segful		movlw	7		;segment full, no CAN_ID allocated
			call	errsub
			setf	IDcount
			bcf		IDcount,7
			return

;***********************************************************************

readNVs		btfss	Mode,1		;in FLiM?
			return
			movlw	LOW	NVstart
			addwf	Rx0d3,W		;add index
			movwf	EEADR
			decf	EEADR,F		;index starts at 1, buffer at 0
			call	eeread
			movwf	Tx1d4		;NV val to transmit buffer
			movff	Rx0d3,Tx1d3	;transfer index
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
			movf	Rx0d3,W
			sublw	4				;1 to 4 are not SVs
			bc		notSV
			
			lfsr	FSR2,SV1
			movlw	5
			subwf	Rx0d3,W
			movff	Rx0d4,PLUSW2	;put in ram
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
			btfsc	Rx0d3,0			;on or off state
			clrf	Intemp
			return
notSV		lfsr	FSR2,NV1
			decf	Rx0d3,W
			movff	Rx0d4,PLUSW2
			movff	NV1,Cutmode		;cutoff mode. Cutoff if bit set.
			return

;***************************************************************

			ORG 0x3000		;speed table
	
Spdtbl		db	.0,.11		;fastest. These values could be modified if wanted
			db	.9,.7
			db	.5,.4
			db	.3,.2
;************************************************************************		
	ORG 0xF00000			;EEPROM data. Defaults
	
CANid	de	B'01111111',0	;CAN id default and module status
NodeID	de	0,0			;Node ID
ENindex	de	0,0		;points to next available EN number (only lo byte used)
					;value actually stored in ENindex+1

	ORG 0xF00006

ENstart	;event numbers stored here. Room for 32 four byte events.

		
	ORG 0xF00086
	
		;event variables stored here. set to defaults initially
		
EVstart	de		0,0,0,0,0,0,0,0	;allows for 2 EV per event
	de		0,0,0,0,0,0,0,0
	de		0,0,0,0,0,0,0,0
	de		0,0,0,0,0,0,0,0
	de		0,0,0,0,0,0,0,0	
	de		0,0,0,0,0,0,0,0
	de		0,0,0,0,0,0,0,0
	de		0,0,0,0,0,0,0,0


		;allows for 4 SVs per servo

NVstart	de 0xFF,0,0,0				;4 NVs for node variables, NV1 is cutoff mode

SVstart	de	.127,.127,0,0
		de	.127,.127,0,0	
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0
		de	.127,.127,0,0



		
		;slowest. 2 is the slowest allowed (corresponds to one decrement per cycle)

		ORG	0xF000FE
		de		0,0		;for boot load
		end


