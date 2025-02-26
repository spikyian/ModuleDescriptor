   	TITLE		"Source for CAN accessory decoder using CBUS"
; filename CANSOL_v1b.asm
; This is for 25K80 PIC.
; use with CANSOL PCB

; For the 12V version, RA1 now drives the doubler. RA0 is the CDU cutoff.

; CANSOL is a renamed version of CANACC4K_2_v1c


; Rev v4a	Changed v2p for 25K80. Uses ECAN and newevhndler_v1b (Mike B 20/07/14)
;			Beta 101. Tested in SLiM and FLiM. Seems OK. 26/07/14.
; New file for the 12V K series. CANACC4K_2_v1a. Starts again at 1a (11/10/14)
; Only change from previous is the doubler drive is now on RA2 (pin 3)so pin 6 can have the capacitor. 
; Rev v1b	Correction for PB pin on kit PCB (now PA2). Remove Beta version. 15/02/15 MB
; Rev v1c	Correction for bootloader  15/02/15  MB 
; Rev 1a	Now called CANSOL and rev v1a  (26/02/15  MB) Uses cbusdefs8h.inc.
; Rev 1b	Current limiter turned on during set-up. CE_BIT. (MB 22/10/15)


;end of comments for CANSOL

;	
; Assembly options
	LIST	P=18F25K80,r=hex,N=75,C=120,T=ON

	include		"p18f25K80.inc"
	include		"cbusdefs8h.inc"


;set config registers

;
	CONFIG	FCMEN = OFF, FOSC = HS1, IESO = OFF, PLLCFG = ON
	CONFIG	PWRTEN = ON, BOREN = SBORDIS, BORV=0, SOSCSEL = DIG
	CONFIG	WDTEN=OFF
	CONFIG	MCLRE = ON, CANMX = PORTB
	CONFIG	BBSIZ = BB1K 
	
	CONFIG	XINST = OFF,STVREN = ON,CP0 = OFF
	CONFIG	CP1 = OFF, CPB = OFF, CPD = OFF,WRT0 = OFF,WRT1 = OFF, WRTB = OFF
	CONFIG 	WRTC = OFF,WRTD = OFF, EBTR0 = OFF, EBTR1 = OFF, EBTRB = OFF
	



;	processor uses 4 MHz resonator but clock is 16 MHz.

;**************************************************************************
;definitions

S_PORT		equ	PORTA	;for PB
SL_PORT		equ PORTB	;for SLiM switches except UNLEARN	
CE_BIT		equ 0	;CANACC2 Charger Enable
S_BIT		equ	2	;PB input
CD_BIT		equ 1	;CANACC2 Charger Doubler Drive (change)
UNLEARN		equ	5	;setup jumper in port A (unlearn)
POLPORT 	equ PORTB
POL			equ 5	
UNLPORT 	equ PORTA

LED_PORT	equ	PORTB
LEARN 		equ 4	;input bits in port B
LED2		equ	7	;PB7 is the green LED on the PCB
LED1		equ 6	;PB6 is the yellow LED on the PCB

CANTX		equ 2	;CAN transmit on PORTB 2
CANRX		equ	3	;CAN receive on PORTB 3



;Defaults
DFFTIM	equ	.5			; Default fire time (units of 10mS)
DFRDLY	equ	.50			; Default recharge delay (units of 10mS)
DFFDLY	equ	.0			; Default fire delay (units of 10mS)
DFCDLY	equ	.3			; Default CANACC2 Charge pump enable delay (units of 10mS)	

CHGFREQ equ .100		; CANACC2 Charge pump frequency (50,100 or 200Hz only)
LPINTI	equ	CHGFREQ*2	; Low Priority Interrupts per second
TMR1CN  equ 10000-(.4000000/LPINTI)	;Timer 1 count (counts UP)

CMD_ON	equ	0x90	;on event
CMD_OFF	equ	0x91	;off event
CMD_REQ	equ	0x92
SCMD_ON	equ	0x98
SCMD_OFF	equ	0x99
SCMD_REQ	equ	0x9A
OPC_PNN	equ 0xB6	;reply to QNN


EN_NUM  equ	.128		;number of allowed events
EV_NUM  equ 2		;number of allowed EVs per event
CANSOL_ID equ .34


#define EVBLK_SZ .16

BLK_SZ equ EVBLK_SZ		;used in new event handler
EVENT_SZ equ BLK_SZ - 8 ;
ERASE_SZ equ .64
HASH_SZ	equ	.32
HASH_MASK equ HASH_SZ - 1
EVSPER_BLK equ ERASE_SZ/BLK_SZ




CONSUMER	equ	1
PRODUCER	equ	2
COMBI		equ	3

Modstat equ 1		;address in EEPROM

MAN_NO      equ MANU_MERG    ;manufacturer number
MAJOR_VER   equ 1
MINOR_VER   equ "B"
MODULE_ID   equ MTYP_CANSOL ; id to identify this type of module
EVT_NUM     equ EN_NUM           ; Number of events
EVperEVT    equ EV_NUM           ; Event variables per event
NV_NUM      equ .16        ; Number of node variables
NODEFLGS    equ B'00001001' ; Node flags  Consumer=Yes, Producer=No, 
							;FliM=No, Boot=YES
CPU_TYPE    equ P18F25K80
BETA_VER	equ 0			;release version


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




;****************************************************************
;	define RAM storage

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
;************************************************************
	
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
	TempCANCON
	TempCANSTAT
	TempINTCON
	TempECAN

	Saved_Fsr1H
	Saved_Fsr1L
	Datmode			;flag for data waiting 
	Count			;counter for loading
	Count1
	Count2
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
	
	ev_opc
	ev0
	ev1
	ev2
	ev3
	
	EVidx		; EV index from learn cmd
	EVdata		; EV data from learn cmd
	ENidx		; event index from commands which access events by index
	CountFb0	; counters used by Flash handling
	CountFb1	


	Cmdtmp		;command temp for number of bytes in frame jump table
	
	DNindex		;holds number of allowed DNs
	Match		;match flag

	ENcount		;which EN matched
	ENcount1
	EVtemp		;holds current EV
	EVtemp1	
	EVtemp2	
	IDcount		;used in self allocation of CAN ID.
	Latcount
	Keepcnt		;keepalive count
	Mode		;for FLiM / SLiM etc
	Mask
	Shift
	Shift1
	
	Temp			;temps
	Temp1
	CanID_tmp	;temp for CAN Node ID
	IDtemph		;used in ID shuffle
	IDtempl
	NN_temph		;node number in RAM
	NN_templ
	ENtemp1			;number of events
	Dlc				;data length for CAN TX
	
	
	
	
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
		
	;***************************************************************
	; new varables at Rev j
	OpTimr		; Output timer (countdown)
	OpTrig		; Output channel trigger mask
	OpFlag		; Output flags	Timout		;used in timer routines
	; new variables at Rev 2_2p
	OpFdly		; Output fire delay
	OpCdly		; Output charge delay
	LPintc		; LPint Counter
	
	
	;*************************************************************
	T1a			;timer registers for each output
	T1b
	T2a
	T2b
	T3a
	T3b
	T4a
	T4b
	; The following added at Rev j
	Trchg		; Recharge Time. Must follow T4b
	; The following added at Rev 2_2o
	Tfdly		; Fire delay. Must follow Trchg
	; The following added at Rev 2_2p
	Tcdly		; Charge delay. Must follow Tfdly
	;End of timer values

	Opm1a		; Mask for output 1a
	Opm1b		; Mask for output 1b
	Opm2a		; Mask for output 2a
	Opm2b		; Mask for output 2b
	Opm3a		; Mask for output 3a
	Opm3b		; Mask for output 3b
	Opm4a		; Mask for output 4a
	Opm4b		; Mask for output 4b
	;****************************************************************

	Roll		;rolling bit for enum
	
	Fsr_tmp1Le	;temp store for FSR1
	Fsr_tmp1He 

	

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
	
	
		
	ENDC

;****************************************************************
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


;	processor uses  4 MHz. Resonator with HSPLL to give a clock of 16MHz

;********************************************************************************



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
;	bcf 	TRISB, CANTX 	; Set the TX pin to output 
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

	movlw	CAN_CIOCON	;	Set IO
	movwf	CIOCON	
	
	clrf	CANCON	; Enter Normal mode
;	movlw	B'00001110'
;	movwf	ADCON1
	bcf	TRISB,7
	bcf	TRISB,6
	bsf	PORTB,7		;gren LED on
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

f	

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
	bsf		CANTX_CD_BIT
	btfss	CAN_CD_BIT 
	bcf		CANTX_CD_BIT
	bsf		TXB0CON, TXREQ
    bra	 	_CANMain	; Setup the command bit

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

;****************************************************************
;
;		start of program code

		ORG		0800h
loadadr
		nop						;for debug
		goto	setup

		ORG		0808h
		goto	hpint			;high priority interrupt

		ORG		0810h			;node type parameters
myName	db		"SOL     "

		ORG		0818h	
		goto	lpint			;low priority interrupt
		
		ORG		0820h

nodeprm     db  MAN_NO, MINOR_VER, MODULE_ID, EVT_NUM, EVperEVT, NV_NUM 
			db	MAJOR_VER,NODEFLGS,CPU_TYPE,PB_CAN    ; Main parameters
            dw  RESET_VECT     ; Load address for module code above bootloader
            dw  0           ; Top 2 bytes of 32 bit address not used
			dw	0
			dw	0
			db	0,BETA_VER
sparprm     fill 0,prmcnt-$ ; Unused parameter space set to zero

PRMCOUNT    equ sparprm-nodeprm ; Number of parameter bytes implemented

             ORG 0838h

prmcnt      dw  PRMCOUNT    ; Number of parameters implemented
nodenam     dw  myName      ; Pointer to module type name
            dw  0 ; Top 2 bytes of 32 bit address not used


PRCKSUM     equ MAN_NO+MINOR_VER+MODULE_ID+EVT_NUM+EVperEVT+NV_NUM+MAJOR_VER+NODEFLGS+CPU_TYPE+PB_CAN+HIGH myName+LOW myName+HIGH loadadr+LOW loadadr+PRMCOUNT +BETA_VER

cksum       dw  PRCKSUM     ; Checksum of parameters



;*******************************************************************

		ORG		0840h			;start of CANSOL program
;	
;
;		high priority interrupt. Used for ECAN overflow and busy mechanism.

hpint	nop		;for debug
				
		movlb	.15
		btfss	PIR5,FIFOWMIF,0		;is it FIFO interrupt?
		bra		no_fifo
		bcf		PIR5,FIFOWMIF,0		;clear FIFO flag
		bcf		PIR5,4,0			;clear busy frame flag
		
		bsf		TXB0CON,TXREQ		;send busy frame
		bcf		PIE5,FIFOWMIE,0		;disable FIFO interrupt 
		movlb	.14
		bsf		TXBIE,TXB0IE		;enable TXB2 IRQ for busy frame sent
		bsf		PIE5,4,0			;enable IRQ for busy frame sent
		movlb	0
		retfie	1
	
no_fifo	bcf		PIR5,4,0			;clear busy frame flag
		movlb	.14
		bcf		TXBIE,TXB0IE		;no busy frame IRQ
		movlb	0
		bsf		PIE5,FIFOWMIE		;wait for next FIFO IRQ

		retfie	1




;**************************************************************
;
;
;		low priority interrupt. Used by output timer overflow. Every 10 millisecs.
;	

lpint	movwf	W_tempL				;used for output timers
		movff	STATUS,St_tempL
		movff	BSR,Bsr_tempL

		movlw	LOW TMR1CN			;Timer 1 lo byte. (adjust if needed)
		movwf	TMR1L				;reset timer 1
		clrf	PIR1				;clear all timer flags
		movf	OpCdly,W			;Is charge delay active?
		bnz		lpint0				;Don't toggle doubler drive then
		btg		S_PORT,CD_BIT		;doubler drive

lpint0
#if CHGFREQ > .50
		incf	LPintc,F			;Increment count
		btfsc	LPintc,0			;skip alternate interrupts
		bra		lpend				;all done
#if CHGFREQ >= .200
		btfsc	LPintc,1			;skip alternate interrupts
		bra		lpend				;all done
#endif
#endif
		
; We get here once every 10mS, no matter what interrupt rate is in use
; Countdown fire delay (time between an event received and an output firing)

		movf	OpFdly,W		; Get fire delay, is it zero?
		bz		lpint1			; Inactive, check for next operation
		decf	OpFdly,F		; Decrement fire delay
lpint1

; Countdown charge delay (time between an output firing and the doubler restarting)

		movf	OpCdly,W		; Get fire delay, is it zero?
		bz		lpint2			; Inactive, check for next operation
		decf	OpCdly,F		; Decrement fire delay
lpint2

; Control PORTC (trigger) outputs

		movf	OpTimr,W		; Get timer value, is it zero?
		bz		donext			; Inactive, check for next operation
		decfsz	OpTimr,F		; Decrement timer, skip if zero (expired)
		bra		lpend			; Timer not expired, all done for now

; Process expired timer

		bsf		S_PORT,CE_BIT	; Enable Charging
		movf	OpFlag,W		; Get output flag to W and set/reset Z
		bz		donext			; Not recharging, do next output
		andwf	PORTC,F			; Turn off last outputs
		clrf	OpFlag			; Clear output flag
		movf	Trchg,W			; Get recharge time
		bz		donext			; None, do next output
		movwf	OpTimr			; Store timer value
		bra		lpend

; Find next bit to trigger

donext	movf	OpTrig,F		; Check trigger
		bz		lpend			; All done
		movf	OpFdly,F		; Check fire delay
		bnz		lpend			; Wait until zero
		btfsc	OpTrig,0		; Do output 1a?
		bra		trig1a
		btfsc	OpTrig,1		; Do output 1b?
		bra		trig1b
		btfsc	OpTrig,2		; Do output 2a?
		bra		trig2a
		btfsc	OpTrig,3		; Do output 2b?
		bra		trig2b
		btfsc	OpTrig,4		; Do output 3a?
		bra		trig3a
		btfsc	OpTrig,5		; Do output 3b?
		bra		trig3b
		btfsc	OpTrig,6		; Do output 4a?
		bra		trig4a
		bra		trig4b			; Do output 4b

; Trigger output 1a

trig1a	bcf		OpTrig,0		; Clear trigger bit
		comf	Opm1b,W			; Get inverted mask for other output into W
		andwf	PORTC,F			; Set other output off
		movf	Opm1a,W			; Get mask for active output into W
		iorwf	PORTC,F			; Active pair on
		movf	T1a,W			; Get timer into W
		bz		lpend			; If timer is zero, then all done
		movwf	OpTimr			; Save timer value
		comf	Opm1a,W			; Get inverted mask for active output into W
		movwf	OpFlag			; Save in flag byte
		bra		trig

; Trigger output 1b

trig1b	bcf		OpTrig,1		; Clear trigger bit
		comf	Opm1a,W			; Get inverted mask for other output into W
		andwf	PORTC,F			; Set other output off
		movf	Opm1b,W			; Get mask for active output into W
		iorwf	PORTC,F			; Active pair on
		movf	T1b,W			; Get timer into W
		bz		lpend			; If timer is zero, then all done
		movwf	OpTimr			; Save timer value
		comf	Opm1b,W			; Get inverted mask for active output into W
		movwf	OpFlag			; Save in flag byte
		bra		trig

; Trigger output 2a

trig2a	bcf		OpTrig,2		; Clear trigger bit
		comf	Opm2b,W			; Get inverted mask for other output into W
		andwf	PORTC,F			; Set other output off
		movf	Opm2a,W			; Get mask for active output into W
		iorwf	PORTC,F			; Active pair on
		movf	T2a,W			; Get timer into W
		bz		lpend			; If timer is zero, then all done
		movwf	OpTimr			; Save timer value
		comf	Opm2a,W			; Get inverted mask for active output into W
		movwf	OpFlag			; Save in flag byte
		bra		trig

; Trigger output 2b

trig2b	bcf		OpTrig,3		; Clear trigger bit
		comf	Opm2a,W			; Get inverted mask for other output into W
		andwf	PORTC,F			; Set other output off
		movf	Opm2b,W			; Get mask for active output into W
		iorwf	PORTC,F			; Active pair on
		movf	T2b,W			; Get timer into W
		bz		lpend			; If timer is zero, then all done
		movwf	OpTimr			; Save timer value
		comf	Opm2b,W			; Get inverted mask for active output into W
		movwf	OpFlag			; Save in flag byte
		bra		trig

; Trigger output 3a

trig3a	bcf		OpTrig,4		; Clear trigger bit
		comf	Opm3b,W			; Get inverted mask for other output into W
		andwf	PORTC,F			; Set other output off
		movf	Opm3a,W			; Get mask for active output into W
		iorwf	PORTC,F			; Active pair on
		movf	T3a,W			; Get timer into W
		bz		lpend			; If timer is zero, then all done
		movwf	OpTimr			; Save timer value
		comf	Opm3a,W			; Get inverted mask for active output into W
		movwf	OpFlag			; Save in flag byte
		bra		trig

; Trigger output 3b

trig3b	bcf		OpTrig,5		; Clear trigger bit
		comf	Opm3a,W			; Get inverted mask for other output into W
		andwf	PORTC,F			; Set other output off
		movf	Opm3b,W			; Get mask for active output into W
		iorwf	PORTC,F			; Active pair on
		movf	T3b,W			; Get timer into W
		bz		lpend			; If timer is zero, then all done
		movwf	OpTimr			; Save timer value
		comf	Opm3b,W			; Get inverted mask for active output into W
		movwf	OpFlag			; Save in flag byte
		bra		trig

; Trigger output 4a

trig4a	bcf		OpTrig,6		; Clear trigger bit
		comf	Opm4b,W			; Get inverted mask for other output into W
		andwf	PORTC,F			; Set other output off
		movf	Opm4a,W			; Get mask for active output into W
		iorwf	PORTC,F			; Active pair on
		movf	T4a,W			; Get timer into W
		bz		lpend			; If timer is zero, then all done
		movwf	OpTimr			; Save timer value
		comf	Opm4a,W			; Get inverted mask for active output into W
		movwf	OpFlag			; Save in flag byte
		bra		trig

; Trigger output 4b

trig4b	bcf		OpTrig,7		; Clear trigger bit
		comf	Opm4a,W			; Get inverted mask for other output into W
		andwf	PORTC,F			; Set other output off
		movf	Opm4b,W			; Get mask for active output into W
		iorwf	PORTC,F			; Active pair on
		movf	T4b,W			; Get timer into W
		bz		lpend			; If timer is zero, then all done
		movwf	OpTimr			; Save timer value
		comf	Opm4b,W			; Get inverted mask for active output into W
		movwf	OpFlag			; Save in flag byte
		
trig	bcf		S_PORT,CE_BIT	; Disable Charging
		movf	OpTimr,W		; Get output timer setting
		addwf	Tcdly,W			; Add a bit more delay
		movwf	OpCdly			; Set charge delay
		
lpend	movff	Bsr_tempL,BSR
		movf	W_tempL,W
		movff	St_tempL,STATUS	
		retfie	
						

;*********************************************************************


;	main waiting loop

main	

		btfsc	Datmode,0		;busy?
		bra		main0
		btfsc	COMSTAT,7		;look for CAN input. 
		bra		getcan
		bra		main0

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
		bra		main0

no_can	bcf		RXB0CON,RXFUL	;clear ECAN


main0	btfsc	Mode,1			;is it SLiM?
		bra		mainf			;no

mains							;is SLiM

		btfss	PIR2,TMR3IF		;flash timer overflow?
		bra		nofl_s			;no SLiM flash
		btg		LED_PORT,LED2	;toggle green LED
		bcf		PIR2,TMR3IF
nofl_s	bra		noflash				;main1
		
; here if FLiM mde

mainf	btfss	INTCON,TMR0IF		;is it flash?
		bra		noflash
		btfss	Datmode,2
		bra		nofl1
		
		btg		LED_PORT,LED1		;flash yellow LED
		
nofl1	bcf		INTCON,TMR0IF
		btfss	Datmode,3		;running mode
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
wait	decfsz	Count2
		goto	wait
		btfss	Datmode,2
		bra		wait2
		btfss	INTCON,TMR0IF		;is it flash?
		bra		wait2
		btg		LED_PORT,LED1		;flash LED
		bcf		INTCON,TMR0IF
wait2	decfsz	Count1
		goto	wait
		btfsc	S_PORT,S_BIT
		bra		main4			;not held long enough
		decfsz	Count
		goto	wait
		btfss	Mode,1			;is it in FLiM?
		bra		go_FLiM
		clrf	Datmode			;back to virgin

		bcf		LED_PORT,LED1	;yellow off
		
		bsf		LED_PORT,LED2	;Green LED on
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
		call	eewrite				;mode to FLiM in EEPROM
		bsf		Mode,1				;to FLiM
		call	self_en				;self enumerate routine
		bcf		Datmode,1
		call	nnack				;send request for NN
		bsf		Datmode,2

		bra		main1


main4	
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
		call	self_en
		bcf		Datmode,1
		call	nnack
		bra		main1

main3	btfss	Datmode,1		;setup mode ?
		bra		main1

		bcf		Datmode,1		;out of setup
		bsf		Datmode,2		;wait for NN
		bra		main1			;continue normally

go_FLiM	bsf		Datmode,1		;FLiM setup mode
		bcf		LED_PORT,LED2	;green off
		bra		wait1
		
		

; common to FLiM and SLiM		
	
	
main1	
		btfss	Datmode,0		;any new CAN frame received?
		bra		main
		
		bra		packet			;yes
		
;		These are here as branch was too long

unset	
		btfss	Datmode,4			;Roger's mod
		bra		main2				;no error message on OPC 0x95
		bsf		Datmode,5
		bra		learn2

readEV	btfss	Datmode,4
		bra		main2			;prevent error message
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
		movff	ev2,ENidx
		movff	ev3,EVidx
		call	evsend
		bra		main2
		
notNNx	goto	notNN

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
		
go_on_x goto	go_on

params	btfss	Datmode,2		;only in setup mode
		bra		main2
		call	parasend
		bra		main2
		
setNV	call	thisNN
		sublw	0
		bnz		notNNx			;not this node
		call	putNV
		call	timload			;Reload NV's into RAM
		bra		main2

short	clrf	ev0
		clrf	ev1
		bra		go_on

short1	goto	short			;branches too long
evns	goto	evns1		
setNVx	goto	setNV
readNVx	goto	readNV
readENx	goto	readEN
readEVx goto	readEV
rden_x	goto	rden

		
;********************************************************************
								;main packet handling is here
								;add more commands for incoming frames as needed
		
packet	movlw	CMD_ON  ;only ON, OFF  events supported
		subwf	ev_opc,W	
		bz		go_on_x
		movlw	CMD_OFF
		subwf	ev_opc,W
		bz		go_on_x
		
		movlw	SCMD_ON
		subwf	ev_opc,W
		bz		short
		movlw	SCMD_OFF
		subwf	ev_opc,W
		bz		short
		
		movlw	0x5C			;reboot
		subwf	ev_opc,W
		bz		reboot
		movlw	0x73
		subwf	ev_opc,W
		bz		para1a			;read individual parameters
	
		
		movlw	0x0d			; QNN
		subwf	ev_opc,W
		bz		doQnn
		movlw	0x10			
		subwf	ev_opc,W
		bz		params			;read node parameters
		movlw	0x11
		subwf	ev_opc,W
		bz		name			;read module name
		btfss	Mode,1			;FLiM?
		bra		main2
		movlw	0x42			;set NN on 0x42
		subwf	ev_opc,W
		bz		setNN
		movlw	0x53			;set to learn mode on 0x53
		subwf	ev_opc,W
		bz		setlrn1		
		movlw	0x54			;clear learn mode on 0x54
		subwf	ev_opc,W
		bz		notlrn1
		movlw	0x55			;clear all events on 0x55
		subwf	ev_opc,W
		bz		clrens1
		movlw	0x56			;read number of events left
		subwf	ev_opc,W
		bz		rden_x
	
		movlw	0x96			;set NV
		subwf	ev_opc,W
		bz		setNVx
		movlw	0x5D			;re-enumerate
		subwf	ev_opc,W
		bz		enum1
		movlw	0x71			;read NVs
		subwf	ev_opc,W
		bz		readNVx
		movlw	0x75			;force new CAN_ID
		subwf	ev_opc,W
		bz		newID1
		movlw	0x96			;set NV
		subwf	ev_opc,W
		bz		setNVx
		movlw	0xD2			;is it set event?
		subwf	ev_opc,W
		bz		chklrn1			;do learn
		movlw	0x95			;is it unset event
		subwf	ev_opc,W			
		bz		unset1
		movlw	0xB2			;read event variables
		subwf	ev_opc,W
		bz		readEVx
	
		movlw	0x57			;is it read events
		subwf	ev_opc,W
		bz		readENx
		movlw	0x72
		subwf	ev_opc,W
		bz		readENi1			;read event by index
		movlw	0x58
		subwf	ev_opc,W
		bz		evns
		movlw	0x9C				;read event variables by EN#
		subwf	ev_opc,W
		bz		reval1

		bra	main2				;end of events lookup

enum1	goto	enum
clrens1	goto	clrens	
newID1	goto	newID
notlrn1	goto	notlrn
chklrn1	goto	chklrn
setlrn1	goto	setlrn
unset1	goto	unset
readENi1 goto	readENi
reval1	goto	reval

		bra		main2


		
reboot	btfss	Mode,1			;FLiM?
		bra		reboots
		call	thisNN
		sublw	0
		bnz		notNN
		
reboot1	movlw	0xFF
		movwf	EEADR
		movlw 	0x3F
		movwf	EEADRH
		movlw	0xFF
		call	eewrite			;set last EEPROM byte to 0xFF
		reset					;software reset to bootloader

reboots
		movf	ev0,W
		addwf	ev1,W
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
		movf	ev0,W
		addwf	ev1,W
		bnz		notNN
		call	para1rd
		bra		main2
			
main2	bcf		RXB0CON,RXFUL
		bcf		Datmode,0
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
		bcf		LED_PORT,LED2	
		bsf		LED_PORT,LED1	;LED ON
		bra		main2

newID	call	thisNN
		sublw	0
		bnz		notNN
		movff	ev2,IDcount

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
	
setlrn	call	thisNN
		sublw	0
		bnz		notNN
		bsf		Datmode,4
		bsf		LED_PORT,LED1			;LED on
		bra		main2

notlrn	call	thisNN
		sublw	0
		bnz		notNN
		bcf		Datmode,4
notln1		;leave in learn mode
		bcf		Datmode,5
		bra		main2
clrens	call	thisNN
		sublw	0
		bnz		notNN
		btfss	Datmode,4
		bra		clrerr
		call	initevdata
		movlw	0x59
		call	nnrel		;send WRACK
		bra		notln1
		
notNN	bra		main2

clrerr	movlw	2			;not in learn mode
		goto	errmsg

		
chklrn	btfss	Datmode,4		;is in learn mode?
		bra		main2			;j if not
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
		call	enrdi
		bra		main2



enum	call	thisNN
		sublw	0
		bnz		notNN1
		call	self_en
		movlw	0x52
		call	nnrel			;send confirm frame
		movlw	B'00001000'		;back to normal running
		movwf	Datmode
		goto	main2
notNN1	goto	notNN
	
	; copy event data to safe buffer
copyev	movff	RXB0D0, ev_opc
		movff	RXB0D1, ev0
		movff	RXB0D2, ev1
		movff	RXB0D3, ev2
		movff	RXB0D4, ev3
		movff	RXB0D5, EVidx		; only used by learn and some read cmds
		movff	RXB0D6, EVdata		; only used by learn cmd
		return


	

go_on	
		btfss	Mode,1			;FLiM?
		bra		go_on_s
		
go_on1	call	enmatch
		sublw	0
		bz		do_it
		bra		main2			;not here

go_on_s	btfss	SL_PORT,LEARN
		bra		learn2			;is in learn mode
		bra		go_on1

paraerr	movlw	3				;error not in setup mode
		goto	errmsg



readNV	call	thisNN
		sublw	0
		bnz		notNN			;not this node
		call	getNV
		bra		main2

readEN	call	thisNN
		sublw	0
		bnz		notNN
		call	enread
		bra		main2
		
do_it
		call	rdfbev
		movff	POSTINC0, EVtemp
		movff	POSTINC0, EVtemp2
		call	ev_set			;do it -  for consumer action
		bra		main2
			
rden1	call	thisNN
		sublw	0
		bnz		notNN
		call	rdFreeSp
		bra		main2		
		
learn1
		bra		learn2
		
learn2	call	enmatch			;is it there already?
		sublw 	0
		bz		isthere
		btfsc	Mode,1			;FLiM?
		bra		learn3
		btfss	UNLPORT,UNLEARN	;if unset and not here
		bra		l_out2			;do nothing else 
		call	learn			;put EN into stack and RAM
		sublw	0
		bz		lrnend
		movlw	4
		goto	errmsg1			;too many
		
		;here if FLiM
learn3	btfsc	Datmode,6		;read EV?
		bra		rdbak1			;not here
		btfsc	Datmode,5		;if unset and not here
		bra		l_out1			;do nothing else 
		
learn4	call	learn			;put EN into stack and RAM
		sublw	0
		bz		lrnend

		movlw	4
		goto	errmsg2	
		
rdbak1	movlw	5				;no match
		goto	errmsg2
		
lrnend
		bra		go_on1
								
isthere
		btfsc	Mode,1
		bra		isthf			;j if FLiM mode
		btfsc	UNLPORT,UNLEARN	;is it here and unlearn...
		bra		dolrn
		call	unlearn			;...goto unlearn	
		bra		l_out1
			
isthf
		btfsc	Datmode, 6		;is it read back
		bra		rdbak
		btfss	Datmode,5		;FLiM unlearn?
		bra		dolrn
		call	unlearn
		movlw	0x59
		call	nnrel
		bra		l_out1
		
dolrn
		call 	learn
		bra		lrnend
		
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
;		bcf		LED_PORT,LED2
l_out1	bcf		Datmode,6
l_out2	bcf		Datmode,0
		
		clrf	PCLATH
		goto	main2

noEV	movlw	6				;invalid EV#
		goto	errmsg2

;***************************************************************************
;		main setup routine
;*************************************************************************

setup	lfsr	FSR0, 0			; clear 128 bytes of ram
nxtram	clrf	POSTINC0
		tstfsz	FSR0L
		bra		nxtram

		movlb	.15
		clrf	ANCON0			;disable A/D
		clrf	ANCON1
		clrf	CM1CON			;disable comparator
		clrf	CM2CON
		clrf	INTCON2			
		bcf		INTCON2,7		;weak pullups on
		setf	WPUB			;pullups on portb
		movlb	0	
		
		clrf	INTCON			;no interrupts yet
		
		
		;port settings will be hardware dependent. RB2 and RB3 are for CAN.
		;set S_PORT and S_BIT to correspond to port used for setup.
		;rest are hardware options
		
	
		movlw	B'00100100'		;Port A. PA2 is setup PB
		movwf	TRISA			;
		movlw	B'00111011'		;RB2 = CANTX, RB3 = CANRX, 	
								;RB6,7 for debug and ICSP and LEDs
								;PORTB has pullups enabled on inputs
		movwf	TRISB
		bcf		LED_PORT,LED1	;LEDs off
		bcf		LED_PORT,LED2
		bsf		PORTB,2			;CAN recessive
		movlw	B'00000000'		;Port C  set to outputs.
		movwf	TRISC
		clrf	PORTC
		bsf		S_PORT,CE_BIT	;turn charger on (MB)
	
		
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

	
		
		


		
mskload	lfsr	0,RXM0SIDH		;Clear masks, point to start
mskloop	clrf	POSTINC0		
		movlw	LOW RXM1EIDL		;end of masks
		incf	WREG
		cpfseq	FSR0L
		bra		mskloop
		
		clrf	CANCON			;out of CAN setup mode
		clrf	CCP1CON

		movlw	B'10000100'
		movwf	T0CON			;set Timer 0 for LED flash
		movlw	0
		movwf	T1GCON			;clear timer 1 gate control
		movlw	B'00000010'		;Timer 1 control.16 bit write
		movwf	T1CON			;Timer 1 is for output duration
		movlw	HIGH TMR1CN
		movwf	TMR1H			;set timer hi byte
		bsf		T1CON,TMR1ON	;start timer
		
		clrf	Tx1con
		movlw	B'00000000'
		movwf	IPR5			;high priority CAN RX and Tx error interrupts disabled(for now)
		clrf	IPR1			;all peripheral interrupts are low priority
		clrf	IPR2
		clrf	PIE2
		movlw	B'00000001'
		movwf	PIE1			;enable interrupt for timer 1


;next segment required
		
		movlw	B'00000001'
		movwf	IDcount			;set at lowest value for starters
		
		
		clrf	INTCON3			;
		clrf	PIR1
		clrf	PIR2
		clrf	PIR5
		
		movlw	B'00010001'
		movwf	IPR5			;FIFOHWM and TXB are high priority
		movlw	B'00010001'
		movwf	PIE5			;FIFOHWM IRQ, and TXB complete only

		movlb	.15				;set  frame for busy in TXB0
		clrf	TXB0SIDH
		clrf	TXB0SIDL
		clrf	TXB0DLC
		movlb	0
	
		
	
		bcf		RXB0CON,RXFUL		;ready for next
		bcf		COMSTAT,RXB0OVFL	;clear overflow flags if set
		bcf		COMSTAT,RXB1OVFL
		clrf	PIR5			;clear all ECAN flags

		clrf	EEADRH			;upper EEPROM page to 0
		call	chkevdata		;set up flash ram if not already done
	
		call	clrvar			;clear variables
		
		;		test for setup mode
;		call	copyEVs
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
		
	
seten_f	bsf		LED_PORT,LED1	;Yellow LED on.
		bcf		LED_PORT,LED2			
		bcf		Datmode,0
		call	timload			;load stuff
		movlw	B'11000000'
		movwf	INTCON			;enable interrupts
		goto	main

slimset	bcf		Mode,1
		clrf	NN_temph
		clrf	NN_templ

		;test for clear all events

		btfss	SL_PORT,LEARN		;ignore the clear if learn is set
		goto	seten
		btfss	UNLPORT,UNLEARN
		call	initevdata			;clear all events if unlearn is set during power up
seten
		call	nv_rest			;if SLiM put default NVs in.
		call	timload			;Load NV's into RAM
	
		movlw	B'11000000'
		movwf	INTCON			;enable interrupts
		bcf		LED_PORT,LED1
		bsf		LED_PORT,LED2	;RUN LED on. Green for SLiM
		goto	main			
	
		

		
;****************************************************************************
;		start of subroutines		
;*************************************************************************


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
;		sets an output  
		

;********************************************************************
;		Do an event.  arrives with EV in EVtemp and POL in EVtemp2
;		Toggles outputs. Turns off before on.
;		Checks which outputs are active for the event
;		Checks command for ON or OFF and checks the POL bit for which way to set output

ev_set
		movff	Tfdly,OpFdly	;Reload fire delay counter
		btfss	EVtemp,0
		bra		ev_set2		;no action on pair 1
		btfss	ev_opc,0		;on or off?
		bra		ev1a
		btfss	EVtemp2,0	;reverse?
		bra		ev1_off
		bra		ev1_on

ev1a	btfss	EVtemp2,0
		bra		ev1_on
		bra		ev1_off			

; Process pair 1

ev1_on						; Output 1 is on, 2 off
		bcf		OpTrig,1	; Clear other output trigger
		bsf		OpTrig,0	; Set output trigger
		bra		ev_set2		; All done for this pair

ev1_off						; Output 1 is off, 2 is on
		bsf		OpTrig,1	; Set active output trigger
		bcf		OpTrig,0	; Clear other output trigger
							; Drop through

; Process pair 2

ev_set2	btfss	EVtemp,1
		bra		ev_set3		;no action on pair 2
		btfss	ev_opc,0		;on or off?
		bra		ev2a
		btfss	EVtemp2,1	;reverse?
		bra		ev2_off
		bra		ev2_on

ev2a	btfss	EVtemp2,1
		bra		ev2_on
		bra		ev2_off			

ev2_on						; Output 3 is on, 4 off
		bcf		OpTrig,3	; Clear other output trigger
		bsf		OpTrig,2	; Set active output trigger
		bra		ev_set3		; All done for this pair

ev2_off						; Output 3 is off, 4 is on
		bsf		OpTrig,3	; Set active output trigger
		bcf		OpTrig,2	; Clear other output trigger
							; Drop through

; Process pair 3

ev_set3	btfss	EVtemp,2
		bra		ev_set4		;no action on pair 3
		btfss	ev_opc,0		;on or off?
		bra		ev3a
		btfss	EVtemp2,2	;reverse?
		bra		ev3_off
		bra		ev3_on

ev3a	btfss	EVtemp2,2
		bra		ev3_on
		bra		ev3_off			

ev3_on						; Output 5 is on, 6 off
		bcf		OpTrig,5	; Clear other output trigger
		bsf		OpTrig,4	; Set active output trigger
		bra		ev_set4		; All done for this pair

ev3_off						; Output 5 is off, 6 is on
		bsf		OpTrig,5	; Set active output trigger
		bcf		OpTrig,4	; Clear other output trigger
							; Drop through

; Process pair 4

ev_set4	btfss	EVtemp,3
		bra		ev_setx		;no action on pair 4
		btfss	ev_opc,0		;on or off?
		bra		ev4a
		btfss	EVtemp2,3	;reverse?
		bra		ev4_off
		bra		ev4_on

ev4a	btfss	EVtemp2,3
		bra		ev4_on
		bra		ev4_off			

ev4_on						; Output 7 is on, 8 off
		bcf		OpTrig,7	; Clear other output trigger
		bsf		OpTrig,6	; Set active output trigger
		bra		ev_setx		; All done for this pair

ev4_off						; Output 7 is off, 8 is on
		bsf		OpTrig,7	; Set active output trigger
		bcf		OpTrig,6	; Clear other output trigger
							; Drop through

; All done

ev_setx	return


;***************************************************************************
;		reloads the timer settings and output masks from EEPROM/Flash to RAM

timload	movlw	LOW NVstart			;reloads timers
		movwf	EEADR
		lfsr	FSR1,T1a
timloop	bsf		EECON1,RD
		movff	EEDATA,POSTINC1
		incf	EEADR
		movlw	LOW NVstart+.11		; 8 outputs and 3 timers
		cpfseq	EEADR
		bra		timloop
		movlw	HIGH Opmap			; Opmap is in Flash
		movwf	TBLPTRH
		movlw	LOW Opmap
		movwf	TBLPTRL	
		clrf	TBLPTRU
		movlw	8
		movwf	Count
		lfsr	FSR1, Opm1a
opmloop
		tblrd*+
		movff	TABLAT, POSTINC1
		decfsz	Count
		bra		opmloop
		return
		
;**************************************************************************
; Clear key variables used for output timing
;**************************************************************************
clrvar	clrf	WREG
		movwf	OpTimr				; Clear working variables
		movwf	OpTrig
		movwf	OpFlag
		movwf	OpFdly
		movwf	OpCdly
		return

;**************************************************************************
;		write to EEPROM, EEADR must be set before this sub.
;		data to write comes in W
eewrite	movwf	EEDATA			
		bcf		EECON1,EEPGD
		bcf		EECON1,CFGS
		bsf		EECON1,WREN
		
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
		movlw	B'11000000'
		movwf	INTCON		;reenable interrupts
		
		return	
		
;************************************************************************************
;		
eeread	bcf		EECON1,EEPGD	;read a EEPROM byte, EEADR must be set before this sub.
		bcf		EECON1,CFGS		;returns with data in W
		bsf		EECON1,RD
		nop
		movf	EEDATA,W
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
	
		return	

;**************************************************************************

;		check if command is for this node

thisNN	movf	NN_temph,W
		subwf	ev0,W
		bnz		not_NN
		movf	NN_templ,W
		subwf	ev1,W
		bnz		not_NN
		retlw 	0			;returns 0 if match
not_NN	retlw	1


#include "newevhndler_v1b.asm"



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

para1rd	movf	ev2,W
		sublw	0
		bz		numParams
		movlw	PRMCOUNT
		movff	ev2, Temp
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
		decf	ev2,W
		addwf	TBLPTRL
		bsf		EECON1,EEPGD
		tblrd*
		movff	TABLAT,Tx1d4
addflags						
		movff	ev2,Tx1d3
		movlw	5
		movwf	Dlc
		call	sendTX
		return	
		
numParams
		movlw	0x9B
		movwf	Tx1d0
		movlw	PRMCOUNT
		movwf	Tx1d4
		movff	ev2,Tx1d3
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
		


;*********************************************************************
;		put in NN from command

putNN	movff	ev0,NN_temph
		movff	ev1,NN_templ
		movlw	LOW NodeID
		movwf	EEADR
		movf	ev0,W
		call	eewrite
		incf	EEADR
		movf	ev1,W
		call	eewrite
		movlw	Modstat
		movwf	EEADR
		movlw	B'00001000'		;Module status has NN set
		call	eewrite
		return

;***********************************************************

;	error message send

errmsg	call	errsub
		goto	main2 
errmsg1	call	errsub
		goto	l_out2
errmsg2	call	errsub
		goto	l_out1

errsub	movwf	Tx1d3		;main eror message send. Error no. in WREG
		movlw	0x6F
		movwf	Tx1d0
		movlw	4
		movwf	Dlc
		call	sendTX
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

;***************************************************************
		
;	a delay routine
;		may be used to allow CDU to recharge between succesive outputs.
;		probably needs to be longer for this.
			
dely	movlw	.10
		movwf	Count1
dely2	clrf	Count
dely1	decfsz	Count,F
		goto	dely1
		decfsz	Count1
		bra		dely2
		return	
	
delay2	clrf	Count2		;long delay for CDU recharge
del2	call	dely
		decfsz	Count2
		bra		del2
		return	

;****************************************************************

;		longer delay

ldely	movlw	.100
		movwf	Count2
ldely1	call	dely
		decfsz	Count2
		bra		ldely1
		
		return	


		
;***************************************************************	
		
getop	movlw	B'00000011'		;get DIP switch setting for output
		andwf	PORTB,W			;mask
				
		movwf	Shift
		movlw	1
		movwf	EVtemp
shift	movf	Shift,F			;is it zero?
		bz		shift2
		rlncf	EVtemp,F		;put rolling bit into EVtemp
		decf	Shift,F
		bra		shift
		
shift2	return


		
	
;****************************************************************************

nnack	movlw	0x50			;request frame for new NN or ack if not virgin
nnrel	movwf	Tx1d0
		movff	NN_temph,Tx1d1
		movff	NN_templ,Tx1d2
		movlw	3
		movwf	Dlc
		call	sendTX
		return

;**************************************************************************

putNV	movlw	NV_NUM + 1		;put new NV in EEPROM and the NV ram.
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

;************************************************************************

getNV	
			movlw	LOW	NVstart
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

nv_rest	movlw	8
		movwf	Count
		movlw	LOW Timers
		movwf	Temp
		movwf	EEADR
nv_rest1 call	eeread
		movwf	Temp1
		movlw	LOW NVstart - LOW Timers
		addwf	EEADR,F
		movf	Temp1,W
		call	eewrite
		decfsz	Count
		bra		nv_rest2
		return
nv_rest2 incf	Temp,F
		movf	Temp,W
		movwf	EEADR
		bra		nv_rest1



;***************************************************************
;
;		self enumeration as separate subroutine

self_en	movff	FSR1L,Fsr_tmp1Le	;save FSR1 just in case
		movff	FSR1H,Fsr_tmp1He 
	
		bsf		Datmode,1		;set to 'setup' mode
		clrf	Tx1con			;CAN ID enumeration. Send RTR frame, start timer
		movlw	.14
		movwf	Count
		lfsr	FSR0, Enum0
clr_en
		clrf	POSTINC0
		decfsz	Count
		bra		clr_en
		
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
		movlw	B'00110011'
		movwf	T3CON			;enable timer 3
		bcf		PIR2,TMR3IF
		movlw	.10
		movwf	Latcount
		
		call	sendTXa			;send RTR frame
		clrf	Tx1dlc			;prevent more RTR frames

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
		call	newid_f			;put new ID in various buffers

			
		movff	Fsr_tmp1Le,FSR1L	;
		movff	Fsr_tmp1He,FSR1H 
		return	0

segful	movlw	7		;segment full, no CAN_ID allocated
		call	errsub
		setf	IDcount
		bcf		IDcount,7
		bra		here3

;***********************************************************************

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

;********************************************************************************


			
Opmap	db	B'00000001',B'10000000'					;output mapping
		db	B'00000010',B'01000000'					;don't change this
		db	B'00100000',B'00000100'
		db	B'00010000',B'00001000'	
	
		ORG	0x3000
evdata				
;************************************************************************
	
		ORG 0xF00000			;EEPROM data. Defaults

CANid	de	B'01111111',0	;CAN id default and module status
NodeID	de	0,0		;Node ID
ENindex	de	0,0		;ENindex contains free space
					;ENindex +1 contains number of events
					;hi byte + lo byte = EN_NUM
					;initialised in initevdata
		
											
		
FreeCh	de	0,0

		
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
	
		
	



NVstart
		de	DFFTIM,DFFTIM,DFFTIM,DFFTIM			;NV 1..4
		de	DFFTIM,DFFTIM,DFFTIM,DFFTIM			;NV 5..8
		de 	DFRDLY,DFFDLY,DFCDLY,0				;NV 9..11
		de	0,0,0,0								;NV 12..16


Timers	de	.5,.5				;Timers (for now)
		de	.5,.5				;These are for each output
		de	.5,.5												
		de	.5,.5	

		ORG 0xF003FE
		de	0,0									;for boot.
		
			
		
		end
