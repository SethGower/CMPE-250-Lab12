            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  <Your name here>
;Date:  <Date completed here>
;Class:  CMPE-250
;Section:  <Your lab section, day, and time here>
;---------------------------------------------------------------
;Keil Template for KL46 Assembly with Keil C startup
;R. W. Melton
;November 13, 2017
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates 
TPM_CnV_PWM_DUTY_2ms EQU 	6050 
TPM_CnV_PWM_DUTY_1ms EQU   	2900 
PWM_2ms  			EQU     TPM_CnV_PWM_DUTY_2ms 
PWM_1ms  			EQU     TPM_CnV_PWM_DUTY_1ms 
DAC0_STEPS			EQU		4096
SERVO_POSITIONS 	EQU 	5
CR          EQU  	0x0D
LF          EQU  	0x0A
NULL        EQU  	0x00
DEL			EQU	 	0x7F
Esc			EQU  	0x1B
LTH			EQU	 	0x3C
GTH			EQU		0x3E
MAX_STRING	EQU	 	79
IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		20
BUF_DEPTH	EQU		4
    
XQ_BUF_SZ   EQU     80
Q_BUF_SZ    EQU     4
Q_REC_SZ    EQU     18
	
PromptLen	EQU		21
LN			EQU		0x2D
SEC			EQU		500
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK

;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK

;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0



;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK

;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;****************************************************************
;MACROs
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
			EXPORT  GetChar
			EXPORT  GetStringSB
			EXPORT  Init_UART0_IRQ
			EXPORT  PutChar
			EXPORT  PutNumHex
			EXPORT  PutNumUB
			EXPORT  PutStringSB
			EXPORT  PWM_duty_table_0
			EXPORT  DAC0_table_0  ;make available to C program 
			EXPORT 	UART0_IRQHandler 
			EXPORT	NewLine
				
;>>>>> begin subroutine code <<<<<
NewLine				PROC	{}
;***************************************************************
;Creates a new line and returns the character  
;	Input:
; 	  None
;	Output:
;	  None
;***************************************************************
			PUSH	{R0-R2, LR}
			MOVS 	R0,#LF
			BL		PutChar
			MOVS	R0,#CR
			BL		PutChar
			POP		{PC, R0-R2}
			ENDP

PutStringSB		PROC	{}
;***************************************************************
;This subroutine displays a null-terminated string to the 
;terminal screen from memory starting at the address in R0.
;
;	Input:
;	  R0: Starting adress of the memory
; 	  R1: Max size of the buffer
;	Output:
;	  None
;***************************************************************
				PUSH		{LR,R2-R6}
				MOVS 		R2,#0				;StringPtr = 0
				MOVS 		R3,R0
LOOP			LDRB		R0,[R3,R2]			;LOAD CHAR
				BL			PutChar				;PRINT CHARACTER
				ADDS		R2,R2,#1			;INCREMENT R2
				CMP			R2,R1				;Buffer Max
				BGT			END_PutStringSB		;END IF MAX STRING SIZE EXCEEDED
				CMP			R0,#NULL			;IS CHAR NULL TERMINATOR
				BHI			LOOP				;IF NOT NULL, CONTINUE
END_PutStringSB	POP			{R2-R6,PC}
				ENDP

PutNumUB	PROC	{R0-R14}
;***************************************************************
;prints the decimal representation of the unsigned byte of R0
;	Input:
; 	  R0: Byte to be outputted
;	Output:
;	  None
;***************************************************************	
			PUSH	{R1-R6, LR}
			LDR		R1,=TMP						;LOAD TEMP STRING ADDRESS
			STR		R0,[R1,#0]					;STORE VALUE IN TEMP LOCATION
			LDRB	R0,[R1,#0]					;LOAD LEAST SIGNIFICANT BYTE
			BL		PutNumU						;PERFORM PRINT
			MOVS	R0,#0x09
			BL		PutChar
			POP		{R1-R6,PC}
			ENDP	

PutNumHex	PROC	{R0-R14}
;***************************************************************
;Prints the unsigned word value of the object in R0 
;	Input:
; 	  R0: Data to be outputted
;	Output:
;	  None
;***************************************************************
			PUSH	{LR,R1,R2,R3,R4,R5,R6}
			LDR		R1,=TMP						;STORE REGISTER IN TEMP VARIABLE
			STR		R0,[R1,#0]
			MOVS	R2,#0x0f
			LDRB	R0,[R1,#3]					;LOAD MOST SIGNIFICANT BYTE
			LSRS	R0,R0,#4
			ADDS	R0,R0,#0x30					;MAKE INTO ASCII
			CMP		R0,#0x3A
			BLT		H_0
			ADDS	R0,R0,#7
H_0			BL		PutChar						;PRINT
			LDRB	R0,[R1,#3]					;LOAD SECOND MOST SIGNIFICANT BYTE
			ANDS	R0,R0,R2
			ADDS	R0,R0,#0x30					;MAKE INTO ASCIICMP		R0,#0X3A
            
			BLT		H_1
			ADDS	R0,R0,#7
H_1			BL		PutChar						;PRINT
			LDRB	R0,[R1,#2]					;LOAD THIRD MOST SIGNIFICANT BYTE
			LSRS	R0,R0,#4
			ADDS	R0,R0,#0x30					;MAKE INTO ASCII
			CMP		R0,#0x3A
			BLT		H_2
			ADDS	R0,R0,#7
H_2			BL		PutChar						;PRINT
			LDRB	R0,[R1,#2]					;LOAD LEAST SIGNIFICANT BYTE
			ANDS	R0,R0,R2
			ADDS	R0,R0,#0x30					;MAKE INTO ASCII
			CMP		R0,#0x3A
			BLT		H_3
			ADDS	R0,R0,#7
H_3			BL		PutChar						;PRINT
			LDRB	R0,[R1,#1]					;LOAD LEAST SIGNIFICANT BYTE
			LSRS	R0,R0,#4
			ADDS	R0,R0,#0x30					;MAKE INTO ASCII
			CMP		R0,#0x3A
			BLT		H_4
			ADDS	R0,R0,#7
H_4			BL		PutChar						;PRINT
			LDRB	R0,[R1,#1]					;LOAD LEAST SIGNIFICANT BYTE
			ANDS	R0,R0,R2
			ADDS	R0,R0,#0x30					;MAKE INTO ASCII
			CMP		R0,#0x3A
			BLT		H_5
			ADDS	R0,R0,#7
H_5			BL		PutChar						;PRINT
			LDRB	R0,[R1,#0]					;LOAD LEAST SIGNIFICANT BYTE
			LSRS	R0,R0,#4
			ADDS	R0,R0,#0x30					;MAKE INTO ASCII
			CMP		R0,#0x3A
			BLT		H_6
			ADDS	R0,R0,#7
H_6			BL		PutChar						;PRINT
			LDRB	R0,[R1,#0]					;LOAD LEAST SIGNIFICANT BYTE
			ANDS	R0,R0,R2
			ADDS	R0,R0,#0x30					;MAKE INTO ASCII
			CMP		R0,#0x3A
			BLT		H_7
			ADDS	R0,R0,#7
H_7			BL		PutChar						;PRINT
			MOVS	R0,#0x09
			BL		PutChar
			POP		{R6,R5,R4,R3,R2,R1,PC}
			ENDP

PutChar				PROC	{}
;**********************************************************************
;Use Polled I/O to print a character
;Input:	R0:	Character to print to the terminal
;Output:	No registers changed upon output
;**********************************************************************
					PUSH	{R1,R2,R4,LR}
					LDR	    R1,=TxQRecord	;Load record adress
TxLoop					CPSID	I				;Mask the interupts
					BL	    Enqueue			;Enques to the equeu
					CPSIE	I				;Unmasks the interupts
					BCS	    TxLoop			;retries if engueue fails
					LDR	    R1,=UART0_BASE
					MOVS	R2,#UART0_C2_TI_RI
					STRB	R2,[R1,#UART0_C2_OFFSET]	;Sets the interupt
					POP 	{PC,R1,R2,R4}
					ENDP

GetChar					PROC	{}
;**********************************************************************
;Use Polled I/O to recieve character from terminal
;Input: No register parameters required
;Output:	R0: Character recieved from terminal
;**********************************************************************
					PUSH	{R1,LR}
					LDR	R1,=RxQRecord		;Load record address
RxLoop					CPSID	I					;Mask interupta
					BL	Dequeue			;Dequeue from recieve queue
					CPSIE	I				;Unmask interupts
					BCS	RxLoop			;Loop if the dequeue fails
					POP	{PC,R1}
					ENDP
				
GetStringSB				PROC	{}
;***************************************************************
;Stores string from the terminal into memory starting at the 
;address given
;
;	Input:
;	  R0: The starting adress of the string
; 	  R1: Buffer Capacity
;	Output:
;	  None
;***************************************************************
				PUSH		{LR,R2-R6}
				MOVS		R6,R0
				MOVS		R2,#0				;StringPtr = 0
WHILE			BL		GetChar				;DO WHILE
				CMP		R0,#CR				;Carriage Return = 13 (CR <= #13)
				BEQ		DEPART				;IF(CR => End SUBROUTINE)
				CMP		R0,#DEL				;CHECK FOR DELETE
				BEQ		CONT				;DO NOT STORE DELETE
				CMP		R0,#Esc				;CHECK FOR CONTROL CHARACTERS
				BLE		WHILE
				CMP		R2,R1				; R1 = MAX
				BEQ		WHILE				;IF MAX_STRING REACHED => WAIT FOR CR
				STRB		R0,[R6,R2]			;STORE CHAR
				BL		PutChar				;ECHO TO TERMINAL
				ADDS		R2,R2,#1			;INCREMENT POINTER
				B		WHILE				;LOOP
CONT			CMP		R2,#0				;DO NOT EXCEED BOUNDS OF THE STRING VARIABLE PAST ZERO
				BEQ		WHILE
				SUBS		R2,R2,#1			;DELETE LAST CHARACTER
				BL		PutChar				;DELETE LAST PRINT
				B		WHILE				;LOOP
DEPART			MOVS 		R3,#0x00			;R3 <= NULL TERMINATOR
				STRB		R3,[R6,R2]			;HANDLE RETURN
				BL		PutChar				;NEW LINE
				POP		{R2-R6,PC}			;RETURN
				ENDP

Init_UART0_IRQ  PROC  {R0-R14}
;Initialize UART0
            ;Preserve registers used
            PUSH    {R0-R3,LR}
            ;initialize receive queue
            LDR     R0,=RxQBuffer
            LDR     R1,=RxQRecord
            MOVS    R2,#XQ_BUF_SZ
            BL      initQueue         ;initialize queue structure
            ;initialize transmit queue
            LDR     R0,=TxQBuffer
            LDR     R1,=TxQRecord
			MOVS	R2,#XQ_BUF_SZ
            BL      initQueue         ;initialize queue structure
            ;Select MCGPLLCLK / 2 as UART0 clock source
            LDR     R0,=SIM_SOPT2
            LDR     R1,=SIM_SOPT2_UART0SRC_MASK
            LDR     R2,[R0,#0]
            BICS    R2,R2,R1
            LDR     R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
            ORRS    R2,R2,R1
            STR     R2,[R0,#0]
            ;Set UART0 for external connection
            LDR     R0,=SIM_SOPT5
            LDR     R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
            LDR     R2,[R0,#0]
            BICS    R2,R2,R1
            STR     R2,[R0,#0]
            ;Enable UART0 module clock
            LDR     R0,=SIM_SCGC4
            LDR     R1,=SIM_SCGC4_UART0_MASK
            LDR     R2,[R0,#0]
            ORRS    R2,R2,R1
            STR     R2,[R0,#0]
            ;Enable PORT A module clock
            LDR     R0,=SIM_SCGC5
            LDR     R1,=SIM_SCGC5_PORTA_MASK
            LDR     R2,[R0,#0]
            ORRS    R2,R2,R1
            STR     R2,[R0,#0]
            ;Select PORT A Pin 1 (J1 Pin 02) for UART0 RX
            LDR     R0,=PORTA_PCR1
            LDR     R1,=PORT_PCR_SET_PTA1_UART0_RX
            STR     R1,[R0,#0]
            ;Select PORT A Pin 2 (J1 Pin 04) for UART0 TX
            LDR     R0,=PORTA_PCR2
            LDR     R1,=PORT_PCR_SET_PTA2_UART0_TX
            STR     R1,[R0,#0]
            ;Disable UART0
            LDR     R0,=UART0_BASE
            MOVS    R1,#UART0_C2_T_R
            LDRB    R2,[R0,#UART0_C2_OFFSET]
            BICS    R2,R2,R1
            STRB    R2,[R0,#UART0_C2_OFFSET]
            ;Set UART0 interrupt priority
            LDR     R0,=UART0_IPR
            ;LDR     R1,=NVIC_IPR_UART0_MASK
            LDR     R2,=NVIC_IPR_UART0_PRI_3
            LDR     R3,[R0,#0]
            ;BICS    R3,R3,R1
            ORRS    R3,R3,R2
            STR     R3,[R0,#0]
            ;Clear any pending UART0 interrupts
            LDR     R0,=NVIC_ICPR
            LDR     R1,=NVIC_ICPR_UART0_MASK
            STR     R1,[R0,#0]
            ;Unmask UART0 interrupts
            LDR     R0,=NVIC_ISER
            LDR     R1,=NVIC_ISER_UART0_MASK
            STR     R1,[R0,#0]
            ;Set for 9600 baud from 96MHz PLL clock
            LDR     R0,=UART0_BASE
            MOVS    R1,#UART0_BDH_9600
            STRB    R1,[R0,#UART0_BDH_OFFSET]
            MOVS    R1,#UART0_BDL_9600
            STRB    R1,[R0,#UART0_BDL_OFFSET]
            MOVS    R1,#UART0_C1_8N1
            STRB    R1,[R0,#UART0_C1_OFFSET]
            MOVS    R1,#UART0_C3_NO_TXINV
            STRB    R1,[R0,#UART0_C3_OFFSET]
            MOVS    R1,#UART0_C4_NO_MATCH_OSR_16
            STRB    R1,[R0,#UART0_C4_OFFSET]
            MOVS    R1,#UART0_C5_NO_DMA_SSR_SYNC
            STRB    R1,[R0,#UART0_C5_OFFSET]
            MOVS    R1,#UART0_S1_CLEAR_FLAGS
            STRB    R1,[R0,#UART0_S1_OFFSET]
            MOVS    R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
            STRB    R1,[R0,#UART0_S2_OFFSET]
            ;Enable UART0 Rx/Tx with Rx IRQ
            MOVS    R1,#UART0_C2_T_RI
            STRB    R1,[R0,#UART0_C2_OFFSET]
            ;Restore registers used and return
            POP     {R0-R3,PC}
            ENDP

UART0_IRQHandler 
UART0_ISR	PROC	{R4-R13}
;**********************************************************************
;UART0 Interupt Service Routine:
;Handles interupts
;Input:  None
;Output:  None
;Modifies: None
;**********************************************************************
			CPSID	I
			PUSH	{LR}
			
			LDR		R2,=UART0_BASE
			LDRB	R3,[R2,#UART0_C2_OFFSET]
			MOVS	R4,#UART0_C2_TIE_MASK
			TST		R3,R4
			BEQ		RxHandle

			LDRB	R3,[R2,#UART0_S1_OFFSET]
			MOVS	R4,#UART0_S1_TDRE_MASK
			TST		R3,R4
			BEQ		RxHandle
			
			LDR		R1,=TxQRecord
			BL		Dequeue
			BCS		DequeueFail
			STRB	R0,[R2,#UART0_D_OFFSET]
			
RxHandle	LDRB	R3,[R2,#UART0_S1_OFFSET]
			MOVS	R4,#UART0_S1_RDRF_MASK
			TST		R3,R4
			BEQ		ISR_END
			
			LDRB	R0,[R2,#UART0_D_OFFSET]
			LDR		R1,=RxQRecord
			BL		Enqueue
			B		ISR_END
			
DequeueFail	MOVS	R4,#UART0_C2_T_RI
			STRB	R4,[R2,#UART0_C2_OFFSET]
			B		RxHandle
			
ISR_END		CPSIE	I
			POP		{PC}
			ENDP
				
PutNumU			PROC		{}
;***************************************************************
;This subroutine prints to the terminal screen the text decimal 
;representation of the unsigned word value in R0.  
;
;	Input:
; 	  R0: Value to be outputted
;	Output:
;	  None
;***************************************************************
				PUSH		{LR,R1-R6}
				MOVS 		R1,R0				;MOVE VALUE TO DIVIDEND
				LDR			R3,=OpLEN			;LOAD NUMBER STORAGE
				MOVS		R4,#0				;Load to use for offset
START			MOVS 		R0,#10				;DIVIDE BY 10
				BL			DIVU				;PERFORM DIVISION
				MOVS		R2,R0				;SAVE QUOTIENT
				MOVS		R0,R1				;MOVE REMAINDER TO R0
				ADDS		R0,R0,#0x30			;CONVERT TO ASCII CHARACTER
				STRB		R0,[R3,R4]			;STORE CHARACTER
				ADDS		R4,R4,#1
				MOVS		R1,R2				;STORE QUOTIENT IN DIVIDEND FOR NEXT RUN
				CMP			R1,#0				;IF X/10 = 0 Remainder X
				BNE			START				;CONTINUE IF QUOTIENT NOT ZERO
Print			SUBS		R4,R4,#1
				LDRB		R0,[R3,R4]
				BL			PutChar
				CMP			R4,#0
				BGT			Print
				POP			{R1-R6,PC}
				ENDP				

DIVU			PROC		{R2-R14}
;***************************************************************
;Computation to divide an operation and include a remandor in the opperation,
;R1/R0 = R0 remainferR 1
;	Input:
; 	  R0: Divisor. Bottom of fraction
; 	  R1: Dividen. Top of fraction
;	Output:
;	  R0: Quotient
; 	  R1: Remainder
;	  C: 1 if invaild (divide by 0) 0 if valid opperation
;***************************************************************
			PUSH	{LR, R2-R5}
			MOVS 	R5,#0		;Puts 0 in R5
			CMP  	R0,R5		;Compairs R0,R5
			BEQ		Divisor0	;Branches when n/0
			CMP		R1,R5		;Compaires R1,R5
			BEQ		Dividen0	;Branches then 0/n
			MOVS 	R4,R5		;takes 0 in R5 and puts it in R4
			MOVS 	R2,#1		;Sets R2 to 1
DivNLoop		CMP		R1,R0		;Dividen <,>,+ Divisor
			BLO		EndLoop	
			SUBS	R1,R1,R0	;Dividen = Dividen - Divisor
			ADDS	R4,R4,R2	;Quotient = Quotient + 1
			B		DivNLoop	;While(Dividend >= Divisor)			
EndLoop			MOVS	R0,R4		;Takes the temp of quotient and moves to proper register
			CMP 	R0,R5		;See if quotient is 0
ClearC			MRS		R2,APSR
			MOVS	R3,#0x20	;Moves mask into register
			LSRS	R3,R3,#24	;Shifts data
			BICS	R2,R3		;Applies mask
			BEQ		CleanUp			
CheckQ			LSRS	R3,R0,#31	;Shifts right so MSB is in LSB
			CMP		R3,R2		;Checks if quotient MSb = 1
			B		CleanUp
Divisor0		MRS		R2,APSR		;Gets the APSR
			MOVS	R3,#0x20	;Moves mask into register
			LSRS	R3,R3,#24	;Shifts data
			ORRS	R2,R2,R3	;Applies mask
			MSR		APSR,R2		;sets just c flag
			B		CleanUp
Dividen0		MOVS	R0,R5		;Sets R0 to 0
			MOVS	R1,R5		;Sets R1 to 0
			B		ClearC		;Branch to set Z
CleanUp			POP		{R2-R5, PC}			
			ENDP
				
Dequeue		PROC	{R1-R14}
;***************************************************************
;Retries a value from the start of the FIFo and allows it to outputted 
;	Input:
; 	  R1: Queue Record Structure
;	Output:
;	  R0: Character removed from queue
;	  APSR C flag: if the opperation failed or success
;   Modified: APSR R0
;***************************************************************
			PUSH	{LR,R2-R6}
			MOVS	R6,#0
			LDRB	R0,[R1,#NUM_ENQD]			;LOAD NUMBER ENQUEUED
			CMP		R0,#0						;IS THERE A VALUE IN THE QUEUE
			BEQ		DEQUEUE_F					;BRANCH  TO FAILUE CONDITION
			SUBS	R0,R0,#1					;DECREMENT NUMBER ENQUEUED
			STRB	R0,[R1,#NUM_ENQD]			;STORE NUMBER ENQUEUED
			LDR		R2,[R1,#OUT_PTR]			;LOAD THE OUT POINTER ADDRESS
			LDRB	R0,[R2,#0]					;DEQUEUE CHARACTER
			STRB	R6,[R2,#0]					;CLEAR VALUE
			ADDS	R2,R2,#1					;INCREMENT OUT POINTER
			LDR		R3,[R1,#BUF_PAST]			;LOAD BUFFER START
			CMP		R2,R3						;IS THE OUT POINTER IN BOUNDS
			BLT		DEQUEUE_CC					;IF IN BOUNDS CONTINUE TO CLEAR
			LDR		R2,[R1,#BUF_STRT]			;SET OUT POINTER TO MIN
DEQUEUE_CC	STR		R2,[R1,#OUT_PTR]			;STORE OUT POINTER IN RECORD
			MRS		R3,APSR						;CLEAR C FLAG
			MOVS	R4,#0x20				
			LSLS	R4,R4,#24
			BICS	R3,R3,R4
			MSR		APSR,R3						;C FLAG CLEARED
			B		DEQUEUE_O					;BRANCH TO  OUTPUT
DEQUEUE_F	MRS		R3,APSR						;SET C FLAG
			MOVS	R4,#0x20
			LSLS	R4,R4,#24
			ORRS	R3,R3,R4
			MSR		APSR,R3						;C FLAG SET
DEQUEUE_O	POP		{R2-R6,PC}
			ENDP
			
Enqueue		PROC	{R0-R14}
;***************************************************************
;Puts a character into the queue when there is space 
;	Input:
;	  R0:  Character to be inputted into the queue
;	  R1:  record structure of the queue
;	Output:
;	  APSR: C determining if the oppertion worked
;	  Modify: APSR
;***************************************************************
			PUSH	{LR,R2-R5}
			LDRB	R2,[R1,#NUM_ENQD]			;LOAD NUMBER ENQUEUED
			MOVS	R3,#BUF_DEPTH				;LOAD THE SIZE OF THE  BUFFER
			CMP		R2,R3						;IS THE QUEUE FULL
			BGE		ENQUEUE_F					;IF TOO MANY VALUES, BRANCH TO END
			ADDS	R2,R2,#1					;INCREMENT NUM ENQUEUED
			STRB	R2,[R1,#NUM_ENQD]			;UPDATE NUMBER ENQUEUED IN MEMORY
			LDR		R2,[R1,#IN_PTR]				;LOAD IN POINTER
			STRB	R0,[R2,#0]					;ENQUEUE CHARACTER
			LDR		R3,[R1,#BUF_PAST]			;LOAD BUFFER PAST
			ADDS	R2,R2,#1					;INCREMENT IN POINTER
			CMP		R2,R3						;IS POINTER PAST BUFFER END
			BLT		ENQUEUE_CC					;BRANCH TO CLEAR IF SO
			LDR		R2,[R1,#BUF_STRT]			;LOAD BUFFER START FOR IN POINTER
ENQUEUE_CC	STR		R2,[R1,#IN_PTR]				;STORE IN POINTER
			MRS		R3,APSR						;CLEAR C FLAG
			MOVS	R4,#0x20				
			LSLS	R4,R4,#24
			BICS	R3,R3,R4
			MSR		APSR,R3						;C FLAG CLEARED
			B		ENQUEUE_O					;BRANCH TO OUTPUT
ENQUEUE_F	MRS		R3,APSR						;SET C FLAG
			MOVS	R4,#0x20
			LSLS	R4,R4,#24
			ORRS	R3,R3,R4
			MSR		APSR,R3						;C FLAG SET
ENQUEUE_O	POP		{PC,R2-R5}
			ENDP

initQueue	PROC	{R0-R14}
;***************************************************************
;Runs the setup needed to create a FIFO   
;	Input: 
; 	  R0: Address of queue buffer in memory
;	  R1: Address of queue structure in memory
;     R2: Size of buffer
;	Output: None
;	Modified: ASPR
;***************************************************************
			PUSH	{R3, R4, LR}
			MOVS	R3,#0						;INIT COUNTER
			MOVS	R4,#0						;INIT ZERO VALUE
INITLOOP	STRB	R4,[R0,R3]					;INIT VALUES
			ADDS	R3,R3,#1					;INCREMENT COUNTER
			CMP		R2,R3						;IF(COUNTER > MAX NUMBER)
			BGT		INITLOOP					;^^ LOOP
			STR		R0,[R1,#IN_PTR]				;INIT IN POINTER
			STR		R0,[R1,#OUT_PTR]			;INIT OUT POINTER
			STR		R0,[R1,#BUF_STRT]			;INIT BUFFER START
			STRB	R2,[R1,#BUF_SIZE]			;INIT BUFFER SIZE IN RECORD
			STRB	R4,[R1,#NUM_ENQD]			;INIT NUMBER ENQUEUED
			ADDS	R4,R0,R2					;COMPUTE BUFFER PAST
			STR		R4,[R1,#BUF_PAST]			;INIT BUFFER PAST
			POP		{R4, R3, PC}				;RETURN
			ENDP

;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
PWM_duty_table 
PWM_duty_table_0      ;include if accessed from C
;Servo positions from 1 (leftmost) to 5 (rightmost)   
			DCW  PWM_2ms      						;-50% of range   
			DCW  ((3*(PWM_2ms-PWM_1ms)/4)+PWM_1ms)	;-25% of range   
			DCW  (((PWM_2ms-PWM_1ms)/2)+PWM_1ms) 	;  0% of range   
			DCW  (((PWM_2ms-PWM_1ms)/4)+PWM_1ms) 	;+25% of range   
			DCW  PWM_1ms     						;+50% of range
			
DAC0_table_0 
DAC0_table
			DCW ((DAC0_STEPS-1) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS-1) * 3) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS-1) * 5) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS-1) * 7) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS-1) * 9) / (SERVO_POSITIONS * 2))

;>>>>>   end constants here <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
RunStopWatch 	SPACE 	1
	
		ALIGN
	
TMP				SPACE 	8
QUEUE			SPACE 	4
RECORD			SPACE	16
OpLEN			SPACE 	16
Count			SPACE	16
StartString 	SPACE	MAX_STRING
    
QBuffer         SPACE   Q_BUF_SZ
        ALIGN

QRecord         SPACE   Q_REC_SZ
RxQBuffer       SPACE   XQ_BUF_SZ
        ALIGN
            
RxQRecord	SPACE	Q_REC_SZ
TxQBuffer	SPACE	XQ_BUF_SZ
        ALIGN
            
TxQRecord	SPACE	Q_REC_SZ
;>>>>>   end variables here <<<<<
            END
