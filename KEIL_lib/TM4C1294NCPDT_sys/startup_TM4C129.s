;/**************************************************************************//**
; * @file     startup_TM4C129.s
; * @brief    CMSIS Cortex-M4 Core Device Startup File for
; *           TI Tiva TM4C129 Snowflake Class Device
; * @version  V1.00
; * @date     15. May 2013
; *
; * @note
; * Copyright (C) 2011 ARM Limited. All rights reserved.
; *
; * @par
; * ARM Limited (ARM) is supplying this software for use with Cortex-M
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/

;******************************************************************************
;
; Allocate space for the stack.
;
;******************************************************************************

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
__stack_base
Stack_Mem       SPACE   Stack_Size
						;provided in command-line option, for example
						; --pd
__stack_limit
__initial_sp

;******************************************************************************
;
; Allocate space for the heap.
;
;******************************************************************************


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
						; provided in command-line option, for example:
                        ; --pd "Heap_Size SETA 0"
__heap_limit


;******************************************************************************
;
; Indicate that the code in this file preserves 8-byte alignment of the stack.
;
;******************************************************************************

                PRESERVE8
                ;THUMB

;******************************************************************************
;
; Place code into the reset code section.
;
;******************************************************************************

; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size


;******************************************************************************
;
; The vector table.
;
;******************************************************************************

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset_Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVCall_IRQHandler               ; SVCall Handler
                DCD     DebugMonitor_IRQHandler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_IRQHandler            ; PendSV Handler
                DCD     SysTick_IRQHandler           ; SysTick Handler

                ; External Interrupts

                DCD     GPIOA_IRQHandler      ;   0: GPIO Port A
                DCD     GPIOB_IRQHandler      ;   1: GPIO Port B
                DCD     GPIOC_IRQHandler      ;   2: GPIO Port C
                DCD     GPIOD_IRQHandler      ;   3: GPIO Port D
                DCD     GPIOE_IRQHandler      ;   4: GPIO Port E
                DCD     UART0_IRQHandler      ;   5: UART0 Rx and Tx
                DCD     UART1_IRQHandler      ;   6: UART1 Rx and Tx
                DCD     SSI0_IRQHandler       ;   7: SSI0 Rx and Tx
                DCD     I2C0_IRQHandler       ;   8: I2C0 Master and Slave
                DCD     PMW0_FAULT_IRQHandler ;   9: PWM Fault
                DCD     PWM0_0_IRQHandler     ;  10: PWM Generator 0
                DCD     PWM0_1_IRQHandler     ;  11: PWM Generator 1
                DCD     PWM0_2_IRQHandler     ;  12: PWM Generator 2
                DCD     QEI0_IRQHandler       ;  13: Quadrature Encoder 0
                DCD     ADC0SS0_IRQHandler    ;  14: ADC Sequence 0
                DCD     ADC0SS1_IRQHandler    ;  15: ADC Sequence 1
                DCD     ADC0SS2_IRQHandler    ;  16: ADC Sequence 2
                DCD     ADC0SS3_IRQHandler    ;  17: ADC Sequence 3
                DCD     WATCHDOG0_IRQHandler  ;  18: Watchdog timer
                DCD     TIMER0A_IRQHandler    ;  19: Timer 0 subtimer A
                DCD     TIMER0B_IRQHandler    ;  20: Timer 0 subtimer B
                DCD     TIMER1A_IRQHandler    ;  21: Timer 1 subtimer A
                DCD     TIMER1B_IRQHandler    ;  22: Timer 1 subtimer B
                DCD     TIMER2A_IRQHandler    ;  23: Timer 2 subtimer A
                DCD     TIMER2B_IRQHandler    ;  24: Timer 2 subtimer B
                DCD     COMP0_IRQHandler      ;  25: Analog Comparator 0
                DCD     COMP1_IRQHandler      ;  26: Analog Comparator 1
                DCD     COMP2_IRQHandler      ;  27: Analog Comparator 2
                DCD     SYSCTL_IRQHandler     ;  28: System Control (PLL, OSC, BO)
                DCD     FLASH_CTRL_IRQHandler ;  29: FLASH Control
                DCD     GPIOF_IRQHandler      ;  30: GPIO Port F
                DCD     GPIOG_IRQHandler      ;  31: GPIO Port G
                DCD     GPIOH_IRQHandler      ;  32: GPIO Port H
                DCD     UART2_IRQHandler      ;  33: UART2 Rx and Tx
                DCD     SSI1_IRQHandler       ;  34: SSI1 Rx and Tx
                DCD     TIMER3A_IRQHandler    ;  35: Timer 3 subtimer A
                DCD     TIMER3B_IRQHandler    ;  36: Timer 3 subtimer B
                DCD     I2C1_IRQHandler       ;  37: I2C1 Master and Slave                    
                DCD     CAN0_IRQHandler       ;  38: CAN0
                DCD     CAN1_IRQHandler       ;  39: CAN1
                DCD     EMAC0_IRQHandler      ;  40: Ethernet
                DCD     HIB_IRQHandler        ;  41: Hibernate
                DCD     USB0_IRQHandler       ;  42: USB0
                DCD     PWM0_3_IRQHandler     ;  43: PWM Generator 3
                DCD     UDMA_IRQHandler       ;  44: uDMA Software Transfer
                DCD     UDMAERR_IRQHandler    ;  45: uDMA Error
                DCD     ADC1SS0_IRQHandler    ;  46: ADC1 Sequence 0
                DCD     ADC1SS1_IRQHandler    ;  47: ADC1 Sequence 1
                DCD     ADC1SS2_IRQHandler    ;  48: ADC1 Sequence 2
                DCD     ADC1SS3_IRQHandler    ;  49: ADC1 Sequence 3
                DCD     EPI0_IRQHandler       ;  50: External Bus Interface 0
                DCD     GPIOJ_IRQHandler      ;  51: GPIO Port J
                DCD     GPIOK_IRQHandler      ;  52: GPIO Port K
                DCD     GPIOL_IRQHandler      ;  53: GPIO Port L
                DCD     SSI2_IRQHandler       ;  54: SSI2 Rx and Tx
                DCD     SSI3_IRQHandler       ;  55: SSI3 Rx and Tx
                DCD     UART3_IRQHandler      ;  56: UART3 Rx and Tx
                DCD     UART4_IRQHandler      ;  57: UART4 Rx and Tx
                DCD     UART5_IRQHandler      ;  58: UART5 Rx and Tx
                DCD     UART6_IRQHandler      ;  59: UART6 Rx and Tx
                DCD     UART7_IRQHandler      ;  60: UART7 Rx and Tx
                DCD     I2C2_IRQHandler       ;  61: I2C2 Master and Slave
                DCD     I2C3_IRQHandler       ;  62: I2C3 Master and Slave
                DCD     TIMER4A_IRQHandler    ;  63: Timer 4 subtimer A
                DCD     TIMER4B_IRQHandler    ;  64: Timer 4 subtimer B
                DCD     TIMER5A_IRQHandler    ;  65: Timer 5 subtimer A
                DCD     TIMER5B_IRQHandler    ;  66: Timer 5 subtimer B
                DCD     SYSEXC_IRQHandler     ;  67: FPU
                DCD     0                         ;  68: Reserved
                DCD     0                         ;  69: Reserved
                DCD     I2C4_IRQHandler       ;  70: I2C4 Master and Slave
                DCD     I2C5_IRQHandler       ;  71: I2C5 Master and Slave
                DCD     GPIOM_IRQHandler      ;  72: GPIO Port M
                DCD     GPION_IRQHandler      ;  73: GPIO Port N
                DCD     0                         ;  74: Reserved
                DCD     TAMPER_IRQHandler            ;  75: Tamper
                DCD     GPIOP0_IRQHandler            ;  76: GPIO Port P (Summary or P0)
                DCD     GPIOP1_IRQHandler            ;  77: GPIO Port P1
                DCD     GPIOP2_IRQHandler            ;  78: GPIO Port P2
                DCD     GPIOP3_IRQHandler            ;  79: GPIO Port P3
                DCD     GPIOP4_IRQHandler            ;  80: GPIO Port P4
                DCD     GPIOP5_IRQHandler            ;  81: GPIO Port P5
                DCD     GPIOP6_IRQHandler            ;  82: GPIO Port P6
                DCD     GPIOP7_IRQHandler            ;  83: GPIO Port P7
                DCD     GPIOQ0_IRQHandler            ;  84: GPIO Port Q (Summary or Q0)
                DCD     GPIOQ1_IRQHandler            ;  85: GPIO Port Q1
                DCD     GPIOQ2_IRQHandler            ;  86: GPIO Port Q2
                DCD     GPIOQ3_IRQHandler            ;  87: GPIO Port Q3
                DCD     GPIOQ4_IRQHandler            ;  88: GPIO Port Q4
                DCD     GPIOQ5_IRQHandler            ;  89: GPIO Port Q5
                DCD     GPIOQ6_IRQHandler            ;  90: GPIO Port Q6
                DCD     GPIOQ7_IRQHandler            ;  91: GPIO Port Q7
                DCD     GPIOR_IRQHandler             ;  92: GPIO Port R
                DCD     GPIOS_IRQHandler             ;  93: GPIO Port S
                DCD     SHAMD5_IRQHandler            ;  94: SHA/MD5 0
                DCD     AES_IRQHandler               ;  95: AES 0
                DCD     DES3DES_IRQHandler           ;  96: DES3DES 0
                DCD     LCDCONTROLLER_IRQHandler     ;  97: LCD Controller 0
                DCD     TIMER6A_IRQHandler           ;  98: Timer 6 subtimer A
                DCD     TIMER6B_IRQHandler           ;  99: Timer 6 subtimer B
                DCD     TIMER7A_IRQHandler           ; 100: Timer 7 subtimer A
                DCD     TIMER7B_IRQHandler           ; 101: Timer 7 subtimer B
                DCD     I2C6_IRQHandler              ; 102: I2C6 Master and Slave
                DCD     I2C7_IRQHandler              ; 103: I2C7 Master and Slave
                DCD     HIMSCANKEYBOARD_IRQHandler   ; 104: HIM Scan Matrix Keyboard 0
                DCD     ONEWIRE_IRQHandler           ; 105: One Wire 0
                DCD     HIMPS2_IRQHandler            ; 106: HIM PS/2 0
                DCD     HIMLEDSEQUENCER_IRQHandler   ; 107: HIM LED Sequencer 0
                DCD     HIMCONSUMERIR_IRQHandler     ; 108: HIM Consumer IR 0
                DCD     I2C8_IRQHandler              ; 109: I2C8 Master and Slave
                DCD     I2C9_IRQHandler              ; 110: I2C9 Master and Slave
                DCD     GPIOT_IRQHandler             ; 111: GPIO Port T
		DCD	0			     ; 112
		DCD	0			     ; 113
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

;******************************************************************************
;
; This is the code for exception handlers.
;
;******************************************************************************

                AREA    |.text|, CODE, READONLY


;******************************************************************************
;
; This is the code that gets called when the processor first starts execution
; following a reset event.
;
;******************************************************************************

; Reset IRQHandler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  main

                LDR     R0, =SystemInit; CMSIS system initialization
                BLX     R0
        	; Call the C library enty point that handles startup. This will copy
        	; the .data section initializers from flash to SRAM and zero fill the
        	; .bss section.
                LDR     R0, =main
                BX      R0
        	; __main calls the main() function, which should not return,
        	; but just in case jump to assert_failed() if main returns.
        	MOVS    R0,#0
        	MOVS    R1,#0       ; error number
        	B       assert_failed
                ENDP


; Dummy Exception IRQHandlers (infinite loops which can be modified)

;******************************************************************************
;
; The NMI handler
;
;******************************************************************************
NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
        	MOVS    R0,#0
        	MOVS    R1,#2       ; NMI exception number
        	B       assert_failed
        	ENDP
;******************************************************************************
;
; The Hard Fault handler
;
;******************************************************************************
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
        	MOVS    R0,#0
        	MOVS    R1,#3       ; HardFault exception number
        	B       assert_failed
        	ENDP
;******************************************************************************
;
; The MPU fault handler
;
;******************************************************************************
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
        	MOVS    r0,#0
        	MOVS    r1,#4       ; MemManage exception number
        	B       assert_failed
		ENDP
;******************************************************************************
;
; The Bus Fault handler
;
;******************************************************************************
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
        	MOVS    R0,#0
        	MOVS    R1,#5       ; BusFault exception number
        	B       assert_failed
        	ENDP
;******************************************************************************
;
; The Usage Fault handler
;
;******************************************************************************
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
        	MOVS    R0,#0
        	MOVS    R1,#6       ; UsageFault exception number
        	B       assert_failed
        	ENDP
;******************************************************************************
;
; The SVC handler
;
;******************************************************************************
SVCall_IRQHandler     PROC
                EXPORT  SVCall_IRQHandler               [WEAK]
        	MOVS    R0,#0
        	MOVS    R1,#11      ; SVCall exception number
        	B       assert_failed
        	ENDP
;******************************************************************************
;
; The Debug Monitor handler
;
;******************************************************************************
DebugMonitor_IRQHandler\
                PROC
                EXPORT  DebugMonitor_IRQHandler          [WEAK]
        	MOVS    R0,#0
        	MOVS    R1,#12      ; DebugMon exception number
        	B       assert_failed
        	ENDP
;******************************************************************************
;
; The PendSV handler
;
;******************************************************************************
PendSV_IRQHandler\
                PROC
                EXPORT  PendSV_IRQHandler            [WEAK]
        	MOVS    R0,#0
        	MOVS    R1,#14      ; PendSV exception number
        	B       assert_failed
        	ENDP
;******************************************************************************
;
; The SysTick handler
;
;******************************************************************************
SysTick_IRQHandler\
                PROC
                EXPORT  SysTick_IRQHandler           [WEAK]
;       			 IMPORT  SysTick_IRQHandler

       			LDR     R0, =SysTick_IRQHandler
        	MOVS    R0,#0
        	MOVS    R1,#15      ; SysTick exception number
        	B       assert_failed
        	ENDP

;******************************************************************************
;
; Define Default_Handledr as dummy for all IRQ handlers
;
;******************************************************************************

Default_Handler PROC
		EXPORT	GPIOA_IRQHandler     		[WEAK]     
		EXPORT	GPIOB_IRQHandler     		[WEAK]   
		EXPORT	GPIOC_IRQHandler     		[WEAK]     
		EXPORT	GPIOD_IRQHandler     		[WEAK]   
		EXPORT	GPIOE_IRQHandler     		[WEAK] 
		EXPORT	UART0_IRQHandler     		[WEAK]  
		EXPORT	UART1_IRQHandler     		[WEAK]
		EXPORT	SSI0_IRQHandler      		[WEAK]  
		EXPORT	I2C0_IRQHandler      		[WEAK]  
		EXPORT	PMW0_FAULT_IRQHandler		[WEAK]   
		EXPORT	PWM0_0_IRQHandler    		[WEAK]
		EXPORT	PWM0_1_IRQHandler    		[WEAK]
		EXPORT	PWM0_2_IRQHandler    		[WEAK]  
		EXPORT	QEI0_IRQHandler			[WEAK]
		EXPORT	ADC0SS0_IRQHandler   		[WEAK]  
		EXPORT	ADC0SS1_IRQHandler   		[WEAK]   
		EXPORT	ADC0SS2_IRQHandler   		[WEAK]    
		EXPORT	ADC0SS3_IRQHandler   		[WEAK]    
		EXPORT	WATCHDOG0_IRQHandler  		[WEAK]     
		EXPORT	TIMER0A_IRQHandler   		[WEAK]        
		EXPORT	TIMER0B_IRQHandler   		[WEAK]     
		EXPORT	TIMER1A_IRQHandler   		[WEAK]     
		EXPORT	TIMER1B_IRQHandler   		[WEAK]        
		EXPORT	TIMER2A_IRQHandler   		[WEAK]      
		EXPORT	TIMER2B_IRQHandler   		[WEAK]     
		EXPORT	COMP0_IRQHandler     		[WEAK]   
		EXPORT	COMP1_IRQHandler     		[WEAK]  
		EXPORT	COMP2_IRQHandler     		[WEAK]    
		EXPORT	SYSCTL_IRQHandler    		[WEAK]     
		EXPORT	FLASH_CTRL_IRQHandler  		[WEAK]     
		EXPORT	GPIOF_IRQHandler     		[WEAK]    
		EXPORT	GPIOG_IRQHandler     		[WEAK]     
		EXPORT	GPIOH_IRQHandler     		[WEAK]     
		EXPORT	UART2_IRQHandler     		[WEAK]   
		EXPORT	SSI1_IRQHandler      		[WEAK]        
		EXPORT	TIMER3A_IRQHandler   		[WEAK]      
		EXPORT	TIMER3B_IRQHandler   		[WEAK]      
		EXPORT	I2C1_IRQHandler      		[WEAK]       
		EXPORT	CAN0_IRQHandler      		[WEAK]    
		EXPORT	CAN1_IRQHandler      		[WEAK]           
		EXPORT	EMAC0_IRQHandler     		[WEAK]       
		EXPORT	HIB_IRQHandler     		[WEAK]                          
		EXPORT	USB0_IRQHandler    		[WEAK]      
		EXPORT	PWM0_3_IRQHandler  		[WEAK]             
		EXPORT	UDMA_IRQHandler    		[WEAK]             
		EXPORT	UDMAERR_IRQHandler 		[WEAK]              
		EXPORT	ADC1SS0_IRQHandler 		[WEAK]          
		EXPORT	ADC1SS1_IRQHandler 		[WEAK]          
		EXPORT	ADC1SS2_IRQHandler 		[WEAK]           
		EXPORT	ADC1SS3_IRQHandler 		[WEAK]           
		EXPORT	EPI0_IRQHandler    		[WEAK]                       
		EXPORT	GPIOJ_IRQHandler   		[WEAK]            
		EXPORT	GPIOK_IRQHandler   		[WEAK]           
		EXPORT	GPIOL_IRQHandler   		[WEAK]        
		EXPORT	SSI2_IRQHandler    		[WEAK]       
		EXPORT	SSI3_IRQHandler    		[WEAK]                            
		EXPORT	UART3_IRQHandler   		[WEAK]             
		EXPORT	UART4_IRQHandler   		[WEAK]         
		EXPORT	UART5_IRQHandler   		[WEAK]           
		EXPORT	UART6_IRQHandler   		[WEAK]       
		EXPORT	UART7_IRQHandler   		[WEAK]             
		EXPORT	I2C2_IRQHandler    		[WEAK]       
		EXPORT	I2C3_IRQHandler    		[WEAK]     
		EXPORT	TIMER4A_IRQHandler 		[WEAK]           
		EXPORT	TIMER4B_IRQHandler 		[WEAK]              
		EXPORT	TIMER5A_IRQHandler 		[WEAK]            
		EXPORT	TIMER5B_IRQHandler 		[WEAK]           
		EXPORT	WTIMER0A_IRQHandler		[WEAK]                
		EXPORT	WTIMER0B_IRQHandler		[WEAK]             
		EXPORT	WTIMER1A_IRQHandler		[WEAK]         
		EXPORT	WTIMER1B_IRQHandler		[WEAK]          
		EXPORT	WTIMER2A_IRQHandler		[WEAK]            
		EXPORT	WTIMER2B_IRQHandler		[WEAK]               
		EXPORT	WTIMER3A_IRQHandler		[WEAK]               
		EXPORT	WTIMER3B_IRQHandler		[WEAK]            
		EXPORT	WTIMER4A_IRQHandler		[WEAK]                
		EXPORT	WTIMER4B_IRQHandler		[WEAK]               
		EXPORT	WTIMER5A_IRQHandler		[WEAK]             
		EXPORT	WTIMER5B_IRQHandler		[WEAK]                  
		EXPORT	SYSEXC_IRQHandler  		[WEAK]                        
		EXPORT	I2C4_IRQHandler    		[WEAK]             
		EXPORT	I2C5_IRQHandler    		[WEAK]       
		EXPORT	GPIOM_IRQHandler   		[WEAK]             
		EXPORT	GPION_IRQHandler                [WEAK]
		EXPORT	TAMPER_IRQHandler  		[WEAK]              
		EXPORT	GPIOP0_IRQHandler  		[WEAK]                
		EXPORT	GPIOP1_IRQHandler  		[WEAK]             
		EXPORT	GPIOP2_IRQHandler  		[WEAK]                  
		EXPORT	GPIOP3_IRQHandler               [WEAK] 
		EXPORT	GPIOP4_IRQHandler         	[WEAK]        
		EXPORT	GPIOP5_IRQHandler         	[WEAK]       
		EXPORT	GPIOP6_IRQHandler         	[WEAK]       
		EXPORT	GPIOP7_IRQHandler         	[WEAK]        
		EXPORT	GPIOQ0_IRQHandler         	[WEAK]     
		EXPORT	GPIOQ1_IRQHandler         	[WEAK]    
		EXPORT	GPIOQ2_IRQHandler         	[WEAK]          
		EXPORT	GPIOQ3_IRQHandler         	[WEAK]       
		EXPORT	GPIOQ4_IRQHandler         	[WEAK]     
		EXPORT	GPIOQ5_IRQHandler         	[WEAK]    
		EXPORT	GPIOQ6_IRQHandler         	[WEAK]      
		EXPORT	GPIOQ7_IRQHandler         	[WEAK]   
		EXPORT	GPIOR_IRQHandler          	[WEAK]   
		EXPORT	GPIOS_IRQHandler          	[WEAK]      
		EXPORT	SHAMD5_IRQHandler         	[WEAK]  
		EXPORT	AES_IRQHandler            	[WEAK]     
		EXPORT	DES3DES_IRQHandler        	[WEAK]     
		EXPORT	LCDCONTROLLER_IRQHandler  	[WEAK]       
		EXPORT	TIMER6A_IRQHandler        	[WEAK]     
		EXPORT	TIMER6B_IRQHandler        	[WEAK]    
		EXPORT	TIMER7A_IRQHandler        	[WEAK]      
		EXPORT	TIMER7B_IRQHandler        	[WEAK]         
		EXPORT	I2C6_IRQHandler           	[WEAK]      
		EXPORT	I2C7_IRQHandler           	[WEAK]         
		EXPORT	HIMSCANKEYBOARD_IRQHandler	[WEAK]             
		EXPORT	ONEWIRE_IRQHandler        	[WEAK]        
		EXPORT	HIMPS2_IRQHandler         	[WEAK]         
		EXPORT	HIMLEDSEQUENCER_IRQHandler	[WEAK]            
		EXPORT	I2C8_IRQHandler           	[WEAK]   
		EXPORT	I2C9_IRQHandler           	[WEAK]
		EXPORT	GPIOT_IRQHandler          	[WEAK]   



GPIOA_IRQHandler
GPIOB_IRQHandler
GPIOC_IRQHandler
GPIOD_IRQHandler
GPIOE_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
SSI0_IRQHandler
I2C0_IRQHandler
PMW0_FAULT_IRQHandler
PWM0_0_IRQHandler
PWM0_1_IRQHandler
PWM0_2_IRQHandler
QEI0_IRQHandler
ADC0SS0_IRQHandler
ADC0SS1_IRQHandler
ADC0SS2_IRQHandler
ADC0SS3_IRQHandler
WATCHDOG0_IRQHandler
TIMER0A_IRQHandler
TIMER0B_IRQHandler
TIMER1A_IRQHandler
TIMER1B_IRQHandler
TIMER2A_IRQHandler
TIMER2B_IRQHandler
COMP0_IRQHandler
COMP1_IRQHandler
COMP2_IRQHandler
SYSCTL_IRQHandler
FLASH_CTRL_IRQHandler
GPIOF_IRQHandler
GPIOG_IRQHandler
GPIOH_IRQHandler
UART2_IRQHandler
SSI1_IRQHandler
TIMER3A_IRQHandler
TIMER3B_IRQHandler
I2C1_IRQHandler
CAN0_IRQHandler
CAN1_IRQHandler
EMAC0_IRQHandler
HIB_IRQHandler
USB0_IRQHandler
PWM0_3_IRQHandler
UDMA_IRQHandler
UDMAERR_IRQHandler
ADC1SS0_IRQHandler
ADC1SS1_IRQHandler
ADC1SS2_IRQHandler
ADC1SS3_IRQHandler
EPI0_IRQHandler              
GPIOJ_IRQHandler
GPIOK_IRQHandler
GPIOL_IRQHandler
SSI2_IRQHandler
SSI3_IRQHandler
UART3_IRQHandler
UART4_IRQHandler
UART5_IRQHandler
UART6_IRQHandler
UART7_IRQHandler
I2C2_IRQHandler
I2C3_IRQHandler
TIMER4A_IRQHandler
TIMER4B_IRQHandler
TIMER5A_IRQHandler
TIMER5B_IRQHandler
WTIMER0A_IRQHandler
WTIMER0B_IRQHandler
WTIMER1A_IRQHandler
WTIMER1B_IRQHandler
WTIMER2A_IRQHandler
WTIMER2B_IRQHandler
WTIMER3A_IRQHandler
WTIMER3B_IRQHandler
WTIMER4A_IRQHandler
WTIMER4B_IRQHandler
WTIMER5A_IRQHandler
WTIMER5B_IRQHandler
SYSEXC_IRQHandler
I2C4_IRQHandler
I2C5_IRQHandler
GPIOM_IRQHandler
GPION_IRQHandler
TAMPER_IRQHandler
GPIOP0_IRQHandler
GPIOP1_IRQHandler
GPIOP2_IRQHandler
GPIOP3_IRQHandler
GPIOP4_IRQHandler
GPIOP5_IRQHandler
GPIOP6_IRQHandler
GPIOP7_IRQHandler
GPIOQ0_IRQHandler
GPIOQ1_IRQHandler
GPIOQ2_IRQHandler
GPIOQ3_IRQHandler
GPIOQ4_IRQHandler
GPIOQ5_IRQHandler
GPIOQ6_IRQHandler
GPIOQ7_IRQHandler
GPIOR_IRQHandler
GPIOS_IRQHandler
SHAMD5_IRQHandler
AES_IRQHandler
DES3DES_IRQHandler
LCDCONTROLLER_IRQHandler
TIMER6A_IRQHandler
TIMER6B_IRQHandler
TIMER7A_IRQHandler
TIMER7B_IRQHandler
I2C6_IRQHandler
I2C7_IRQHandler
HIMSCANKEYBOARD_IRQHandler
ONEWIRE_IRQHandler
HIMPS2_IRQHandler
HIMLEDSEQUENCER_IRQHandler
HIMCONSUMERIR_IRQHandler
I2C8_IRQHandler
I2C9_IRQHandler
GPIOT_IRQHandler
        MOVS    r0,#0
        MOVS    r1,#-1      ; 0xFFFFFFF
        B       assert_failed
        ENDP

        ALIGN               ; make sure the end of this section is aligned


;******************************************************************************
;
; The function expected of the C library startup code for defining the stack
; and heap memory locations.  For the C library version of the startup code,
; provide this function so that the C library initialization code can find out
; the location of the stack and heap.
;
;******************************************************************************

; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
				EXPORT  __stack_limit
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  __heap_base
                LDR     R1, =  __stack_limit
                LDR     R2, =  __heap_limit
                LDR     R3, =  __stack_base
                BX      LR

		ENDIF

;******************************************************************************
;
; The function assert_failed defines the error/assertion handling policy
; for the application. After making sure that the stack is OK, this function
; calls Q_onAssert, which should NOT return (typically reset the CPU).
;
; NOTE: the function Q_onAssert should NOT return.
;
; The C proptotype of the assert_failed() and Q_onAssert() functions are:
; void assert_failed(char const *file, int line);
; void Q_onAssert   (char const *file, int line);
;******************************************************************************
        EXPORT  assert_failed
        IMPORT  Q_onAssert
			
assert_failed PROC

        LDR    sp,=__initial_sp  ; re-set the SP in case of stack overflow
		LDR	   R0, =Q_onAssert
        BLX    R0        ; call the application-specific handler
        ;B      .                 ; should not be reached, but just in case...

        ENDP

        ALIGN                    ; make sure the end of this section is aligned

        END                      ; end of module


;                PROC
;                EXPORT  SysTick_IRQHandler           [WEAK]
;       			 IMPORT  SysTick_IRQHandler

;       			LDR     R0, =SysTick_IRQHandler
;        	MOVS    R0,#0
;        	MOVS    R1,#15      ; SysTick exception number
;        	B       assert_failed

