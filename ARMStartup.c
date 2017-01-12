
/**********************************************************************************************************************
  COPYRIGHT
-----------------------------------------------------------------------------------------------------------------------
  \par      copyright
  \verbatim
  Copyright (c) 2014 by Vector Informatik GmbH.                                                  All rights reserved.

                This software is copyright protected and proprietary to Vector Informatik GmbH.
                Vector Informatik GmbH grants to you only those rights as set out in the license conditions.
                All other rights remain with Vector Informatik GmbH.
  \endverbatim
-----------------------------------------------------------------------------------------------------------------------
  FILE DESCRIPTION
-----------------------------------------------------------------------------------------------------------------------
  \file  File:  ARMStartup.c
      Project:  Vector Basic Runtime System
       Module:  BrsHw for all platforms with ARM core
    Generator:  -

  \brief Description:  This is a global, hardware-independent file for the ARM-BRS.
                       It contains the general startup-code for ARM-Core based Systems.
                       All the (hardware depending) rest needs to be defined in BrsHw.c
  
  \attention Please note:
    The demo and example programs only show special aspects of the software. With regard to the fact
    that these programs are meant for demonstration purposes only, Vector Informatik´s liability shall be
    expressly excluded in cases of ordinary negligence, to the extent admissible by law or statute.
**********************************************************************************************************************/

/**********************************************************************************************************************
  AUTHOR IDENTITY
-----------------------------------------------------------------------------------------------------------------------
  Name                          Initials      Company
-----------------------------------------------------------------------------------------------------------------------
  Philipp Duller                vispdr        Vector Informatik GmbH
  Jens Haerer                   visjhr        Vector Informatik GmbH
  Benjamin Walter               visbwa        Vector Informatik GmbH
  Carlos Sanchez                viscsz        Vector Informatik GmbH
  Tobias Mueller                vismto        Vector Informatik GmbH
  Nguyen Le                     vislnn        Vector Informatik GmbH
-----------------------------------------------------------------------------------------------------------------------
  REVISION HISTORY
 ----------------------------------------------------------------------------------------------------------------------
  Version   Date        Author  Description
 ----------------------------------------------------------------------------------------------------------------------
  01.00.00  2013-10-17  vispdr  initial version for Cortex M3
  01.00.01  2013-11-13  vispdr  support for Cortex M4
  01.00.02  2013-11-29  vispdr  support for ARM7
  01.00.03  2013-12-12  vispdr  support for Cortex A8
  01.00.04  2013-12-18  vispdr  support for Cortex M0
  01.00.05  2014-01-15  vispdr  support for ARM9
  01.01.00  2014-01-17  vispdr  diverse arrangements
  01.01.01  2014-03-07  visjhr  added EXPORT Startup_Handler
  01.01.02  2014-03-21  visbwa  added support for ARM compiler
  01.01.03  2014-03-26  viscsz  fixed vector_table_core[] for iMX6 support
  01.01.04  2014-04-03  vismto  added support for iMX6 (A9 Core) with GHS
  01.01.05  2014-04-04  vismto  bugfix for ASM indention
  01.01.06  2014-04-14  vismto  bug-fix in IRQ_ and FIQ_Handler for GHS
  01.01.07  2014-04-28  vismto  added support for IAR with Cortex-M4
  01.01.08  2014-10-14  vislnn  added inline assembler for COMP_GNU ( GNU Linaro )  
                                <Warning> : STACK_SIZE defined in Makefile cannot be passed to 
                                with inline assembler GNU
  01.01.09  2014-10-27  vissgr  bugfix for cortex A8
  01.01.10  2014-11-03  vislnn  Added support for Cortex A15 with GNU
  02.00.00  2014-11-17  visbwa  Complete rework of Brs, according to styleguide Vector_BrsHw_2.0
  02.00.01  2014-11-24  vismto  Adapted file for Sta1095 (Cortex-M3 core) with GNU
  02.00.02  2014-11-27  visbwa  Codestyle Review, introduction of BRS_ISR_KEYWORD,
                                added '.' to all section defines (for compatibility reasons)
  02.00.03  2014-11-28  visbwa  Added description to manually set-able stack pointer address
  02.00.04  2014-12-17  vismto  Adapted for Cortex-R5F (Traveo, S6J326CKSA) with GHS
  02.00.05  2014-12-18  vislnn  Added support for compiler TI ARM (Mx) with TI's built-in library
***********************************************************************************************************************/

/*#########################################################################*/
/*### ---------------------> ARMx Series, Cortex-Ax Series <----------- ###*/
/*#########################################################################*/
#if (defined (BRS_CPU_CORE_ARM7) || defined (BRS_CPU_CORE_ARM9) || defined (BRS_CPU_CORE_ARM11) || \
     defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined( BRS_CPU_CORE_CORTEX_A15 ) || \
     defined (BRS_CPU_CORE_CORTEX_R5) )

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~~ -----> COMPILER ABSTRACTION <----- ~~~*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
# if defined (BRS_COMP_GHS)
  #define Section_3(a,b,c)        .section a
  #define Section_4(a,b,c,d)      .section a
  #define Label(a)                a:
  #define EQU                     .equ
  #define EndlessLoopHandler(a)   a: B a
  #define _OR_                    |
  #define main                    main
# endif

# if defined (BRS_COMP_GHS)
  #pragma asm
# endif

;----------------------------------------------------------------------------;
; --------------------------> S T A C K Section <--------------------------- ;
; ~~~> HINT: User32/System mode use the same StackPointer !!!                ;
;            This Stacksize has to be defined in makefile.config             ;
;            Rest sizes have to be defined here.                             ;
;----------------------------------------------------------------------------;

#  if defined (BRS_COMP_GHS)
;/* Stack is defined in linker script! */
#  else /* TODO: vismto - Adapt linker script for ARM Compiler to remove below code! (see linker script GHS!) */
;/* -----> Stacksizes <----- */
FIQ_Stack_Size  EQU     0x00000200
IRQ_Stack_Size  EQU     0x00000400
SVC_Stack_Size  EQU     0x00000200
ABT_Stack_Size  EQU     0x00000200
UND_Stack_Size  EQU     0x00000200
USR_Stack_Size  EQU     BRS_STACK_SIZE  ; Check Makefile.config
Stack_Sum       EQU     (UND_Stack_Size + SVC_Stack_Size + ABT_Stack_Size + FIQ_Stack_Size + IRQ_Stack_Size + USR_Stack_Size)
#  endif

#  if defined (BRS_COMP_GHS)
;/* Stack is defined in linker script! */
#  else /* TODO: vismto - Adapt linker script for ARM Compiler to remove below code! (see linker script GHS!) */
;/* -----> Definition of the Stack <----- */
  Section_4(.STACK,NOINIT,READWRITE,ALIGN=3)

Label(Stack_Mem)       SPACE   Stack_Sum     ; The SPACE directive reserves a zeroed block of memory,
Label(Stack_Top)                             ; so this is now the top of the stack, used by Reset-Handler!
#  endif

;/*------------------------------------------------------------------*/
;/* Common defines (Current Program Status Register (CPSR)-Bits):    */
;/*------------------------------------------------------------------*/
;/* --> Mode, correspording to bits in CPSR-Register */
CPSR_A    EQU   0x100     ; bit8: disable abort execption - flag
CPSR_I    EQU   0x80      ; bit7: disable interrupts - flag
CPSR_F    EQU   0x40      ; bit6: disable fast interrupts - flag
CPSR_T    EQU   0x20      ; bit5: indicate thumb execution - flag
CPSR_MODE EQU   0x1F      ; bit0-4: select all
CPSR_USR  EQU   0x10      ; select mode 0: usr
CPSR_FIQ  EQU   0x11      ; select mode 1: fiq
CPSR_IRQ  EQU   0x12      ; select mode 2: irq
CPSR_SVC  EQU   0x13      ; select mode 3: svc
CPSR_ABT  EQU   0x17      ; select mode 7: abt
CPSR_UND  EQU   0x1B      ; select mode 11: und
CPSR_SYS  EQU   0x1F      ; select mode 15: sys

#  if defined (BRS_OS_USECASE_BRS)
;/*----------------------------------------*/
;/* Exception Table:                       */
;/*----------------------------------------*/
  EXPORT _vector_table

  Section_3(.EXCEPTIONTABLE,CODE,READONLY)
Label(_vector_table)
  LDR     pc,=Startup_Handler             ; Reset
  LDR     pc,=UndefInstruction_Handler    ; Undefined Instruction
  LDR     pc,=SoftwareInterrupt_Handler   ; Software Interrupt
  LDR     pc,=PrefetchAbort_Handler       ; Prefetch Abort
  LDR     pc,=DataAbort_Handler           ; Data Abort
  NOP                                     ; reserved
  LDR     pc,=IRQ_Handler                 ; IRQ
  LDR     pc,=FIQ_Handler                 ; FIQ
#  endif /*BRS_OS_USECASE_BRS*/

;/*----------------------------------------*/
;/* Reset-Handler (Startup_Handler)        */
;/*----------------------------------------*/
  EXPORT Startup_Handler

  Section_3(.STARTUP,CODE,READONLY)
Label(Startup_Handler)

  LDR     R0, =Stack_Top
  
#  if defined (BRS_COMP_GHS)
;  FIQ (fiq) - Designed to support data transfer or channel process
  MSR     CPSR_c, #(CPSR_FIQ _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  LDR     R1, =FIQ_Stack_Size
  SUB     R0, R0, R1

;  IRQ (irq) - Used for general purpose interrupt handling
  MSR     CPSR_c, #(CPSR_IRQ _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  LDR     R1, =IRQ_Stack_Size
  SUB     R0, R0, R1

;  Supervisor (svc) - Protected mode for the operating system
  MSR     CPSR_c, #(CPSR_SVC _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  LDR     R1, =SVC_Stack_Size
  SUB     R0, R0, R1

;  Abort mode (abt) - Entered after data or inSTRuction prefetch abort
  MSR     CPSR_c, #(CPSR_ABT _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  LDR     R1, =ABT_Stack_Size
  SUB     R0, R0, R1

;  Undefined (und) - Entered when an undefined inSTRuction is executed
  MSR     CPSR_c, #(CPSR_UND _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  LDR     R1, =UND_Stack_Size
  SUB     R0, R0, R1

;  System (sys) - A privileged user mode for the operating system
  MSR     CPSR_c, #(CPSR_SYS)
  MOV     SP, R0
  LDR     R1, =USR_Stack_Size
  SUB     SL, SP, R1            ; Write the stack bottom address to the stack limit pointer (SL)

#  else /* TODO: vismto - Adapt linker script for ARM Compiler to remove below code! (see linker script GHS!) */
;  FIQ (fiq) - Designed to support data transfer or channel process
  MSR     CPSR_c, #(CPSR_FIQ _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  SUB     R0, R0, #FIQ_Stack_Size

;  IRQ (irq) - Used for general purpose interrupt handling
  MSR     CPSR_c, #(CPSR_IRQ _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  SUB     R0, R0, #IRQ_Stack_Size

;  Supervisor (svc) - Protected mode for the operating system
  MSR     CPSR_c, #(CPSR_SVC _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  SUB     R0, R0, #SVC_Stack_Size

;  Abort mode (abt) - Entered after data or inSTRuction prefetch abort
  MSR     CPSR_c, #(CPSR_ABT _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  SUB     R0, R0, #ABT_Stack_Size

;  Undefined (und) - Entered when an undefined inSTRuction is executed
  MSR     CPSR_c, #(CPSR_UND _OR_ CPSR_I _OR_ CPSR_F)
  MOV     SP, R0
  SUB     R0, R0, #UND_Stack_Size

;  System (sys) - A privileged user mode for the operating system
  MSR     CPSR_c, #(CPSR_SYS)
  MOV     SP, R0
  SUB     SL, SP, #USR_Stack_Size
#  endif /*BRS_COMP_GHS*/

# if defined (BRS_COMP_GHS)
  ; init .bss
  ; clear the .bss and .sbss sections (zero init)

  LDR     r1,=__bss_start
  LDR     r2,=__bss_end
  MOV     r3,#0

Label(bss_loop)
  CMP    R2, R1
  BLT    bss_loop_end
  STR    R3, [R1]       ; must be an aligned memory access!
  ADD    R1, R1, #4
  B      bss_loop

Label(bss_loop_end)
#  endif

;  ----- Branch to Main-Function ----------------------------------
  IMPORT  main
  LDR     R0, =main
  BX      R0          ; Branch to Main-Function

; Just in the case of an unexpected return!
EndlessLoopHandler(Loop)

;/*---------------------------------------*/
;/* Undefined Instruction Handler         */
;/*---------------------------------------*/
  EXPORT UndefInstruction_Handler

EndlessLoopHandler(UndefInstruction_Handler)

;/*---------------------------------------*/
;/* Software Interrupt Handler            */
;/*---------------------------------------*/
  EXPORT SoftwareInterrupt_Handler

EndlessLoopHandler(SoftwareInterrupt_Handler)

;/*---------------------------------------*/
;/* Invalid Prefetch Instruction Handler  */
;/*---------------------------------------*/
  EXPORT PrefetchAbort_Handler

EndlessLoopHandler(PrefetchAbort_Handler)

;/*---------------------------------------*/
;/* Data Abort Handler                    */
;/*---------------------------------------*/
  EXPORT DataAbort_Handler

DataAbort_Handler:

; /* Collect some information about the cause of the data abort! */
 MRC p15, 0, r0, c5, c0, 0    ; /* Data Fault Status Register (DFSR)            */
 MRC p15, 0, r1, c5, c1, 0    ; /*Auxiliary Data Fault Status Register (ADFSR)  */
 MRC p15, 0, r2, c6, c0, 0    ; /* Data Fault Address Register (DAFR)           */

Abort_Loop:
 B Abort_Loop

;/*---------------------------------------*/
;/* Interrupt (IRQ) Handler               */
;/*---------------------------------------*/

#  if defined (BRS_COMP_GHS) && (defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_R5) )
  EXPORT IRQ_Handler

Label(IRQ_Handler)

; Save important registers to the stack and
; adjust LR for correct return after the IRQ_Handler!

  SUB     LR, LR, #4          ; Pre-adjust lr
  SRSFD   SP!, #CPSR_IRQ      ; Save LR and SPRS to IRQ mode stack and store final address in SP
  PUSH    {R0-R12}            ; Save registers R0-R12 to FIQ mode stack

  BL __IRQ_Handler            ; Jump to real IRQ_Handler in BrsHw.c

  POP     {R0-R12}            ; Restore stacked registers
  RFEFD   SP!                 ; Restore saved LR and SPRS from IRQ mode stack

#  else
  EXTERN IRQ_Handler
; Check BrsIntTB.c or BrsHw.c for implementation!

#  endif /*BRS_COMP_GHS && BRS_CPU_CORE_CORTEX_Ax*/

;##############################;
; Fast Interrupt (FIQ) Handler ;
;##############################;

#  if defined (BRS_COMP_GHS ) && (defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_R5) )
  EXPORT FIQ_Handler

Label(FIQ_Handler)

; Save important registers to the stack and
; adjust LR for correct return after the FIQ_Handler!
  SUB     LR, LR, #4          ; Pre-adjust lr
  SRSFD   SP!, #CPSR_FIQ      ; Save LR and SPRS to FIQ mode stack and store final address in SP
  PUSH    {R0-R12}            ; Save registers R0-R12 to FIQ mode stack

  BL __FIQ_Handler            ; Jump to real FIQ_Handler in BrsHw.c

  POP     {R0-R12}            ; Restore stacked registers
  RFEFD   SP!                 ; Restore saved LR and SPRS from FIQ mode stack

#  else
  EXTERN FIQ_Handler
; Check BrsIntTB.c or BrsHw.c for implementation!

#  endif /*BRS_COMP_GHS && BRS_CPU_CORE_CORTEX_Ax*/

;---------------------------;
; Compiler specific things: ;
;---------------------------;

# if defined (BRS_COMP_GHS)
  #pragma endasm
# endif

#endif /*BRS_CPU_CORE_ARMx || BRS_CPU_CORE_CORTEX_Ax*/

/*#########################################################################*/
/*### -----------------------> Cortex M-Series <----------------------- ###*/
/*#########################################################################*/
#if defined (BRS_CPU_CORE_CORTEX_M0) || defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)

/******************************************************
*              Includes                               *
******************************************************/
/*
 * Description: The ARMBrsHw header provides the prototype for BrsHwTimeBaseInterrupt()
 */
#include "ARMBrsHw.h"

/*
 * Description: BrsTypes header is the abstraction for MSR/CBD to get access to types definitions
 */
#include "BrsTypes.h"

# if defined (BRS_COMP_GHS) 
/******************************************************
*              Stack & Heap Definitions               *
******************************************************/
#  if defined (BRS_COMP_GHS) 
  __attribute__ ((section(".STACK"))) uint8 Stack[BRS_STACK_SIZE];
  __attribute__ ((section(".HEAP")))  uint8 Heap[0x00000010];
#  endif

/*******************************************************
*            External functions                        *
*******************************************************/
  extern int main(void);    /* Called @ Reset-Routine */

/*******************************************************
*            Prototypes                                *
*******************************************************/

void BRS_ISR_KEYWORD Startup_Handler(void);  /* See implementation below! */
void BRS_ISR_KEYWORD NMI_Handler(void);
void BRS_ISR_KEYWORD HardFault_Handler(void);
void BRS_ISR_KEYWORD SVC_Handler(void);
void BRS_ISR_KEYWORD PendSV_Handler(void);
  
#  if defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
void BRS_ISR_KEYWORD MemManage_Handler(void);
void BRS_ISR_KEYWORD BusFault_Handler(void);
void BRS_ISR_KEYWORD UsageFault_Handler(void);
void BRS_ISR_KEYWORD DebugMon_Handler(void);
#  endif

/*******************************************************
*            ISR Definitions                           *
*******************************************************/
void BRS_ISR_KEYWORD NMI_Handler(void)         { while(1); }
void BRS_ISR_KEYWORD HardFault_Handler(void)
{
  volatile uint8 bafr = *(uint8*)0xE000ED29;      /* read out bus fault status register                */
  volatile uint32 address = *(uint32*)0xE000ED38; /* read out the address where the bus fault occurred */
  while(1);
}
void BRS_ISR_KEYWORD SVC_Handler(void)         { while(1); }
void BRS_ISR_KEYWORD PendSV_Handler(void)      { while(1); }

#  if defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
void BRS_ISR_KEYWORD MemManage_Handler(void)  { while(1); }
void BRS_ISR_KEYWORD BusFault_Handler(void)   { while(1); }
void BRS_ISR_KEYWORD UsageFault_Handler(void) { while(1); }
void BRS_ISR_KEYWORD DebugMon_Handler(void)   { while(1); }
#  endif /*BRS_CPU_CORE_CORTEX_M3 || BRS_CPU_CORE_CORTEX_M4*/

/******************************************************
*            (Core-)Interrupt Vector Table            *
******************************************************/

void (* const vector_table_core[])(void) = {

/* (Core-)Internal Interrupts */
  (void (*)())&Stack[BRS_STACK_SIZE-1],   /* Stack initialization */
  Startup_Handler,                    /* Reset Handler (see definition below) */
  NMI_Handler,                        /* Non-maskable Interrupt Handler */
  HardFault_Handler,                  /* Hard Fault Handler */

#  if defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
  MemManage_Handler,                  /* MPU Fault Handler */
  BusFault_Handler,                   /* Bus Fault Handler */
  UsageFault_Handler,                 /* Usage Fault Handler */
#  else /* Cortex-M0 */
  0,
  0,
  0,
#  endif

  0,
  0,
  0,
  0,
  SVC_Handler,                        /* SVCall Handler */

#  if defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
  DebugMon_Handler,                   /* Debug Monitor Handler */
#  else /* Cortex-M0 */
  0,
#  endif

  0,
  PendSV_Handler,                     /* PendSV Handler */
  BrsHwTimeBaseInterrupt,             /* SysTick Handler */
};

/*************************************************************************
*            ~~~~~> StackPointer-Placement-Routine <~~~~~                *
* If Vectors are located @ RAM, you'll need to init SP & PC by your own. *
* PC is initialized by Linkerflag (-e / --entry), SP by this routine     *
*************************************************************************/
#  if defined (BRS_VECTOR_TABLE_LOCATION_RAM)
#  endif /*BRS_VECTOR_TABLE_LOCATION_RAM*/

/*******************************************************
*                    Reset Routine                     *
*******************************************************/
#  if defined (BRS_COMP_GHS)
  __attribute__ ((section(".STARTUP")))
#  endif

void BRS_ISR_KEYWORD Startup_Handler(void)
{
  uint32 stack_init_cnt;   /* Counter to initialize the stack */

/** ----- SP initialisation - (if it's not automatically done) -------- **/
#  if defined (BRS_VECTOR_TABLE_LOCATION_RAM)
#   if defined (BRS_COMP_GHS)
  #pragma asm

  LDR r0, =Stack      ; Load Stack-Adress -> R0
  MOV r1, #BRS_STACK_SIZE ; Load Stack-Size -> R1
  SUB r1, r1, #1      ; Reduce Stack-Size about 1 Bytes
  ADD r0, r0, r1      ; Compute Stack-Top-Adresse
  MOV sp, r0          ; Stack-Top-Adresse -> SP

  #pragma endasm
#   endif /*BRS_COMP_x*/
#  endif /*BRS_VECTOR_TABLE_LOCATION_RAM*/

/** ----- Stack initialization ---------------------------------------- **/
  for( stack_init_cnt = 0; stack_init_cnt < BRS_STACK_SIZE; stack_init_cnt++ )
    {
      Stack[stack_init_cnt] = 0xFF;
    }

/** ----- Jump to main() - NEVER RETURNS! ----------------------------- **/
  main();

/** ----- Endless Loop ------------------------------------------------ **/
  while(1); /* Catch the case of an unexpected return from main()! */
}

/*******************************************************
*               Compiler specific things               *
*******************************************************/

# endif /*BRS_COMP_GNU || BRS_COMP_GHS || BRS_COMP_ARM || BRS_COMP_KEIL || BRS_COMP_IAR*/

#endif /*Cortex-M Series*/
