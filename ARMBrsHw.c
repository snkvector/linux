
/**********************************************************************************************************************
  COPYRIGHT
-----------------------------------------------------------------------------------------------------------------------
  \par      copyright
  \verbatim
  Copyright (c) 2015 by Vector Informatik GmbH.                                                  All rights reserved.

                This software is copyright protected and proprietary to Vector Informatik GmbH.
                Vector Informatik GmbH grants to you only those rights as set out in the license conditions.
                All other rights remain with Vector Informatik GmbH.
  \endverbatim
-----------------------------------------------------------------------------------------------------------------------
  FILE DESCRIPTION
-----------------------------------------------------------------------------------------------------------------------
  \file  File:  ARMBrsHw.c
      Project:  Vector Basic Runtime System
       Module:  BrsHw for all platforms with ARM core
    Generator:  -

  \brief Description:  This is a global, hardware-independent file for the ARM-BRS.
                       This file includes all non-hardware dependent functions, e.g. the timer-configuration for the
                       Millisecond-Timebase. All the (hardware depending) rest needs to be defined in BrsHw.c
  
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
  01.00.00  2013-10-17  vispdr  Initial Version
  01.10.00  2014-01-17  vispdr  Changed it into a Cortex-M Timer-File
  01.20.00  2014-01-22  vispdr  Update
  01.30.00  2014-01-27  vispdr  Included Interrupt Functions
  01.35.00  2014-01-29  vispdr  Included Vector-Table-Movement Function
  01.35.01  2014-03-07  visjhr  Removed call of BrsHwInterruptControllerPowerOn() for all except M-Cores
  01.36.00  2014-03-21  visbwa  Added support for ARM compiler
  01.37.00  2014-03-26  viscsz  Adaptions for iMX6 GHS support
  01.38.00  2014-04-03  vismto  Minor adaptions in BrsHwInterruptDisableNow() and BrsHwInterruptEnableNow()
  01.39.00  2014-04-14  vismto  Added support for Cortex-A5 and improvement of BrsHwVectorTableMovement()
  01.39.01  2014-04-17  viscsz  Fix for iMX6X.
  01.40.00  2014-04-28  vismto  Added support for IAR with Cortex-M4 and bug-fix
  01.41.00  2014-08-21  vismto  Added support for GNU with Cortex-A5
  02.00.00  2014-10-30  visbwa  Complete rework of Brs, according to styleguide Vector_BrsHw_2.0,
                                added support for OS VectorTable
  02.00.01  2014-11-24  vismto  Adapted file for Sta1095 (Cortex-M3 core) with GNU
  02.00.02  2014-11-27  visbwa  Codestyle Review, introduction of BRS_ISR_KEYWORD
  02.00.03  2014-11-28  visbwa  Added description to manually set-able VectorTable location
  02.00.04  2014-12-17  vismto  Adapted for Cortex-R5F (Traveo, S6J326CKSA) with GHS
  02.00.05  2014-12-18  vislnn  Added support for compiler TI ARM (Mx) with TI's built-in library
  02.00.06  2015-01-09  visbwa  Added comment within BrsHwEnableInterruptAtPowerOn() for Cortex-Mx
**********************************************************************************************************************/

/**********************************************************************************************************************
  INCLUDES
**********************************************************************************************************************/
/*
 * Description: The ARMBrsHw header provides all interfaces common to ARM Cortex derivatives
 */
#include "ARMBrsHw.h"

/*
 * Description: The BrsHw header provides all the necessary interfaces to
 *              the micros hardware features like ports, timers, LEDs, ...
 *              This file is part of the BRS.
 */
#include "BrsHw.h"

/*
 * Description: The BrsMain header provides all the necessary interfaces to
 *              the BRS main file.
 *              This file is part of the BRS.
 */
#include "BrsMain.h"

/**********************************************************************************************************************
  VERSION CHECK
**********************************************************************************************************************/
#if (ARMBRSHW_VERSION != 0x0200)
  #error "Header and source file are inconsistent!"
#endif
#if (ARMBRSHW_BUGFIX_VERSION != 0x06)
  #error "Different versions of bugfix in Header and Source used!"
#endif

/**********************************************************************************************************************
  CONFIGURATION CHECK
**********************************************************************************************************************/
#if defined (BRS_COMP_GHS)   
#else
  #error "Unknown compiler specified!"
#endif

#if defined (BRS_VECTOR_TABLE_LOCATION_RAM)   || \
    defined (BRS_VECTOR_TABLE_LOCATION_FLASH)
#else
  #error "Unknown location for VectorTable specified. Please correct the setting of VECTOR_TABLE_LOCATION within Makefile.config!"
#endif

#if defined (BRS_PROGRAM_CODE_LOCATION_RAM)   || \
    defined (BRS_PROGRAM_CODE_LOCATION_FLASH)
#else
  #error "Unknown location for ProgramCode specified. Please correct the setting of PROGRAM_CODE_LOCATION within Makefile.config!"
#endif

#if defined (BRS_CPU_CORE_ARM7) || defined (BRS_CPU_CORE_ARM9) || defined (BRS_CPU_CORE_ARM11) || \
    defined (BRS_CPU_CORE_CORTEX_M0) || defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4) || \
    defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_A15) || \
    defined (BRS_CPU_CORE_CORTEX_R5)
#else
  #error "Unknown ARM Core specified. Please correct the setting of CPU_CORE within Makefile.Platform.config!"
#endif

/**********************************************************************************************************************
  Definition + Macros
**********************************************************************************************************************/
  #define BRSHWNOP10() __asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");

/******************************************************************************
 * Local defines                                                              *
 ******************************************************************************/
#if defined (BRS_CPU_CORE_CORTEX_M0) || defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
  /* SysTick-Registers */
  #define BRS_SYST_CSR    BRS_IOS(uint32, (0xE000E010UL))  /* SysTick Control and Status Register */
  #define BRS_SYST_RVR    BRS_IOS(uint32, (0xE000E014UL))  /* SysTick Reload Value Register       */
  #define BRS_SYST_CVR    BRS_IOS(uint32, (0xE000E018UL))  /* SysTick Current Value Register      */
  #define BRS_SYST_CALIB  BRS_IOS(uint32, (0xE000E01CUL))  /* SysTick Calibration Value Register  */

  #define BRS_SYST_CSR_CONFIG       0x00000007UL           /* Configuration of the SysTick Timer  */

  /* NVIC-Registers */
  #define NVIC_ISER0       BRS_IOS(uint32, (0xE000E100UL))
  #define NVIC_ISER1       BRS_IOS(uint32, (0xE000E104UL))
  #define NVIC_ISER2       BRS_IOS(uint32, (0xE000E108UL))
  #define NVIC_ISER3       BRS_IOS(uint32, (0xE000E10CUL))
  #define NVIC_ISER4       BRS_IOS(uint32, (0xE000E110UL))
  #define NVIC_ISER5       BRS_IOS(uint32, (0xE000E114UL))
  #define NVIC_ISER6       BRS_IOS(uint32, (0xE000E118UL))
  #define NVIC_ISER7       BRS_IOS(uint32, (0xE000E11CUL))

    /* Vector Table Offset Register */
  #define VTOR             BRS_IOS(uint32, (0xE000ED08UL))
#endif /*Cortex-M*/

#if defined (BRS_OS_USECASE_BRS)
  #define BRS_VECTOR_TABLE_SECTION _vector_table
#else
  /* set up the section name of the intvect table of your OS here */
  /* for SingleCore OS, this could be: intvect_table              */
  /* for MultiCore OS, this could be: intvect_table_c0            */
  /* Don't forget the external define of this variable,           */
  /*   some lines below (if Keil or ARM compiler)                 */
  #define BRS_VECTOR_TABLE_SECTION intvect_table_c0
#endif

/******************************************************************************
 * Extern variables                                                           *
 ******************************************************************************/

/******************************************************************************
 * Local variables                                                            *
 ******************************************************************************/
/*
 * Description: This counter is used to count the number of nested calls to
 *              disable and restore the state of the global INT enable bit.
 *              Please note: This variable is used only in this file.
 * Values     : 0 if no BrsHwDisableInterrupt is called and INT is allowed by the
 *              BRS. Value > 0 if INT is locked by the BRS interrupt control API.
 */
static uint8 bBrsHwIntDiCounter;

/******************************************************************************
 * Function definitions                                                            *
 ******************************************************************************/

#if defined (BRS_CPU_CORE_CORTEX_M0) || defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
/*******************************************************************************
* NAME          : BrsHwTimeBaseInitPowerOn
* CALLED BY     : main@BrsMain at power on initialization
* PRECONDITIONS : Interrupt vector must be correct configured and the ISR
*                 function itself should exist ...
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : Programmable Interrupt Timer, PIT initialization for 'FakeOS'
*                 1ms time base generation
********************************************************************************/
void BrsHwTimeBaseInitPowerOn(void)
{
# if defined (BRS_OS_USECASE_BRS)
  BRS_SYST_CSR = 0x00000000UL;                         /* Stop timer                                                     */
  BRS_SYST_RVR = (uint32)BRS_TIMEBASE_CLOCK * 1000UL;  /* Timer reload value (BRS_TIMEBASE_CLOCK is frequency of M-Core) */
  BRS_SYST_CVR = 0x00000000UL;                         /* Clear current count                                            */
  BRS_SYST_CSR = BRS_SYST_CSR_CONFIG;                  /* Use core clock, SysTick IRQ enabled, Enable SysTick counter    */
# endif /*BRS_OS_USECASE_BRS*/

 
}

# if defined (BRSHW_ENABLE_TIMER_INTERRUPT)
/*******************************************************************************
 * NAME          : BrsHwTimeBaseInterrupt
 * CALLED BY     : ARMStartup when counter reaches 0 & interrupt occurs
 * PRECONDITIONS : Interrupt vector must be correct configured and the ISR 
 *                 function itself should exist ...
 * PARAMETERS    : none
 * RETURN VALUE  : none
 * DESCRIPTION   : Increment the one millisecond timer flag and clear the 
                   interrupt flag in the corresponding register.
 ********************************************************************************/
void BRS_ISR_KEYWORD BrsHwTimeBaseInterrupt(void)
{
  BRS_SYST_CSR &= ~(1 << 16);   /* Clear COUNTFLAG */
  /* Increment the "fakeOS" 1ms timer tick now. This interrupt should occur
   * approximately every 1ms. The resolution of the timer is 1000Hz not including
   * interrupt latency. */   
  gbBrsMainIsrMilliSecondFlag++;
}
# endif /*BRSHW_ENABLE_TIMER_INTERRUPT*/
#endif /*Cortex-M Series*/

#if defined (BRS_VECTOR_TABLE_LOCATION_RAM)
# if (defined (BRS_CPU_CORE_CORTEX_M0) || defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)) && !defined (BRS_DERIVATIVE_iMX6X)
void BrsHwVectorTableMovement(void)
{
/**************************************************/
/**               Cortex-M-Series                **/
/** Note: VTOR needn't be supported by every MCU **/
/**************************************************/
  /* Configure the location of the InterruptVectorTable */
  /* Possible values for already evaluated platforms:   */
  /*   Sta1095 (Cortex_M3): VTOR =  0x10000000          */
  #error "Specify the address for BrsHwVectorTableMovement here manually";
#  if defined (BRS_CPU_CORE_CORTEX_M0)
  VTOR &= ~0xFFFFFF00;      /* TBLOFF [31:7] - low bits are always zero */
  VTOR  =  0x________;      /* Fill in full new address */
#  elif defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
  VTOR  =  0x________;      /* Write address of the vector table to VTOR */
#  endif /*BRS_CPU_CORE_CORTEX_M0*/
}

# else /*!BRS_CPU_CORE_CORTEX_Mx*/
/*******************************************************************************************
 * THIS FUNCTION IS ONLY MANDATORY, IF YOU DON'T LINK THE VECTORS TO THE EXPECTED LOCATION *
 *                                                                                         *
 * @brief Function to align the new Vector-Table-Location to the MCU                       *
 * @pre   Startup without any interrupt has to be accomplished                             *
 * @hint  A Vector-Table-Shifting of derivatives older than ARMv7 (not Cortex)             *
 *        isn't possible, because of a fixed Vector-Table-Address.                         *
 *        But you can use the following trick:                                             *
 *        Simply copy the vectors to the excepted location (see below)                     *
 *                                                                                         *
 *  >> PLEASE OVERWRITE THE UNDERLINES WITH YOUR DESIRED OFFSET-VALUE/SRC-DEST-ADDRESS <<  *
 *******************************************************************************************/

#  if defined (BRS_COMP_GHS)
#   if defined (BRS_CPU_CORE_ARM7) || defined (BRS_CPU_CORE_ARM9) || defined (BRS_CPU_CORE_ARM11) || \
        defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_A15) || \
        defined (BRS_CPU_CORE_CORTEX_R5)
#    if defined (BRS_COMP_GHS)
  void BrsHwVectorTableMovement(void)
#    endif /*BRS_COMP_XXX*/
{
#    if defined (BRS_COMP_GHS)
  #pragma asm
#    endif
#    if defined (BRS_CPU_CORE_ARM7) || defined (BRS_CPU_CORE_ARM9) || defined (BRS_CPU_CORE_ARM11)

;/***********************************************/
;/**            ARM7 / ARM9 / ARM11            **/
;/***********************************************/
  ; Copy Exception Vectors to excepted location 
  LDR     R8, =0x________ ; Source
  LDR     R9, =0x________ ; Destination
  LDMIA   R8!, {R0-R7}    ; Load Vectors 
  STMIA   R9!, {R0-R7}    ; Store Vectors 
  LDMIA   R8!, {R0-R7}    ; Load Handler Addresses 
  STMIA   R9!, {R0-R7}    ; Store Handler Addresses
  
#    elif defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_A15) || \
          defined (BRS_CPU_CORE_CORTEX_R5)
  
;/***********************************************/
;/**              Cortex-Ax/Rx Series          **/
;/***********************************************/  

  ;/* Attention: Not every Derivative supports the VBAR register! */

  ; /*Deselect high exception vectors to allow remapping of the vector table */
  MRC p15, 0, r1, c1, c0, 0   ; Read SCTLR (System Control Register)
  BIC r1, r1, #0x2000         ; Clear V (bit 13) in SCTLR -> vector table base address = 0x0 or VBAR!
  MCR p15, 0, r1, c1, c0, 0   ; Write SCTLR

  ; /* Define the base address of the vector table (!!! Only possible if Security Extension (VBAR) is implemented!!!) */
  MRC p15, 0, r1, c12, c0, 0      ; Read  VBAR (Vector Base Address Register)
#     if defined (BRS_COMP_GHS)
  LDR r1, =BRS_VECTOR_TABLE_SECTION  ; Load address of the vector table
#     endif /* BRS_COMP_XXX */
  MCR p15, 0, r1, c12, c0, 0      ; Write VBAR

#    endif /* BRS_CPU_CORE_ARMx [elif] BRS_CPU_CORE_CORTEX_Ax */
    
#    if defined (BRS_COMP_GHS)
  #pragma endasm 
#    endif /*BRS_COMP_XXX*/
#   endif /*BRS_CPU_CORE_ARMx || BRS_CPU_CORE_CORTEX_Ax*/
#  endif /*BRS_COMP_KEIL || BRS_COMP_ARM || BRS_COMP_GHS*/
}
# endif /*(BRS_CPU_CORE_CORTEX_M0 || BRS_CPU_CORE_CORTEX_M3 || BRS_CPU_CORE_CORTEX_M4) && !BRS_DERIVATIVE_iMX6X*/

#endif /*BRS_VECTOR_TABLE_LOCATION_RAM*/

/*******************************************************************************
* NAME          : BrsHwDisableInterruptAtPowerOn
* CALLED BY     : main@BrsMain/BrsAsrMainInit at power on initialization
* PRECONDITIONS : Must be the first function call in main@BrsMain/BrsAsrMainInit
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : Disable the global system interrupt and initialize the INT
*                 lock handler variables.
********************************************************************************/
void BrsHwDisableInterruptAtPowerOn(void)
{
  bBrsHwIntDiCounter = 0;
  
  BrsHwDisableInterrupt(); 
}

/*******************************************************************************
 * NAME          : BrsHwEnableInterruptAtPowerOn
 * CALLED BY     : main@BrsMain at power on initialization
 * PRECONDITIONS : Must be called after all initializations are done
 * PARAMETERS    : none
 * RETURN VALUE  : none
 * DESCRIPTION   : Enable the global system interrupt the first time
 ********************************************************************************/
void BrsHwEnableInterruptAtPowerOn(void)
{
#if defined (BRS_OS_USECASE_BRS)
  /* visbwa: the following sequence is enabling all peripheral interrupts.
             If this causes problems, try to only enable the interrupts,
             used by your actual controller and your actual configuration */
# if defined (BRS_CPU_CORE_CORTEX_M0)
  NVIC_ISER0 = 0xFFFFFFFF;   /* Cortex-M0 only supports 32 Peripheral-Interrupts */
# elif defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
  NVIC_ISER0 = 0xFFFFFFFF;   /* Enable 1st 32 interrupts */
  NVIC_ISER1 = 0xFFFFFFFF;   /* Enable 2nd 32 interrupts */
  NVIC_ISER2 = 0xFFFFFFFF;   /* Enable 3rd 32 interrupts */
  NVIC_ISER3 = 0xFFFFFFFF;   /* Enable 4th 32 interrupts */
  NVIC_ISER4 = 0xFFFFFFFF;   /* Enable 5th 32 interrupts */
  NVIC_ISER5 = 0xFFFFFFFF;   /* Enable 6th 32 interrupts */
  NVIC_ISER6 = 0xFFFFFFFF;   /* Enable 7th 32 interrupts */
  NVIC_ISER7 = 0xFFFFFFFF;   /* Enable 8th 32 interrupts */
# endif

  /* Call platform specific interrupt enabling function in here */
  BrsHwConfigureInterruptsAtPowerOn();
#endif /*BRS_OS_USECASE_BRS*/
  
  BrsHwRestoreInterrupt();
}

/*******************************************************************************
 * NAME          : BrsHwDisableInterrupt
 * CALLED BY     : All modules to disable the global interrupt
 * PRECONDITIONS : none
 * PARAMETERS    : none
 * RETURN VALUE  : none
 * DESCRIPTION   : Disables the global interrupt of the micro. This is done in a 
 *                 "save way" to allow also nested calls of BrsHwDisableInterrupt
 *                 and BrsHwRestoreInterrupt. The first call of BrsHwDisableInterrupt
 *                 stores the current state of the global INT flag and the last 
 *                 call to BrsHwRestoreInterrupt restores the state.
 ********************************************************************************/
void BrsHwDisableInterrupt(void)
{
  if (0 == bBrsHwIntDiCounter)
  {

#if defined (BRS_COMP_GHS) 
# if defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
    /* Global disable IRQs, EXCEPT HardFault and NMI! */
    __asm("MOV R0, #1       ");
    __asm("MSR primask,   R0");
    __asm("MSR faultmask, R0");
# endif /*BRS_CPU_CORE_CORTEX_M3 || BRS_CPU_CORE_CORTEX_M4*/

# if defined (BRS_CPU_CORE_ARM7) || defined (BRS_CPU_CORE_ARM11) || defined (BRS_CPU_CORE_ARM9) || \
     defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_A15) || \
     defined (BRS_CPU_CORE_CORTEX_R5)
    /* Disable IRQ, FIQ */
    __asm("MRS R0, CPSR       "); /* Read CPSR Register                     */
    __asm("ORR R0, R0, 0x040  "); /* Set Asynchronous FIQ Mask bit          */
    __asm("ORR R0, R0, 0x080  "); /* Set Asynchronous IRQ Mask bit          */
    __asm("MSR CPSR_c, R0     "); /* Write CPSR Register (only bits [7:0])  */

#  if defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_A15) || \
    defined (BRS_CPU_CORE_CORTEX_R5)
    /* Disable Data Abort Handler */
    __asm("MRS R0, CPSR       "); /* Read CPSR Register                     */
    __asm("ORR R0, R0, 0x100  "); /* Set Asynchronous Abort Mask bit        */
    __asm("MSR CPSR_x, R0     "); /* Write CPSR Register (only bits [15:8]) */
#  endif /*BRS_CPU_CORE_CORTEX_Ax*/
# endif /*BRS_CPU_CORE_ARMx || BRS_CPU_CORE_CORTEX_Ax*/
#endif /*BRS_COMP_GHS || BRS_COMP_IAR || BRS_COMP_GNU*/
  }
  
  bBrsHwIntDiCounter++;
}

/*******************************************************************************
 * NAME          : BrsHwDisableInterrupt
 * CALLED BY     : All modules to disable the global interrupt
 * PRECONDITIONS : none
 * PARAMETERS    : none
 * RETURN VALUE  : none
 * DESCRIPTION   : Disables the global interrupt of the micro. This is done in a 
 *                 "save way" to allow also nested calls of BrsHwDisableInterrupt
 *                 and BrsHwRestoreInterrupt. The first call of BrsHwDisableInterrupt
 *                 stores the current state of the global INT flag and the last 
 *                 call to BrsHwRestoreInterrupt restores the state.
 ********************************************************************************/
void BrsHwRestoreInterrupt(void)
{
  /* Check for illegal call of BrsHwRestoreInterrupt. If this function is called 
  *  to often, the INT lock works incorrect. */
  if (bBrsHwIntDiCounter == 0)
  {
    BrsMainExceptionHandler( kBrsIllegalInterruptRestoration, BRSERROR_MODULE_BRSHW, (uint16)(__LINE__) );
  }
  bBrsHwIntDiCounter--;
  
  if (bBrsHwIntDiCounter == 0) 
  {  /* enable, if zero */
  
#if defined (BRS_COMP_GHS) 
# if defined (BRS_CPU_CORE_CORTEX_M3) || defined (BRS_CPU_CORE_CORTEX_M4)
    /* Global enable IRQs */
    __asm("MOV R0, #0       ");
    __asm("MSR primask,   R0");
    __asm("MSR faultmask, R0");
# endif /*BRS_CPU_CORE_CORTEX_M3 || BRS_CPU_CORE_CORTEX_M4*/

# if defined (BRS_CPU_CORE_ARM7) || defined (BRS_CPU_CORE_ARM11) || defined (BRS_CPU_CORE_ARM9) || \
     defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_A15) || \
     defined (BRS_CPU_CORE_CORTEX_R5)
    /* Enable IRQ, FIQ */
    __asm("MRS R0, CPSR      "); /* Read CPSR Register                     */
    __asm("BIC R0, R0, 0x040 "); /* Enable FIQ                             */
    __asm("BIC R0, R0, 0x080 "); /* Enable IRQ                             */
    __asm("MSR CPSR_c, R0    "); /* Write CPSR Register (only bits [15:8]) */

#  if defined (BRS_CPU_CORE_CORTEX_A5) || defined (BRS_CPU_CORE_CORTEX_A8) || defined (BRS_CPU_CORE_CORTEX_A9) || defined (BRS_CPU_CORE_CORTEX_A15) || \
    defined (BRS_CPU_CORE_CORTEX_R5)
    /* Enable Data Abort Handler */
    __asm("MRS R0, CPSR      "); /* Read CPSR Register                     */
    __asm("BIC R0, R0, 0x100 "); /* Clear Asynchronous Abort Mask bit      */
    __asm("MSR CPSR_x, R0    "); /* Write CPSR Register (only bits [15:8]) */
#  endif /*BRS_CPU_CORE_CORTEX_Ax*/
# endif /*BRS_CPU_CORE_ARMx || BRS_CPU_CORE_CORTEX_Ax*/
#endif /*BRS_COMP_GHS || BRS_COMP_IAR || BRS_COMP_GNU*/
  }
}

/*******************************************************************************
* NAME          : BrsHwTime100NOP
* CALLED BY     : TimeMeasurement
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : This API is used for the BRS time measurement support to get a
*                 default time value for all measurements with this platform to
*                 be able to compare time measurements on different dates based
*                 on this time result.
********************************************************************************/
void BrsHwTime100NOP(void)
{
  BrsHwDisableInterrupt();
  BRSHWNOP10();
  BRSHWNOP10();
  BRSHWNOP10();
  BRSHWNOP10();
  BRSHWNOP10();
  BRSHWNOP10();
  BRSHWNOP10();
  BRSHWNOP10();
  BRSHWNOP10();
  BRSHWNOP10();
  BrsHwRestoreInterrupt();
}


