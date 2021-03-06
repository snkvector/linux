
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
  \file  File:  BrsAsrMain.c
      Project:  Vector Basic Runtime System for MICROSAR4
       Module:  BrsAsrMain
    Generator:  -

  \brief Description:  Main File of BrsAsr4 contains
                       - Main function (start of initialization (EcuM)
                       - InitTask, Main and Background Task which can be used by the operating system
                       - Exception handling
  
  \attention Please note:
    The demo and example programs only show special aspects of the software. With regard to the fact
    that these programs are meant for demonstration purposes only, Vector Informatik liability shall be
    expressly excluded in cases of ordinary negligence, to the extent admissible by law or statute.
**********************************************************************************************************************/

/**********************************************************************************************************************
  AUTHOR IDENTITY
-----------------------------------------------------------------------------------------------------------------------
  Name                          Initials      Company
-----------------------------------------------------------------------------------------------------------------------
  Benjamin Walter               visbwa        Vector Informatik GmbH
-----------------------------------------------------------------------------------------------------------------------
  REVISION HISTORY
 ----------------------------------------------------------------------------------------------------------------------
  Version   Date        Author  Description
 ----------------------------------------------------------------------------------------------------------------------
  01.00.00  2014-02-03  visbwa  Initial creation of new BrsAsr4 code basis
  01.00.01  2014-04-11  visbwa  Removed DeclareTask() calls as this is not needed by AsrOs
  01.00.02  2014-08-19  visbwa  Added for SafeContext and/or MultiCore OS (inclusive MemMap usage),
                                added legal wording within header information
  01.00.03  2014-09-11  visbwa  Encapsulated mainTask and backgroundTask to be configuration dependent,
                                added support for UseCase w/o OS
  01.00.04  2014-10-16  visbwa  IdleTask_Core0 renamed to IdleTask and always available with OS-support (for new OS),
                                osInitialize() always called with OS-support,
                                added support for MultiCore SafeWatchdog Stack
  01.00.05  2014-10-20  visbwa  encapsulated OS Hook templates by BRSASR_ENABLE_OSSUPPORT,
                                eliminated use of BRS_ENABLE_TIMER_INTERRUPT
  01.00.06  2015-02-06  visbwa  Replaced MSEC by OS_MS2TICKS to support new OS versions
  01.00.07  2015-02-09  visbwa  Replaced OS_MS2TICKS by OS_MS2TICKS_SystemTimer and added error-check,
                                added call of BrsHwPreInitPowerOn (encapsulated by BRSHW_PREINIT_AVAILABLE)
  01.00.08  2015-03-12  visbwa  Encapsuled call of BrsHwSetLed() with BRS_ENABLE_SUPPORT_LEDS
  01.00.09  2015-03-31  visbwa  Changed ECUM_CORE_ID_MASTER to ECUM_CORE_ID_STARTUP within main() for MultiCore
  01.00.10  2015-07-14  visbwa  Removed usage of v_cfg.h (replaced platform defines by compiler.h defines)
  01.01.00  2015-09-03  visbwa  Enhanced VTT support, removed CANoe-Emu support, added support of StartApplication,
                                removed OS-Hooks, as they are now generated by Os_Proxy within OS_Callout_Stubs.c
  01.01.01  2015-09-10  visbwa  Fixed VTT-filter bug (BrsAsrMainInit), 
                                added call of SchM_Init()(VTT-UseCase) and Rte_Start() to InitTasks for MultiCore
  01.01.02  2015-09-11  visbwa  Added sample for ESCAN00078832 workaround, fixed VTT MultiCore StartUp, 
                                included Rte_Main.h for MultiCore StartUp
**********************************************************************************************************************/

/**********************************************************************************************************************
  INCLUDES
**********************************************************************************************************************/
#include "BrsAsrMain.h"

#include "BrsAsrAppl.h"

#if defined (BRSASR_ENABLE_SAFEWDGSUPPORT)
# include "WdgM.h"
#endif

/* Ecu State Manager has to be available in system (BSW module or BRS Stubb) */
#include "EcuM.h"

#if defined (BRSASR_ENABLE_SAFECTXSUPPORT)
# include "osekext.h"
#endif

# include "BrsHw.h"

#if defined (BRSASR_ENABLE_OS_MULTICORESUPPORT)
# include "Rte_Main.h"
#endif /*BRSASR_ENABLE_OS_MULTICORESUPPORT*/

#include "ARMBrsHw.h"
/**********************************************************************************************************************
  VERSION CHECK
**********************************************************************************************************************/
#if (  (BRSASRMAIN_MAJOR_VERSION != (0x01)) \
    || (BRSASRMAIN_MINOR_VERSION != (0x01)) \
    || (BRSASRMAIN_PATCH_VERSION != (0x02)) )
# error "Vendor specific version numbers of BrsAsrMain.c and BrsAsrMain.h are inconsistent"
#endif

#ifndef OS_MS2TICKS_SystemTimer
  #error "Please configure here the name of your OS SystemTimer"
  #define OS_MS2TICKS_SystemTimer OS_MS2TICKS_YourTimer
#endif

/**********************************************************************************************************************
  LOCAL CONSTANT MACROS
**********************************************************************************************************************/
#define BRSMAIN_ERRORBUFFERSIZE    120

#if defined (BRSASR_ENABLE_OS_MULTICORESUPPORT)
# if !defined (ECUM_CORE_ID_STARTUP)
  /* compatibility to older EcuM versions */
  #define ECUM_CORE_ID_STARTUP ECUM_CORE_ID_MASTER
# endif
#endif /*BRSASR_ENABLE_OS_MULTICORESUPPORT*/

/**********************************************************************************************************************
  LOCAL FUNCTION MACROS
**********************************************************************************************************************/

/**********************************************************************************************************************
  LOCAL DATA TYPES AND STRUCTURES
**********************************************************************************************************************/

/**********************************************************************************************************************
  LOCAL DATA PROTOTYPES
**********************************************************************************************************************/
#define START_SEC_VAR_ZERO_INIT_16BIT 
#include "MemMap.h"

/**
 * \var wBrsMainCallCounter1ms
 *      Counter for calls of the function BrsAsrMainCyclic1ms
 */
static uint16 wBrsMainCallCounter1ms;

#define STOP_SEC_VAR 
#include "MemMap.h"

#define START_SEC_VAR_ZERO_INIT_16BIT 
#include "MemMap.h"

/**
 * \var bBrsMainToggleLedCounter
 *      Counter for LED toggling within  BrsMainMilliSecondHandler
 */
static uint8 bBrsMainToggleLedCounter;

#define STOP_SEC_VAR
#include "MemMap.h"

/**********************************************************************************************************************
  GLOBAL DATA
**********************************************************************************************************************/
#if !defined (BRSASR_ENABLE_OSSUPPORT)
/**
 * \var gbBrsMainIsrMilliSecondFlag
 *      The BRS internal timing is based on a cyclic called function. The call 
 *      must be performed each 1ms. This is done by an endless loop polling 
 *      this variable for being '1'. The value is set to '1' by a periodic 
 *      interrupt function in the hardware dependent module HW_CTRL. The value 
 *      is cleared to '0', if the 1ms event has been handled. The separation of 
 *      hardware independent and hardware dependent parts is the only reason to 
 *      export this variable here.
 *      Values: 0, wait for next 1ms event; 1, 1ms event occured
 */
 volatile uint8 gbBrsMainIsrMilliSecondFlag;
#endif /*!BRSASR_ENABLE_OSSUPPORT*/

/**********************************************************************************************************************
  LOCAL FUNCTION PROTOTYPES
**********************************************************************************************************************/
static void BrsAsrMainInit(void);
static void BrsAsrMainCyclic1ms(void);

 
/**********************************************************************************************************************
  LOCAL FUNCTIONS
**********************************************************************************************************************/
#if !defined (_MICROSOFT_C_VTT_)
/***********************************************************************************************************************
 *  BrsAsrMainInit
 **********************************************************************************************************************/
/*! \brief      Main initialization routine. 
 *              Contains initialisation of BRSModules and BrsMain specific initialization
 *  \param[in]  -
 *  \param[out] -
 *  \return     -
 *  \context    Function is called from main 
 **********************************************************************************************************************/
void BrsAsrMainInit(void)
{
  BrsHwDisableInterruptAtPowerOn();
  
#if defined (BRSHW_PREINIT_AVAILABLE)
  BrsHwPreInitPowerOn();
#endif
  
#if defined (VGEN_ENABLE_CAN_DRV)
  /* visbwa: up to now, only DrvCan relevant things done inside there */
  BrsAsrAppl_Init();
#endif /*VGEN_ENABLE_CAN_DRV*/

#if defined (BRSASR_ENABLE_MULTICONFIG)
  BrsAsr_Configuration = NULL_PTR;
#endif

#if defined (BRS_ENABLE_TESTCONTROL)
  BrsTcc_Init();
#endif

#if defined (BRSASR_ENABLE_TGFSUPPORT)
  TsiTestGeneratorFrameworkInitPowerOn();
#endif

#if !defined (VGEN_ENABLE_DRVWD)
  BrsHwWatchdogInitPowerOn();
#endif

#if !defined (VGEN_ENABLE_DRVMCU)
  BrsHwPllInitPowerOn();
#endif

#if !defined (VGEN_ENABLE_DRVPORT)
  BrsHwPortInitPowerOn();
#endif

  BrsHwEvaBoardInitPowerOn();

#if !defined (BRSASR_ENABLE_OSSUPPORT)
  gbBrsMainIsrMilliSecondFlag = 0;

  /** If no OSEK OS is used, the system has to create the time base by its own.
  *  Please take care that the ISR function is alredy defined and the ISR vector
  *  is set up correctly at this point.*/
  BrsHwTimeBaseInitPowerOn();
#endif /*!BRSASR_ENABLE_OSSUPPORT*/

#if defined (BRSASR_ENABLE_CPULOAD_MEASUREMENT)
  CpuLoad_Init();
#endif

  wBrsMainCallCounter1ms   = 0;
  bBrsMainToggleLedCounter = 0;
}
#endif /*_MICROSOFT_C_VTT_*/

/**********************************************************************************************************************
   GLOBAL FUNCTIONS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 *  BrsMainExceptionHandler
 **********************************************************************************************************************/
/*! \brief      This is the central exeption handler of BRS.
 *              All BRS modules and all CANbedded specific fatal error handler will
 *              call this API in case an assertion has failed.
 *
 *  \param[in]  ErrorCode shall hold the caller specific error code (uint8)
 *  \param[in]  ModuleCode shall describe the caller; please use the CI constant as parameter (uint8)
 *  \param[in]  LineNumber shall be the line where the assertion has failed, or, 
 *              if not available on caller location, the line where this API is 
 *              called from
 *  \param[out] -
 *  \return     -
 *
 *  \attention  This function implements an endless loop with locked interrupts.
 *              The recommendation is to set a breakpoint on top of this function 
 *              to see if any assertion has failed during the code execution.
 *
 *  \post       Due to an assertion has failed and the endless loop runs with
 *              locked global interrupts there will be no life after the call ...
 **********************************************************************************************************************/
void BrsMainExceptionHandler(uint8  ErrorCode, uint8  ModuleCode, uint16 LineNumber) 
{
#if defined (BRSASR_ENABLE_SAFECTXSUPPORT)
  while(1)
  {}
  
#else

# if defined (_MICROSOFT_C_VTT_)
  char error[BRSMAIN_ERRORBUFFERSIZE];

  sprintf_s(error, BRSMAIN_ERRORBUFFERSIZE, "BrsMainExceptionHandler Code: [0x%x] ModuleCode: [0x%x] LineNumber: [0x%x]", ErrorCode, ModuleCode, LineNumber);

  CANoeAPI_WriteString(error);
  ShutdownOS(0);

# else
  volatile uint8 BrsMain_Continue;
  BrsMain_Continue = 0;
  BrsHwDisableInterrupt(); 
  while (BrsMain_Continue == 0) 
  { 
    /* Set BrsMain_Continue to 1 to continue here. 
     *  If the debugger is not able to show the stack properly, this mechanism can be used to find the 
     *  source of the exception.
     */
  }
# endif /*_MICROSOFT_C_VTT_*/
#endif /*BRSASR_ENABLE_SAFECTXSUPPORT*/
}

/***********************************************************************************************************************
 *  BrsAsrMainCyclic1ms
 **********************************************************************************************************************/
/*! \brief      One millisecond handler for BrsMain 
 *               - Executes retransmission of BRS TCC messages
 *               - Toggling of LED (alive signal)
 *               - BRS Test code (1s cyclic negative TCC response message)
 *  \param[in]  -
 *  \param[out] -
 *  \return     -
 *  \context    Function is called each millisecond either from the main loop or from the BRS main task (TASK(mainTask))
 **********************************************************************************************************************/
static void BrsAsrMainCyclic1ms(void)
{
  wBrsMainCallCounter1ms++;

  if (wBrsMainCallCounter1ms == 500)
  {
    wBrsMainCallCounter1ms = 0;
    bBrsMainToggleLedCounter++;
    
#if defined (BRS_ENABLE_SUPPORT_LEDS)
    /* Cyclic - 500ms for LED */
    BrsHwSetLed(BRSHW_LED_SYSTEM_OK, (uint8) (bBrsMainToggleLedCounter & 0x01));
#endif
  }
}

/***********************************************************************************************************************
 *  InitTask
 **********************************************************************************************************************/
/*! \brief      InitTask to call EcuM_StartupTwo()
 *              
 *  \param[in]  -
 *  \param[out] -
 *  \return     -
 *  \context    The task is started once by the OS
 **********************************************************************************************************************/
//david: vorher InitTask
TASK(InitTaskHw)
{
#if defined (BRSASR_ENABLE_OS_MULTICORESUPPORT)
  /* Workaround for RTE ESCAN00078832 */
  /* Use this code, if you get a Det Error at the end of Rte_Start() on MasterCore */
  /* Rte_Start() on the SlaveCores has to be called first, before Rte_Start() on MasterCore */
  /* SET THIS InitTask TO FULL PREEMPTIVE (OsTaskSchedule) within OsConfig! */
  /*while(Rte_InitState_1 != RTE_STATE_INIT)
  {
    (void)Schedule();
  }*/
#endif /*BRSASR_ENABLE_OS_MULTICORESUPPORT*/

  EcuM_StartupTwo();

  (void)TerminateTask();
}

#if defined (BRSASR_ENABLE_CPULOAD_MEASUREMENT) || defined (BRS_ENABLE_SUPPORT_LEDS)
/***********************************************************************************************************************
 *  mainTask
 **********************************************************************************************************************/
/*! \brief      mainTask executes the 1 millisecond handler if an operating system is used
 *              The function initiates calls to BrsAsrMainCyclic1ms
 *              
 *  \param[in]  -
 *  \param[out] -
 *  \return     -
 *  \context    The task is started once by the OS and activated by the OS event EvCyclicAlarm_1ms
 **********************************************************************************************************************/
TASK(mainTask)
{
#if defined (BRSASR_ENABLE_OSSUPPORT)
  EventMaskType ev;

  (void)SetRelAlarm(CyclicAlarm_1ms, OS_MS2TICKS_SystemTimer(1), OS_MS2TICKS_SystemTimer(1));
#endif /*BRSASR_ENABLE_OSSUPPORT*/

  for(;;)
  {
#if defined (BRSASR_ENABLE_OSSUPPORT)
    (void)WaitEvent(EvCyclicAlarm_1ms);
    (void)GetEvent(mainTask, &ev);
    (void)ClearEvent(ev);
    if(ev & EvCyclicAlarm_1ms)
    {
#else
    if (gbBrsMainIsrMilliSecondFlag > 0)
    {
      gbBrsMainIsrMilliSecondFlag = 0;
#endif /*BRSASR_ENABLE_OSSUPPORT*/

      /* 1ms event detected, call the ms handler */
#if defined (BRSASR_ENABLE_CPULOAD_MEASUREMENT)
      if (CpuLoad_MainFunction() != 0) 
#endif
      BrsAsrMainCyclic1ms();
    }
 
  }
}
#endif /*BRSASR_ENABLE_CPULOAD_MEASUREMENT || BRS_ENABLE_SUPPORT_LEDS*/

#if defined (BRSASR_ENABLE_OSSUPPORT)

# if defined (BRSASR_ENABLE_CPULOAD_MEASUREMENT) || defined (BRSASR_ENABLE_TGFSUPPORT)
/***********************************************************************************************************************
 *  backgroundTask
 **********************************************************************************************************************/
/*! \brief      backgroundTask 
 *              
 *  \param[in]  -
 *  \param[out] -
 *  \return     -
 *  \context    The task is started once by the OS looping forever unless the OS processes interrupts or 
 *              tasks with higher priority
 **********************************************************************************************************************/
TASK(backgroundTask)
{
  for(;;) 
  {
# if defined (BRSASR_ENABLE_CPULOAD_MEASUREMENT)
    SuspendAllInterrupts();
    CpuLoad_BackgroundFunction();
    ResumeAllInterrupts();
# endif
# if defined (BRSASR_ENABLE_TGFSUPPORT)
    TsiTestGeneratorFrameworkBackgroundTask();
# endif
    (void)Schedule();
  }
}
# endif /*BRSASR_ENABLE_CPULOAD_MEASUREMENT || BRSASR_ENABLE_TGFSUPPORT*/

TASK(IdleTask)
{
  for(;;) 
  {
    (void)Schedule();
  }
}

# if defined (BRSASR_ENABLE_OS_MULTICORESUPPORT)
TASK(IdleTask_Core1)
{
  for(;;) 
  {
    (void)Schedule();
  }
}

TASK(IdleTask_Core2)
{
  for(;;) 
  {
    (void)Schedule();
  }
}

TASK(InitTask_Core1)
{
  EcuM_StartupTwo();

  Rte_Start();

  (void)TerminateTask();
}

TASK(InitTask_Core2)
{
  EcuM_StartupTwo();

  Rte_Start();

  (void)TerminateTask();
}

# endif /*BRSASR_ENABLE_OS_MULTICORESUPPORT*/

# if defined (BRSASR_ENABLE_SAFEWDGSUPPORT)
/***********************************************************************************************************************
 *  WdgStack_ASILD_Task
 **********************************************************************************************************************/
/*! \brief      Initialization / Cyclic triggering of Safe Watch Dog (trusted code)
 *              
 *  \param[in]  -
 *  \param[out] -
 *  \return     -
 *  \context    The task is started once by the OS looping forever unless the OS processes interrupts or 
 *              tasks with higher priority
 *              The task activated by the OS event WdgStack_ASILD_Alarm
  **********************************************************************************************************************/
TASK(WdgStack_ASILD_Task)
{
  EventMaskType ev;
  
  BrsAsrInit_SafeWdg();
  
  (void)SetRelAlarm(WdgStack_ASILD_Alarm, OS_MS2TICKS_SystemTimer(1), OS_MS2TICKS_SystemTimer(10));
  
  for(;;)
  {
    (void)WaitEvent(WdgStack_ASILD_Event);
    (void)GetEvent(WdgStack_ASILD_Task, &ev);
    (void)ClearEvent(ev);
    
    WdgM_MainFunction();
  }
}

#  if defined (BRSASR_ENABLE_SAFEWDG_MULTICORESUPPORT)
TASK(WdgStack_ASILD_Task_Core1)
{
  EventMaskType ev;
  
  BrsAsrInit_SafeWdg_Core1();
  
  (void)SetRelAlarm(WdgStack_ASILD_Alarm_Core1, OS_MS2TICKS_SystemTimer(1), OS_MS2TICKS_SystemTimer(10));
  
  for(;;)
  {
    (void)WaitEvent(WdgStack_ASILD_Event_Core1);
    (void)GetEvent(WdgStack_ASILD_Task_Core1, &ev);
    (void)ClearEvent(ev);
    
    WdgM_MainFunction();
  }
}

TASK(WdgStack_ASILD_Task_Core2)
{
  EventMaskType ev;
  
  BrsAsrInit_SafeWdg_Core2();
  
  (void)SetRelAlarm(WdgStack_ASILD_Alarm_Core2, OS_MS2TICKS_SystemTimer(1), OS_MS2TICKS_SystemTimer(10));
  
  for(;;)
  {
    (void)WaitEvent(WdgStack_ASILD_Event_Core2);
    (void)GetEvent(WdgStack_ASILD_Task_Core2, &ev);
    (void)ClearEvent(ev);
    
    WdgM_MainFunction();
  }
}
#  endif /*BRSASR_ENABLE_SAFEWDG_MULTICORESUPPORT*/

# endif /*BRSASR_ENABLE_SAFEWDGSUPPORT*/

#endif /*BRSASR_ENABLE_OSSUPPORT*/

#if defined (BRSASR_ENABLE_SAFEWDGSUPPORT)
/* Necessary APIs for Interrupt handling in TTTech WdgM */
void GlobalSuspendInterrupts(void){
  SuspendAllInterrupts();
}

void GlobalRestoreInterrupts(void){
  ResumeAllInterrupts();
}
#endif /*BRSASR_ENABLE_SAFEWDGSUPPORT*/

/***********************************************************************************************************************
 *  main / CANoeAPI_Main
 **********************************************************************************************************************/
/*! \brief      Main function  
 *              
 *  \param[in]  -
 *  \param[out] -
 *  \return     always 0 as the function is not expected to return
 *  \context    Called by the startup code
 **********************************************************************************************************************/
int main(void) 
{
# if defined (BRSASR_ENABLE_OS_MULTICORESUPPORT)
  /* some calls are only necessary on master core */
  
// if(GetCoreID()==ECUM_CORE_ID_STARTUP)
 //{
# endif /*BRSASR_ENABLE_OS_MULTICORESUPPORT*/
  /*
 __asm("MRC p15, 0, r1, c1, c0, 0  ");     
    __asm("BIC r1, r1, #0x1   ");  
    __asm("MCR p15, 0, r1, c1, c0, 0  ");      
    __asm("MRC p15, 0, r1, c1, c0, 0  ");  
    __asm("BIC r1, r1, #(0x1 << 12) ");  
    __asm("BIC r1, r1, #(0x1 << 2)  ");  
    __asm("MCR p15, 0, r1, c1, c0, 0 ");     
    __asm("MOV r1, #0 "); 
    __asm("MCR p15, 0, r1, c7, c5, 0 ");  */
  
    
  BrsAsrMainInit();
  
# if defined (BRSASR_ENABLE_OSSUPPORT)
  /* This call is necessary to use OS APIs (e.g. to disable/enable interrupts) before StartOs() */
  osInitialize();

#  if defined (BRSASR_ENABLE_SAFECTXSUPPORT)
#   if defined (V_CPU_RH850) ||\
       defined (BRS_CPU_RH850)
  /* For FeatureSet1 && Version 6.1.x and earlier */
  /*  osInitializeExceptions();
  osInitSysCallRegisters(); */
  /* For FeatureSet1 && Version 6.2.x */
  osInitINTCFG();
#   endif

  /* Check, if the SafeCtx OS of your hardware platform needs this call */
  /* e.g. MPC56xx needs it, V850 does not provide it */
#   if defined (V_CPU_V85X) || defined (V_CPU_RH850) || defined (V_CPU_TRICORE) ||\
       defined (BRS_CPU_V85X) || defined (BRS_CPU_RH850) || defined (BRS_CPU_AURIX)
#   else
  osInitInterruptUnit();
#   endif

#  endif /*BRSASR_ENABLE_SAFECTXSUPPORT*/
# endif /*BRSASR_ENABLE_OSSUPPORT*/

# if defined (BRSASR_ENABLE_OS_MULTICORESUPPORT)
    osInitMultiCoreOS();
 // }
# endif /*BRSASR_ENABLE_OS_MULTICORESUPPORT*/

  EcuM_Init(); /* never returns */

  return 0;
}
