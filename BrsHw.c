
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
  \file  File:  BrsHw.c
      Project:  Vector Basic Runtime System
       Module:  BrsHw for Platform iMX
    Generator:  -

  \brief Description:  This is the hardware specific code file for Vector Basic Runtime System (BRS).
                       This file supports: Freescale i.MX53. i.MX25, i.MX35, i.MX6X, i.MX6
  
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
  Carlos Sanchez                viscsz        Vector Informatik GmbH
  Benjamin Walter               visbwa        Vector Informatik GmbH
  Tobias Mueller                vismto        Vector Informatik GmbH
  Stefany Gouvea Vanzeler       vissgr        Vector Informatik GmbH
-----------------------------------------------------------------------------------------------------------------------
  REVISION HISTORY
 ----------------------------------------------------------------------------------------------------------------------
  Version   Date        Author  Description
 ----------------------------------------------------------------------------------------------------------------------
  03.00.00  2013-10-17  vispdr  Initial creation of new BrsHw code basis (usage of ARMcommon)
                                based on 2.03.00 (First version for i.MX53)
  03.01.00  2014-03-26  viscsz  added support for i.MX6X (Cortex-M4 Core)
  03.01.01  2014-03-26  visbwa  rework to combine all derivatives within one file,
                                removed empty Transceiver templates
  03.02.00  2014-04-03  vismto  added support for i.MX6 (Cortex-A9 Core)
  03.02.01  2014-04-04  vismto  added include of v_def.h in header
  03.02.02  2014-04-14  visbwa  added empty dummy functions BrsHwWatchdogInitPowerOn() and 
                                BrsHwEvaBoardInitPowerOn() (mandatory for BrsAsr)
  03.02.03  2014-04-16  vismto  Bug-fix to support new name of vector table in ARMBrsHw.c (ARMCommon)
  03.02.04  2014-04-17  viscsz  Corrected BRS_DERIVATIVE_IMX6X for the iMX6X pin mux functions.
                                Added the FlexCan ISR configuration for iMX6S.
  03.02.05  2014-04-17  vissgr  Bugfix for iMX53
  04.00.00  2014-11-17  visbwa  Complete rework of Brs, according to styleguide Vector_BrsHw_2.0
  04.00.01  2014-11-27  visbwa  Removed BRSHWNOP10() define within header (static part of ArmCommon)
  04.00.02  2015-01-07  visbwa  Removed several obsolete declarations within header
**********************************************************************************************************************/

/**********************************************************************************************************************
  INCLUDES
**********************************************************************************************************************/
/*
 * Description: The BrsHw header provides all the necessary interfaces to
 *              the micros hardware features like ports, timers, LEDs, ...
 *              This file is part of the BRS.
 */
#include "BrsHw.h"

#include "ARMBrsHw.h"

#if defined (VGEN_ENABLE_CAN_DRV)
/*
 * Description: The CAN driver header will provide all the necessary interfaces
 *              to the driver and API functions. This header will be include
 *              with other high level modules like IL, TP or DIAG, too.
 */
# if defined (VGEN_ENABLE_IF_ASRIFCAN)
  /* UseCase MICROSAR */
  #include "Can.h"
# else
  /* UseCase CANbedded */
  #include "can_inc.h"
  #include "can_def.h"
# endif /*VGEN_ENABLE_IF_ASRIFCAN*/
#endif /*VGEN_ENABLE_CAN_DRV*/

/*
 * Description: The BrsMain header provides all the necessary interfaces to
 *              the BRS main file.
 *              This file is part of the BRS.
 */
#include "BrsMain.h"

/**********************************************************************************************************************
  VERSION CHECK
**********************************************************************************************************************/
#if (BRSHW_VERSION != 0x0400)
  #error "Header and source file are inconsistent!"
#endif
#if (BRSHW_BUGFIX_VERSION != 0x02)
  #error "Different versions of bugfix in Header and Source used!"
#endif

/**********************************************************************************************************************
  CONFIGURATION CHECK
**********************************************************************************************************************/
#if defined (BRS_COMP_GHS)
#else
  #error "Unknown compiler specified!"
#endif

#define PORT_INPUT       0
#define PORT_OUTPUT      1
#define PORT_INPUT_ALL   0x00
#define PORT_OUTPUT_ALL  0xFF

/**********************************************************************************************************************
  Global variables
**********************************************************************************************************************/

/**********************************************************************************************************************
  Global const variables
**********************************************************************************************************************/
/*
 * Description: These constants are used to propagate the Versions over
 *              module boundaries.The version numbers are BCD coded. E.g. a sub
 *              version of 23 is coded with 0x23, a bug fix version of 9 is
 *              coded 0x09.
 */
uint8 const kBrsHwMainVersion   = (uint8)(BRSHW_VERSION >> 8);
uint8 const kBrsHwSubVersion    = (uint8)(BRSHW_VERSION & 0xFF);
uint8 const kBrsHwBugfixVersion = (uint8)(BRSHW_BUGFIX_VERSION);

/**********************************************************************************************************************
  Local variables and local HW register
**********************************************************************************************************************/
#if defined (BRS_DERIVATIVE_IMX25)
  /* Module base addresses */
  #define BRSHW_CCM_CCMR (*((volatile uint32 *)CCM_BASE))
  #define BRSHW_CCM_CGR0 (*((volatile uint32 *)(CCM_BASE + 0x002C)))

   /*PAD input/output signals*/
  #define IOMUXC_CAN1_IPP_IND_CANRX_SELECT_INPUT  BRS_IOS(uint32,0x43FAC480)
  #define IOMUXC_CAN2_IPP_IND_CANRX_SELECT_INPUT  BRS_IOS(uint32,0x43FAC484)

  #define IOMUXC_MODE_CAN1              0x06
  #define IOMUXC_SELECT_INPUT_CAN1      0x01
  #define IOMUXC_SW_MUX_CTL_PAD_CANTX1  BRS_IOS(uint32,0x43FAC1F4) /* IOMUXC_SW_MUX_CTL_PAD_GPIO_A */  /* ALT6 */
  #define IOMUXC_SW_MUX_CTL_PAD_CANRX1  BRS_IOS(uint32,0x43FAC1F8) /* IOMUXC_SW_MUX_CTL_PAD_GPIO_B */  /* ALT6 */
  #define IOMUXC_SW_PAD_CTL_PAD_CANTX1  BRS_IOS(uint32,0x43FAC3F0) /* IOMUXC_SW_PAD_CTL_PAD_GPIO_A */
  #define IOMUXC_SW_PAD_CTL_PAD_CANRX1  BRS_IOS(uint32,0x43FAC3F4) /* IOMUXC_SW_PAD_CTL_PAD_GPIO_B */
  #define IOMUXC_MODE_CAN2              0x06
  #define IOMUXC_SELECT_INPUT_CAN2      0x01
  #define IOMUXC_SW_MUX_CTL_PAD_CANTX2  BRS_IOS(uint32,0x43FAC1FC) /* IOMUXC_SW_MUX_CTL_PAD_GPIO_C */  /* ALT6 */
  #define IOMUXC_SW_MUX_CTL_PAD_CANRX2  BRS_IOS(uint32,0x43FAC200) /* IOMUXC_SW_MUX_CTL_PAD_GPIO_D */  /* ALT6 */
  #define IOMUXC_SW_PAD_CTL_PAD_CANTX2  BRS_IOS(uint32,0x43FAC3F8) /* IOMUXC_SW_PAD_CTL_PAD_GPIO_C */
  #define IOMUXC_SW_PAD_CTL_PAD_CANRX2  BRS_IOS(uint32,0x43FAC3FC) /* IOMUXC_SW_PAD_CTL_PAD_GPIO_D */

/* BRS_DERIVATIVE_IMX25 */

#elif defined (BRS_DERIVATIVE_IMX35)
  /* Module base addresses */
  #define AIPS1_BASE   ((void *)0x43f00000)
  #define MAX_BASE     ((void *)0x43f04000)
  #define UART1_BASE   ((void *)0x43f90000)
  #define UART2_BASE   ((void *)0x43f94000)
  #define UART3_BASE   ((void *)0x5000c000)
  #define AIPS2_BASE   ((void *)0x53f00000)
  #define CCM_BASE     ((void *)0x53f80000)
  #define GPT_BASE     ((void *)0x53f90000)
  #define EPIT1_BASE   ((void *)0x53f94000)
  #define EPIT2_BASE   ((void *)0x53f98000)
  #define AVIC_BASE    ((void *)0x68000000)
  #define ESDCTL_BASE  ((void *)0xb8001000)
  #define WEIM_BASE    ((void *)0xb8002000)
  #define M3IF_BASE    ((void *)0xb8003000)

  /* Physical base of SDRAM */
  #define BANK0_BASE  ((void *)0x80000000)

  /* For drivers that support more than one instance, these constants are the
   * number of instances to support. */
  #define EPIT_MAX  2
  #define GPT_MAX   1
  #define UART_MAX  3

  /* Interrupt numbers (into the AVIC) */
  #define UART3_INTNUM  18
  #define EPIT2_INTNUM  27
  #define EPIT1_INTNUM  28
  #define GPT_INTNUM    29
  #define UART2_INTNUM  32
  #define CAN0_INTNUM   43
  #define CAN1_INTNUM   44
  #define UART1_INTNUM  45

  #define EPIT1_INTLVL  5
  #define CAN0_INTLVL   9
  #define CAN1_INTLVL   7

 /* EPIT Register offsets from base address */
  #define EPIT_REG(B, R)  ((volatile uint32 *)((char *)(B) + (R)))

  #define EPIT_CR    0x00
  #define EPIT_SR    0x04
  #define EPIT_LR    0x08
  #define EPIT_CMPR  0x0c
  #define EPIT_CNR   0x10

  /*PAD input/output signals*/

  #define IOMUXC_MODE_CAN1              0x01
  #define IOMUXC_SW_MUX_CTL_PAD_CANTX1  BRS_IOS(uint32,0x43FAC118) /* IOMUXC_SW_MUX_CTL_PAD_I2C2_CLK */
  #define IOMUXC_SW_MUX_CTL_PAD_CANRX1  BRS_IOS(uint32,0x43FAC11C) /* IOMUXC_SW_MUX_CTL_PAD_I2C2_DAT */
  #define IOMUXC_SW_PAD_CTL_PAD_CANTX1  BRS_IOS(uint32,0x43FAC55C) /* IOMUXC_SW_PAD_CTL_PAD_I2C2_CLK */
  #define IOMUXC_SW_PAD_CTL_PAD_CANRX1  BRS_IOS(uint32,0x43FAC560) /* IOMUXC_SW_PAD_CTL_PAD_I2C2_DAT */

  /* FEC_MDC and  FEC_MDIO */
  #define IOMUXC_MODE_CAN2              0x01
  #define IOMUXC_SW_MUX_CTL_PAD_CANTX2  BRS_IOS(uint32,0x43FAC2FC) /* IOMUXC_SW_MUX_CTL_PAD_FEC_MDC */
  #define IOMUXC_SW_MUX_CTL_PAD_CANRX2  BRS_IOS(uint32,0x43FAC300) /* IOMUXC_SW_PAD_CTL_PAD_FEC_MDC */
  #define IOMUXC_SW_PAD_CTL_PAD_CANTX2  BRS_IOS(uint32,0x43FAC760) /* IOMUXC_SW_PAD_CTL_PAD_FEC_MDIO */
  #define IOMUXC_SW_PAD_CTL_PAD_CANRX2  BRS_IOS(uint32,0x43FAC764) /* IOMUXC_SW_PAD_CTL_PAD_FEC_MDIO */

  #define IOMUXC_SELECT_INPUT_CAN1                0x00
  #define IOMUXC_SELECT_INPUT_CAN2                0x02
  #define IOMUXC_CAN1_IPP_IND_CANRX_SELECT_INPUT  BRS_IOS(uint32,0x43FAC7C8)
  #define IOMUXC_CAN2_IPP_IND_CANRX_SELECT_INPUT  BRS_IOS(uint32,0x43FAC7CC)

  #define IOMUXC_SW_MUX_CTL_SION  0x00000010
  #define IOMUXC_SW_MUX_CTL_MODE  0x00000007

  #define IOMUXC_SW_PAD_CTL_DRV   0x00002000 /* drive voltage */
  #define IOMUXC_SW_PAD_CTL_HYS   0x00000100 /* hysteresis */
  #define IOMUXC_SW_PAD_CTL_PULL  0x000000F0 /* pull/keep ctl */
  #define IOMUXC_SW_PAD_CTL_ODE   0x00000008 /* open drain */
  #define IOMUXC_SW_PAD_CTL_DSE   0x00000006 /* drive strength */
  #define IOMUXC_SW_PAD_CTL_SRE   0x00000001 /* slew rate */

  #define AVIC_REG(R)    ((volatile uint32 *)((uint8 *)AVIC_BASE + (R)))

  #define AVIC_INTCNTL     0x00
  #define AVIC_NIMASK      0x04
  #define AVIC_INTENNUM    0x08
  #define AVIC_INTDISNUM   0x0c
  #define AVIC_INTENABLEH  0x10
  #define AVIC_INTENABLEL  0x14
  #define AVIC_INTTYPEH    0x18
  #define AVIC_INTTYPEL    0x1c
  #define AVIC_NIPRIORITY(N)  (0x20 + 4 * (7 - (N)))
  #define AVIC_NIVECSR    0x40
  #define AVIC_FIVECSR    0x44
  #define AVIC_INTSRCH    0x48
  #define AVIC_INTSRCL    0x4c
  #define AVIC_INTFRCH    0x50
  #define AVIC_INTFRCL    0x54
  #define AVIC_NIPNDH     0x58
  #define AVIC_NIPNDL     0x5c
  #define AVIC_FIPNDH     0x60
  #define AVIC_FIPNDL     0x64
  #define AVIC_VECTOR(N)  (0x100 + 4 * (N))

  #define IC_INT_NUM_MAX    64
  #define IC_INT_LEVEL_MAX  16

  #define INT_NEST_MAX 4

/* UART Register offsets from base address */
  #define UART_REG(B, R)  ((volatile uint32 *)((uint8 *)(B) + (R)))

  #define UART_RXD    0x00
  #define UART_TXD    0x40
  #define UART_CR1    0x80
  #define UART_CR2    0x84
  #define UART_CR3    0x88
  #define UART_CR4    0x8c
  #define UART_FCR    0x90
  #define UART_SR1    0x94
  #define UART_SR2    0x98
  #define UART_ESC    0x9c
  #define UART_TIM    0xa0
  #define UART_BIR    0xa4
  #define UART_BMR    0xa8
  #define UART_BRC    0xac
  #define UART_ONEMS  0xb0
  #define UART_TS     0xb4

  #define UART_SR2_TXDC  0x0008
  #define UART_SR2_RDR   0x0001

  void *con_handle;
  void *epit_handle;

  struct uart_info_s
  {
    void    *base;
    unsigned  int_num;
  } uart_info[UART_MAX] = {
    { UART1_BASE, UART1_INTNUM },
    { UART2_BASE, UART2_INTNUM },
    { UART3_BASE, UART3_INTNUM },
  };

  unsigned int_nest = 0;
  unsigned int_nest_cnt[INT_NEST_MAX];

  /* A "handler" is a C function to call, and a void * argument to pass it. */
  static struct Ic_handler_s {
    void (*func)(void *);
    void *arg;
  } Ic_handler[IC_INT_NUM_MAX];
  
  static struct epit_info_s  {
    void    *base;
    unsigned  int_num;
  } epit_info[EPIT_MAX] = {
    { EPIT1_BASE, EPIT1_INTNUM },
    { EPIT2_BASE, EPIT2_INTNUM },
  } ;

  volatile uint32 brsHwIntStatus;
  volatile uint32 brsHwIntCounter;

  void BrsHwAvicDisable(unsigned int_num);
  void BrsHwAvicEnable(unsigned int_num);
  void BrsHwAvicConnect(unsigned int_num, unsigned int_level, void (*func)(void*), void *arg);
  void assert_fail(const char * file, int line, const char * assertion);
  void *epit_init(unsigned epit_num, unsigned prescale, uint32 count);
  void epit_ack(void *epit_handle);
  void epit_enable(void *epit_handle);
  void epit_disable(void *epit_handle);
  void epit_connect(void *epit_handle, unsigned level, void (*func)(void*), void *arg);
  void epit_config(void *epit_handle, unsigned prescale, uint32 count);
  
/* BRS_DERIVATIVE_IMX35 */

#elif defined (BRS_DERIVATIVE_IMX53)
  /* Module base addresses */
  #define UART1_BASE  ((void *)0x53FBC000)
  #define UART2_BASE  ((void *)0x53FC0000)
  #define UART3_BASE  ((void *)0x5000C000)
  #define UART4_BASE  ((void *)0x53FF0000)
  #define CCM_BASE    ((void *)0x53FD4000)
  #define EPIT1_BASE  ((void *)0x53FAC000)
  #define EPIT2_BASE  ((void *)0x53FB0000)
  #define TZIC_BASE   ((void *)0x0FFFC000)

    /*  GPIO  */
  #define GPIO_BASE_1 ((void *)0x53F84000)
  #define GPIO_BASE_2 ((void *)0x53F88000)
  #define GPIO_BASE_3 ((void *)0x53F8C000)
  #define GPIO_BASE_4 ((void *)0x53F90000)
  #define GPIO_BASE_5 ((void *)0x53FDC000)
  #define GPIO_BASE_6 ((void *)0x53FE0000)
  #define GPIO_BASE_7 ((void *)0x53FE4000)

  /* For drivers that support more than one instance, these constants are the
   * number of instances to support. */
  #define EPIT_MAX  2
  #define UART_MAX  4

  /* Interrupt numbers for TZIC */
  #define UART1_INTNUM  31
  #define UART2_INTNUM  32
  #define UART3_INTNUM  33
  #define EPIT1_INTNUM  40
  #define EPIT2_INTNUM  41
  #define GPT_INTNUM    39
  #define CAN0_INTNUM   82
  #define CAN1_INTNUM   83

  #define EPIT1_INTLVL  5
  #define CAN0_INTLVL   9
  #define CAN1_INTLVL   7

  /* EPIT Register offsets from base address */
  #define EPIT_REG(B, R)  ((volatile uint32 *)((char *)(B) + (R)))
  #define EPIT_CR    0x00
  #define EPIT_SR    0x04
  #define EPIT_LR    0x08
  #define EPIT_CMPR  0x0c
  #define EPIT_CNR   0x10

  /* Register offsets from base address */
  
  /* LED */
  /*  select alt1 mux port: pin 7 of instance gpio7 */
  #define IOMUXC_MODE_LED                  0x01
  #define IOMUXC_SW_MUX_CTL_PAD_PATA_DA_1  BRS_IOS(uint32,0x53FA8294)
  #define IOMUXC_SW_PAD_CTL_PAD_PATA_DA_1  BRS_IOS(uint32,0x53FA8614)
  
  #define GPIO_REG(BASE,REG) ((volatile uint32 *)((uint8 *)(BASE) + (REG)))
  /*  data register  */
  #define GPIO_DR 0x00
  /*  direction register  */
  #define GPIO_GDIR 0x04

  /* CAN 1 */
  /*010: Select mux mode: ALT2 mux port: RXCAN of instance: can1.  IOMUXC_SW_MUX_CTL_PAD_KEY_ROW2*/
  /*010: Select mux mode: ALT2 mux port: TXCAN of instance: can1.  IOMUXC_SW_MUX_CTL_PAD_KEY_COL2*/
  #define IOMUXC_MODE_CAN1              0x02
  /*  Muxregister(TX):  IOMUXC_SW_MUX_CTL_PAD_KEY_ROW2  */
  #define IOMUXC_SW_MUX_CTL_PAD_CANTX1  BRS_IOS(uint32,0x53FA8038) /*CANTX1*/
  /*  Muxregister(RX):  IOMUXC_SW_MUX_CTL_PAD_KEY_COL2  */
  #define IOMUXC_SW_MUX_CTL_PAD_CANRX1  BRS_IOS(uint32,0x53FA8034) /*CANRX1*/
  /*  Padregister(TX):  IOMUXC_SW_PAD_CTL_PAD_KEY_ROW2  */
  #define IOMUXC_SW_PAD_CTL_PAD_CANTX1  BRS_IOS(uint32,0x53FA8360) /*CANTX1*/
  /*  Padregister(RX):  IOMUXC_SW_PAD_CTL_PAD_KEY_COL2  */
  #define IOMUXC_SW_PAD_CTL_PAD_CANRX1  BRS_IOS(uint32,0x53FA835C) /*CANRX1*/
  
  #define IOMUXC_SELECT_INPUT_CAN1                0x00
  #define IOMUXC_CAN1_IPP_IND_CANRX_SELECT_INPUT  BRS_IOS(uint32,0x53FA8760)

  /* CAN 2 */
  /*010: Select mux mode: ALT2 mux port: TXCAN of instance: can2. IOMUXC_SW_MUX_CTL_PAD_KEY_COL4*/
  /*010: Select mux mode: ALT2 mux port: RXCAN of instance: can2. IOMUXC_SW_MUX_CTL_PAD_KEY_ROW4*/
  #define IOMUXC_MODE_CAN2                0x02
  /* Muxregister(TX):  IOMUXC_SW_MUX_CTL_PAD_KEY_COL4  */
  #define IOMUXC_SW_MUX_CTL_PAD_CANTX2  BRS_IOS(uint32,0x53FA8044) /*CANTX2*/
  /* Muxregister(RX):  IOMUXC_SW_MUX_CTL_PAD_KEY_ROW4  */
  #define IOMUXC_SW_MUX_CTL_PAD_CANRX2  BRS_IOS(uint32,0x53FA8048) /*CANRX2*/
  /* Padregister(TX):  IOMUXC_SW_PAD_CTL_PAD_KEY_COL4  */
  #define IOMUXC_SW_PAD_CTL_PAD_CANTX2  BRS_IOS(uint32,0x53FA836C) /*CANTX2*/
  /* Padregister(RX): IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4   */
  #define IOMUXC_SW_PAD_CTL_PAD_CANRX2  BRS_IOS(uint32,0x53FA8370) /*CANRX2*/

  #define IOMUXC_SELECT_INPUT_CAN2                0x00
  #define IOMUXC_CAN2_IPP_IND_CANRX_SELECT_INPUT  BRS_IOS(uint32,0x53FA8764)
  
  /* MUX register mask */
  /* force input mode */
  #define IOMUXC_SW_MUX_CTL_SION  0x00000010
  /* Mux mode */
  #define IOMUXC_SW_MUX_CTL_MODE  0x00000007

  /* PAD register mask */
  /* hysteresis */
  #define IOMUXC_SW_PAD_CTL_HYS  0x00000100
  /* Pull / Keep Enable Field */
  #define IOMUXC_SW_PAD_CTL_PKE  0x00000080
  /* Pull / Keep Select Field */
  #define IOMUXC_SW_PAD_CTL_PUE  0x00000040
  /* Pull Up / Down Config. Field */
  #define IOMUXC_SW_PAD_CTL_PUS  0x00000030
  /* Open Drain Enable Field */
  #define IOMUXC_SW_PAD_CTL_ODE  0x00000008
  /* Drive Strength Field */
  #define IOMUXC_SW_PAD_CTL_DSE  0x00000006
  
/****************************************************************************
 *                      Ports                  *
 ****************************************************************************/
  #define IOMUXC_SW_MUX_CTL_PAD_CAN2_STBY 0x00D4 /* CSI0_D5 */
  #define IOMUXC_SW_PAD_CTL_PAD_CAN2_STBY 0x0400
  #define IOMUXC_SW_MUX_CTL_PAD_CAN2_EN   0x00D8 /* CSI0_D6 */
  #define IOMUXC_SW_PAD_CTL_PAD_CAN2_EN   0x0404

  #define IOMUXC_SW_MUX_CTL_PAD_GPIO_14 BRS_IOS(uint32,0x53FA8224)
  #define IOMUXC_SW_PAD_CTL_PAD_GPIO_14 BRS_IOS(uint32,0x53FA8550)

  #define IOMUXC_SW_MUX_CTL_PAD_GPIO_17 BRS_IOS(uint32,0x53FA8340)
  #define IOMUXC_SW_PAD_CTL_PAD_GPIO_17 BRS_IOS(uint32,0x53FA86D0)

  #define IOMUXC_SW_MUX_CTL_PAD_GPIO_18 BRS_IOS(uint32,0x53FA8344)
  #define IOMUXC_SW_PAD_CTL_PAD_GPIO_18 BRS_IOS(uint32,0x53FA86D4)

  #define GPIOBIT(x) (uint32)((uint32)(0x01) << (x))

/****************************************************************************
 *                      TrustZoneInterruptController(TZIC)                  *
 ****************************************************************************/
  #define IC_INT_NUM_MAX    128
  #define IC_INT_LEVEL_MAX  256

  #define INT_NEST_MAX 4
  
  #define TZIC_REG(R) ((volatile uint32 *)((uint8 *)TZIC_BASE + (R)))

  #define TZIC_INTCTRL  0x0000
  #define TZIC_PRIOMASK 0x000C
  
  /* Priority Registers */
  #define TZIC_PRIORITY_BASE 0x0400
  #define TZIC_PRIORITY(x) TZIC_PRIORITY_BASE+(x*4)

  /*The High Priority Pending Registers (HIPND) are used to determine which 
  interrupts are currently pending at the highest active priority.*/
  #define TZIC_HIPND_BASE 0x0D80
  #define TZIC_HIPND(x) TZIC_HIPND_BASE+(x*4)

  /*The Interrupt Security Registers (INTSEC) are used to set interrupts as secure or non-secure */
  #define TZIC_INTSEC_BASE 0x0080   
  #define TZIC_INTSEC(x) TZIC_INTSEC_BASE+(x*4)
    
  /*The Source Set Registers (SRCSET) are used to determine which interrupts are currently asserted and to
  force interrupts to an asserted state. */
  #define TZIC_SRCSET_BASE 0x0200
  #define TZIC_SRCSET(x) TZIC_SRCSET_BASE+(x*4)

  /*The Source Clear Registers (SRCCLEAR) are used to determine which interrupts are currently asserted
  and to clear interrupts from the asserted state*/
  #define TZIC_SRCCLEAR_BASE 0x0280
  #define TZIC_SRCCLEAR(x) TZIC_SRCCLEAR_BASE+(x*4) 
 
  /*The interrupt controller software interrupt register (SWINT) can be used by software to assert or negate
  an interrupt at a specific interrupt number.*/
  #define TZIC_SWINT 0x0F00

  /*The Enable Set Registers (ENSET) are used to enable interrupt requests to the core.*/
  #define TZIC_ENSET_BASE 0x0100    
  #define TZIC_ENSET(x) TZIC_ENSET_BASE+(x*4)

  /*The Enable Clear Registers (ENCLEAR) are used to disable interrupt requests to the core.*/
  #define TZIC_ENCLEAR_BASE 0x0180
  #define TZIC_ENCLEAR(x) TZIC_ENCLEAR_BASE+(x*4)

/**************************************************************************
 *                                UART                                    *
 **************************************************************************/
  #define UART_REG(B, R)  ((volatile uint32 *)((uint8 *)(B) + (R)))

  #define UART_RXD    0x00
  #define UART_TXD    0x40
  #define UART_CR1    0x80
  #define UART_CR2    0x84
  #define UART_CR3    0x88
  #define UART_CR4    0x8c
  #define UART_FCR    0x90
  #define UART_SR1    0x94
  #define UART_SR2    0x98
  #define UART_ESC    0x9c
  #define UART_TIM    0xa0
  #define UART_BIR    0xa4
  #define UART_BMR    0xa8
  #define UART_BRC    0xac
  #define UART_ONEMS  0xb0
  #define UART_TS     0xb4

  #define UART_SR2_TXDC  0x0008
  #define UART_SR2_RDR   0x0001

/****************************************************************************
 *                        Start of Declaration                                 *
 ****************************************************************************/
  void *con_handle;

  struct uart_info_s
  {
    void *base;
    unsigned  int_num;
  } uart_info[UART_MAX] = {
    { UART1_BASE, UART1_INTNUM },
    { UART2_BASE, UART2_INTNUM },
    { UART3_BASE, UART3_INTNUM },
  };

  unsigned int_nest = 0;
  unsigned int_nest_cnt[INT_NEST_MAX];

  /* A "handler" is a C function to call, and a void * argument to pass it. */
  static struct Ic_handler_s {
    void (*func)(void *);
    void *arg;
  } Ic_handler[IC_INT_NUM_MAX];
  
  static struct epit_info_s {
    void *base;
    unsigned  int_num;
  } epit_info[EPIT_MAX] = {
    { EPIT1_BASE, EPIT1_INTNUM },
    { EPIT2_BASE, EPIT2_INTNUM },
  };

  volatile uint32 brsHwIntStatus;
  volatile uint32 brsHwIntCounter;

  void BrsHwTzicConnect(unsigned int_num, unsigned int_level, void (*func)(void*), void *arg);
  void BrsHwTzicEnable(unsigned int_num);
  void assert_fail(const char * file, int line, const char * assertion);
  void *epit_init(unsigned epit_num, unsigned prescale, uint32 count);
  void epit_ack(void *epit_handle);
  void epit_enable(void *epit_handle);
  void epit_disable(void *epit_handle);
  void epit_connect(void *epit_handle, unsigned level, void (*func)(void*), void *arg);
  void epit_config(void *epit_handle, unsigned prescale, uint32 count);
  
  uint8 highestPriorityPendingInt(void);

/* BRS_DERIVATIVE_IMX53 */

#elif defined (BRS_DERIVATIVE_IMX6X) || defined (BRS_DERIVATIVE_IMX6S)

# if defined (BRS_DERIVATIVE_IMX6X)
#  ifndef uint64
  typedef unsigned long long uint64;
#  endif
  void BrsHwIoMuxV3SetupPad(uint64 pad);
#  if defined (VGEN_ENABLE_CAN_DRV)
  static inline void BrsHwGpioSetDir (uint32 gpio_base, uint32 bit, uint8 dir);
  static inline void BrsHwGpioSetVal(uint32 gpio_base, uint32 bit, uint32 value);
#  endif /* VGEN_ENABLE_CAN_DRV */
  
  /* General Read and Write macros */
  #define BRS_RAW_WR(v,a)         BRS_IOS(uint32,a) = (v)
  #define BRS_RAW_RD(a)           BRS_IOS(uint32,a)

  /* Pad defines */ 
  #define PAD_CTL_HYS             (1 << 16)
  #define PAD_CTL_PUS_100K_DOWN   (0 << 14)
  #define PAD_CTL_PUS_47K_UP      (1 << 14)
  #define PAD_CTL_PUS_100K_UP     (2 << 14)
  #define PAD_CTL_PUS_22K_UP      (3 << 14)

  #define PAD_CTL_PUE             (1 << 13)
  #define PAD_CTL_PKE             (1 << 12)
  #define PAD_CTL_ODE             (1 << 11)

  #define PAD_CTL_SPEED_LOW       (1 << 6)
  #define PAD_CTL_SPEED_MED       (2 << 6)
  #define PAD_CTL_SPEED_HIGH      (3 << 6)

  #define PAD_CTL_DSE_DISABLE     (0 << 3)
  #define PAD_CTL_DSE_240ohm      (1 << 3)
  #define PAD_CTL_DSE_120ohm      (2 << 3)
  #define PAD_CTL_DSE_80ohm       (3 << 3)
  #define PAD_CTL_DSE_60ohm       (4 << 3)
  #define PAD_CTL_DSE_40ohm       (5 << 3)
  #define PAD_CTL_DSE_34ohm       (6 << 3)

  #define PAD_CTL_SRE_FAST        (1 << 0)
  #define PAD_CTL_SRE_SLOW        (0 << 0)

  #define MUX_CTRL_OFS_SHIFT      0
  #define MUX_CTRL_OFS_MASK       ((uint64)0xfff << MUX_CTRL_OFS_SHIFT)
  #define MUX_PAD_CTRL_OFS_SHIFT  12
  #define MUX_PAD_CTRL_OFS_MASK   ((uint64)0xfff << MUX_PAD_CTRL_OFS_SHIFT)
  #define MUX_SEL_INPUT_OFS_SHIFT 24
  #define MUX_SEL_INPUT_OFS_MASK  ((uint64)0xfff << MUX_SEL_INPUT_OFS_SHIFT)
  #define MUX_MODE_SHIFT          36
  #define MUX_MODE_MASK           ((uint64)0x1f << MUX_MODE_SHIFT)
  #define MUX_PAD_CTRL_SHIFT      41
  #define MUX_PAD_CTRL_MASK       ((uint64)0x3ffff << MUX_PAD_CTRL_SHIFT)
  #define MUX_SEL_INPUT_SHIFT     59
  #define MUX_SEL_INPUT_MASK      ((uint64)0xf << MUX_SEL_INPUT_SHIFT)

  #define NO_PAD_CTRL             (1 << 17)

  #define MUX_PAD_CTRL(x)         ((uint64)(x) << MUX_PAD_CTRL_SHIFT)

  #define BRS_GPIO_DIR_IN  0
  #define BRS_GPIO_DIR_OUT 1

  #define BRS_LED_ON 1
  #define BRS_LED_OFF 0

  #define IOMUX_PAD(pad_ctrl_ofs, mux_ctrl_ofs, mux_mode, sel_input_ofs, \
          sel_input, pad_ctrl) \
          (((uint64)(mux_ctrl_ofs)  << MUX_CTRL_OFS_SHIFT) | \
           ((uint64)(mux_mode)      << MUX_MODE_SHIFT) | \
           ((uint64)(pad_ctrl_ofs)  << MUX_PAD_CTRL_OFS_SHIFT) | \
           ((uint64)(pad_ctrl)      << MUX_PAD_CTRL_SHIFT) | \
           ((uint64)(sel_input_ofs) << MUX_SEL_INPUT_OFS_SHIFT) | \
           ((uint64)(sel_input)     << MUX_SEL_INPUT_SHIFT))

  /* USER_LED */
  #define MX6SLX_PAD_KEY_COL3__GPIO2_IO_13 \
      IOMUX_PAD(0x03F8, 0x00B0, 5, 0x0000, 0, NO_PAD_CTRL)

  /* CAN 1 TX - Pin: KEY_COL2 */
  #define MX6SLX_PAD_KEY_COL2__CAN1_TX \
      IOMUX_PAD(0x03F4, 0x00AC, 3, 0x0000, 0, NO_PAD_CTRL)
  /* CAN FD TX1 - Pin: KEY_COL2 */
  #define MX6SLX_PAD_KEY_COL2__CANFD_TX1 \
      IOMUX_PAD(0x03F4, 0x00AC, 4, 0x0000, 0, NO_PAD_CTRL)

  /* CAN 1 TX - Pin: QSPI1B_DQS */
  #define MX6SLX_PAD_QSPI1B_DQS__CAN1_TX \
      IOMUX_PAD(0x04F8, 0x01B0, 1, 0x0000, 0, NO_PAD_CTRL)
  /* CAN FD TX1 - Pin: QSPI1B_DQS */
  #define MX6SLX_PAD_QSPI1B_DQS__CANFD_TX1 \
      IOMUX_PAD(0x04F8, 0x01B0, 2, 0x0000, 0, NO_PAD_CTRL)

  /* CAN 1 TX - Pin: SD3_DATA5 */
  #define MX6SLX_PAD_SD3_DATA5__CAN1_TX \
      IOMUX_PAD(0x05B4, 0x026C, 1, 0x0000, 0, NO_PAD_CTRL)
  /* CAN FD TX1 - Pin: SD3_DATA5 */
  #define MX6SLX_PAD_SD3_DATA5__CANFD_TX1 \
      IOMUX_PAD(0x05B4, 0x026C, 2, 0x0000, 0, NO_PAD_CTRL)

  /* CAN 1 RX - Pin: KEY_ROW2 */
  #define MX6SLX_PAD_KEY_ROW2__CAN1_RX \
      IOMUX_PAD(0x0408, 0x00C0, 3, 0x068C, 1, NO_PAD_CTRL)
  /* CAN FD RX1 - Pin: KEY_ROW2 */
  #define MX6SLX_PAD_KEY_ROW2__CANFD_RX1 \
      IOMUX_PAD(0x0408, 0x00C0, 4, 0x0694, 1, NO_PAD_CTRL)

  /* CAN 1 RX - Pin: QSPI1A_SS1_B */
  #define MX6SLX_PAD_QSPI1A_SS1_B__CAN1_RX \
      IOMUX_PAD(0x04E4, 0x019C, 1, 0x068C, 2, NO_PAD_CTRL)
  /* CAN FD RX1 - Pin: QSPI1A_SS1_B */
  #define MX6SLX_PAD_QSPI1A_SS1_B__CANFD_RX1 \
      IOMUX_PAD(0x04E4, 0x019C, 2, 0x0694, 2, NO_PAD_CTRL)

  /* CAN 1 RX - Pin: SD3_DATA7 */
  #define MX6SLX_PAD_SD3_DATA7__CAN1_RX \
      IOMUX_PAD(0x05BC, 0x0274, 1, 0x068C, 0, NO_PAD_CTRL)
  /* CAN FD RX1 - Pin: SD3_DATA7 */
  #define MX6SLX_PAD_SD3_DATA7__CANFD_RX1 \
      IOMUX_PAD(0x05BC, 0x0274, 2, 0x0694, 0, NO_PAD_CTRL)

  /* CAN 2 TX - Pin: KEY_COL3 */
  #define MX6SLX_PAD_KEY_COL3__CAN2_TX \
      IOMUX_PAD(0x03F8, 0x00B0, 3, 0x0000, 0, NO_PAD_CTRL)
  /* CAN FD TX2 - Pin: KEY_COL3 */
  #define MX6SLX_PAD_KEY_COL3__CANFD_TX2 \
      IOMUX_PAD(0x03F8, 0x00B0, 4, 0x0000, 0, NO_PAD_CTRL)

  /* CAN 2 RX - Pin: KEY_ROW3 */
  #define MX6SLX_PAD_KEY_ROW3__CAN2_RX \
      IOMUX_PAD(0x040C, 0x00C4, 3, 0x0690, 1, NO_PAD_CTRL)
  /* CAN FD RX2 - Pin: KEY_ROW3 */
  #define MX6SLX_PAD_KEY_ROW3__CANFD_RX2 \
      IOMUX_PAD(0x040C, 0x00C4, 4, 0x0698, 1, NO_PAD_CTRL)

  /* CAN 2 TX - Pin: QSPI1A_DQS */
  #define MX6SLX_PAD_QSPI1A_DQS__CAN2_TX \
      IOMUX_PAD(0x04D8, 0x0190, 1, 0x0000, 0, NO_PAD_CTRL)
  /* CAN FD TX2 - Pin: QSPI1A_DQS */
  #define MX6SLX_PAD_QSPI1A_DQS__CANFD_TX2 \
      IOMUX_PAD(0x04D8, 0x0190, 2, 0x0000, 0, NO_PAD_CTRL)

  /* CAN 2 RX - Pin: QSPI1B_SS1_B */
  #define MX6SLX_PAD_QSPI1B_SS1_B__CAN2_RX \
      IOMUX_PAD(0x0504, 0x01BC, 1, 0x0690, 2, NO_PAD_CTRL)
  /* CAN FD RX2 - Pin: QSPI1B_SS1_B */
  #define MX6SLX_PAD_QSPI1B_SS1_B__CANFD_RX2 \
      IOMUX_PAD(0x0504, 0x01BC, 2, 0x0698, 2, NO_PAD_CTRL)

  /* CAN 2 RX - Pin: SD3_DATA4 */
  #define MX6SLX_PAD_SD3_DATA4__CAN2_RX \
      IOMUX_PAD(0x05B0, 0x0268, 1, 0x0690, 0, NO_PAD_CTRL)
  /* CAN FD RX2 - Pin: SD3_DATA4 */
  #define MX6SLX_PAD_SD3_DATA4__CANFD_RX2 \
      IOMUX_PAD(0x05B0, 0x0268, 2, 0x0698, 0, NO_PAD_CTRL)

  /* CAN 2 TX - Pin: SD3_DATA6 */
  #define MX6SLX_PAD_SD3_DATA6__CAN2_TX \
      IOMUX_PAD(0x05B8, 0x0270, 1, 0x0000, 0, NO_PAD_CTRL)
  /* CAN FD TX2 - Pin: SD3_DATA6 */
  #define MX6SLX_PAD_SD3_DATA6__CANFD_TX2 \
      IOMUX_PAD(0x05B8, 0x0270, 2, 0x0000, 0, NO_PAD_CTRL)

  /* CAN Transceiver CAN1_2_STBY_B Pin: QSPI1B_DATA3 */
  #define MX6SLX_PAD_QSPI1B_DATA3__GPIO4_IO_27 \
      IOMUX_PAD(0x04F4, 0x01AC, 5, 0x0000, 0, NO_PAD_CTRL)
#  if defined (BRS_CPU_CORE_CORTEX_M4)
 
  void BrsHwStartupRest(void);
  void BrsHwEnableHandler(void);
  extern void (* const vector_table_core[])(void);

  /* Vector table offset register */
  #define BRS_VECTOR_TABLE_OFFSET BRS_IOS(uint32,0xE000ED08)

  /* timer */
  #define BRS_SYS_TICK_CTRL       BRS_IOS(uint32,0xE000E010)
  #define BRS_SYS_TICK_LOAD       BRS_IOS(uint32,0xE000E014)
  #define BRS_SYS_TICK_VALUE      BRS_IOS(uint32,0xE000E018)
  #define BRS_SYS_TICK_CALIB      BRS_IOS(uint32,0xE000E01C)
  #define BRS_CCM_CSCMR2          BRS_IOS(uint32,0x420C4020)
  #define BRS_CCM_CACRR           BRS_IOS(uint32,0x420C4010)
  #define BRS_CCM_ANALOG_PLL_ARM  BRS_IOS(uint32,0x420C8000)

  #define BRS_SYS_TICK_COUNT_FLAG (1<<16)
  #define BRS_SYS_TICK_CLK_SOURCE (1<<2)
  #define BRS_SYS_TICK_INTERRUPT  (1<<1)
  #define BRS_SYS_TICK_ENABLE     (1<<0)
  
  /* Base Addresses */
  #define BRS_CM4_BASE_ADDR       ((uint32)0xE0000000)
  #define BRS_GPIO2_BASE_ADDR     ((uint32)(0x420A0000))
  #define BRS_GPIO4_BASE_ADDR     ((uint32)(0x420A8000))
  #define BRS_UART_BASE           ((uint32)(0x421E8000))

  #define BRS_CCM_BASE_ADDR       ((uint32)0x420C4000)
  #define BRS_CCM_CSCDR1          (CCM_BASE_ADDR + 24)

  #define BRS_AIPS_BASE_ADDR      ((uint32)0x420E0000) /* static uint32_t base = 0x420E0000; */
  #define BRS_EPIT1_BASE_ADDR     ((uint32)0x420D0000)

  /*Interrupts*/
  #define BRS_EPIT1_IRQ 56
  #define BRS_EPIT1_INT_SRC (BRS_EPIT1_IRQ/32)
  #define BRS_EPIT1_INT_EN (1<<(BRS_EPIT1_IRQ % 32))

  #define BRS_FLEXCAN1_IRQ 110
  #define BRS_FLEXCAN1_INT_SRC (BRS_FLEXCAN1_IRQ/32)
  #define BRS_FLEXCAN1_INT_EN (1<<(BRS_FLEXCAN1_IRQ % 32))

  #define BRS_FLEXCAN2_IRQ 111
  #define BRS_FLEXCAN2_INT_SRC (BRS_FLEXCAN2_IRQ/32)
  #define BRS_FLEXCAN2_INT_EN (1<<(BRS_FLEXCAN2_IRQ % 32))

  #define BRS_CANFD1_IRQ 114
  #define BRS_CANFD1_INT_SRC (BRS_CANFD1_IRQ/32)
  #define BRS_CANFD1_INT_EN (1<<(BRS_CANFD1_IRQ % 32))

  #define BRS_CANFD2_IRQ 115
  #define BRS_CANFD2_INT_SRC (BRS_CANFD2_IRQ/32)
  #define BRS_CANFD2_INT_EN (1<<(BRS_CANFD2_IRQ % 32))

  /*
   * System Control Space (SCS) Register Struct
   * Structure containing all the SCS registers with appropriate padding
   */
  #define BRS_SCS_BASE_ADDR     (0xE000E000)

  typedef volatile struct {
    int MasterCtrl;
    int IntCtrlType;

    int zReserved008_00c[2];

    struct {
      int Ctrl;
      int Reload;
      int Value;
      int Calibration;
    } SysTick;

    int zReserved020_0fc[(0x100-0x20)/4];

    /* Offset 0x0100 */
    struct {
      int Enable[32];
      int Disable[32];
      int Set[32];
      int Clear[32];
      int Active[64];
      int Priority[64];
    } NVIC;

    int zReserved0x500_0xcfc[(0xd00-0x500)/4];

    /* Offset 0x0d00 */
    int CPUID;
    int IRQcontrolState;
    int ExceptionTableOffset;
    int AIRC;
    int SysCtrl;
    int ConfigCtrl;
    int SystemPriority[3];

    int zReserved0xd24_0xd88[(0xd88-0xd24)/4];

    /* Offset 0x0d88 */
    int CPACR;
    int Pad;

    /* Offset 0x0d90 */
    struct {
      int Type;
      int Ctrl;
      int RegionNumber;
      int RegionBaseAddr;
      int RegionAttrSize;
    } MPU;

  } BRS_IMX6SLX_SCS;
  
#  endif /*BRS_CPU_CORE_CORTEX_M4*/

#   if defined (BRS_CPU_CORE_CORTEX_A9)
  #define BRS_GPIO2_BASE_ADDR     ((vuint32)(0x20A0000))
  #define BRS_GPIO4_BASE_ADDR     ((vuint32)(0x20A8000))
  #define BRS_AIPS_BASE_ADDR      ((vuint32)0x020E0000) 
#  endif /*BRS_CPU_CORE_CORTEX_A9*/
# endif /*BRS_DERIVATIVE_IMX6X*/

# if defined (BRS_CPU_CORE_CORTEX_A9)
  /**********************************************************
   * Local fuction declarations
   *********************************************************/
  void __IRQ_Handler(void);
  void __FIQ_Handler(void);

  /******************************************************************************
   * Controller configuration registers
   *****************************************************************************/

  /**********************************************************
   * Generic Interrupt Controller (GIC)
   *********************************************************/
  /* Base addresses of GIC */
  #define BRS_GIC_INTERFACE_BASE      0x00A00100UL                                            /* Base address of GIC Interface registers    */
  #define BRS_GIC_DISTRIBUTOR_BASE    0x00A01000UL                                            /* Base address of GIC Distributor registers  */

  /* Distributor register definitions */
  #define BRS_GICD_CTLR               BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE))            /* Distributor Control Register               */

  #define BRS_GICD_ISENABLER1         BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x104UL))  /* Interrupt Set-Enable Registers 1           */
  #define BRS_GICD_ISENABLER2         BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x108UL))  /* Interrupt Set-Enable Registers 2           */
  #define BRS_GICD_ISENABLER3         BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x10CUL))  /* Interrupt Set-Enable Registers 3           */
  #define BRS_GICD_ISENABLER4         BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x110UL))  /* Interrupt Set-Enable Registers 4           */

  #define BRS_GICD_ISPENDR1           BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x204UL))  /* Interrupt Interrupt Set-Pending Register 1 */
  #define BRS_GICD_ISPENDR2           BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x208UL))  /* Interrupt Interrupt Set-Pending Register 2 */
  #define BRS_GICD_ISPENDR3           BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x20CUL))  /* Interrupt Interrupt Set-Pending Register 3 */
  #define BRS_GICD_ISPENDR4           BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x210UL))  /* Interrupt Interrupt Set-Pending Register 4 */

  #define BRS_GICD_IPRIORITYR22       BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x458UL))  /* Interrupt Priority Register 22             */
  #define BRS_GICD_IPRIORITYR35       BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x48CUL))  /* Interrupt Priority Register 35             */

  #define BRS_GICD_ITARGETSR22        BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x858UL))  /* Interrupt Processor Targets Register 22    */
  #define BRS_GICD_ITARGETSR35        BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0x88CUL))  /* Interrupt Processor Targets Register 35    */

  #define BRS_GICD_ICFGR4             BRS_IOS(uint32, (BRS_GIC_DISTRIBUTOR_BASE + 0xC10UL))  /* Interrupt Configuration Register 4         */

  /* CPU interface registers definitions */
  #define BRS_GICC_IAR                BRS_IOS(uint32, (BRS_GIC_INTERFACE_BASE + 0x00CUL))    /* Interrupt Acknowledge Register             */
  #define BRS_GICC_EOIR               BRS_IOS(uint32, (BRS_GIC_INTERFACE_BASE + 0x010UL))    /* End of Interrupt Register                  */
  #define BRS_GICC_CTLR               BRS_IOS(uint32, (BRS_GIC_INTERFACE_BASE))              /* CPU Interface Control Register             */
  #define BRS_GICC_PMR                BRS_IOS(uint32, (BRS_GIC_INTERFACE_BASE + 0x004UL))    /* Interrupt Priority Mask Register           */
  #define BRS_GICC_BPR                BRS_IOS(uint32, (BRS_GIC_INTERFACE_BASE + 0x008UL))    /* Binary Point Register                      */

  /* Used IRQ-IDs */
  #define BRS_IRQ_ID_EPIT1            0x58UL  /* EPIT1-ID    = 88  */
  #define BRS_IRQ_ID_FLEXCAN1         0x8EUL  /* FLEXCAN1-ID = 142 */
  #define BRS_IRQ_ID_FLEXCAN2         0x8FUL  /* FLEXCAN2-ID = 143 */
  #define BRS_IRQ_ID_ENET             150UL   /* ENET-ID = 150 */

  /* GIC Bit definitions */
  #define BRS_GICD_ISENABLER2_EPIT1_BIT    24U    /* Bit to enable EPIT1        (ID=  88)      */
  #define BRS_GICD_ISENABLER4_FLEXCAN1_BIT 14U    /* Bit to enable FlexCAN1 IRQ (ID= 142)      */
  #define BRS_GICD_ISENABLER4_FLEXCAN2_BIT 15U    /* Bit to enable FlexCAN2 IRQ (ID= 143)      */
  #define BRS_GICD_ISENABLER4_ENET_BIT     22U    /* Bit to enable ENET IRQ (ID= 150)      */

  /* GIC Configuration */
  #define BRS_GICC_BPR_CONFIG         0x03UL    /* Set the count of bits for the group priority and sub priority to -> g: 4bit, s: 4bit     */
  #define BRS_GICC_PMR_CONFIG         0xFFUL    /* Set Priority Mask Register to the lowest interrupt priority                              */
  #define BRS_GICC_CTLR_CONFIG        0x03UL    /* Enable forwarding of group 0 and 1 interrupts to the processor                           */
  #define BRS_GICD_CTLR_CONFIG        0x03UL    /* Enable forwarding of group 0 and 1 interrupts by the Distributor                         */
  
  /**********************************************************
   * Clock Controller Module (CCM)
   *********************************************************/
  /* Register structure of the CCM */
  typedef struct
  {
    volatile uint32 CCR;
    volatile uint32 CCDR;
    volatile uint32 CSR;
    volatile uint32 CCSR;
    volatile uint32 CACRR;
    volatile uint32 CBCDR;
    volatile uint32 CBCMR;
    volatile uint32 CSCMR1;
    volatile uint32 CSCMR2;
    volatile uint32 CSCDR1;
    volatile uint32 CS1CDR;
    volatile uint32 CS2CDR;
    volatile uint32 CDCDR;
    volatile uint32 CHSCCDR;
    volatile uint32 CSCDR2;
    volatile uint32 CSCDR3;
    volatile uint32 CDHIPR;
    volatile uint32 CLPCR;
    volatile uint32 CISR;
    volatile uint32 CIMR;
    volatile uint32 CCOSR;
    volatile uint32 CGPR;
    volatile uint32 CCGR0;
    volatile uint32 CCGR1;
    volatile uint32 CCGR2;
    volatile uint32 CCGR3;
    volatile uint32 CCGR4;
    volatile uint32 CCGR5;
    volatile uint32 CCGR6;
    volatile uint32 CMEOR;
  } BRS_CCM_TypeDef;

  #define BRS_CCM_BASE          0x020C4000UL                                  /* Base address of CCM Module   */
  #define BRS_CCM               ((BRS_CCM_TypeDef      *) BRS_CCM_BASE    )   /* Definition of the CCM Module */

  /* CCM configurations */

  /* Enable all clocks during all modes, except stop mode */
  #define BRS_CCM_CONFIG_CCGR0    0xFFFFFFFFUL
  #define BRS_CCM_CONFIG_CCGR1    0xFFFFFFFFUL
  #define BRS_CCM_CONFIG_CCGR2    0xFFFFFFFFUL
  #define BRS_CCM_CONFIG_CCGR3    0xFFFFFFFFUL
  #define BRS_CCM_CONFIG_CCGR4    0xFFFFFFFFUL
  #define BRS_CCM_CONFIG_CCGR5    0xFFFFFFFFUL
  #define BRS_CCM_CONFIG_CCGR6    0xFFFFFFFFUL

  #define BRS_CCM_CACRR_CONFIG    0x00000000UL  /* ARM PLL divided by 1                                            */
  #define BRS_CCM_CBCMR_CONFIG    0x00820324UL  /* PLL2 main clock (528MHz) selected for 'pre_periph clock' (EPIT) */
  #define BRS_CCM_CBCDR_CONFIG    0x00018D00UL  /* 'AHB_PODF' divider = 4, IPG_PODF divider = 2    (EPIT) */
  #define BRS_CCM_CSCMR1_CONFIG   0x00F00000UL  /* 'PERCLK_PODF' divider = 1                       (EPIT) */

#  if defined (BRS_EVA_BOARD_VEBN01084) /* == if defined (BRS_DERIVATIVE_IMX6S)*/
  /* Adapt configuration for FlexCAN clock divider here ! */
  /*Divider for can clock podf.
  000000 divide by 1
  000111 divide by 8
  111111 divide by 2^6*/
  
  #define BRS_CCM_CSCMR2_CONFIG   0x02B92F06UL  /* CAN_CLK_PODF = 2 */

#  elif defined (BRS_EVA_BOARD_VEBN01013) /* == #if defined (BRS_DERIVATIVE_IMX6X)*/
  /*The CAN controller frequency is controlled by registers CCM_CSCMR2.can_clk_sel
  and CCM_CSCMR2.can_clk_podf

  CCM_CSCMR2.can_clk_sel:
  00: derive clock from pll3 divided clock (60M)
  01: derive clock from OSC clock (24M)
  10: derive clock from pll3 divided clock (80M)

  CCM_CSCMR2.can_clk_podf:
  000000 divide by 1
  000111 divide by 8
  ...
  111111 divide by 2^6
  */
  
  /* set the CAN frequency to 60 Mhz */
  #define BRS_CCM_CSCMR2_CONFIG   0x02B92c06UL  /* CAN_CLK_PODF = 2, derive clock from pll3 divided clock (60M) */ 
#  else
  #error "Unknown EvalBoard specified. Please configure BRS_CCM_CSCMR2_CONFIG for PLL clock manually!"
#  endif /*BRS_EVA_BOARD_x*/

  /**********************************************************
   *  CCM Analog
   *********************************************************/
  #define BRS_CCM_ANALOG_BASE           0x020C8000UL                    /* Base address of the CCM_ANALOG Module    */

  /* CCM Analog register definitions */
  #define BRS_ANALOG_PLL_ARM_0          BRS_IOS(uint32, (BRS_CCM_ANALOG_BASE))           /* Control register PLL1 Core0 (ARM PLL)     */
  #define BRS_ANALOG_PLL_USB1_0         BRS_IOS(uint32, (BRS_CCM_ANALOG_BASE + 0x10UL))  /* Control register PLL3 Core0 (USB1 PLL)    */
  #define BRS_ANALOG_PLL_SYS_0          BRS_IOS(uint32, (BRS_CCM_ANALOG_BASE + 0x30UL))  /* Control register PLL2 Core0 (System PLL)  */

  /* CCM Analog configurations */
  #define BRS_ANALOG_PLL_ARM_CONFIG     0x00002042UL  /* PLL1 792 MHz */
  #define BRS_ANALOG_PLL_SYS_CONFIG     0x00002001UL  /* PLL2 528 MHz */
  #define BRS_ANALOG_PLL_USB1_CONFIG    0x00003000UL  /* PLL3 480 MHz */

  /* CCM Analog Bit definitions */
  #define BRS_PLL_X_BYPASS_BIT          16U  /* Bit to configure PLL bypass mode */
  #define BRS_PLL_ARM_LOCK_BIT          31U
  #define BRS_PLL_SYS_LOCK_BIT          31U
  #define BRS_PLL_USB1_LOCK_BIT         31U

  /**********************************************************
   * Periodic Interrupt Timer (EPIT)
   *********************************************************/
  /* Register structure of the EPIT */
  typedef struct
  {
    volatile uint32 CR;
    volatile uint32 SR;
    volatile uint32 LR;
    volatile uint32 CMPR;
    volatile uint32 CNR;
  } BRS_EPIT_TypeDef;

  /* Base address definitions for the EPITs */
  #define BRS_EPIT1_BASE                0x020D0000UL
  #define BRS_EPIT2_BASE                0x020D4000UL

  /* Definition of the EPITs */
  #define BRS_EPIT1                     ((BRS_EPIT_TypeDef      *) BRS_EPIT1_BASE    )
  #define BRS_EPIT2                     ((BRS_EPIT_TypeDef      *) BRS_EPIT2_BASE    )

  /* EPIT bit definitions */
  #define BRS_EPIT1_PRESCALER_VALUE     0x00000042UL   /* 66MHz /   '66' = 1MHz            */
  #define BRS_EPIT1_LOAD_VALUE          0x000003E8UL   /*  1MHz / '1000' = 1kHz -> T=1ms   */

  /* EPIT register bit definitions */

  /* CR register */
  #define BRS_EPIT_CR_CLKSRC_BITS       24U
  #define BRS_EPIT_CR_OM_BITS           22U
  #define BRS_EPIT_CR_STOPEN_BIT        21U
  #define BRS_EPIT_CR_WAITEN_BIT        19U
  #define BRS_EPIT_CR_DBGEN_BIT         18U
  #define BRS_EPIT_CR_IOVW_BIT          17U
  #define BRS_EPIT_CR_SWR_BIT           16U
  #define BRS_EPIT_CR_PRESCALAR_BITS     4U
  #define BRS_EPIT_CR_RLD_BIT            3U
  #define BRS_EPIT_CR_OCIEN_BIT          2U
  #define BRS_EPIT_CR_ENMOD_BIT          1U
  #define BRS_EPIT_CR_EN_BIT             0U

  /**********************************************************
   * General Purpose Input/Output (GPIO)
   *********************************************************/
  /* Register structure of the GPIO */
  typedef struct
  {
    volatile uint32 DR;
    volatile uint32 GDIR;
    volatile uint32 PSR;
    volatile uint32 ICR1;
    volatile uint32 ICR2;
    volatile uint32 IMR;
    volatile uint32 ISR;
    volatile uint32 EDGE_SEL;
  } BRS_GPIO_TypeDef;

  /* Base address definitions for the GPIOs */
  #define BRS_GPIO1_BASE          (0x209C000UL)
  #define BRS_GPIO2_BASE          (0x20A0000UL)
  #define BRS_GPIO3_BASE          (0x20A4000UL)
  #define BRS_GPIO4_BASE          (0x20A8000UL)
  #define BRS_GPIO5_BASE          (0x20AC000UL)
  #define BRS_GPIO6_BASE          (0x20B0000UL)
  #define BRS_GPIO7_BASE          (0x20B4000UL)

  /* Definition of the GPIOs */
  #define BRS_GPIO1               ((BRS_GPIO_TypeDef *) BRS_GPIO1_BASE)
  #define BRS_GPIO2               ((BRS_GPIO_TypeDef *) BRS_GPIO2_BASE)
  #define BRS_GPIO3               ((BRS_GPIO_TypeDef *) BRS_GPIO3_BASE)
  #define BRS_GPIO4               ((BRS_GPIO_TypeDef *) BRS_GPIO4_BASE)
  #define BRS_GPIO5               ((BRS_GPIO_TypeDef *) BRS_GPIO5_BASE)
  #define BRS_GPIO6               ((BRS_GPIO_TypeDef *) BRS_GPIO6_BASE)
  #define BRS_GPIO7               ((BRS_GPIO_TypeDef *) BRS_GPIO7_BASE)

  /**********************************************************
   * Definition of used IOMUXC registers
   *********************************************************/
  /* Definition of the LED pin registers*/
  #define BRS_IOMUXC_MUX_CTL_GPIO2    BRS_IOS(uint32, (0x020E0224UL))  /* Definition of the IOMUXC MUX control register for the LED */
  #define BRS_IOMUXC_PAD_CTL_GPIO2    BRS_IOS(uint32, (0x020E05F4UL))  /* Definition of the IOMUXC PAD control register for the LED */

  /* Definition of the FLEXCAN pin registers */
  #define BRS_IOMUXC_MUX_CTL_GPIO7    BRS_IOS(uint32, (0x020E0238UL))  /* Definition of the IOMUXC MUX control register for FLEXCAN1_TX */
  #define BRS_IOMUXC_PAD_CTL_GPIO7    BRS_IOS(uint32, (0x020E0608UL))  /* Definition of the IOMUXC PAD control register for FLEXCAN1_TX */

  #define BRS_IOMUXC_MUX_CTL_GPIO8    BRS_IOS(uint32, (0x020E023CUL))  /* Definition of the IOMUXC MUX control register for FLEXCAN1_RX */
  #define BRS_IOMUXC_PAD_CTL_GPIO8    BRS_IOS(uint32, (0x020E060CUL))  /* Definition of the IOMUXC PAD control register for FLEXCAN1_RX */

  /************************************
  * Configurations for the LED
  ************************************/
  /* SION = Disabled, Function GPIO1_IO02 (ALT5) selected */
  #define BRS_GPIO2_MUX_CONFIG        0x00000005UL

  /* CMOS input , 100k pull-up, pull-up selected, pull-up disabled, CMOS output, Speed = 50MHz, driver = 34 Ohm, slow slew rate  */
  #define BRS_GPIO2_PAD_CONFIG        0x0000A078UL

  /************************************
  * Configuration for FLEXCAN1
  ************************************/
  /* SION = Disabled, Function FLEXCAN1_TX (ALT3) selected */
  #define BRS_GPIO7_MUX_CONFIG        0x00000003UL

  /* CMOS input, 100k pull-up, pull-up selected, pull-up disabled, CMOS output, Speed = 50MHz, driver = 34 Ohm, fast slew rate  */
  #define BRS_GPIO7_PAD_CONFIG        0x0000A079UL

  /* SION = Disabled, Function FLEXCAN1_RX (ALT3) selected */
  #define BRS_GPIO8_MUX_CONFIG        0x00000003UL

  /* CMOS input, 100k pull-up, pull-up selected, pull-up disabled, CMOS output, Speed = 50MHz, driver = 34 Ohm, fast slew rate  */
  #define BRS_GPIO8_PAD_CONFIG        0x0000A079UL

# endif /*BRS_CPU_CORE_CORTEX_A9*/

#else
  #error "Unknown DERIVATIVE specified!"
#endif

/********************************************************************************
 *                          Start of Implementation                             *
 ********************************************************************************/
#if defined (BRS_DERIVATIVE_IMX25) || defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX53)
/* Some functions in libgcc.a want to call this for exceptions, like division
 * by zero. This is not intended to do anything useful, just satisfy the
 * linker. */
# if defined (BRS_DERIVATIVE_IMX25) || defined (BRS_DERIVATIVE_IMX35)
int raise(int sig)
{
  BrsHwDisableInterrupt();
  ConPuts("SIGNAL RAISED by libgcc.a: signal ");
  ConPutd(sig);
  ConPuts("\r\n");
  while(1)
  {
    ;
  }
}

void *memcpy(void *dst, const void *src, int n)
{
  void *r = dst;
  char *d8 = (char *)dst;
  const char *s8 = (const char *)src;
  if (dst < src)
  {
    /* low to high */
    while (n-- > 0) *d8++ = *s8++;
  }
  else
  {
    /* high to low, in case dst and src overlap */
    d8 += n;
    s8 += n;
    while (n-- > 0)
      *--d8 = *--s8;
  }
  return r;
}

void *memset(void *dst, int c, int n)
{
  void *r = dst;
  char *d8 = (char *)dst;
  while(n-- > 0) *d8++ = c;
  return r;
}
# endif /*BRS_DERIVATIVE_IMX25 || BRS_DERIVATIVE_IMX35*/

void assert_fail(const char * file, int line, const char * assertion)
{
  BrsHwDisableInterrupt();
  ConPuts("ASSERTION FAILED: file \"");
  ConPuts(file);
  ConPuts("\", line ");
  ConPutd(line);
  ConPuts(", \"");
  ConPuts(assertion);
  ConPuts("\"\r\n");
  while(1)
  {
    ;
  }
}

# if defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX53)
/****************************************************************************
* uart_init
*
* For 115200 baud, RedBoot sets UBMR = 0x7a11 and UBIR = 0x047f, which
* matches freq_ref = 50.000 MHz. We use RedBoot's values here, but that
* freq_ref value may be useful if a different baud rate is needed.
*
*/
void *uart_init(unsigned uart_num)
{
  void *b;

  if (uart_num >= UART_MAX)
    return NULL_PTR;

  b = uart_info[uart_num].base;

  /* wait for FIFO empty */
  while ((*UART_REG(b, UART_SR2) & 0x8) == 0);

  /* reinitialize */
  *UART_REG(b, UART_CR1) = 0x0001;    /* enable UART */
  *UART_REG(b, UART_CR2) = 0x0000;    /* reset UART */
  *UART_REG(b, UART_CR3) = 0x0004;    /* rx pin config, all interrupts disabled */
  *UART_REG(b, UART_CR4) = 0x8000;    /* leave at reset values */
  *UART_REG(b, UART_FCR) = 0x0a01;    /* reset value, set RFDIV */
  *UART_REG(b, UART_SR1) = 0xffff;    /* clear all */
  *UART_REG(b, UART_SR2) = 0xffff;    /* clear all */
  *UART_REG(b, UART_ESC) = 0x002b;    /* reset value */
  *UART_REG(b, UART_TIM) = 0x0000;    /* reset value */
  *UART_REG(b, UART_BIR) = 0x047f;    /* RedBoot value */
  *UART_REG(b, UART_BMR) = 0x7a11;    /* RedBoot value */
  *UART_REG(b, UART_ONEMS) = 0xc350;  /* RedBoot value */
  *UART_REG(b, UART_TS) = 0x0060;     /* reset value */
  *UART_REG(b, UART_CR2) = 0x4027;    /* no RTS, no par, 1s, 8b, en */

  return uart_info + uart_num;
}

int uart_getc(void *uart_handle)
{
  struct uart_info_s *p = uart_handle;
  if (*UART_REG(p->base, UART_SR2) & UART_SR2_RDR)
    return *UART_REG(p->base, UART_RXD) & 0xff;
  else
    return -1;
}

void uart_putc(void *uart_handle, int c)
{
  struct uart_info_s *p = uart_handle;
  void *b = p->base;
  *UART_REG(b, UART_TXD) = c;
  while ((*UART_REG(b, UART_SR2) & UART_SR2_TXDC) == 0);
}

/****************************************************************************
 * Below here is not really "driver" any more; it's formatted I/O.
 ****************************************************************************/
void uart_puts(void *uart_handle, const char *s)
{
  while (*s != '\0')
    uart_putc(uart_handle, *s++);
}

void uart_putx(void *uart_handle, unsigned u)
{
  int i;
  uart_puts(uart_handle, "0x");
  for (i = 0; i < 8; i++) {
    int n = (u >> 28) & 0xf;
    if (n < 10)
      n = n + '0';
    else
      n = (n - 10) + 'A';
    uart_putc(uart_handle, n);
    u = u << 4;
  }
}

void uart_putu(void *uart_handle, unsigned u)
{
  int started = 0; /* to know if we're filling zeros yet */
  unsigned m = 1000000000;

  while (m != 0) {
    if (u >= m || started || m == 1) {
      /* integer divide and mod pull in helper functions */
      unsigned d = 0;
      while (u >= m) {
        d++;
        u -= m;
      }
      started = 1;
      uart_putc(uart_handle, d + '0');
    }
    m = m / 10;
  }
}

void uart_putd(void *uart_handle, int d)
{
  if (d < 0) {
    uart_putc(uart_handle, '-');
    d = -d;
  }
  uart_putu(uart_handle, d);
}

/**************************************************************************
 *                              EPIT                                      *
 **************************************************************************/
/* Input clock is 66,500,000 Hz
 * prescale = 665 for a count rate of 100,000 Hz (10 usec/tick)
 * count is units of 10 usec (count = 100 means 1 msec)
 * (Note that the counter ticks at both zero and the reload value) */
void *epit_init(unsigned epit_num, unsigned prescale, uint32 count)
{
  BrsAssert(epit_num < EPIT_MAX);
  epit_config(epit_info + epit_num, prescale, count);
  return epit_info + epit_num;
}

void epit_ack(void *epit_handle)
{
  struct epit_info_s *p = epit_handle;
  *EPIT_REG(p->base, EPIT_SR) = 1;
}

void epit_enable(void *epit_handle)
{
  struct epit_info_s *p = epit_handle;
  *EPIT_REG(p->base, EPIT_CR) |= 1;
#ifdef AVIC_BASE
    BrsHwAvicEnable(p->int_num);
#endif
#ifdef TZIC_BASE
    BrsHwTzicEnable((*p).int_num);
#endif
}

/* EPIT disable */
void epit_disable(void *epit_handle)
{
  struct epit_info_s *p = epit_handle;
  *EPIT_REG(p->base, EPIT_CR) &= ~1;
}

/*  EPIT Connect  */
#ifdef AVIC_BASE
void epit_connect(void *epit_handle, unsigned level, void (*func)(void*), void *arg)
{
  struct epit_info_s *p = epit_handle;
  BrsHwAvicConnect(p->int_num, level, func, arg); 
}
#endif

#ifdef TZIC_BASE
void epit_connect(void *epit_handle, unsigned level, void (*func)(void*), void *arg)
{
  BrsHwTzicConnect((*(struct epit_info_s *)epit_handle).int_num, level, func, arg);
}
#endif

/*  EPIT Configure */
void epit_config(void *epit_handle, unsigned prescale, uint32 count)
{
  struct epit_info_s *p = epit_handle;
  void *base = p->base;

  BrsAssert(prescale >= 1 && prescale <= 4096);

  *EPIT_REG(base, EPIT_CR) = 0;    /* disabled */
  *EPIT_REG(base, EPIT_CR) = (1 << 16);  /* software reset */
  while (*EPIT_REG(base, EPIT_CR) & (1 << 16))
    ;        /* wait for reset to finish */
  *EPIT_REG(base, EPIT_CR) =
      (1 << 24)  /* ipg_clk */
    | (1 << 17)  /* setting load sets count */
    | ((prescale - 1) << 4)  /* prescale */
    | (1 << 3)  /* reload */
    | (1 << 2)  /* interrupt enable */
    | (1 << 1);  /* start at load when enabled */
  *EPIT_REG(base, EPIT_LR) = count - 1;
  *EPIT_REG(base, EPIT_CMPR) = count - 1;
  *EPIT_REG(base, EPIT_SR) = 1;
}
# endif /*BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX53*/
#endif /*BRS_DERIVATIVE_IMX25 || BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX53*/

#if defined (BRS_DERIVATIVE_IMX25)
/*******************************************************************************
 powerdown handling
*******************************************************************************/
void BrsHwDozeMode(void)
{
  uint32 tmpVar;

  tmpVar = (BRSHW_CCM_CGR0 & (uint32)0xFFFFFC3F) | (uint32)0x00000280;
  BRSHW_CCM_CGR0 = tmpVar;      

  /* temp = ((mem32_read(CCM_CCMR) & 0xffff3fff) | 0x00008000); */
  tmpVar = (BRSHW_CCM_CCMR & (uint32)0xFFFF3FFF) | (uint32)0x00008000;
  /* mem32_write((CCM_BASE_ADDR+CCM_CCMR_OFFSET),temp);  */
  BRSHW_CCM_CCMR = tmpVar;
  
  tmpVar = BRSHW_CCM_CCMR | (uint32)0x40000000;
  BRSHW_CCM_CCMR = tmpVar;  
  
  __asm(" MRS r0, CPSR ");
  __asm(" AND r1, r0, #(0x0+0x0+0x1F) ");
  __asm(" MSR CPSR_c,r1 ");
  __asm(" NOP ");
  __asm(" NOP ");
  __asm(" MOV r1,#0x0 ");
  __asm(" MCR p15,0,r1,c7,c0,4 ");
  __asm(" NOP ");
  __asm(" NOP ");
  __asm(" NOP ");
  __asm(" NOP ");
  __asm(" NOP ");
}
 
void BrsHwStopMode(void)
{
  uint32 tmpVar; 
  
  /* temp = ((mem32_read(CCM_CCMR) & 0xffff3fff) | 0x0000C000); */
  tmpVar = (BRSHW_CCM_CCMR & (uint32)0xFFFF3FFF) | (uint32)0x0000C000;
  /* mem32_write((CCM_BASE_ADDR + CCM_CCMR_OFFSET),temp); */
  BRSHW_CCM_CCMR = tmpVar;
  
  __asm(" MRS r0, CPSR ");
  __asm(" AND r1, r0, #(0x0+0x0+0x1F) ");
  __asm(" MSR CPSR_c,r1 ");
  __asm(" NOP ");
  __asm(" NOP ");
  __asm(" MOV r1,#0x0 ");
  __asm(" MCR p15,0,r1,c7,c0,4 ");
  __asm(" NOP ");
  __asm(" NOP ");
  __asm(" NOP ");
  __asm(" NOP ");
  __asm(" NOP ");
}

void BrsHwSetDozeMode(void)
{
  uint32 tmpVar;
  
  tmpVar = (BRSHW_CCM_CGR0 & (uint32)0xFFFFFC3F) | (uint32)0x00000280;
  BRSHW_CCM_CGR0 = tmpVar;      
  tmpVar = (BRSHW_CCM_CCMR & (uint32)0xFFFF3FFF) | (uint32)0x00008000;
  BRSHW_CCM_CCMR = tmpVar;
  tmpVar = BRSHW_CCM_CCMR | (uint32)0x40000000;
  BRSHW_CCM_CCMR = tmpVar;
}
#endif /*BRS_DERIVATIVE_IMX25*/

/********************************************************************************
 *                                                                              *
 *                              INTERRUPT CONTROLLER                            *
 *                                                                              *
 ********************************************************************************/
#if defined (AVIC_BASE) /* == BRS_DERIVATIVE_IMX35*/
/**********************************************************************
 *                            AVIC                                    *
 **********************************************************************/
/* "BrsHwIrqHandler" is called by the IRQ exception stub */
void BrsHwIrqHandler(void)
{
  uint32 old_mask;
  uint32 vecsr;
  unsigned int_num;

  old_mask = *AVIC_REG(AVIC_NIMASK);
  vecsr = *AVIC_REG(AVIC_NIVECSR);

  /* mask any interrupts of equal or lower priority */
  *AVIC_REG(AVIC_NIMASK) = vecsr & 0x1f;

  /* read back to make sure the mask is set before enabling interrupts */
  *AVIC_REG(AVIC_NIMASK);

  /* update histogram of nesting levels:
   * int_nest_cnt[0] is the number of ints that didn't preempt
   * int_nest_cnt[1] is the number of ints that preempted a 1st-level int */
  int_nest_cnt[int_nest < INT_NEST_MAX ? int_nest : INT_NEST_MAX - 1]++;

  int_nest++;
  BrsHwRestoreInterrupt();

  int_num = (vecsr >> 16) & 0xffff;

  if (int_num < IC_INT_NUM_MAX)
  {
    struct Ic_handler_s *p = Ic_handler + int_num;
    /* avic_force_clr(int_num); */
    if (p->func != NULL) {
      (*(p->func))(p->arg);
    } else {
      BrsMainExceptionHandler( kBrsInterruptHandlerNotInstalled, BRSERROR_MODULE_BRSHW, int_num );
    }
  }

  BrsHwDisableInterrupt();
  int_nest--;

  /* restore previous mask */
  *AVIC_REG(AVIC_NIMASK) = old_mask;
}

/*  INIT  */
void BrsHwAvicInit(void)
{
  unsigned i;

  *AVIC_REG(AVIC_INTCNTL) = 0;
  /* all levels enabled */
  *AVIC_REG(AVIC_NIMASK) = 0x1f;
  /* all interrupts disabled */
  *AVIC_REG(AVIC_INTENABLEH) = 0;
  *AVIC_REG(AVIC_INTENABLEL) = 0;
  /* all interrupts routed to IRQ (not FIQ) */
  *AVIC_REG(AVIC_INTTYPEH) = 0;
  *AVIC_REG(AVIC_INTTYPEL) = 0;
  /* all interrupts are priority 0 (lowest) */
  *AVIC_REG(AVIC_NIPRIORITY(0)) = 0;
  *AVIC_REG(AVIC_NIPRIORITY(1)) = 0;
  *AVIC_REG(AVIC_NIPRIORITY(2)) = 0;
  *AVIC_REG(AVIC_NIPRIORITY(3)) = 0;
  *AVIC_REG(AVIC_NIPRIORITY(4)) = 0;
  *AVIC_REG(AVIC_NIPRIORITY(5)) = 0;
  *AVIC_REG(AVIC_NIPRIORITY(6)) = 0;
  *AVIC_REG(AVIC_NIPRIORITY(7)) = 0;
  /* all handlers are NULL */
  for (i = 0; i < IC_INT_NUM_MAX; i++)
  {
    Ic_handler[i].func= NULL;
    Ic_handler[i].arg = NULL;
  }
}

/*
 * Connect a handler to an interrupt number. Interrupts are not disabled since
 * it is assumed that the interrupt number of interest has not been enabled
 * yet. If a handler is changed after enabling it, interrupts should be
 * disabled while doing so.
 */
void BrsHwAvicConnect(unsigned int_num, unsigned int_level,
      void (*func)(void*), void *arg)
{
  unsigned pri_reg;
  unsigned pri_shift;
  uint32 r;

  BrsAssert(int_num < IC_INT_NUM_MAX);
  BrsAssert(int_level < IC_INT_LEVEL_MAX);

  pri_reg = int_num / 8;    /* which NIPRIORITY() reg to use */
  pri_shift = (int_num % 8) * 4;  /* bit position in that reg */

  r = *AVIC_REG(AVIC_NIPRIORITY(pri_reg));
  r &= ~(0xf << pri_shift);
  r |= (int_level << pri_shift);
  *AVIC_REG(AVIC_NIPRIORITY(pri_reg)) = r;

  Ic_handler[int_num].func = func;
  Ic_handler[int_num].arg = arg;
}

/* Enable an interrupt number. */
void BrsHwAvicEnable(unsigned int_num)
{
  BrsAssert(int_num < IC_INT_NUM_MAX);
  *AVIC_REG(AVIC_INTENNUM) = int_num;
}

/* Disable an interrupt number. */
void BrsHwAvicDisable(unsigned int_num)
{
  BrsAssert(int_num < IC_INT_NUM_MAX);
  *AVIC_REG(AVIC_INTDISNUM) = int_num;
  /* make sure it's disabled before returning */
  *AVIC_REG(AVIC_INTDISNUM);
}

#endif /*AVIC_BASE*/

#if defined (TZIC_BASE) /* == BRS_DERIVATIVE_IMX53*/
/********************************************************************
 *                            TZIC                                  *
 ********************************************************************/
uint8 highestPriorityPendingInt(void)
{
  /* return : vector-no. of the pending interrupt with the highest priority */
  uint32 msk = 0x00000001;
  uint8 i = 0;
  volatile uint32 *r = TZIC_REG(TZIC_HIPND(0));

  while (((*r & msk)==0) && (i<IC_INT_NUM_MAX))
  {
    if(msk == 0x80000000) {  // last index of current register
      msk=0x00000001;         // reset mask
      r++;            // step to the next HIPND register
    } else {
      msk=msk<<1;        // shift left
    }
    i++;          
  }
  if((*r&0xffffffff) != 0)
  {
    return i; /* return number of pending interrupt */
  }
  else
  {
    return IC_INT_NUM_MAX; /* no interrupt found */
  }
}

void BrsHwIrqHandler(void)
{
  uint32 vecsr;      /*  priority of selected interrupt  */
  unsigned int_num;   /*  number of interrupt source  */
  uint32 old_mask = *TZIC_REG(TZIC_PRIOMASK);  
  int_num = highestPriorityPendingInt();
  vecsr = TZIC_PRIORITY(int_num / 4) << ((int_num % 4) * 8);

  /* mask any interrupts of equal or lower priority */
  *TZIC_REG(TZIC_PRIOMASK) = vecsr & 0xFF;

  /* update histogram of nesting levels:
   * int_nest_cnt[0] is the number of ints that didn't preempt
   * int_nest_cnt[1] is the number of ints that preempted a 1st-level int */
  int_nest_cnt[int_nest < INT_NEST_MAX ? int_nest : INT_NEST_MAX - 1]++;

  if (int_num < IC_INT_NUM_MAX)
  {
    struct Ic_handler_s *p = Ic_handler + int_num;
    if ((*p).func != NULL_PTR)
    {
      (*((*p).func))((*p).arg);
    }
    else
    {
      BrsMainExceptionHandler( kBrsInterruptHandlerNotInstalled, BRSERROR_MODULE_BRSHW, int_num );
    } 
  }
  /* restore previous mask */
  *TZIC_REG(TZIC_PRIOMASK) = old_mask;
}

void BrsHwTzicInit(void)
{
  uint8 i;
  
  /* all levels enabled */
  *TZIC_REG(TZIC_PRIOMASK)= 0xFF;
  *TZIC_REG(TZIC_INTCTRL) = 0x00010001; 

  /* all interrupts disabled */
  /* all interrupts are defined (in)secure */
  for(i=0;i<4;i++)
  {
    *TZIC_REG(TZIC_ENCLEAR(i)) = 0;
    *TZIC_REG(TZIC_INTSEC(i)) = 0;
  }
  /* all interrupts are priority 0 (lowest) */
  for ( i=0;i<32;i++)
  {
    *TZIC_REG(TZIC_PRIORITY(i)) = 0;
  }
  /* all handlers are NULL */
  for (i = 0; i < IC_INT_NUM_MAX; i++)
  {
    Ic_handler[i].func = NULL_PTR;
    Ic_handler[i].arg = NULL_PTR;
  }
}

void BrsHwTzicConnect(unsigned int_num, unsigned int_level, void (*func)(void*), void *arg)
{
  unsigned pri_reg;
  unsigned pri_shift;
  uint32 r;

  BrsAssert(int_num < IC_INT_NUM_MAX);
  BrsAssert(int_level < IC_INT_LEVEL_MAX);

  pri_reg = int_num /32;          /* determine priority register */
  pri_shift = (int_num % 4) * 8;
  r = *TZIC_REG(TZIC_PRIORITY(pri_reg));
  r &= ~(0xFF << pri_shift);
  r |= (int_level << pri_shift);  /* jump to the offset of the corresponding interrupt / set prio */
  *TZIC_REG(TZIC_PRIORITY(pri_reg)) = r; 

  Ic_handler[int_num].func = func;
  Ic_handler[int_num].arg = arg;
}

void BrsHwTzicEnable(unsigned int_num)
{
  BrsAssert(int_num < IC_INT_NUM_MAX);
  /* *TZIC_REG(TZIC_INTCTRL)=1; */
  *TZIC_REG(TZIC_ENSET(int_num/32)) |= (1 << (int_num%32));
}

/* Disable an interrupt number. */
void BrsHwTzicDisable(unsigned int_num)
{
  BrsAssert(int_num < IC_INT_NUM_MAX);
  *TZIC_REG(TZIC_ENCLEAR(int_num/32)) |= (1 << (int_num%32));
  *TZIC_REG(TZIC_ENCLEAR(int_num/32));
}

/* Assert an interrupt. */
void BrsHwTzicForceSet(unsigned int_num)
{
  BrsAssert(int_num < IC_INT_NUM_MAX);
  *TZIC_REG(TZIC_SRCSET(int_num/32)) |= 1 << ((int_num%32));
}

/* Deassert an interrupt. */
void BrsHwTzicForceClr(unsigned int_num)
{
  BrsAssert(int_num < IC_INT_NUM_MAX);
  *TZIC_REG(TZIC_SRCCLEAR(int_num/32)) &= ~(1 << (int_num%32));
  *TZIC_REG(TZIC_SRCCLEAR(int_num/32));
}
#endif /*TZIC_BASE*/

/*******************************************************************************
* NAME          : BrsHwWatchdogInitPowerOn
* CALLED BY     : BrsHwHardwareInitPowerOn or EcuM at power on initialization
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : This function has to be used to initialize the Watchdog.
********************************************************************************/
void BrsHwWatchdogInitPowerOn(void)
{
  /* Function mandatory for BrsAsr */
  /* nothing to be done here */
}

/*******************************************************************************
* NAME          : BrsHwPllInitPowerOn
* CALLED BY     : BrsHwHardwareInitPowerOn or EcuM at power on initialization
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : This function has to be used to initialize the PLL.
********************************************************************************/
void BrsHwPllInitPowerOn(void)
{
#if !defined (VGEN_ENABLE_DRVMCU)

# if defined (BRS_DERIVATIVE_IMX6S) || defined (BRS_DERIVATIVE_IMX6X)
#  if defined (BRS_CPU_CORE_CORTEX_M4)
  /* After reset the PLLs have the following configuration:
    ARM PLL (PLL0): 792 Mhz
    System PLL (PLL1): 528 Mhz
    USB PLL (PLL3): 480 Mhz

    The CAN controller frequency is controlled by registers CCM_CSCMR2.can_clk_sel
    and CCM_CSCMR2.can_clk_podf

    CCM_CSCMR2.can_clk_sel:
    00: derive clock from pll3 divided clock (60M)
    01: derive clock from OSC clock (24M)
    10: derive clock from pll3 divided clock (80M)

    CCM_CSCMR2.can_clk_podf:
    000000 divide by 1
    000111 divide by 8
    ...
    111111 divide by 2^6
  */

  /* Set the CPU frequecy to the one ARM PLL provides */
  BRS_CCM_CACRR &= ~((uint32)0x00000007); 

  /* set the CAN frequency to 60 Mhz (480MHz/8) */
  BRS_CCM_CSCMR2 &= ~((uint32)0x000003FC); 

  /*
     So this Brs operates with following default frequencies:
     Oscillator: 24Mhz
     ARM Cortex-M4 CPU freq: 792Mhz
     FlexCAN freq: 60 MHz (The CAN freq can be selected to come directly from the Osc. in the configuration tool!)
  */
#  endif /*BRS_CPU_CORE_CORTEX_M4*/

#  if defined (BRS_CPU_CORE_CORTEX_A9)
  /* Clock Setup:
   *              ARM_CLK_ROOT    (A9-Core)  = 792 MHz
   *              PERCLK_CLK_ROOT (EPIT)     =  66 MHz
   *              CAN_CLK_ROOT               =  30 MHz (FLEXCAN)
   *
   * ATTENTION: Configure the clock settings of the FlexCAN Module to your needs ! (CAN_CLK_ROOT)
   *************************************************************************/

  /* Configure ARM PLL (PLL1) */
  //BRS_CCM->CCSR &= ~(1 << 2);                                          /* Ensure that the source of pll1_sw_clk (ARM Core) is PLL1 */
  //BRS_ANALOG_PLL_ARM_0 |= (1U << BRS_PLL_X_BYPASS_BIT);                /* Bypass PLL1 for configuration!                           */
 // BRS_ANALOG_PLL_ARM_0  = BRS_ANALOG_PLL_ARM_CONFIG;                   /* Configure PLL1 for 792 MHz (pll1_main_clk)               */
 // while( !( BRS_ANALOG_PLL_ARM_0 & (1U << BRS_PLL_ARM_LOCK_BIT) ) ){}  /* Check for PLL lock (PLL is ready to use)                 */

  /* Configure System PLL (PLL2) */
//  BRS_CCM->CCSR &= ~(1 << 1);                                          /* Ensure that the source of pll2_sw_clk is pll2_main_clk   */
 // BRS_ANALOG_PLL_SYS_0 |= (1U << BRS_PLL_X_BYPASS_BIT);                /* Bypass PLL2 for configuration!                           */
 // BRS_ANALOG_PLL_SYS_0  = BRS_ANALOG_PLL_SYS_CONFIG;                   /* Configure PLL2 (pll2_main_clk) for 528 MHz               */
 // while( !( BRS_ANALOG_PLL_SYS_0 & (1U << BRS_PLL_SYS_LOCK_BIT) ) ){}  /* Check for PLL lock (PLL is ready to use)                 */

  /* Configure USB1 PLL (PLL3) */
 // BRS_CCM->CCSR &= ~(1 << 0);                                           /* Ensure that the source of pll3_sw_clk is pll3_main_clk   */
//  BRS_ANALOG_PLL_USB1_0 |= (1U << BRS_PLL_X_BYPASS_BIT);                /* Bypass PLL3 for configuration!                           */
//  BRS_ANALOG_PLL_USB1_0 = BRS_ANALOG_PLL_USB1_CONFIG;                   /* Configure PLL3 (pll3_main_clk) for 480 MHz               */
 // while( !( BRS_ANALOG_PLL_USB1_0 & (1U << BRS_PLL_USB1_LOCK_BIT) ) ){} /* Check for PLL lock (PLL is ready to use)                 */

  /* Configure the selection of the clock sources */
  BRS_CCM->CACRR  = BRS_CCM_CACRR_CONFIG;   /* Configure clock divider for A9-Core */
  BRS_CCM->CBCMR  = BRS_CCM_CBCMR_CONFIG;   /* Configure clock dividers for EPIT   */
  BRS_CCM->CBCDR  = BRS_CCM_CBCDR_CONFIG;
  BRS_CCM->CSCMR1 = BRS_CCM_CSCMR1_CONFIG;

  BRS_CCM->CSCMR2 = BRS_CCM_CSCMR2_CONFIG;  /* Configure FlexCAN clock divider */

  /* Enable all clocks during all modes, except stop mode (LPCG) */
  BRS_CCM->CCGR0  = BRS_CCM_CONFIG_CCGR0;
  BRS_CCM->CCGR1  = BRS_CCM_CONFIG_CCGR1;
  BRS_CCM->CCGR2  = BRS_CCM_CONFIG_CCGR2;
  BRS_CCM->CCGR3  = BRS_CCM_CONFIG_CCGR3;
  BRS_CCM->CCGR4  = BRS_CCM_CONFIG_CCGR4;
  BRS_CCM->CCGR5  = BRS_CCM_CONFIG_CCGR5;
  BRS_CCM->CCGR6  = BRS_CCM_CONFIG_CCGR6;
#  endif /*BRS_CPU_CORE_CORTEX_A9*/
# endif /*BRS_DERIVATIVE_IMX6S || BRS_DERIVATIVE_IMX6X*/


  /* clocks for ethernet module (ENET) */
  // CCM_CCGR1[CG5] must be enabled (should have been initialized by BRS before)
  // enable CCM_ANALOG_PLL_ENET
#define CCM_ANALOG_PLL_ENET0 BRS_IOS(uint32, 0x020C80E0)
#define CCM_ANALOG_PLL_ENET1 BRS_IOS(uint32, 0x020C80E4)
#define CCM_ANALOG_PLL_ENET2 BRS_IOS(uint32, 0x020C80E8)
#define CCM_ANALOG_PLL_ENET3 BRS_IOS(uint32, 0x020C80EC)

CCM_ANALOG_PLL_ENET0 = 0x182003;

#endif /*!VGEN_ENABLE_DRVMCU*/
}

/*******************************************************************************
* NAME          : BrsHwPortInitPowerOn
* CALLED BY     : BrsHwHardwareInitPowerOn or EcuM at power on initialization
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : This function has to be used to initialize the used ports.
********************************************************************************/
void BrsHwPortInitPowerOn(void)
{
#if !defined (VGEN_ENABLE_DRVPORT)
# if defined (BRS_DERIVATIVE_IMX25) || defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX53)
  uint32 tmpVar;

#  if defined (kCanPhysToLogChannelIndex_0)
  /* select pin function */
  tmpVar = *IOMUXC_SW_MUX_CTL_PAD_CANTX1;
  tmpVar &= ~(IOMUXC_SW_MUX_CTL_SION | IOMUXC_SW_MUX_CTL_MODE);
  tmpVar |= IOMUXC_MODE_CAN1;
  *IOMUXC_SW_MUX_CTL_PAD_CANTX1 = tmpVar;

  /* configure pin */
  tmpVar = *IOMUXC_SW_PAD_CTL_PAD_CANTX1;
#   if defined (BRS_DERIVATIVE_IMX53)
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS; // hysteresis disabled
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PKE; // pull disabled
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUE; // keep selected field
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUS; // 360KOhm PD
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE; // open drain disabled
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE; // low drive strength
#   endif /*BRS_DERIVATIVE_IMX53*/
#   if defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX25)
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DRV;  /* 3.3 V */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS;  /* hysteresis disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PULL; /* pull/keep disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE;  /* CMOS output */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE;  /* normal drive strength */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_SRE;  /* slow slew rate */
#   endif /*BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX25*/
  *IOMUXC_SW_PAD_CTL_PAD_CANTX1 = tmpVar;

  /* RX pin */ /* select pin function */
  tmpVar = *IOMUXC_SW_MUX_CTL_PAD_CANRX1;
  tmpVar &= ~(IOMUXC_SW_MUX_CTL_SION | IOMUXC_SW_MUX_CTL_MODE);
  tmpVar |= IOMUXC_MODE_CAN1;
  *IOMUXC_SW_MUX_CTL_PAD_CANRX1 = tmpVar;

  /* configure pin */
  tmpVar = *IOMUXC_SW_PAD_CTL_PAD_CANRX1;
#   if defined (BRS_DERIVATIVE_IMX53)
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS; // hysteresis disabled
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PKE; // pull disabled
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUE; // keep selected field
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUS; // 360KOhm PD
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE; // open drain disabled
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE; // low drive strength
#   endif /*BRS_DERIVATIVE_IMX53*/
#   if defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX25)
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DRV;  /* 3.3 V */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS;  /* hysteresis disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PULL; /* pull/keep disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE;  /* CMOS output */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE;  /* normal drive strength */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_SRE;  /* slow slew rate */
#   endif /*BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX25*/
  *IOMUXC_SW_PAD_CTL_PAD_CANRX1 = tmpVar;

  tmpVar = IOMUXC_SELECT_INPUT_CAN1;
  *IOMUXC_CAN1_IPP_IND_CANRX_SELECT_INPUT = tmpVar;
#  endif /*kCanPhysToLogChannelIndex_0*/

#  if defined (kCanPhysToLogChannelIndex_1)
  /* TX pin */ /* select pin function */
  tmpVar = *IOMUXC_SW_MUX_CTL_PAD_CANTX2;
  tmpVar &= ~(IOMUXC_SW_MUX_CTL_SION | IOMUXC_SW_MUX_CTL_MODE);
  tmpVar |= IOMUXC_MODE_CAN2;
  *IOMUXC_SW_MUX_CTL_PAD_CANTX2 = tmpVar;
  
  /*  configure pin */
  tmpVar = *IOMUXC_SW_PAD_CTL_PAD_CANTX2;
#   if defined (BRS_DERIVATIVE_IMX53)
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS; /* hysteresis disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PKE; /* pull disabled       */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUE; /* keep selected field */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUS; /* 360KOhm PD          */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE; /* open drain disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE; /* low drive strength  */
#   endif /*BRS_DERIVATIVE_IMX53*/
#   if defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX25)
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DRV;  /* 3.3 V */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS;  /* hysteresis disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PULL; /* pull/keep disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE;  /* CMOS output */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE;  /* normal drive strength */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_SRE;  /* slow slew rate */
#   endif /*BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX25*/
  *IOMUXC_SW_PAD_CTL_PAD_CANTX2 = tmpVar;

  /* RX pin */
  /* select pin function */
  tmpVar = *IOMUXC_SW_MUX_CTL_PAD_CANRX2;
  tmpVar &= ~(IOMUXC_SW_MUX_CTL_SION | IOMUXC_SW_MUX_CTL_MODE);
  tmpVar |= IOMUXC_MODE_CAN2;
  *IOMUXC_SW_MUX_CTL_PAD_CANRX2 = tmpVar;

  /* configure pin */
  tmpVar = *IOMUXC_SW_PAD_CTL_PAD_CANRX2;
#   if defined (BRS_DERIVATIVE_IMX53)
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS; /* hysteresis disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PKE; /* pull disabled       */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUE; /* keep selected field */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUS; /* 360KOhm PD          */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE; /* open drain disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE; /* low drive strength  */
#   endif /*BRS_DERIVATIVE_IMX53*/
#   if defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX25)
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DRV;  /* 3.3 V */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS;  /* hysteresis disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PULL; /* pull/keep disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE;  /* CMOS output */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE;  /* normal drive strength */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_SRE;  /* slow slew rate */
#   endif /*BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX25*/
  *IOMUXC_SW_PAD_CTL_PAD_CANRX2 = tmpVar;
  
  tmpVar = IOMUXC_SELECT_INPUT_CAN2;
  *(IOMUXC_CAN2_IPP_IND_CANRX_SELECT_INPUT) = tmpVar;
#  endif /*kCanPhysToLogChannelIndex_1*/

#  if defined (BRS_DERIVATIVE_IMX53)
  *(IOMUXC_SW_MUX_CTL_PAD_GPIO_17) = (uint32)0x01;
  *(IOMUXC_SW_PAD_CTL_PAD_GPIO_17) = (uint32)0x00; 
  *(IOMUXC_SW_MUX_CTL_PAD_GPIO_18) = (uint32)0x01;
  *(IOMUXC_SW_PAD_CTL_PAD_GPIO_18) = (uint32)0x00;
  *GPIO_REG(GPIO_BASE_7,GPIO_GDIR) |= GPIOBIT(12) | GPIOBIT(13);
  *GPIO_REG(GPIO_BASE_7,GPIO_DR)   |= GPIOBIT(12) | GPIOBIT(13);

  *(IOMUXC_SW_MUX_CTL_PAD_GPIO_14) = (uint32)0x00;
  *(IOMUXC_SW_PAD_CTL_PAD_GPIO_14) = (uint32)0xE3;
  *GPIO_REG(GPIO_BASE_4,GPIO_GDIR) |= GPIOBIT(4);
  *GPIO_REG(GPIO_BASE_4,GPIO_DR)   |= GPIOBIT(4);

  /* LED Port initialization (i.MX53 EVK PCB REV. A) */
#   if defined (BRS_ENABLE_SUPPORT_LEDS)
  tmpVar = *(IOMUXC_SW_MUX_CTL_PAD_PATA_DA_1);
  tmpVar &= ~(IOMUXC_SW_MUX_CTL_SION | IOMUXC_SW_MUX_CTL_MODE);  /* clear SION, MUX_MODE */
  tmpVar |= IOMUXC_MODE_LED;                                     /* select mux_mode 1 (gpio) */
  *(IOMUXC_SW_MUX_CTL_PAD_PATA_DA_1) = tmpVar;                   /* write back */

  tmpVar = *(IOMUXC_SW_PAD_CTL_PAD_PATA_DA_1);
  tmpVar &= ~IOMUXC_SW_PAD_CTL_HYS; /* hysteresis disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PKE; /* pull disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUE; /* keep selected field */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_PUS; /* 360KOhm PD */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_ODE; /* open drain disabled */
  tmpVar &= ~IOMUXC_SW_PAD_CTL_DSE; /* low drive strength */
  *(IOMUXC_SW_PAD_CTL_PAD_PATA_DA_1) = tmpVar;
  *GPIO_REG(GPIO_BASE_7,GPIO_GDIR) |= GPIOBIT(7); /* set gpio7 pin7 to output */
#   endif /*BRS_ENABLE_SUPPORT_LEDS*/
#  endif /*BRS_DERIVATIVE_IMX53*/
# endif /*BRS_DERIVATIVE_IMX25 || BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX53*/

# if defined (BRS_DERIVATIVE_IMX6X)
#  if defined (VGEN_ENABLE_CAN_DRV)
#   if defined (kCanPhysToLogChannelIndex_0) /*FlexCAN1 */
  /* CAN1 TX Possible options:
       MX6SLX_PAD_KEY_COL2__CAN1_TX
       MX6SLX_PAD_QSPI1B_DQS__CAN1_TX 
       MX6SLX_PAD_SD3_DATA5__CAN1_TX <- In VEBN01059 this one needs R44 to be set to "B" to use the onboard transceiver.
  */
  BrsHwIoMuxV3SetupPad(MX6SLX_PAD_QSPI1B_DQS__CAN1_TX);

  /* CAN1 RX Possible options:
       MX6SLX_PAD_KEY_ROW2__CAN1_RX
       MX6SLX_PAD_QSPI1A_SS1_B__CAN1_RX <- In VEBN01059 this one needs R690 to be set to "B" to use the onboard transceiver.
       MX6SLX_PAD_SD3_DATA7__CAN1_RX
  */
  BrsHwIoMuxV3SetupPad(MX6SLX_PAD_QSPI1A_SS1_B__CAN1_RX);

  /* CAN Transceiver CAN1_2_STBY_B*/
  BrsHwIoMuxV3SetupPad(MX6SLX_PAD_QSPI1B_DATA3__GPIO4_IO_27);

  /* Set pin QSPI1B_DATA3 direction to output */
  BrsHwGpioSetDir(BRS_GPIO4_BASE_ADDR, 27,(uint8)BRS_GPIO_DIR_OUT);

  /* Pull down pin QSPI1B_DATA3 */
  BrsHwGpioSetVal(BRS_GPIO4_BASE_ADDR,27,(uint8)BRS_LED_OFF);
#   endif /*kCanPhysToLogChannelIndex_0*/
#   if defined (kCanPhysToLogChannelIndex_1) /*FlexCAN2 */
  /* CAN2 TX Possible options:
       MX6SLX_PAD_KEY_COL3__CAN2_TX <- In VEBN01059 this pin is also used for the user LED, so plase don't use this pin
       MX6SLX_PAD_QSPI1A_DQS__CAN2_TX <- In VEBN01059 this pin is connected to the PORT3_P68 signal from the J503 expansion connector.
       MX6SLX_PAD_SD3_DATA6__CAN2_TX
  */
  BrsHwIoMuxV3SetupPad(MX6SLX_PAD_QSPI1A_DQS__CAN2_TX);

  /* CAN2 RX Possible options:
       MX6SLX_PAD_KEY_ROW3__CAN2_RX
       MX6SLX_PAD_QSPI1B_SS1_B__CAN2_RX <- In VEBN01059 this pin is connected to the PORT3_P66 signal from the J503 expansion connector.
       MX6SLX_PAD_SD3_DATA4__CAN2_RX
  */
  BrsHwIoMuxV3SetupPad(MX6SLX_PAD_QSPI1B_SS1_B__CAN2_RX);
#   endif /*kCanPhysToLogChannelIndex_1*/
#  endif /*VGEN_ENABLE_CAN_DRV*/

#  if defined (BRS_ENABLE_SUPPORT_LEDS)
  /* Configure function GPIO2_IO_13 to pin KEY_COL */
  BrsHwIoMuxV3SetupPad(MX6SLX_PAD_KEY_COL3__GPIO2_IO_13);

  /* Configure GPIO2_IO_13 as output */
  BrsHwGpioSetDir(BRS_GPIO2_BASE_ADDR, 13,(uint8)BRS_GPIO_DIR_OUT);

  /* Turn the user LED on */
  BrsHwSetLed(BRSHW_LED_SYSTEM_OK, BRSHW_LED_ON);
#  endif /*BRS_ENABLE_SUPPORT_LEDS*/
# endif /*BRS_DERIVATIVE_IMX6X*/

# if defined (BRS_DERIVATIVE_IMX6S)
#  if defined (VGEN_ENABLE_CAN_DRV)
  BRS_IOMUXC_MUX_CTL_GPIO7 = BRS_GPIO7_MUX_CONFIG;  /* Configure the pin function as FLEXCAN1_TX  */
  BRS_IOMUXC_PAD_CTL_GPIO7 = BRS_GPIO7_PAD_CONFIG;  /* Configure the PAD settings for FLEXCAN1_TX */

  BRS_IOMUXC_MUX_CTL_GPIO8 = BRS_GPIO8_MUX_CONFIG;  /* Configure the pin function as FLEXCAN1_RX  */
  BRS_IOMUXC_PAD_CTL_GPIO8 = BRS_GPIO8_PAD_CONFIG;  /* Configure the PAD settings for FLEXCAN1_RX */
#  endif /*VGEN_ENABLE_CAN_DRV*/

#  if defined (BRS_ENABLE_SUPPORT_LEDS)
#   if defined (BRS_EVA_BOARD_VEBN01084)
  BRS_IOMUXC_MUX_CTL_GPIO2 = BRS_GPIO2_MUX_CONFIG;  /* Configure the pin for the LED as GPIO  */
  BRS_IOMUXC_PAD_CTL_GPIO2 = BRS_GPIO2_PAD_CONFIG;  /* Configure the PAD settings for the LED */
#   endif

  BRS_GPIO1->GDIR |= (1<<BRSHW_LED_SYSTEM_OK);      /* Set the pin of the LED as output       */

  /* Set one LED to show the system is alive */
  BrsHwSetLed(BRSHW_LED_SYSTEM_OK, BRSHW_LED_ON);
#  endif /*BRS_ENABLE_SUPPORT_LEDS*/
# endif /*BRS_DERIVATIVE_IMX6S*/


#  if defined (VGEN_ENABLE_DRVETH__BASEASR)

  /* Portconfig for Ethernet on Sabre Lite Board */

  /* defines */
#define BRS_ETH_PAD_CFG_0                            0x439//0x100b0  //davor: 0x439
#define BRS_ETH_PAD_CFG_1                            0x439//0x1b0b0
#define BRS_ETH_DRV_STRENGTH                         0x000C0000
#define BRS_ETH_DRV_TERM                             0x00000300
#define BRS_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII     BRS_IOS(uint32, 0x20E0790)
#define BRS_IOMUXC_SW_PAD_CTL_GRP_RGMII_TERM         BRS_IOS(uint32, 0x20E07AC)

#define BRS_IOMUXC_GPR1                              BRS_IOS(uint32, 0x20E0004)
#define BRS_IOMUXC_ENET_REF_CLK_SELECT_INPUT         BRS_IOS(uint32, 0x20E083C)

#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RXC          BRS_IOS(uint32, 0x20E01D0)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TXC          BRS_IOS(uint32, 0x20E0058)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_ENET_REF_CLK       BRS_IOS(uint32, 0x20E01D4)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RX_CTL       BRS_IOS(uint32, 0x20E006C)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TX_CTL       BRS_IOS(uint32, 0x20E0074)   //was ist der Unterschied???
//#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TX_EN        BRS_IOS(uint32, 0x20E01E8)   //was ist der Unterschied???
#define BRS_IOMUXC_SW_MUX_CTL_PAD_ENET_MDIO          BRS_IOS(uint32, 0x20E01D0)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_ENET_MDC           BRS_IOS(uint32, 0x20E01F4)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD0          BRS_IOS(uint32, 0x20E01E0)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD1          BRS_IOS(uint32, 0x20E0078)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD2          BRS_IOS(uint32, 0x20E007C)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD3          BRS_IOS(uint32, 0x20E0080)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD0          BRS_IOS(uint32, 0x20E005C)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD1          BRS_IOS(uint32, 0x20E0060)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD2          BRS_IOS(uint32, 0x20E0064)
#define BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD3          BRS_IOS(uint32, 0x20E0068)

#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RXC          BRS_IOS(uint32, 0x20E0398)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TXC          BRS_IOS(uint32, 0x20E036C)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK       BRS_IOS(uint32, 0x20E04E8)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RX_CTL       BRS_IOS(uint32, 0x20E0380)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_CTL       BRS_IOS(uint32, 0x20E0388)   //was ist der Unterschied???
//#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_EN        BRS_IOS(uint32, 0x20E04FC)   //was ist der Unterschied???
#define BRS_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO          BRS_IOS(uint32, 0x20E04E4)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC           BRS_IOS(uint32, 0x20E0508)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD0          BRS_IOS(uint32, 0x20E0384)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD1          BRS_IOS(uint32, 0x20E038C)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD2          BRS_IOS(uint32, 0x20E0390)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD3          BRS_IOS(uint32, 0x20E0394)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD0          BRS_IOS(uint32, 0x20E0370)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD1          BRS_IOS(uint32, 0x20E0374)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD2          BRS_IOS(uint32, 0x20E0378)
#define BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD3          BRS_IOS(uint32, 0x20E037C)


  //BRS_IOMUXC_SW_PAD_CTL_GRP_DDR_TYPE_RGMII = BRS_ETH_DRV_STRENGTH;
  //BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TXC      = BRS_ETH_DRV_TERM;
  /* Clock */
  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RXC = 0x00000001;  /* Configure the pin function as ALT1: RXC  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RXC = BRS_ETH_PAD_CFG_1;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TXC = 0x00000001;  /* Configure the pin function as ALT1: TXC  */
  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TXC = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_ENET_REF_CLK = 0x00000001;  /* Configure the pin function as ALT1: REF_CLK  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_ENET_REF_CLK = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  /* Control */
  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RX_CTL = 0x00000001;  /* Configure the pin function as ALT1: RX_CTL (RXDV)  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RX_CTL = BRS_ETH_PAD_CFG_1;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TX_CTL = 0x00000001;  /* Configure the pin function as ALT1: TX_CTL */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_CTL = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  //BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TX_EN = 0x00000001;  /* Configure the pin function as ALT1: TX_EN (TXEN) */
  //BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TX_EN = BRS_ETH_PAD_CFG;  /* Configure the PAD settings for ETH */

  /* MDIO */
  BRS_IOMUXC_SW_MUX_CTL_PAD_ENET_MDIO = 0x00000001;  /* Configure the pin function as ALT1: MDIO */
  BRS_IOMUXC_SW_PAD_CTL_PAD_ENET_MDIO = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_ENET_MDC = 0x00000001;  /* Configure the pin function as ALT1: MDC */
  BRS_IOMUXC_SW_PAD_CTL_PAD_ENET_MDC = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  //!!!!!!!!!!!!!!!!!!!!!!!!RX_ERROR ist falsch verbunden!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  /* Data */
  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD0 = 0x00000001;  /* Configure the pin function as ALT1: RD0  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD0 = BRS_ETH_PAD_CFG_1;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD1 = 0x00000001;  /* Configure the pin function as ALT1: RD1  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD1 = BRS_ETH_PAD_CFG_1;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD2 = 0x00000001;  /* Configure the pin function as ALT1: RD2  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD2 = BRS_ETH_PAD_CFG_1;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_RD3 = 0x00000001;  /* Configure the pin function as ALT1: RD3  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_RD3 = BRS_ETH_PAD_CFG_1;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD0 = 0x00000001;  /* Configure the pin function as ALT1: TD0  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD0 = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD1 = 0x00000001;  /* Configure the pin function as ALT1: TD1  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD1 = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD2 = 0x00000001;  /* Configure the pin function as ALT1: TD2  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD2 = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  BRS_IOMUXC_SW_MUX_CTL_PAD_RGMII_TD3 = 0x00000001;  /* Configure the pin function as ALT1: TD3  */
  BRS_IOMUXC_SW_PAD_CTL_PAD_RGMII_TD3 = BRS_ETH_PAD_CFG_0;  /* Configure the PAD settings for ETH */

  //BRS_IOMUXC_GPR1                    &= ~(1 << 21);
  BRS_IOMUXC_ENET_REF_CLK_SELECT_INPUT                    &= ~0x00000001;
#  endif /*VGEN_ENABLE_DRVETH__BASEASR*/

#endif /*!VGEN_ENABLE_DRVPORT*/
}

/*******************************************************************************
* NAME          : BrsHwEvaBoardInitPowerOn
* CALLED BY     : BrsHwHardwareInitPowerOn or EcuM at power on initialization
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : This function has to be used to initialize misc features not
*                 covered by the three functions above.
********************************************************************************/
void BrsHwEvaBoardInitPowerOn(void)
{
  /* Function mandatory for BrsAsr */
  /* nothing to be done here */
}

# if !defined (BRS_CPU_CORE_CORTEX_M4)
/*******************************************************************************
* NAME          : BrsHwTimeBaseInitPowerOn
* CALLED BY     : main@BrsMain at power on initialization
* PRECONDITIONS : Interrupt vector must be correct configured and the ISR
*                 function itself should exist ...
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : Programmable Interrupt Timer, Timer initialization for 'FakeOS'
*                 1ms time base generation
********************************************************************************/
void BrsHwTimeBaseInitPowerOn(void)
{
#  if defined (BRS_OS_USECASE_BRS)
/* This function is not necessary if an OS is running! */
#   if defined (BRS_DERIVATIVE_IMX6S) || defined (BRS_DERIVATIVE_IMX6X)
  /* Initialize EPIT1 for 1ms IRQ */
  BRS_EPIT1->CR  = 0;                                                               /* Disable EPIT1                */
  BRS_EPIT1->CR |= (1U << BRS_EPIT_CR_SWR_BIT);                                     /* Software Reset of EPIT1      */
  while( BRS_EPIT1->CR & (1U << BRS_EPIT_CR_SWR_BIT) );                             /* Wait until reset is finished */

  BRS_EPIT1->CR  = (1U << BRS_EPIT_CR_CLKSRC_BITS);                                 /* Clock source is set to Peripheral clock (ipg_clk)                        */
  BRS_EPIT1->CR |= (1U << BRS_EPIT_CR_IOVW_BIT);                                    /* Write to load register results in immediate overwriting of counter value */
  BRS_EPIT1->CR |= ((BRS_EPIT1_PRESCALER_VALUE - 1) << BRS_EPIT_CR_PRESCALAR_BITS); /* Set prescaler value                                                      */
  BRS_EPIT1->CR |= (1U << BRS_EPIT_CR_RLD_BIT);                                     /* Use reload value when counter reaches zero                               */
  BRS_EPIT1->CR |= (1U << BRS_EPIT_CR_OCIEN_BIT);                                   /* Compare interrupt is enabled                                             */
  BRS_EPIT1->CR |= (1U << BRS_EPIT_CR_ENMOD_BIT);                                   /* Counter starts count from load value                                     */

  BRS_EPIT1->LR   = (BRS_EPIT1_LOAD_VALUE - 1);                                     /* EPIT1 reload value                         */
  BRS_EPIT1->CMPR = (BRS_EPIT1_LOAD_VALUE - 1);                                     /* EPIT1 compare value                        */
  BRS_EPIT1->SR   = 1;                                                              /* Clear Status register of the compare event */

  BRS_EPIT1->CR |= (1U << BRS_EPIT_CR_EN_BIT);                                      /* Enable EPIT1 */
  /* IRQ initialization for EPIT see 'BrsHwHardwareInitPowerOn()'  */
#   else
  void *epit_handle;
#    if defined (BRS_DERIVATIVE_IMX53)
    epit_handle = epit_init(0, 32, 1000);    //timer 1(offset =0) , prescaler =32 , load value =1000 ; (32MHz/32=1MHz->1000counts=1ms)
#    elif defined (BRS_DERIVATIVE_IMX25) || defined (BRS_DERIVATIVE_IMX35)
    epit_handle = epit_init(0, 665, 100);    
#    endif

  epit_connect(epit_handle, EPIT1_INTLVL, BrsHwTimeBaseInterrupt, epit_handle);
  epit_enable(epit_handle);
  
#   endif
#  endif /*BRS_OS_USECASE_BRS*/
}

#  if defined (BRS_OS_USECASE_BRS)
/*******************************************************************************
* NAME          : BrsHwTimeBaseInterrupt
* CALLED BY     : Via interrupt vector table if the assigned timer has set its
*                 interrupt flag.
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : Timer interrupt for the 'FakeOS' 1ms time base generation.
********************************************************************************/
#   if defined (BRS_DERIVATIVE_IMX6S) || defined (BRS_DERIVATIVE_IMX6X)
void BrsHwTimeBaseInterrupt(void)
#   else
void BrsHwTimeBaseInterrupt(void *epit_handle)
#   endif
{
#   if defined (BRS_DERIVATIVE_IMX6S) || defined (BRS_DERIVATIVE_IMX6X)
  BRS_EPIT1->SR   = 1;  /* Clear Status register of the compare event */
#   else
  epit_ack(epit_handle);  
  /*ConPuts("ms Timer Interrupt\r\n");*/
#   endif

  /*Increment the "fakeOS" 1ms timer tick now. This interrupt should occur
   * approximately every 1ms. The resolution of the timer is 1000Hz not including
   * interrupt latency.*/
  gbBrsMainIsrMilliSecondFlag++;
}
#  endif /*BRS_OS_USECASE_BRS*/

# endif /*!BRS_CPU_CORE_CORTEX_M4*/

/*******************************************************************************
* NAME          : BrsHwConfigureInterruptsAtPowerOn
* CALLED BY     : BrsHwHardwareInitPowerOn or EcuM at power on initialization
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : This function has to be used to initialize the used interrupts.
********************************************************************************/
void BrsHwConfigureInterruptsAtPowerOn(void)
{
#if defined (BRS_DERIVATIVE_IMX25) || defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX53)
  BrsHwVectorTableMovement();
  BrsHwConInit(0);
#endif

#if defined (BRS_OS_USECASE_BRS)

# if defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX25)
  BrsHwExceptionIrqInit();
  BrsHwAvicInit();
# endif /*BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX25*/
  
# if defined (BRS_DERIVATIVE_IMX53)
  BrsHwTzicInit();
  
#  if defined (C_ENABLE_FLEXCAN_A) || defined (kCanPhysToLogChannelIndex_0)
#   if defined (C_ENABLE_CAN_RXTX_INTERRUPT) || defined (C_ENABLE_CAN_BUSOFF_INTERRUPT) || defined (C_ENABLE_CAN_WAKEUP_INTERRUPT)
  BrsHwTzicConnect(CAN0_INTNUM, CAN0_INTLVL, CanIsr_0, (void *)0);
  BrsHwTzicEnable(CAN0_INTNUM);
  /*ConPuts("\r\nCANISR0 initialized");*/
#   endif
#  endif /*C_ENABLE_FLEXCAN_A || kCanPhysToLogChannelIndex_0*/

#  if defined (C_ENABLE_FLEXCAN_B) || defined (kCanPhysToLogChannelIndex_1)
#   if defined (C_ENABLE_CAN_RXTX_INTERRUPT) || defined (C_ENABLE_CAN_BUSOFF_INTERRUPT) || defined (C_ENABLE_CAN_WAKEUP_INTERRUPT)
  BrsHwTzicConnect(CAN1_INTNUM, CAN1_INTLVL, CanIsr_1, (void *)0);
  BrsHwTzicEnable(CAN1_INTNUM);
  /*ConPuts("\r\nCANISR1 initialized");*/
#   endif
#  endif /*C_ENABLE_FLEXCAN_B || kCanPhysToLogChannelIndex_1*/
# endif /*BRS_DERIVATIVE_IMX53*/
  
# if defined (BRS_DERIVATIVE_IMX6X) || defined (BRS_DERIVATIVE_IMX6S)
#  if defined (BRS_CPU_CORE_CORTEX_M4)
#   if defined (VGEN_ENABLE_CAN_DRV)
  /* Set the base address to access the SCS register block */
  BRS_IMX6SLX_SCS * pSCS = (BRS_IMX6SLX_SCS *)BRS_SCS_BASE_ADDR;
#   endif /* VGEN_ENABLE_CAN_DRV */
  
  /* Load Vector table address offset */
  BRS_VECTOR_TABLE_OFFSET = (uint32)vector_table_core;
  
  /* Enable Core-Interrupt-Handlers, cleaning .bss section */
  BrsHwStartupRest();  
  
#   if defined (VGEN_ENABLE_CAN_DRV)
#    if defined (kCanPhysToLogChannelIndex_0) /*FlexCAN1 */
  /* Activate FLEXCAN1 IRQ */
  pSCS->NVIC.Enable[BRS_FLEXCAN1_INT_SRC] = BRS_FLEXCAN1_INT_EN;
#    endif /*kCanPhysToLogChannelIndex_0*/
  
#    if defined ( kCanPhysToLogChannelIndex_1) /*FlexCAN2 */
  /* Activate FLEXCAN2 IRQ */
  pSCS->NVIC.Enable[BRS_FLEXCAN2_INT_SRC] = BRS_FLEXCAN2_INT_EN;
#    endif /*kCanPhysToLogChannelIndex_1*/
#   endif /*VGEN_ENABLE_CAN_DRV*/
#  endif /*BRS_CPU_CORE_CORTEX_M4*/
  
#  if defined (BRS_CPU_CORE_CORTEX_A9)
  /* Write the address of the Vector Table to VBAR Register */
  asm("LDR r0, =_vector_table");
  asm("MCR p15, 0, r0, c12, c0, 0");
  
  BRS_GICC_BPR  = BRS_GICC_BPR_CONFIG;   /* Set the count of bits for the group priority and sub priority to -> g: 4bit, s: 4bit */
  BRS_GICC_PMR  = BRS_GICC_PMR_CONFIG;   /* Set Priority Mask Register to the lowest interrupt priority                          */
  
  BRS_GICC_CTLR = BRS_GICC_CTLR_CONFIG;  /* Enable forwarding of group 0 and 1 interrupts to the processor                       */
  BRS_GICD_CTLR = BRS_GICD_CTLR_CONFIG;  /* Enable forwarding of group 0 and 1 interrupts by the Distributor                     */
  
  /* EPIT init */
  BRS_GICD_ISENABLER2  |= (1U << BRS_GICD_ISENABLER2_EPIT1_BIT);  /* Enable EPIT1 IRQ in the GIC    */
  BRS_GICD_IPRIORITYR22 = 0x0;                                    /* Highest priority for EPIT1 IRQ */
  BRS_GICD_ITARGETSR22  =0x02020202;                             /* Set EPIT1 IRQ to target CPU0   */

  /* ENET IRQ */
#   if defined (VGEN_ENABLE_DRVETH__BASEASR)
  BRS_GICD_ISENABLER4  |= (1U << BRS_GICD_ISENABLER4_ENET_BIT);     /* Enable ENET IRQ in the GIC    */
  BRS_GICD_IPRIORITYR35 = 0x0;                                      /* Highest priority for ENET IRQ */
  BRS_GICD_ITARGETSR35  =0x02020202;                               /* Set ENET IRQ to target CPU0   */
#   endif /* VGEN_ENABLE_DRVETH__BASEASR */
  
#   if defined (VGEN_ENABLE_CAN_DRV)
#    if defined (kCanPhysToLogChannelIndex_0) /* FlexCAN1 */
  BRS_GICD_ISENABLER4  |= (1U << BRS_GICD_ISENABLER4_FLEXCAN1_BIT); /* Enable FlexCAN1 IRQ in the GIC    */
  BRS_GICD_IPRIORITYR35 = 0x0;                                      /* Highest priority for FLEXCAN1 IRQ */
  BRS_GICD_ITARGETSR35  =0x02020202;                               /* Set FLEXCAN1 IRQ to target CPU0   */
#    endif /*kCanPhysToLogChannelIndex_0*/
  
#    if defined (kCanPhysToLogChannelIndex_1) /* FlexCAN2 */
  BRS_GICD_ISENABLER4 |= (1U << BRS_GICD_ISENABLER4_FLEXCAN2_BIT); /* Enable FlexCAN2 IRQ in the GIC    */
  BRS_GICD_IPRIORITYR35 = 0x0;                                     /* Highest priority for FLEXCAN2 IRQ */
  BRS_GICD_ITARGETSR35  =0x02020202;                              /* Set FLEXCAN2 IRQ to target CPU0   */
#    endif /*kCanPhysToLogChannelIndex_1*/
#   endif /*VGEN_ENABLE_CAN_DRV*/
#  endif /*BRS_CPU_CORE_CORTEX_A9*/
# endif /*BRS_DERIVATIVE_IMX6X || BRS_DERIVATIVE_IMX6S*/

#endif /*BRS_OS_USECASE_BRS*/
}

/**************************************************************************
 * IRQ Handlers
 *************************************************************************/
#if defined (BRS_DERIVATIVE_IMX25) || defined (BRS_DERIVATIVE_IMX35) || defined (BRS_DERIVATIVE_IMX53)
void __FIQ_Handler(void)
{
  BrsHwIrqHandler();
}
void __IRQ_Handler(void)
{
  /* Only FIQ is used! */
}
#endif /*BRS_DERIVATIVE_IMX25 || BRS_DERIVATIVE_IMX35 || BRS_DERIVATIVE_IMX53*/

#if defined (BRS_DERIVATIVE_IMX6S) || defined (BRS_DERIVATIVE_IMX6X)
# if defined (BRS_CPU_CORE_CORTEX_A9)
/*****************************************************************************
 * IRQ-Handler definitions
 *****************************************************************************/
void __IRQ_Handler(void)
{
  uint32 irq_id;
  irq_id = BRS_GICC_IAR;  /* Acknowledge the IRQ by reading the IRQ-ID from the IAR register  */

  /* Jump to the IRQ-Handler belonging to the current IRQ-ID */
  switch(irq_id)
  {
# if defined (BRS_OS_USECASE_BRS)
    /* This ISR function is not necessary if the OSEK OS is used!     */
    case BRS_IRQ_ID_EPIT1:       /* IRQ-ID of EPIT1 */
      BrsHwTimeBaseInterrupt();
    break;
# endif /*BRS_OS_USECASE_BRS*/

# if defined (VGEN_ENABLE_DRVETH__BASEASR)
#if  (ETH_ENABLE_RX_INTERRUPT == STD_ON)
    case BRS_IRQ_ID_ENET:       /* IRQ-ID of ENET  */
      EthIsr_EthCtrlConfig_EthInterruptServiceRoutine();
        break;
#  endif
# endif

# if defined (VGEN_ENABLE_CAN_DRV)
#  if defined (kCanPhysToLogChannelIndex_0)
#   if defined (C_ENABLE_CAN_RXTX_INTERRUPT)   || \
       defined (C_ENABLE_CAN_BUSOFF_INTERRUPT) || \
       defined (C_ENABLE_CAN_WAKEUP_INTERRUPT)
    case BRS_IRQ_ID_FLEXCAN1:       /* IRQ-ID of FlexCAN1   */
      CanIsr_0();
    break;
#   endif /* C_ENABLE_CAN_RXTX_INTERRUPT || C_ENABLE_CAN_BUSOFF_INTERRUPT || C_ENABLE_CAN_WAKEUP_INTERRUPT */
#  endif /* kCanPhysToLogChannelIndex_0 */

#  if defined (kCanPhysToLogChannelIndex_1)
#   if defined (C_ENABLE_CAN_RXTX_INTERRUPT)   || \
       defined (C_ENABLE_CAN_BUSOFF_INTERRUPT) || \
       defined (C_ENABLE_CAN_WAKEUP_INTERRUPT)
    case BRS_IRQ_ID_FLEXCAN2:       /* IRQ-ID of FlexCAN2   */
      CanIsr_1();
    break;
#   endif /* C_ENABLE_CAN_RXTX_INTERRUPT || C_ENABLE_CAN_BUSOFF_INTERRUPT || C_ENABLE_CAN_WAKEUP_INTERRUPT */
#  endif /* kCanPhysToLogChannelIndex_1 */
# endif /* VGEN_ENABLE_CAN_DRV */

    default:
      while(1){}               /* ERROR: unexpected irq_id occurred */
  }

  BRS_GICC_EOIR = irq_id;      /* Signal the GIC that the IRQ was successfully processed */
}

void __FIQ_Handler(void)
{
  while(1){}
}
# endif /*BRS_CPU_CORE_CORTEX_A9*/

#endif /*BRS_DERIVATIVE_IMX6S || BRS_DERIVATIVE_IMX6X*/

/*******************************************************************************
* NAME          : BrsHwSoftwareResetECU
* CALLED BY     : can driver testsuite
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : restart ECU
********************************************************************************/
void BrsHwSoftwareResetECU(void)
{
  /* TODO: call reset function, if possible*/
  while(1){}; /* Function currently not implemented! */
}

/*******************************************************************************
* NAME          : BrsHwSetLed
* CALLED BY     : All modules to set or clear a LED
* PRECONDITIONS : none
* PARAMETERS    : none
* RETURN VALUE  : none
* DESCRIPTION   : Up to 8 LEDs are supported by the BRS. 
*                 Please note, that not every EVA hardware supports LEDs and due 
*                 to this do not use this feature for a general test case!
********************************************************************************/
void BrsHwSetLed(uint8 LedNumber, uint8 NewState)
{
#if defined (BRS_ENABLE_SUPPORT_LEDS)
# if defined (BRS_EVA_BOARD_VEBN01013) /* IMX6X, cortex A9 + M4 */
  if (NewState)
  {
    /* Set LED on PTB3 */
    BrsHwGpioSetVal(BRS_GPIO2_BASE_ADDR,LedNumber,(uint8)BRS_LED_ON);
  }
  else
  {
    /*Clear LED on PTB3 */
    BrsHwGpioSetVal(BRS_GPIO2_BASE_ADDR,LedNumber,(uint8)BRS_LED_OFF);
  }
# elif defined (BRS_EVA_BOARD_VEBN01084) /* IMX6S, cortex A9 */
  if (NewState > 0)
  {
    /* Set LED */
    BRS_GPIO1->DR |= (1U << LedNumber);
  }
  else
  {
    /* Clear LED */
    BRS_GPIO1->DR &= ~(1U << LedNumber);
  }
# else
#  if defined (BRSHW_LED_PORT) /* imx25, imx35, imx53 */
  uint16 usPortMask = (uint16)(1 << LedNumber);
  if (usPortMask & BRSHW_LED_PORTMASK) /* check if LED is available */
  {
    if(!NewState) 
    {
      BRSHW_LED_PORT |= usPortMask;
    }
    else 
    {
      BRSHW_LED_PORT &= ~usPortMask;
    }
  }
#  endif /*BRSHW_LED_PORT*/

  if (NewState)
  {
    *GPIO_REG(GPIO_BASE_7,GPIO_DR) |= 0x00000080;
  }
  else
  {
    *GPIO_REG(GPIO_BASE_7,GPIO_DR) &= 0xFFFFFF7F;
  }
# endif /*BRS_EVA_BOARD_x*/
#endif /*BRS_ENABLE_SUPPORT_LEDS*/
}

/*****************************************************************************/
/**
 * @brief Forwards wakeup request to CCL.
 * @pre   BrsHwConfigWakeUpIsr() has been called.
 *
 * In case of a wakeup interrupt, forward to CCL.
 *
 */
/*****************************************************************************/
void BrsHwWakeUpIsr_0(void)
{
  /*BrsHwClearWakeupIRQ(0);*/

#if defined (VGEN_ENABLE_CCL)
# if defined (CCL_ENABLE_EXTERNAL_REQUEST) && \
     defined (CCL_ENABLE_TRCV_PORT_INT)
#  if defined (C_MULTIPLE_RECEIVE_CHANNEL)
#   if(kCclNrOfChannels > 1)
  CclCanWakeUpInt(0);
#   else
  CclCanWakeUpInt();
#   endif
#  else
  CclCanWakeUpInt();
#  endif
# endif
#endif /*VGEN_ENABLE_CCL*/

#if defined (VGEN_ENABLE_SYSSERVICE_ASRECUM)
  EcuM_SetWakeupEvent(CanIf_Config.eWakeUpConfig[0].eWakeUpSource);
#endif /*VGEN_ENABLE_SYSSERVICE_ASRECUM*/
}

void BrsHwWakeUpIsr_1(void)
{
  /*BrsHwClearWakeupIRQ(1);*/

#if defined (VGEN_ENABLE_CCL)
# if defined (CCL_ENABLE_EXTERNAL_REQUEST) && \
     defined (CCL_ENABLE_TRCV_PORT_INT)
#  if defined (C_MULTIPLE_RECEIVE_CHANNEL)
#   if(kCclNrOfChannels > 1)
  CclCanWakeUpInt(1);
#   else
  CclCanWakeUpInt();
#   endif
#  else
  CclCanWakeUpInt();
#  endif
# endif
#endif  /*VGEN_ENABLE_CCL*/

#if defined (VGEN_ENABLE_SYSSERVICE_ASRECUM)
  EcuM_SetWakeupEvent(CanIf_Config.eWakeUpConfig[1].eWakeUpSource);
#endif /*VGEN_ENABLE_SYSSERVICE_ASRECUM*/
}

void BrsHwWakeUpIsr_2(void)
{
  /*BrsHwClearWakeupIRQ(2);*/

#if defined (VGEN_ENABLE_CCL)
# if defined (CCL_ENABLE_EXTERNAL_REQUEST) && \
     defined (CCL_ENABLE_TRCV_PORT_INT)
#  if defined (C_MULTIPLE_RECEIVE_CHANNEL)
#   if(kCclNrOfChannels > 1)
  CclCanWakeUpInt(2);
#   else
  CclCanWakeUpInt();
#   endif
#  else
  CclCanWakeUpInt();
#  endif
# endif
#endif  /* VGEN_ENABLE_CCL */

#if defined (VGEN_ENABLE_SYSSERVICE_ASRECUM)
  EcuM_SetWakeupEvent(CanIf_Config.eWakeUpConfig[2].eWakeUpSource);
#endif /*VGEN_ENABLE_SYSSERVICE_ASRECUM*/
}

#if defined (BRS_DERIVATIVE_IMX6X)
/*****************************************************************************/
/**
 * \brief Routine to finish startup
 * 
 * Tasks: Enable diverse Fault-Handler, clearing the .bss section.
 */
/*****************************************************************************/
# if defined (BRS_CPU_CORE_CORTEX_M4)
void BrsHwStartupRest(void)
{
  /* Enable MemManage Fault, Bus Fault and Usage Fault exceptions */
  BrsHwEnableHandler();
}

void BrsHwEnableHandler(void)
{
# if defined (BRS_COMP_GHS)
#pragma asm
# endif
  ldr r0, =0xE000ED24
  ldr r1, =0x00070000
  str r1, [r0]
# if defined (BRS_COMP_GHS)
#pragma endasm
# endif 
}
#endif /*BRS_CPU_CORE_CORTEX_M4*/
/*****************************************************************************/
/**
 * \brief Set the operation mode of a pin
 *
 * \param[in] uint64 pad register parameter
 */
/*****************************************************************************/
void BrsHwIoMuxV3SetupPad(uint64 pad) /*iomux_v3_setup_pad*/
{
  uint32 mux_ctrl_ofs = (pad & MUX_CTRL_OFS_MASK) >> MUX_CTRL_OFS_SHIFT;
  uint32 mux_mode = (pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT;
  uint32 sel_input_ofs = (pad & MUX_SEL_INPUT_OFS_MASK) >> MUX_SEL_INPUT_OFS_SHIFT;
  uint32 sel_input = (pad & MUX_SEL_INPUT_MASK) >> MUX_SEL_INPUT_SHIFT;
  uint32 pad_ctrl_ofs = (pad & MUX_PAD_CTRL_OFS_MASK) >> MUX_PAD_CTRL_OFS_SHIFT;
  uint32 pad_ctrl = (pad & MUX_PAD_CTRL_MASK) >> MUX_PAD_CTRL_SHIFT;

  if (mux_ctrl_ofs)
    BRS_RAW_WR(mux_mode, BRS_AIPS_BASE_ADDR + mux_ctrl_ofs);

  if (sel_input_ofs)
    BRS_RAW_WR(sel_input, BRS_AIPS_BASE_ADDR + sel_input_ofs);

  if (!(pad_ctrl & NO_PAD_CTRL) && pad_ctrl_ofs)
    BRS_RAW_WR(pad_ctrl, BRS_AIPS_BASE_ADDR + pad_ctrl_ofs);
}

# if defined (VGEN_ENABLE_CAN_DRV)
static inline void BrsHwGpioSetDir (uint32 gpio_base, uint32 bit, uint8 dir) /* gpio_set_dir */
{
  uint32 gpio_dir_addr = gpio_base + 4;
  uint32 gpio_dir =  BRS_IOS(uint32,gpio_dir_addr);
  switch (dir)
  {
    case BRS_GPIO_DIR_OUT:
      gpio_dir |= 1 << bit;
      break;
    case BRS_GPIO_DIR_IN:
      gpio_dir &= ~(1 << bit);
      break;
  }

  BRS_RAW_WR(gpio_dir, gpio_dir_addr);
}

static inline void BrsHwGpioSetVal(uint32 gpio_base, uint32 bit, uint32 value) /* gpio_set_val */
{
  uint32 gpio_dr_addr = gpio_base;
  uint32 gpio_dr = BRS_IOS(uint32,gpio_base);
  if (value)
    gpio_dr |= 1 << bit;
  else
    gpio_dr &= ~(1 << bit);

  BRS_RAW_WR(gpio_dr, gpio_dr_addr);
}
# endif /*VGEN_ENABLE_CAN_DRV*/

#endif /*BRS_DERIVATIVE_IMX6X*/
