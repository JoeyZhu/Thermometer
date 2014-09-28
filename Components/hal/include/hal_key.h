/**************************************************************************************************
  Filename:       hal_key.h
  Revised:        $Date: 2007-07-06 10:42:24 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Description:    This file contains the interface to the KEY Service.


  Copyright 2005-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef HAL_KEY_H
#define HAL_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_KEY_RISING_EDGE   0
#define HAL_KEY_FALLING_EDGE  1

#define HAL_KEY_DEBOUNCE_VALUE  25

/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF P0IF
#define HAL_KEY_CPU_PORT_2_IF P2IF

#define HAL_KEY_SW_1 0x01  // Joystick up
   
#define RELEASE_TEMPEVER

#ifdef RELEASE_TEMPEVER
/* SW_1 is at P0.1 */
#define HAL_KEY_SW_1_PORT   P0
#define HAL_KEY_SW_1_BIT    BV(2)
#define HAL_KEY_SW_1_SEL    P0SEL
#define HAL_KEY_SW_1_DIR    P0DIR

#define HAL_KEY_SW_1_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_1_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_1_ICTLBIT  BV(2) /* P0IEN - P0.1 enable/disable bit */
#define HAL_KEY_SW_1_IENBIT   BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_1_PXIFG    P0IFG /* Interrupt flag at source */

#define HAL_KEY_SW_1_EDGEBIT    BV(0)

#else

#define HAL_KEY_SW_1_PORT   P1
#define HAL_KEY_SW_1_BIT    BV(2)
#define HAL_KEY_SW_1_SEL    P1SEL
#define HAL_KEY_SW_1_DIR    P1DIR

#define HAL_KEY_SW_1_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_1_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_1_ICTLBIT  BV(2) /* P1IEN - P1.2 enable/disable bit */
#define HAL_KEY_SW_1_IENBIT   BV(4) /* Mask bit for all of Port_1 */
#define HAL_KEY_SW_1_PXIFG    P1IFG /* Interrupt flag at source */

#define HAL_KEY_SW_1_EDGEBIT    BV(1)   /* low nibble for port1*/

#endif
/* Interrupt option - Enable or disable */
#define HAL_KEY_INTERRUPT_DISABLE    0x00
#define HAL_KEY_INTERRUPT_ENABLE     0x01

/* Key state - shift or nornal */
#define HAL_KEY_STATE_NORMAL          0x00
#define HAL_KEY_STATE_SHIFT           0x01

/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/
typedef void (*halKeyCBack_t) (uint8 keys, uint8 state);

/**************************************************************************************************
 *                                             GLOBAL VARIABLES
 **************************************************************************************************/
extern bool Hal_KeyIntEnable;

/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize the Key Service
 */
extern void HalKeyInit( void );

/*
 * Configure the Key Service
 */
extern void HalKeyConfig( bool interruptEnable, const halKeyCBack_t cback);

/*
 * Read the Key status
 */
extern uint8 HalKeyRead( void);

/*
 * Enter sleep mode, store important values
 */
extern void HalKeyEnterSleep ( void );

/*
 * Exit sleep mode, retore values
 */
extern uint8 HalKeyExitSleep ( void );

/*
 * This is for internal used by hal_driver
 */
extern void HalKeyPoll ( void );

/*
 * This is for internal used by hal_sleep
 */
extern bool HalKeyPressed( void );

extern uint8 hal_key_keys(void);                                           

extern uint8 hal_key_int_keys(void);

extern void halProcessKeyInterrupt (void);
/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
