/*
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    board.c
 * @brief   Board initialization file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#include <stdint.h>
#include "board.h"
#include "hal.h"

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 800/*800000*/; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

#if 0
void Boot_Cpu_SystemReset(void) {
  /* SCB_AIRCR: VECTKEY=0x05FA,SYSRESETREQ=1 */
  SCB_AIRCR = (uint32_t)((SCB_AIRCR & (uint32_t)~(uint32_t)(SCB_AIRCR_VECTKEY(0xFA05))) | 
  			  (uint32_t)(SCB_AIRCR_VECTKEY(0x05FA) | SCB_AIRCR_SYSRESETREQ_MASK));                      /* Request system reset */
  while(1) {                           /* Wait until reset */
  }
}
#endif

/**
 * @brief Set up and initialize all required blocks and functions related to the board hardware.
 */
void BOARD_InitDebugConsole(void) {
	/* The user initialization should be placed here */
}

/* Test LED blinker (Uses PD7). Should work in any conditions! */
void __blink(int n) {
#if 1
	PCC->CLKCFG[PCC_PORTD_INDEX] |= PCC_CLKCFG_CGC_MASK; // enable clock on PORT D
    PORTD->PCR[7U] = (PORTD->PCR[7U] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(1/*kPORT_MuxAsGpio*/);
	GPIOD->PSOR = 1U << 7U;
    GPIOD->PDDR |= (1U << 7U);
    int k;
    for (k = 0; k < 1; k++) {
    	GPIOD->PCOR = 1U << 7U;
   		for (int i = 0; i < 2*n; i++)
		{
	        GPIOD->PTOR = (1U << 7U);
    	    delay();
    	}
    	GPIOD->PSOR = 1U << 7U;
/*
    	for (int kk = 0; kk < 5; kk++) {
	    	delay();
	    }
*/
    }
/*
   	for (k = 0; k < 10; k++) {
    	delay();
    }
*/
#endif
}

void disableWatchdog(void) {
  WDOG->CNT = WDOG_UPDATE_KEY;
  WDOG->TOVAL = 0xFFFF;
  WDOG->CS = (uint32_t) ((WDOG->CS) & ~WDOG_CS_EN_MASK) | WDOG_CS_UPDATE_MASK;
}

void __early_init(void) {
	ke1xf_init();
}

void boardInit(void) {
}
