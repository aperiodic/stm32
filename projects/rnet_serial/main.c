/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdbool.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"
#include "cmddetail.h"

/*! The goal of this code is to run the shell through the serial terminal
 * and not the usb subsystem. Connect an FTDI serial/usb connector to the
 * appropriate (in rocketnet hub land this is pa9 and pa10)
 * These are configured as alternate functions in the board.h file.
 *
 * See the rocketnet-hub schematic for pinout.
 *
 *     stm		rnethub      ftdi
 * RX: pa10     ?            tx
 * TX: pa9      ?            rx
 *
 * In the mcuconf.h file enable the proper serial subsystem:
 * #define STM32_SERIAL_USE_USART1             TRUE
 *
 * In the halconf.h enable the serial system
 * #define HAL_USE_SERIAL                      TRUE
 */


static uint32_t           led_wait_time         =        500;

static const ShellCommand commands[] = {
#if DEBUG_KSZ
		{"ksz_nodes_en_n", cmd_ksz_nodes_en_n},
		{"ksz_n1en_n", cmd_ksz_n1en_n},
		{"ksz_rst_n", cmd_ksz_rst_n},
		{"ksz_pwr", cmd_ksz_pwr},
#endif
		{"phy", cmd_phy},
		{"show"    , cmd_show},
		{"mem"    , cmd_mem},
		{"threads", cmd_threads},
		{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
		(BaseSequentialStream *)&SD1,
		commands
};

static void led_init(void) {

    palClearPad(GPIOD, GPIO_D12_RGB_G);
    palClearPad(GPIOD, GPIO_D13_RGB_R);
    palClearPad(GPIOD, GPIO_D11_RGB_B);

    int i = 0;
    for(i=0; i<5; ++i) {
        palClearPad(GPIOD, GPIO_D12_RGB_G);
        chThdSleepMilliseconds(150);
        palSetPad(GPIOD, GPIO_D12_RGB_G);
        palClearPad(GPIOD, GPIO_D13_RGB_R);
        chThdSleepMilliseconds(150);
        palSetPad(GPIOD, GPIO_D13_RGB_R);
        palClearPad(GPIOD, GPIO_D11_RGB_B);
        chThdSleepMilliseconds(150);
        palSetPad(GPIOD, GPIO_D11_RGB_B);
    }
}

static WORKING_AREA(waThread_blinker, 64);
/*! \brief Green LED blinker thread
 */
static msg_t Thread_blinker(void *arg) {
	(void)arg;
	chRegSetThreadName("blinker");
	while (TRUE) {
		palTogglePad(GPIOD, GPIO_D12_RGB_G);
		chThdSleepMilliseconds(led_wait_time);
	}
	return -1;
}

//static WORKING_AREA(waThread_25mhz, 64);
///*! \brief 25 Mhz output clock hack
// */
//static msg_t Thread_25mhz(void *arg) {
//	(void)arg;
//
//	uint32_t i = 0;
//	while(TRUE) {
//		palClearPad(GPIOC, GPIO_C9_KSZ_25MHZ);
//
//		for(i=0; i<20; ++i) {
//			asm volatile("mov r0, r0");
//			asm volatile("mov r0, r0");
//		}
//		palSetPad(GPIOC, GPIO_C9_KSZ_25MHZ);
//		for(i=0; i<20; ++i) {
//			asm volatile("mov r0, r0");
//			asm volatile("mov r0, r0");
//		}
//	}
//	return -1;
//}

void init_rnet(void) {
	palSetPad(GPIOD, GPIO_D14_KSZ_EN);
	palClearPad(GPIOD, GPIO_D4_ETH_N_RST);
	palClearPad(GPIOD, GPIO_D14_KSZ_EN);
	chThdSleepMilliseconds(1000);  // sleep 10
	palSetPad(GPIOD, GPIO_D14_KSZ_EN);     // enable pwr
    palClearPad(GPIOD, GPIO_D13_RGB_R);
	chThdSleepMilliseconds(1000);  // sleep 10
	// Turn on clock from HSE -> PC9 function MCO2 see board file
	RCC->CFGR |=  (1<<31);      // MCO0
	RCC->CFGR &= ~(1<<29);

//	RCC->CFGR &=  ~(0b11<<22);      // MCO1
//	RCC->CFGR |=  (0b10<<21);
//
//	RCC->CFGR &= ~(0b111<<26); // clear MCO1 prescaler
//	RCC->CFGR |= (0b???<<26);  // set MCO1 prescaler
// Timer 1 channel one instead?

	palSetPad(GPIOD, GPIO_D4_ETH_N_RST);   // disable reset
	palClearPad(GPIOD, GPIO_D11_RGB_B);
}

/*
 * Application entry point.
 */
int main(void) {
	static Thread            *shelltp       = NULL;
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	led_init();
    init_rnet();

	// start the serial port
	sdStart(&SD1, NULL);

	/*
	 * Shell manager initialization.
	 */
	shellInit();

	chThdCreateStatic(waThread_blinker  , sizeof(waThread_blinker)          , NORMALPRIO    , Thread_blinker         , NULL);
	//chThdCreateStatic(waThread_25mhz    , sizeof(waThread_25mhz)            , NORMALPRIO    , Thread_25mhz           , NULL);


	/*
	 * Normal main() thread activity, in this demo it enables and disables the
	 * button EXT channel using 5 seconds intervals.
	 */

	while (true) {
		if (!shelltp )
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(shelltp)) {
			chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
			shelltp = NULL;           /* Triggers spawning of a new shell.        */
		}
		chThdSleepMilliseconds(1000);
	}

	exit(0);
}

