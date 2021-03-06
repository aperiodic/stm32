/*! \file main.c
 *
 * Development for ADIS IMU on ChibiOS
 *
 * This implementation is specific to the Olimex stm32-e407 board.
 */


/*!
 * \defgroup mainapp Application
 * @{
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "shell.h"

#include "iwdg_lld.h"
#include "usbdetail.h"
#include "extdetail.h"
#include "cmddetail.h"

#include "ADIS16405.h"

#include "main.h"

static const ShellCommand commands[] = {
		{"mem", cmd_mem},
		{"threads", cmd_threads},
		{NULL, NULL}
};

/*! \brief ADIS SPI configuration
 *
 * 656250Hz, CPHA=1, CPOL=1, MSb first.
 *
 * For burst mode ADIS SPI is limited to 1Mhz.
 */
#if 1
const SPIConfig adis_spicfg = {
		adis_spi_cb,
		GPIOA,
		4,
		SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR_2 | SPI_CR1_BR_1
};
#else
const SPIConfig adis_spicfg = {
		NULL,
		GPIOA,
		4,
		SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR_2 | SPI_CR1_BR_1
};

#endif




static WORKING_AREA(waThread_blinker, 64);
/*! \brief Green LED blinker thread
 */
static msg_t Thread_blinker(void *arg) {
	(void)arg;
	chRegSetThreadName("blinker");
	while (TRUE) {
		palTogglePad(GPIOC, GPIOC_LED);
		chThdSleepMilliseconds(500);
	}
	return -1;
}

static WORKING_AREA(waThread_adis_newdata, 256);
/*! \brief ADIS Newdata Thread
 */
static msg_t Thread_adis_newdata(void *arg) {
	(void)arg;
	chRegSetThreadName("adis_newdata");

	static const evhandler_t evhndl_newdata[]       = {
			adis_newdata_handler
	};
	struct EventListener     evl_spi_cb2;

	chEvtRegister(&adis_spi_cb_data_captured, &evl_spi_cb2, 0);

	while (TRUE) {
		chEvtDispatch(evhndl_newdata, chEvtWaitOneTimeout((eventmask_t)1, US2ST(50)));
	}
	return -1;
}


static WORKING_AREA(waThread_adis_dio1, 128);
/*! \brief ADIS DIO1 thread
 *
 * For burst mode transactions t_readrate is 1uS
 *
 */
static msg_t Thread_adis_dio1(void *arg) {
	(void)arg;
	static const evhandler_t evhndl_dio1[]       = {
			adis_burst_read_handler,
			//adis_read_id_handler,
			adis_spi_cb_txdone_handler,
			adis_release_bus
	};
	struct EventListener     evl_dio;
	struct EventListener     evl_spi_ev;
	struct EventListener     evl_spi_release;

	chRegSetThreadName("adis_dio");

	chEvtRegister(&adis_dio1_event,           &evl_dio,         0);
	chEvtRegister(&adis_spi_cb_txdone_event,  &evl_spi_ev,      1);
	chEvtRegister(&adis_spi_cb_releasebus,    &evl_spi_release, 2);

	while (TRUE) {
		chEvtDispatch(evhndl_dio1, chEvtWaitOneTimeout((EVENT_MASK(2)|EVENT_MASK(1)|EVENT_MASK(0)), US2ST(50)));
	}
	return -1;
}

static WORKING_AREA(waThread_indwatchdog, 64);
/*! \brief  Watchdog thread
 */
static msg_t Thread_indwatchdog(void *arg) {
	(void)arg;

	chRegSetThreadName("iwatchdog");
	while (TRUE) {
		iwdg_lld_reload();
		chThdSleepMilliseconds(250);
	}
	return -1;
}

int main(void) {
	static const evhandler_t evhndl_main[]       = {
			extdetail_WKUP_button_handler
	};
	struct EventListener     el0;

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();
	usbSerialShellStart(commands);
	extdetail_init();

	palSetPad(GPIOC, GPIOC_LED);

	spiStart(&SPID1, &adis_spicfg);       /* Set transfer parameters.  */

	chThdSleepMilliseconds(300);

	adis_init();
	adis_reset();

	/*! Activates the EXT driver 1. */
	extStart(&EXTD1, &extcfg);

	chThdCreateStatic(waThread_blinker,      sizeof(waThread_blinker),      NORMALPRIO, Thread_blinker,      NULL);
	chThdCreateStatic(waThread_adis_dio1,    sizeof(waThread_adis_dio1),    NORMALPRIO, Thread_adis_dio1,    NULL);
	chThdCreateStatic(waThread_adis_newdata, sizeof(waThread_adis_newdata), NORMALPRIO, Thread_adis_newdata, NULL);
	chThdCreateStatic(waThread_indwatchdog,  sizeof(waThread_indwatchdog),  NORMALPRIO, Thread_indwatchdog,  NULL);

	chEvtRegister(&extdetail_wkup_event, &el0, 0);
	while (TRUE) {
		chEvtDispatch(evhndl_main, chEvtWaitOneTimeout((eventmask_t)1, MS2ST(500)));
	}
}


//! @}
