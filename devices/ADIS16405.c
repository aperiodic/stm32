/*! \file ADIS16405.c
 *
 * API to support transactions through the SPI port to
 * the Analog Devices ADIS16405 series IMU.
 *
 */

/*! \defgroup adis16405 ADIS IMU
 *
 * @{
 */

#include <stdbool.h>

#include "ch.h"
#include "hal.h"

#include "usbdetail.h"
#include "chprintf.h"

#include "ADIS16405.h"

ADIS_Driver        adis_driver;

adis_cache         adis_cache_data;

EventSource        adis_dio1_event;
EventSource        adis_spi_cb_txdone_event;
EventSource        adis_spi_cb_newdata;
EventSource        adis_spi_cb_releasebus;

/*! \brief Reset the ADIS
 */
void adis_reset() {
	palClearPad(adis_connections.reset_port, adis_connections.reset_pad);
	chThdSleepMilliseconds(ADIS_RESET_MSECS);
	palSetPad(adis_connections.reset_port, adis_connections.reset_pad);
	chThdSleepMilliseconds(ADIS_RESET_MSECS);
}

/*! \brief Initialize ADIS driver
 *
 */
void adis_init() {
	uint8_t     i              = 0;

	//chMtxInit(&adis_driver.adis_mtx);
	//chCondInit(&adis_driver.adis_cv1);

	adis_driver.spi_instance    = &SPID1;
	adis_driver.state           = ADIS_IDLE;
	adis_driver.reg             = ADIS_PRODUCT_ID;
	adis_driver.debug_cb_count  = 0;
	adis_driver.debug_spi_count = 0;

	for(i=0; i<ADIS_MAX_TX_BUFFER; ++i) {
		adis_driver.adis_txbuf[i]        = 0;
		adis_cache_data.adis_tx_cache[i] = 0;
	}
	for(i=0; i<ADIS_MAX_RX_BUFFER; ++i) {
		adis_driver.adis_rxbuf[i]        = 0xa5;
		adis_cache_data.adis_rx_cache[i] = 0xa5;
	}
	chEvtInit(&adis_dio1_event);
	chEvtInit(&adis_spi_cb_txdone_event);
	chEvtInit(&adis_spi_cb_newdata);
	chEvtInit(&adis_spi_cb_releasebus);
}

/*!
 * t_stall is 9uS according to ADIS datasheet.
 */
void adis_tstall_delay() {
	volatile uint32_t i, j;

	/*!
	 *  This is the ADIS T_stall time.
	 *  Counting to 100 takes about 11uS.
	 *  Measured on oscilloscope.
	 *
	 *  \todo Use a hw (not virtual) timer/counter unit to generate T_stall delays
	 */
	j = 0;
	for(i=0; i<100; ++i) {
		j++;
	}
}

/*!
 *
 * @param spip
 */
void adis_release_bus(eventid_t id) {
	(void) id;
	spiReleaseBus(adis_driver.spi_instance);
}


/*! \brief Process an adis_newdata_event
 */
void adis_newdata_handler(eventid_t id) {
	(void)                id;
	uint8_t               i      = 0;
	static uint32_t       j      = 0;
	static uint32_t       xcount = 0;

	BaseSequentialStream    *chp = (BaseSequentialStream *)&SDU1;

	spiUnselect(adis_driver.spi_instance);                /* Slave Select de-assertion.       */

	/*! \todo Package for UDP transmission to fc here */
	++j;
	if(j>2000) {
		chprintf(chp, "\r\n%d rx bytes: ", xcount);
		for(i=0; i<adis_cache_data.current_rx_numbytes; ++i) {
			chprintf(chp, "0x%x ", adis_cache_data.adis_rx_cache[i]);
		}
		chprintf(chp,"\r\n");
		j=0;
		++xcount;
	}
	adis_driver.state             = ADIS_IDLE;   /* don't go to idle until data processed */
}

void adis_read_id_handler(eventid_t id) {
	(void) id;
	adis_read_id(&SPID1);
}

/*! \brief Process information from a adis_spi_cb event
 *
 */
void adis_spi_cb_txdone_handler(eventid_t id) {
	(void) id;
	switch(adis_driver.reg) {
		case ADIS_PRODUCT_ID:
			spiUnselect(adis_driver.spi_instance);
			spiReleaseBus(adis_driver.spi_instance);
			adis_tstall_delay();
			spiAcquireBus(adis_driver.spi_instance);
			spiSelect(adis_driver.spi_instance);
			spiStartReceive(adis_driver.spi_instance, adis_driver.rx_numbytes, adis_driver.adis_rxbuf);
			adis_driver.state             = ADIS_RX_PEND;
			break;
		default:
			spiUnselect(adis_driver.spi_instance);
			spiReleaseBus(adis_driver.spi_instance);
			adis_driver.state             = ADIS_IDLE;
			break;
	}
}

/*! \brief adis_spi_cb
 *
 * What happens at end of ADIS SPI transaction
 *
 * This is executed during the SPI interrupt.
 *
 * @param spip
 */
void adis_spi_cb(SPIDriver *spip) {
	chSysLockFromIsr();

	uint8_t       i                              = 0;

	chDbgCheck(adis_driver.spi_instance == spip, "adis_spi_cb driver mismatch");
	if(adis_driver.state == ADIS_TX_PEND) {
		chEvtBroadcastI(&adis_spi_cb_txdone_event);
	} else {
		for(i=0; i<adis_driver.tx_numbytes; ++i) {
			adis_cache_data.adis_tx_cache[i] = adis_driver.adis_txbuf[i];
		}
		for(i=0; i<adis_driver.rx_numbytes; ++i) {
			adis_cache_data.adis_rx_cache[i] = adis_driver.adis_rxbuf[i];
		}
		adis_cache_data.reg                  = adis_driver.reg;
		adis_cache_data.current_rx_numbytes  = adis_driver.rx_numbytes;
		adis_cache_data.current_tx_numbytes  = adis_driver.tx_numbytes;
		chEvtBroadcastI(&adis_spi_cb_newdata);
		chEvtBroadcastI(&adis_spi_cb_releasebus);
	}
	chSysUnlockFromIsr();
}

/*! \brief Create a read address
 *
 * @param s
 * @return  formatted read address for adis
 */
static uint8_t adis_create_read_addr(adis_regaddr s) {
	return (s & 0b01111111);
}

/*! \brief Create a write address
 *
 * @param s
 * @return  formatted write address for adis
 */
//static adis_regaddr adis_create_write_addr(adis_regaddr s) {
//    return (s | 0b10000000);
//}

void adis_read_id(SPIDriver *spip) {
	if(adis_driver.state == ADIS_IDLE) {
		adis_driver.spi_instance        = spip;
		adis_driver.adis_txbuf[0]       = adis_create_read_addr(ADIS_PRODUCT_ID);
		adis_driver.adis_txbuf[1]       = (adis_data) 0x0;
		adis_driver.adis_txbuf[2]       = (adis_data) 0x0;
		adis_driver.reg                 = ADIS_PRODUCT_ID;
		adis_driver.rx_numbytes         = 2;
		adis_driver.tx_numbytes         = 2;
		adis_driver.debug_cb_count      = 0;

		spiAcquireBus(spip);                /* Acquire ownership of the bus.    */
		spiSelect(spip);                    /* Slave Select assertion.          */
		spiStartSend(spip, adis_driver.tx_numbytes, adis_driver.adis_txbuf);
		adis_driver.state             = ADIS_TX_PEND;
	}
}

// @}
