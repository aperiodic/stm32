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

#if !defined(ADIS_DEBUG) || defined(__DOXYGEN__)
#define 	ADIS_DEBUG                   0
#endif

ADIS_Driver        adis_driver;

adis_cache         adis_cache_data;
adis_burst_data    burst_data;

EventSource        adis_dio1_event;
EventSource        adis_spi_cb_txdone_event;
EventSource        adis_spi_cb_newdata;
EventSource        adis_spi_cb_releasebus;

#if ADIS_DEBUG || defined(__DOXYGEN__)
	/*! \brief Convert an ADIS 14 bit accel. value to micro-g
	 *
	 * @param decimal
	 * @param accel reading
	 * @return   TRUE if less than zero, FALSE if greater or equal to zero
	 */
	static bool adis_accel2ug(uint32_t* decimal, uint16_t* twos_num) {
		uint16_t ones_comp;
		bool     isnegative = false;

		//! bit 13 is 14-bit two's complement sign bit
		isnegative   = (((uint16_t)(1<<13) & *twos_num) != 0) ? true : false;

		if(isnegative) {
			ones_comp    = ~(*twos_num & (uint16_t)0x3fff) & 0x3fff;
			*decimal     = (ones_comp) + 1;
		} else {
			*decimal     = *twos_num;
		}
		*decimal     *= 3330;
		return isnegative;
	}
#endif

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

static void adis_read_id(SPIDriver *spip) {
	if(adis_driver.state == ADIS_IDLE) {
		adis_driver.spi_instance        = spip;
		adis_driver.adis_txbuf[0]       = adis_create_read_addr(ADIS_PRODUCT_ID);
		adis_driver.adis_txbuf[1]       = (adis_reg_data) 0x0;
		adis_driver.adis_txbuf[2]       = (adis_reg_data) 0x0;
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

static void adis_burst_read(SPIDriver *spip) {
	if(adis_driver.state == ADIS_IDLE) {
		adis_driver.spi_instance        = spip;
		adis_driver.adis_txbuf[0]       = adis_create_read_addr(ADIS_GLOB_CMD);
		adis_driver.adis_txbuf[1]       = (adis_reg_data) 0x0;
		adis_driver.reg                 = ADIS_GLOB_CMD;
		adis_driver.rx_numbytes         = (ADIS_NUM_BURSTREAD_REGS *2);
		adis_driver.tx_numbytes         = 2;
		adis_driver.debug_cb_count      = 0;

		spiAcquireBus(spip);                /* Acquire ownership of the bus.    */
		spiSelect(spip);                    /* Slave Select assertion.          */
		spiStartSend(spip, adis_driver.tx_numbytes, adis_driver.adis_txbuf);
		adis_driver.state             = ADIS_TX_PEND;
	}
}
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
	 *  \todo Use a HW (not virtual) timer/counter unit to generate T_stall delays
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

/*! \brief Process an adis_newdata_event
 */
void adis_newdata_handler(eventid_t id) {
	(void)                id;

	spiUnselect(adis_driver.spi_instance);                /* Slave Select de-assertion.       */

	if(adis_driver.reg == ADIS_GLOB_CMD) {
		burst_data.adis_supply_out   = (((adis_cache_data.adis_rx_cache[0]  << 8) | adis_cache_data.adis_rx_cache[1] ) & ADIS_14_BIT_MASK );
		burst_data.adis_xgyro_out    = (((adis_cache_data.adis_rx_cache[2]  << 8) | adis_cache_data.adis_rx_cache[3] ) & ADIS_14_BIT_MASK );
		burst_data.adis_ygyro_out    = (((adis_cache_data.adis_rx_cache[4]  << 8) | adis_cache_data.adis_rx_cache[5] ) & ADIS_14_BIT_MASK );
		burst_data.adis_zgyro_out    = (((adis_cache_data.adis_rx_cache[6]  << 8) | adis_cache_data.adis_rx_cache[7] ) & ADIS_14_BIT_MASK );
		burst_data.adis_xaccl_out    = (((adis_cache_data.adis_rx_cache[8]  << 8) | adis_cache_data.adis_rx_cache[9] ) & ADIS_14_BIT_MASK );
		burst_data.adis_yaccl_out    = (((adis_cache_data.adis_rx_cache[10] << 8) | adis_cache_data.adis_rx_cache[11]) & ADIS_14_BIT_MASK );
		burst_data.adis_zaccl_out    = (((adis_cache_data.adis_rx_cache[12] << 8) | adis_cache_data.adis_rx_cache[13]) & ADIS_14_BIT_MASK );
		burst_data.adis_xmagn_out    = (((adis_cache_data.adis_rx_cache[14] << 8) | adis_cache_data.adis_rx_cache[15]) & ADIS_14_BIT_MASK );
		burst_data.adis_ymagn_out    = (((adis_cache_data.adis_rx_cache[16] << 8) | adis_cache_data.adis_rx_cache[17]) & ADIS_14_BIT_MASK );
		burst_data.adis_zmagn_out    = (((adis_cache_data.adis_rx_cache[18] << 8) | adis_cache_data.adis_rx_cache[19]) & ADIS_14_BIT_MASK );
		burst_data.adis_temp_out     = (((adis_cache_data.adis_rx_cache[20] << 8) | adis_cache_data.adis_rx_cache[21]) & ADIS_12_BIT_MASK );
		burst_data.adis_aux_adc      = (((adis_cache_data.adis_rx_cache[22] << 8) | adis_cache_data.adis_rx_cache[23]) & ADIS_12_BIT_MASK );
	}

	/*! \todo Package for UDP transmission to fc here */

#if ADIS_DEBUG
	bool                  negative = false;
	uint32_t              result_ug = 0;
	static uint32_t       j        = 0;
	static uint32_t       xcount   = 0;

	BaseSequentialStream    *chp = (BaseSequentialStream *)&SDU1;

	++j;
	if(j>4000) {
    	if(adis_driver.reg == ADIS_GLOB_CMD) {
    		// chprintf(chp, "%d: supply: %x %d uV\r\n", xcount, burst_data.adis_supply_out, ( burst_data.adis_supply_out * 2418));
    		//negative = twos2dec(&burst_data.adis_xaccl_out);

    		negative   = adis_accel2ug(&result_ug, &burst_data.adis_zaccl_out);
    		chprintf(chp, "%d: z: 0x%x  %s%d ug\r\n", xcount, burst_data.adis_zaccl_out, (negative) ? "-" : "", result_ug);
    		negative   = adis_accel2ug(&result_ug, &burst_data.adis_xaccl_out);
    		chprintf(chp, "%d: x: 0x%x  %s%d ug\r\n", xcount, burst_data.adis_xaccl_out, (negative) ? "-" : "", result_ug);
    		negative   = adis_accel2ug(&result_ug, &burst_data.adis_yaccl_out);
    		chprintf(chp, "%d: y: 0x%x  %s%d ug\r\n\r\n", xcount, burst_data.adis_yaccl_out, (negative) ? "-" : "", result_ug);

    	} else if (adis_driver.reg == ADIS_PRODUCT_ID) {
    		chprintf(chp, "%d: Prod id: %x\r\n", xcount, ((adis_cache_data.adis_rx_cache[0]<< 8)|(adis_cache_data.adis_rx_cache[1])) );
    	}

		//		for(i=0; i<adis_cache_data.current_rx_numbytes; ++i) {
//		    chprintf(chp, "%x ", adis_cache_data.adis_rx_cache[i]);
//		}
//		chprintf(chp,"\r\n");
		j=0;
		++xcount;
	}
#endif

	adis_driver.state             = ADIS_IDLE;   /* don't go to idle until data processed */
}

void adis_read_id_handler(eventid_t id) {
	(void) id;
	adis_read_id(&SPID1);
}

void adis_burst_read_handler(eventid_t id) {
	(void) id;
	adis_burst_read(&SPID1);
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
		case ADIS_GLOB_CMD:
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

//! @}
