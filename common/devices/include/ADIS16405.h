/*! \file ADIS16405.h
 *
 * Intended for use with ChibiOS RT
 */

/*! \addtogroup adis16405
 * @{
 */

#ifndef _ADIS16405_H
#define _ADIS16405_H


#ifdef __cplusplus
extern "C" {
#endif

#include "ch.h"
#include "hal.h"

#define     ADIS_14_BIT_MASK                  0x3fff
#define     ADIS_12_BIT_MASK                  0x0fff

#define     ADIS_RESET_MSECS                  500
#define     ADIS_TSTALL_US                    10
#define     ADIS_TSTALL_US_LOWPOWER           75

#define     ADIS_MAX_RX_BUFFER                64
#define     ADIS_MAX_TX_BUFFER                16

#define     ADIS_NUM_BURSTREAD_REGS           12

#if !defined(ADIS_DEBUG) || defined(__DOXYGEN__)
#define 	ADIS_DEBUG                   0
#endif

/*! \typedef adis_regaddr
 *
 * ADIS Register addresses
 */
typedef enum {
	// Name         address         default    function
	ADIS_FLASH_CNT    = 0x00,        //  N/A     Flash memory write count
	ADIS_SUPPLY_OUT   = 0x02,        //  N/A     Power supply measurement
	ADIS_XGYRO_OUT    = 0x04,        //  N/A     X-axis gyroscope output
	ADIS_YGYRO_OUT    = 0x06,        //  N/A     Y-axis gyroscope output
	ADIS_ZGYRO_OUT    = 0x08,        //  N/A     Z-axis gyroscope output
	ADIS_XACCL_OUT    = 0x0A,        //  N/A     X-axis accelerometer output
	ADIS_YACCL_OUT    = 0x0C,        //  N/A     Y-axis accelerometer output
	ADIS_ZACCL_OUT    = 0x0E,        //  N/A     Z-axis accelerometer output
	ADIS_XMAGN_OUT    = 0x10,        //  N/A     X-axis magnetometer measurement
	ADIS_YMAGN_OUT    = 0x12,        //  N/A     Y-axis magnetometer measurement
	ADIS_ZMAGN_OUT    = 0x14,        //  N/A     Z-axis magnetometer measurement
	ADIS_TEMP_OUT     = 0x16,        //  N/A     Temperature output
	ADIS_AUX_ADC      = 0x18,        //  N/A     Auxiliary ADC measurement
	ADIS_XGYRO_OFF    = 0x1A,        //  0x0000  X-axis gyroscope bias offset factor
	ADIS_YGYRO_OFF    = 0x1C,        //  0x0000  Y-axis gyroscope bias offset factor
	ADIS_ZGYRO_OFF    = 0x1E,        //  0x0000  Z-axis gyroscope bias offset factor
	ADIS_XACCL_OFF    = 0x20,        //  0x0000  X-axis acceleration bias offset factor
	ADIS_YACCL_OFF    = 0x22,        //  0x0000  Y-axis acceleration bias offset factor
	ADIS_ZACCL_OFF    = 0x24,        //  0x0000  Z-axis acceleration bias offset factor
	ADIS_XMAGN_HIF    = 0x26,        //  0x0000  X-axis magnetometer, hard-iron factor
	ADIS_YMAGN_HIF    = 0x28,        //  0x0000  Y-axis magnetometer, hard-iron factor
	ADIS_ZMAGN_HIF    = 0x2A,        //  0x0000  Z-axis magnetometer, hard-iron factor
	ADIS_XMAGN_SIF    = 0x2C,        //  0x0800  X-axis magnetometer, soft-iron factor
	ADIS_YMAGN_SIF    = 0x2E,        //  0x0800  Y-axis magnetometer, soft-iron factor
	ADIS_ZMAGN_SIF    = 0x30,        //  0x0800  Z-axis magnetometer, soft-iron factor
	ADIS_GPIO_CTRL    = 0x32,        //  0x0000  Auxiliary digital input/output control
	ADIS_MSC_CTRL     = 0x34,        //  0x0006  Miscellaneous control
	ADIS_SMPL_PRD     = 0x36,        //  0x0001  Internal sample period (rate) control
	ADIS_SENS_AVG     = 0x38,        //  0x0402  Dynamic range and digital filter control
	ADIS_SLP_CNT      = 0x3A,        //  0x0000  Sleep mode control
	ADIS_DIAG_STAT    = 0x3C,        //  0x0000  System status
	ADIS_GLOB_CMD     = 0x3E,        //  0x0000  System command
	ADIS_ALM_MAG1     = 0x40,        //  0x0000  Alarm 1 amplitude threshold
	ADIS_ALM_MAG2     = 0x42,        //  0x0000  Alarm spi_master_xact_data* caller, spi_master_xact_data* spi_xact, void* data2 amplitude threshold
	ADIS_ALM_SMPL1    = 0x44,        //  0x0000  Alarm 1 sample size
	ADIS_ALM_SMPL2    = 0x46,        //  0x0000  Alarm 2 sample size
	ADIS_ALM_CTRL     = 0x48,        //  0x0000  Alarm control
	ADIS_AUX_DAC      = 0x4A,        //  0x0000  Auxiliary DAC data
	//          =0x4C,to 0x55        //        Reserved
	ADIS_PRODUCT_ID   = 0x56         //          Product identifier

} adis_regaddr;

//typedef struct{
//    memory_test = 0,
//    auto_self_test = 0,
//    manual_test_neg = 0,
//    manual_test_pos = 0,
//    gyro_lin_accel_bias_compensation = 0,
//    lin_accel_origin_alignment = 0,
//    data_ready_enable = 0,
//    data_ready_polarity = 0,
//    data_ready_lineselect = 0
//} adis_MSC_CTRL;

/*! \typedef
 * Burst data collection. This establishes what we consider the right datatype
 * for the registers because trying to work with 12 or 14 bit twos complement
 * that doesn't sign extend to 16 bits is nuts and we consider it a bug.
 */
typedef struct ADIS16405_burst_data {
	uint16_t supply_out;//  Power supply measurement
	int16_t xgyro_out;  //  X-axis gyroscope output
	int16_t ygyro_out;  //  Y-axis gyroscope output
	int16_t zgyro_out;  //  Z-axis gyroscope output
	int16_t xaccl_out;  //  X-axis accelerometer output
	int16_t yaccl_out;  //  Y-axis accelerometer output
	int16_t zaccl_out;  //  Z-axis accelerometer output
	int16_t xmagn_out;  //  X-axis magnetometer measurement
	int16_t ymagn_out;  //  Y-axis magnetometer measurement
	int16_t zmagn_out;  //  Z-axis magnetometer measurement
	int16_t temp_out;   //  Temperature output
	uint16_t aux_adc;   //  Auxiliary ADC measurement
} __attribute__((packed)) ADIS16405_burst_data;

/*! \typedef adis_config
 *
 * Configuration for the ADIS connections EXCEPT SPI
 */

struct pin {
    ioportid_t port;
    uint16_t pad;
};

typedef struct {
	struct pin spi_sck;  /*! \brief The SPI SCK wire */
	struct pin spi_miso; /*! \brief The SPI MISO wire */
	struct pin spi_mosi; /*! \brief The SPI MOSI wire */
	struct pin spi_cs;   /*! \brief The SPI CS wire */
	struct pin reset;    /*! \brief The reset line */
	struct pin dio1;     /*! \brief The DIO1 port */
	struct pin dio2;     /*! \brief The DIO2 port */
	struct pin dio3;     /*! \brief The DIO3 port */
	struct pin dio4;     /*! \brief The DIO4 port */
} adis_pins;

extern const adis_pins adis_olimex_e407;

extern EventSource adis_data_ready;

void adis_init(const adis_pins * pins);
uint16_t adis_get(adis_regaddr addr);
void adis_set(adis_regaddr addr, uint16_t value);
void adis_get_burst(ADIS16405_burst_data * data);

void 	     adis_tstall_delay(void);

void         adis_reset(void);

//void         adis_spi_cb(SPIDriver *spip) ;
//void         adis_newdata_handler(eventid_t id) ;
//void         adis_read_id_handler(eventid_t id) ;
//void         adis_read_dC_handler(eventid_t id) ;
//void         adis_burst_read_handler(eventid_t id) ;
//void         adis_spi_cb_txdone_handler(eventid_t id) ;

/*!
 * @}
 */

#ifdef __cplusplus
}
#endif


#endif


