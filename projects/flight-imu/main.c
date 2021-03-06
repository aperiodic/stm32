/*! \file main.c
 *
 * This is specific to the Olimex stm32-e407 board modified for flight with IMU sensors.
 *
 */

/*!
 * \defgroup mainapp flight-imu Flight Application IMU
 * @{
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include "board.h"
#include "lwip/ip_addr.h"
#include "lwipopts.h"
#include "lwipthread.h"
#include "ff.h"

#include "MPU9150.h"
#include "ADIS16405.h"
#include "MPL3115A2.h"

#include "device_net.h"
#include "fc_net.h"

#include "iwdg_lld.h"
#include "usbdetail.h"
#include "extdetail.h"
#include "cmddetail.h"

#include "sensor_mpl.h"
#include "sensor_mpu.h"

#include "data_udp.h"
#include "psas_rtc.h"
#include "psas_sdclog.h"
#include "sdcdetail.h"

#include "main.h"

static const int    FLIGHT_IMU_WATCHDOG_MS = 250;

static const ShellCommand commands[] = {
    {"tree",        cmd_tree},
    {"sdchalt",     cmd_sdchalt},
    {"date",        cmd_date},
    {"mem",         cmd_mem},
    {"threads",     cmd_threads},
    {NULL,          NULL}
};

/*! configure the i2c module on stm32
 *
 */
const I2CConfig IMU_I2C_Config = {
    OPMODE_I2C,
    400000,                // i2c clock speed. Test at 400000 when r=4.7k
    FAST_DUTY_CYCLE_2,
    // STD_DUTY_CYCLE,
};

/*! \typedef mpu9150_config
 *
 * Configuration for the MPU IMU connections
 */
const mpu9150_connect mpu9150_connections = {
    GPIOF,                // i2c sda port
    0,                    // i2c_sda_pad
    GPIOF,                // i2c_scl_port
    1,                    // i2c scl pad
    GPIOF,                // interrupt port
    13,                   // interrupt pad;
};

static WORKING_AREA(waThread_blinker, 64);
/*! \brief Green LED blinker thread
*/
static msg_t Thread_blinker(void *arg) {
    (void)arg;
    chRegSetThreadName("blinker");
    while (TRUE) {
        palTogglePad(GPIOC, GPIOC_LED);
        chThdSleepMilliseconds(fs_ready ? 125 : 500);
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
        chThdSleepMilliseconds(FLIGHT_IMU_WATCHDOG_MS);
    }
    return -1;
}

int main(void) {
    static const evhandler_t evhndl_main[]  = {
        sdc_insert_handler,
        sdc_remove_handler
    };
    struct EventListener     el0, el1;

    //struct lwipthread_opts   ip_opts;

    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    psas_rtc_lld_init();

    psas_rtc_set_fc_boot_mark(&RTCD1); 

        /*
     * Activates the serial driver 6 and SDC driver 1 using default
     * configuration.
     */
    sdStart(&SD6, NULL);
    sdcStart(&SDCD1, NULL);

    /*
     * Activates the card insertion monitor.
     */
    sdc_tmr_init(&SDCD1);

    iwdg_begin();

    chThdSleepMilliseconds(300);
    usbSerialShellStart(commands);
    //adis_init();
    //adis_reset();

    mpu9150_start(&I2CD2);
    mpl3115a2_start(&I2CD2);

    /*
     * I2C2 I/O pins setup
     */
    palSetPadMode(mpu9150_connections.i2c_sda_port , mpu9150_connections.i2c_sda_pad,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_OSPEED_HIGHEST |PAL_STM32_PUDR_FLOATING );
    palSetPadMode(mpu9150_connections.i2c_scl_port, mpu9150_connections.i2c_scl_pad,
            PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST  | PAL_STM32_PUDR_FLOATING);

    palSetPad(mpu9150_connections.i2c_scl_port,  mpu9150_connections.i2c_scl_pad );

    // the mpu and the mpl sensor share the same I2C instance
    i2cStart(mpu9150_driver.i2c_instance, &IMU_I2C_Config);

    // Activates the EXT driver
    extStart(&EXTD1, &extcfg);

    // Maintenance threads
    chThdCreateStatic(waThread_blinker          , sizeof(waThread_blinker)          , NORMALPRIO    , Thread_blinker         , NULL);
    chThdCreateStatic(waThread_indwatchdog      , sizeof(waThread_indwatchdog)      , NORMALPRIO    , Thread_indwatchdog     , NULL);

    // MPL pressure sensor
    chThdCreateStatic(waThread_mpl_int_1        , sizeof(waThread_mpl_int_1)        , NORMALPRIO    , Thread_mpl_int_1       , NULL);

    // MPU 6 axis IMU sensor
    chThdCreateStatic(waThread_mpu9150_int      , sizeof(waThread_mpu9150_int)      , NORMALPRIO    , Thread_mpu9150_int     , NULL);

    /* SPI ADIS - As of: Mon 18 November 2013 11:11:42 (PST) Unavailable for testing. */
    //chThdCreateStatic(waThread_adis_dio1,    sizeof(waThread_adis_dio1),    NORMALPRIO, Thread_adis_dio1,    NULL);
    //chThdCreateStatic(waThread_adis_newdata, sizeof(waThread_adis_newdata), NORMALPRIO, Thread_adis_newdata, NULL);

    /*
     *    static       uint8_t      IMU_macAddress[6]           = IMU_A_MAC_ADDRESS;
     *    struct       ip_addr      ip, gateway, netmask;
     *    IMU_A_IP_ADDR(&ip);
     *    IMU_A_GATEWAY(&gateway);
     *    IMU_A_NETMASK(&netmask);
     *
     *    ip_opts.macaddress = IMU_macAddress;
     *    ip_opts.address    = ip.addr;
     *    ip_opts.netmask    = netmask.addr;
     *    ip_opts.gateway    = gateway.addr;
     *
     *    chThdCreateStatic(wa_lwip_thread            , sizeof(wa_lwip_thread)            , NORMALPRIO + 2, lwip_thread            , &ip_opts);
     *    chThdCreateStatic(wa_data_udp_send_thread   , sizeof(wa_data_udp_send_thread)   , NORMALPRIO    , data_udp_send_thread   , NULL);
     *    chThdCreateStatic(wa_data_udp_receive_thread, sizeof(wa_data_udp_receive_thread), NORMALPRIO    , data_udp_receive_thread, NULL);
     */

    chThdCreateStatic(wa_sdlog_thread           , sizeof(wa_sdlog_thread)           , NORMALPRIO    , sdlog_thread           , NULL);

    chThdSleepMilliseconds(10);

    chEvtRegister(&sdc_inserted_event,   &el0, 0);
    chEvtRegister(&sdc_removed_event,    &el1, 1);

    // It is possible the card is already inserted. Check now by calling insert handler directly.
    sdc_insert_handler(0);

    while (TRUE) {
       chEvtDispatch(evhndl_main, chEvtWaitOneTimeout(ALL_EVENTS, MS2ST(50)));
    }
}

//! @}
