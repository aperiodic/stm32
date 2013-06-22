/* experimental control of pwm vars over udp */

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

#include <lwip/ip_addr.h>

#include "data_udp.h"
#include "lwipopts.h"
#include "lwipthread.h"
#include "pwm.h"
#include "pwm_config.h"

#include "main.h"


/*shell commands*/
static const ShellCommand commands[] = {
        {"mem", cmd_mem},
        {"threads", cmd_threads},
        {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
        (BaseSequentialStream *)&SDU_PSAS,
        commands
};

static WORKING_AREA(waThread_pwmtest, 512);
static msg_t Thread_pwmtest(void *arg) {
    (void)arg;
    chRegSetThreadName("pwmtest");

    //    uint32_t i = 0;
    // BaseSequentialStream *chp =  (BaseSequentialStream *)&SDU_PSAS;
    //
    chThdSleepMilliseconds(2000);
    while(1) {
        pwmcnt_t pulse = 0;

   //     chprintf(chp, ".");
//        pwm_set_pulse_width_ticks(pwm_us_to_ticks(333));
//        chThdSleepMilliseconds(200);
//
//        pwm_set_pulse_width_ticks(pwm_us_to_ticks(1050));
//        chThdSleepMilliseconds(200);
//
//        pwm_set_pulse_width_ticks(pwm_us_to_ticks(1900));
//        chThdSleepMilliseconds(200);
//
//        pwmDisableChannel(&PWMD4, 3);
        chThdSleepMilliseconds(250);
        for (pulse = 333; pulse <= 1900; pulse += 50) {
            pwm_set_pulse_width_ticks(pwm_us_to_ticks(pulse));
            chThdSleepMilliseconds(50);
        }
    }
    return -1;
}


/*blinker thread vars and functions*/
static WORKING_AREA(waThread_blinker, 128);
static msg_t Thread_blinker(void *arg) {
    (void)arg;
    chRegSetThreadName("blinker");
    while (TRUE) {
        palTogglePad(GPIOC, GPIOC_LED);
        chThdSleepMilliseconds(500);
    }
    return -1;
}

/*watchdog timer thread vars and functions*/
static WORKING_AREA(waThread_indwatchdog, 128);
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
    static Thread            *shelltp       = NULL;
    /*this is needed in order for the usb over serial to work*/
    static const evhandler_t evhndl_main[]       = {
            extdetail_WKUP_button_handler
    };
    struct EventListener     el0;
    /*lwip options data structure*/
    struct lwipthread_opts   ip_opts;

    /*initialize HAL*/
    halInit();
    chSysInit();

    /*need to become familiar with this still*/
    extdetail_init();
    palSetPad(GPIOC, GPIOC_LED);

    /*initialize serial over usb driver*/
    sduObjectInit(&SDU_PSAS);
    sduStart(&SDU_PSAS, &serusbcfg);

    /*activate usb driver*/
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    /*start shell*/
    shellInit();

    /*start watchdog*/
    iwdg_begin();

    /*start serial driver 6 and sdc driver 1 using default config*/
    sdStart(&SD6, NULL);

    /*sleep 300 ms*/
    chThdSleepMilliseconds(300);

    /*start ext1 driver*/
    extStart(&EXTD1, &extcfg);

    /*create blinker thread*/
    chThdCreateStatic(waThread_blinker,      sizeof(waThread_blinker),      NORMALPRIO, Thread_blinker,      NULL);

    /*start pwm*/
    pwm_start();

    /*configure ethernet, ip stuff*/
    static       uint8_t      macAddress[6]    =     {0xC2, 0xAF, 0x51, 0x03, 0xCF, 0x46};
    struct ip_addr ip, gateway, netmask;
    IP4_ADDR(&ip,      10, 0, 0, 2);
    IP4_ADDR(&gateway, 10, 0, 0, 254);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    ip_opts.address    = ip.addr;
    ip_opts.netmask    = netmask.addr;
    ip_opts.gateway    = gateway.addr;
    ip_opts.macaddress = macAddress;

    /*create watchdog thread*/
    chThdCreateStatic(waThread_indwatchdog,  sizeof(waThread_indwatchdog),  NORMALPRIO, Thread_indwatchdog,  NULL);

    /*create lwip thread*/
    //chThdCreateStatic(wa_lwip_thread, LWIP_THREAD_STACK_SIZE, NORMALPRIO + 2, lwip_thread, &ip_opts);

    /*create pwmtest thread*/
    chThdCreateStatic(waThread_pwmtest, sizeof(waThread_pwmtest), NORMALPRIO, Thread_pwmtest, NULL);

        /*create udp send thread */
    /*chThdCreateStatic(wa_data_udp_send_thread, sizeof(wa_data_udp_send_thread), NORMALPRIO, data_udp_send_thread, NULL);*/

    /*create udp receive thread*/
   // chThdCreateStatic(wa_data_udp_receive_thread, sizeof(wa_data_udp_receive_thread), NORMALPRIO, data_udp_receive_thread, NULL);
    chEvtRegister(&extdetail_wkup_event, &el0, 0);

    while (TRUE) {
        if (!shelltp && (SDU_PSAS.config->usbp->state == USB_ACTIVE))
            shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
        else if (chThdTerminated(shelltp)) {
            chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
            shelltp = NULL;           /* Triggers spawning of a new shell.        */
        }
        /*this is needed for the usb over serial to work */
        chEvtDispatch(evhndl_main, chEvtWaitOneTimeout((eventmask_t)1, MS2ST(500)));
    }
}
