/* \file psas_rtc.h
 *
 */

#ifndef PSAS_RTC_H_
#define PSAS_RTC_H_

/*!
 * \addtogroup psas_rtc
 * @{
 */

#include <stdbool.h>
#include <time.h>

#include "ch.h"
#include "hal.h"
#include "rtc.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PSAS_RTC_COE_DEBUG
#define     PSAS_RTC_COE_DEBUG      0          // Enable the calibration output on PC13
#endif

#define PSAS_RTC_NS_BYTES 6

    typedef struct {
        bool    initialized;
        time_t  fc_boot_time_mark;
    } psas_rtc_state;

    struct psas_timespec {
        // this array contains a 48-bit number, byte-by-byte, in LSB order
        uint8_t ns[PSAS_RTC_NS_BYTES];
    } __attribute__((packed));
    typedef struct psas_timespec psas_timespec;

    extern  psas_rtc_state     psas_rtc_s;
    extern  bool               fs_ready;

    void   psas_rtc_lld_init(void) ;

    void   psas_rtc_set_fc_boot_mark(RTCDriver* rtcp) ;
    void   psas_rtc_to_psas_ts(psas_timespec* ts, RTCTime* rtc) ;
    void   psas_ts_to_psas_rtc(RTCTime* rtc, psas_timespec* ts) ;

    void   psas_stm32_rtc_bcd2tm(struct tm *timp, RTCTime *timespec) ;
    time_t psas_rtc_dr_tr_to_unixtime(RTCTime* timespec) ;
    void   psas_rtc_lld_get_time( RTCDriver *rtcp, RTCTime *timespec) ;
    int    psas_rtc_get_unix_time( RTCDriver *rtcp, RTCTime *timespec) ;
    void   psas_rtc_lld_set_time( RTCDriver *rtcp, RTCTime *timespec) ;


#ifdef __cplusplus
}
#endif

#endif

//! @}

