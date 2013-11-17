/*! \file psas_sdclog.c
 *  SD Card Logging
 */

/*!
 * \defgroup psas_sdclog PSAS SDCard Logging Utilities
 * @{
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "chrtclib.h"
#include "psas_rtc.h"

#include "MPU9150.h"

#include "crc_16_reflect.h"
#include "ff.h"

#include "psas_sdclog.h"

/*
 * Sat 16 November 2013 11:24:15 (PST)
 * Dan's experiment:
 * ....I tested for data loss by running ten trials of removing the card while
 * logging MPU results every 6 ms, and comparing the index of the last
 * non-truncated message to the debug output coming in via the serial USB
 * channel. In every trial, the last message that was reported to be
 * successfully logged from the debug output was in fact present on the card,
 * though the next message (which the debug output indicated was not logged
 * succesfully) was usually truncated. I deemed this to be acceptable data loss
 * given the ~5x speedup this approach has over the previous approach calling
 * f_sync 3 times per message....
 */
#define         SDC_F_SYNC_COUNT                     ((int8_t) 40)

#define         DEBUG_SDC

#ifdef DEBUG_SDC
    #include "usbdetail.h"
    BaseSequentialStream    *sdcchp   =  (BaseSequentialStream *)&SDU_PSAS;
    #define SDCDEBUG(format, ...) chprintf( sdcchp, format, ##__VA_ARGS__ )
#else
    #define SDCDEBUG(...) do{ } while ( false )
#endif

// static declarations
static const    unsigned        sdc_polling_interval             = 10;
static const    unsigned        sdc_polling_delay                = 10;

static          sdc_eod_marker  sdc_eod;

static          unsigned        sdc_debounce_count               = 0;

static          VirtualTimer    sdc_tmr;

// extern declarations
bool                            fs_ready                         = FALSE;

FATFS                           SDC_FS;

DWORD                           sdc_fp_index                     = 0;
DWORD                           sdc_fp_index_old                 = 0;

EventSource                     sdc_inserted_event;
EventSource                     sdc_removed_event;

/*!
 * @brief           Insertion monitor timer callback function.
 *
 * @param[in] p     pointer to the @p BaseBlockDevice object
 *
 * @notapi
 */
static void sdc_tmrfunc(void *p) {
    BaseBlockDevice *bbdp = p;

    chSysLockFromIsr();
    if (sdc_debounce_count > 0) {
        if (blkIsInserted(bbdp)) {
            if (--sdc_debounce_count == 0) {
                chEvtBroadcastI(&sdc_inserted_event);
            }
        }
        else
            sdc_debounce_count = sdc_polling_interval;
    }
    else {
        if (!blkIsInserted(bbdp)) {
            sdc_debounce_count = sdc_polling_interval;
            chEvtBroadcastI(&sdc_removed_event);
        }
    }
    chVTSetI(&sdc_tmr, MS2ST(sdc_polling_delay), sdc_tmrfunc, bbdp);
    chSysUnlockFromIsr();
}

/*! \brief Init end of data fiducials.
 */
void sdc_init_eod (uint8_t marker_byte) {
    unsigned int i;

    sdc_eod.marker = (marker_byte << 8) | marker_byte ;
    for(i=0; i<SDC_MARKER_BYTES ; ++i) {
        sdc_eod.sdc_eodmarks[i] = marker_byte ;
    }
}

/**
 * @brief   Polling monitor start.
 *
 * @param[in] p         pointer to an object implementing @p BaseBlockDevice
 *
 * @notapi
 */
void sdc_tmr_init(void *p) {

    chEvtInit(&sdc_inserted_event);
    chEvtInit(&sdc_removed_event);
    chSysLock();
    sdc_debounce_count = sdc_polling_interval;
    chVTSetI(&sdc_tmr, MS2ST(sdc_polling_delay), sdc_tmrfunc, p);
    chSysUnlock();
}

/*!
 * \brief Card insertion event.
 *
 * Also use at power up to see if card was or remained
 * inserted while power unavailable.
 */
void sdc_insert_handler(eventid_t id) {
    FRESULT err;

    (void)id;

    /*! \todo generate a mailbox event here */

    /*! \todo test event message system */

    /*!
     * On insertion SDC initialization and FS mount.
     */
    if (sdcConnect(&SDCD1)) {
        if(sdcConnect(&SDCD1)) {   // why does it often fail the first time but not the second?
            return;
        }
    }

    err = f_mount(0, &SDC_FS);
    if (err != FR_OK) {
        err = f_mount(0, &SDC_FS);
        if (err != FR_OK) {
            sdcDisconnect(&SDCD1);
            return;
        }
    }
    sdc_reset_fp_index();
    fs_ready = TRUE;
}

/*!
 *  \brief Card removal event.
 */
void sdc_remove_handler(eventid_t id) {
    (void)id;
    bool_t ret;

    /*! \todo generate a mailbox event here */

    /*! \todo test event message system */

    ret = sdcDisconnect(&SDCD1);
    if(ret) {
        SDCDEBUG("sdcDiscon fail\r\n");   // this happens a lot!
        ret = sdcDisconnect(&SDCD1);
        if(ret) {
            SDCDEBUG("sdcDiscon fail2\r\n");
        }
    }
    sdc_reset_fp_index();
    fs_ready = FALSE;
}

/*! \brief used in cmd_tree function
 *
 * Find files on sd card.
 */
FRESULT sdc_scan_files(BaseSequentialStream *chp, char *path) {
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;

#if _USE_LFN
    fno.lfname = 0;
    fno.lfsize = 0;
#endif
    res = f_opendir(&dir, path);
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
                break;
            if (fno.fname[0] == '.')
                continue;
            fn = fno.fname;
            if (fno.fattrib & AM_DIR) {
                path[i++] = '/';
                strcpy(&path[i], fn);
                res = sdc_scan_files(chp, path);
                if (res != FR_OK)
                    break;
                path[--i] = 0;
            }
            else {
                SDCDEBUG("%s/%s\r\n", path, fn);
            }
        }
    }
    return res;
}

/*! \brief Set the current filepointer (fp) index
 *
 *  sdcp_fp_index tracks what byte we are pointing to
 *  from start of file (byte0)
 */
SDC_ERRORCode sdc_set_fp_index(FIL* DATAFil, DWORD ofs) {
    FRESULT f_ret;
    f_ret = f_lseek(DATAFil, ofs);
    if(f_ret != FR_OK) {
        f_ret = f_lseek(DATAFil, ofs);
        if(f_ret != FR_OK) {
            SDCDEBUG("f_lseek error: %d\r\n", f_ret) ;
            return SDC_FSEEK_ERROR;
        }
    }
    sdc_fp_index      = ofs;
    return SDC_OK;
}

/*! \brief Store a checksum and end of data fiducial marks
 *
     A line of data would look like this:
        [GENERIC_message][chksum]
     We want a line at the end-of-data(eod) to look like this:
        [GENERIC_message][chksum][0xa5a5]

     What happens if power fails partway through a write? There will
     be NO eod marker. So write a length of eod markers every write past the
     length of the next write.
        [GENERIC_message][checksum][0xa5a5..(len(GENERIC_message+checksum+some extra)..0xa5a5]
                                   ^
                                   Seek fp-index (sdc_set_fp_index) to here.

     Now, in the event of failure during the next write, we have already placed
     eod markers. A partial filled line will have an incorrect checksum.

     If we pre-allocate the file on the sd card, then the file will already exist
     in the FAT and shouldn't be damaged due to a partial file size reallocation.

 */
SDC_ERRORCode sdc_write_checksum(FIL* DATAFil, crc_t* crcd, uint32_t* bw) {
    SDC_ERRORCode      sdc_ret;

    if((DATAFil == NULL) || (crcd==NULL) || (bw == NULL)) {
        return SDC_NULL_PARAMETER_ERROR;
    }
    sdc_ret = sdc_f_write(DATAFil, (void *)(crcd), sizeof(crc_t), (unsigned int*) bw);
    if (sdc_ret)  {
        return sdc_ret;
    }

    sdc_ret = sdc_f_write(DATAFil, (void *)(&sdc_eod.sdc_eodmarks), sizeof(sdc_eod_marker), (unsigned int*) bw);
    if (sdc_ret)  {
        return sdc_ret;
    }

     SDCDEBUG("%s:\t%d\tgm:\t%d\r\n",__func__,  sizeof(sdc_eod_marker), sizeof(GENERIC_message)) ;
    // be kind, rewind.
    sdc_ret = sdc_set_fp_index(DATAFil, sdc_fp_index - sizeof(sdc_eod_marker)) ;
    if (sdc_ret)  {
        return sdc_ret;
    }


   return FR_OK;
}

/*! \brief Store GENERIC_message to the SD card
*/
SDC_ERRORCode sdc_write_log_message(FIL* DATAFil, GENERIC_message* d, uint32_t* bw) {
    SDC_ERRORCode rc;

    if((DATAFil == NULL) || (d==NULL) || (bw == NULL)) {
        return SDC_NULL_PARAMETER_ERROR;
    }

    rc = sdc_f_write(DATAFil, (void *)(d), sizeof(GENERIC_message), (unsigned int*) bw);
    if (rc)  {
        return SDC_FWRITE_ERROR;
    }
    return SDC_OK;
}

/*
 * Wrapper around fatfs f_write function.
 *  Try to write twice before failing
 *  Track the sdc_fp_index
 *  Do periodic f_sync to sd card
 */
SDC_ERRORCode sdc_f_write(FIL* fp, void* buff, unsigned int btr,  unsigned int*  bytes_written) {
    static  int8_t     sync_wait = SDC_F_SYNC_COUNT;
    FRESULT f_ret;

    f_ret = f_write(fp, buff, btr,  bytes_written);
    if (f_ret) {
        SDCDEBUG("%s: f_write error: %d\r\n",__func__, f_ret) ;
        f_ret = f_write(fp, buff, btr,  bytes_written);
        if (f_ret) {
            SDCDEBUG("%s: f_write error final: %d\r\n",__func__, f_ret) ;
            return SDC_FWRITE_ERROR;
        }
    }
    sdc_fp_index += *bytes_written;

    if(sync_wait-- <= 0) {
        f_ret = f_sync(fp);
        if (f_ret)  {
            f_ret = f_sync(fp);
            if (f_ret)  {
                SDCDEBUG("f_sync error: %d\r\n", f_ret);
                return SDC_SYNC_ERROR;
            }
        }
        SDCDEBUG("*");
        sync_wait = SDC_F_SYNC_COUNT;
    }
    return SDC_OK;
}

/*
 * Wrapper around fatfs f_read function.
 *  Track the sdc_fp_index
 *  Try to read twice before failing
 */
SDC_ERRORCode sdc_f_read(FIL* fp, void* buff, unsigned int btr,  unsigned int*  bytes_read) {
    FRESULT f_ret;

    f_ret  = f_read(fp, buff, btr, bytes_read);
    if(f_ret != FR_OK) {
        SDCDEBUG("%s: read fail: %d\r\n", __func__, f_ret );
        f_ret  = f_read(fp, buff, btr, bytes_read);
        if(f_ret != FR_OK) {
            SDCDEBUG("%s: read message fail final: %d\r\n", __func__, f_ret );
        }
        return SDC_FREAD_ERROR;
    }
    sdc_fp_index += *bytes_read;

    return SDC_OK;
}

SDC_ERRORCode sdc_check_message(FIL* df, DWORD ofs) {
    SDC_ERRORCode   sdc_ret;

    DWORD           saved_ofs = sdc_fp_index;
    crc_t           crcd, crcd_calc;
    uint8_t         rd[sizeof(GENERIC_message)];
    unsigned int    bytes_read;

    sdc_ret = sdc_set_fp_index(df, ofs) ;
    if(sdc_ret != SDC_OK) { return sdc_ret; }

    sdc_ret  = sdc_f_read(df, rd, sizeof(GENERIC_message), &bytes_read);
    if(sdc_ret != SDC_OK) { return sdc_ret; }

    sdc_ret = sdc_f_read(df, &crcd, sizeof(crc_t), &bytes_read);
    if(sdc_ret != SDC_OK) { return sdc_ret; }

    // calc checksum
    crcd_calc                   = crc_init();
    crcd_calc                   = crc_update(crcd_calc, (const unsigned char*) &rd, sizeof(GENERIC_message));
    crcd_calc                   = crc_finalize(crcd_calc);
    if(crcd != crcd_calc) {
        SDCDEBUG("%s: No valid checksum in data. Data: %u\tvs.\tCalc: %u\r\n", __func__, crcd, crcd_calc);

        sdc_ret = sdc_set_fp_index(df, saved_ofs) ;
        if(sdc_ret != SDC_OK) { return sdc_ret; }

        return SDC_CHECKSUM_ERROR;
    }
    sdc_ret = sdc_set_fp_index(df, saved_ofs) ;
    if(sdc_ret != SDC_OK) { return sdc_ret; }

    return SDC_OK;
}

/*!
 * Instead of restarting from byte0 in opened existing data file, continue
 * from last successful data write.
 *
 * Track from reset due to power cycle or watchdog for instance.
 *
 */
SDC_ERRORCode sdc_seek_eod(FIL* DATAFil ) {
    SDC_ERRORCode   sdc_ret;

    int32_t         i ;
    unsigned int    bytes_read;
    uint16_t        eod_marker = 0xff;
    uint16_t        bom_marker = 0xff;
    unsigned int    bw       = 0;
    DWORD           jumpsize =  (DWORD) (sizeof(GENERIC_message) + sizeof(crc_t));

    sdc_reset_fp_index();

    /* step 0: If first line has valid (checksum) data, then seek to end of data , else return  */
    sdc_ret = sdc_check_message(DATAFil, 2) ;
    if(sdc_ret != SDC_OK) {
        SDCDEBUG("%s: First message failed checksum\t%lu\r\n", __func__ );
        return sdc_ret;
    }

    sdc_ret = sdc_set_fp_index(DATAFil, sdc_fp_index + jumpsize) ;
    if(sdc_ret != SDC_OK) {
        sdc_reset_fp_index();
        return sdc_ret;
    }

    // check next halfword for eod marker....
    sdc_ret = sdc_f_read(DATAFil, &eod_marker, sizeof(uint16_t), &bytes_read);
    if(sdc_ret != SDC_OK) {
        sdc_reset_fp_index();
        return sdc_ret;
    }

    while(eod_marker != sdc_eod.marker) {
        sdc_ret = sdc_f_read(DATAFil, &eod_marker, sizeof(uint16_t), &bytes_read);
        if(sdc_ret != SDC_OK) {
            sdc_reset_fp_index();
            return sdc_ret;
        }

        SDCDEBUG("%s:eod: 0x%x(%u)\t0x%x\r\n",__func__, sdc_fp_index, sdc_fp_index, eod_marker);
    }

    // Found eod marker back up two previous messages
    for(i=0 ; i<2; ++i) {
        do {
            sdc_ret = sdc_set_fp_index(DATAFil, sdc_fp_index - 4 ) ;
            sdc_ret = sdc_f_read(DATAFil, &bom_marker, sizeof(uint16_t), &bytes_read);
            if(sdc_ret != SDC_OK) {
                sdc_reset_fp_index();
                return sdc_ret;
            }
            SDCDEBUG("%s:bom: 0x%x(%u)\t0x%x\r\n",__func__, sdc_fp_index, sdc_fp_index, bom_marker);
        } while (bom_marker != SDC_BOM_MARK) ;
    }
    SDCDEBUG("%s: 0x%x(%u)\t0x%x\r\n",__func__, sdc_fp_index, sdc_fp_index, bom_marker);

    sdc_ret = sdc_check_message(DATAFil, (DWORD) sdc_fp_index) ;
    if(sdc_ret != SDC_OK) {
        sdc_reset_fp_index();
        return sdc_ret;
    }

    SDCDEBUG("%s:checkmessage passed, index\t%u\r\n", __func__, sdc_fp_index);
    sdc_ret = sdc_set_fp_index(DATAFil, sdc_fp_index+jumpsize);
    if(sdc_ret != SDC_OK) {
        sdc_reset_fp_index();
        return sdc_ret;
    }

    SDCDEBUG("%s:jump to end of good message \t%u\r\n", __func__, sdc_fp_index);
    sdc_ret = sdc_f_write(DATAFil, (void *)(&sdc_eod), sizeof(sdc_eod_marker), (unsigned int*) &bw);
    if (sdc_ret)  {
        sdc_reset_fp_index();
        return sdc_ret;
    }

    sdc_ret = sdc_set_fp_index(DATAFil, sdc_fp_index - sizeof(sdc_eod_marker)) ;
    if (sdc_ret)  {
        sdc_reset_fp_index();
        return sdc_ret;
    }

    SDCDEBUG("%s:after rewind:\t%u\r\n", __func__, sdc_fp_index);
    /* phase 2: Use eeprom (not avail on e407, wait for new PSAS GFE) as scratch pad to end of data. */

    return SDC_OK;
}


//! @}

