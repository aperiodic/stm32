/*!
 * \file data_udp.c
 */

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

/*
 * This file is a modified version of the lwIP web server demo. The original
 * author is unknown because the file didn't contain any license information.
 */

/*!
 * \brief Experiment with sending sensor data over UDP connection.
 * \defgroup dataudp Data UDP PSAS Experiment
 * @{
 */

#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"

#include "usbdetail.h"
#include "data_udp.h"
#include "pwm_config.h"
#include "device_net.h"

#define LWIP_NETCONN 1
#if LWIP_NETCONN

ip_addr_t ip_addr_fc;

WORKING_AREA(wa_data_udp_send_thread, DATA_UDP_SEND_THREAD_STACK_SIZE);

msg_t data_udp_send_thread(void *p) {
	void * arg __attribute__ ((unused)) = p;
	uint8_t               count = 0;
	struct     netconn    *conn;
	char                   msg[DATA_UDP_MSG_SIZE] ;
	struct     netbuf     *buf;
	char*                  data;

	  chRegSetThreadName("data_udp_send_thread");

//    netconn_connect(tftptxcon,&modsrv_addr,ip_data_out.port);    // Open connection
//    tftptxcon->pcb.udp->local_port=69;   // Set local (source) port
//    tftpsrvbuf=netbuf_new();        // Create netbuf
//      .....
//
	conn       = netconn_new( NETCONN_UDP );
	netconn_bind(conn, NULL, 35000 ); //local port

	netconn_connect(conn, IP_ADDR_BROADCAST, DATA_UDP_THREAD_PORT );
	for( ;; ){
		buf     =  netbuf_new();
		data    =  netbuf_alloc(buf, sizeof(msg));
		sprintf(msg, "PSAS Rockets! %d", count++);
		memcpy (data, msg, sizeof (msg));
		netconn_send(conn, buf);
		netbuf_delete(buf); // De-allocate packet buffer
	    chThdSleepMilliseconds(500);
	}
}

static void data_udp_rx_serve(struct netconn *conn) {
  BaseSequentialStream *chp =  (BaseSequentialStream *)&SDU_PSAS;

  struct netbuf        *inbuf;
  char                 *buf;

  uint16_t             buflen = 0;
  char cmdbuf[DATA_UDP_CMDBUF_SIZE];

  //uint16_t        i      = 0;
  err_t           err    = 0;

  /*fill buffer with nulls*/
  memset(cmdbuf, 0, DATA_UDP_CMDBUF_SIZE);

  /* Read the data from the port, blocking if nothing yet there.
   We assume the request (the part we care about) is in one netbuf */
  chprintf(chp, ".w.\r\n");
  err = netconn_recv(conn, &inbuf);

  if (err == ERR_OK) {
    netbuf_data(inbuf, (void **)&buf, &buflen);

    chprintf(chp, "buffer: %s\r\n", buf);

//	if (strncmp("GETPWMWIDTH", (const char *) cmdbuf, 11) == 0) {
//		char respBuf[64];
//		unsigned int pulseWidth = pwm_getPulseWidth();
//		sprintf(respBuf, "PULSE WIDTH %d", pulseWidth);
//		sendResponsePacket(respBuf);
//	} else if (strncmp("SETPWMWIDTH", (const char *) cmdbuf, 11) == 0) {
//		pwm_set_pulse_width_ticks(20000);
//	} else {
//		sendResponsePacket("CMDUNDEF");
//	};

  }
  netconn_close(conn);

  /* Delete the buffer (netconn_recv gives us ownership,
   * so we have to make sure to deallocate the buffer)
   */
  netbuf_delete(inbuf);
}

WORKING_AREA(wa_data_udp_receive_thread, DATA_UDP_RECEIVE_THREAD_STACK_SIZE);
msg_t data_udp_receive_thread(void *p) {
    void * arg __attribute__ ((unused)) = p;

    err_t             err;

    struct netconn    *conn;

    ip_addr_t         ip_addr_rc;

    chRegSetThreadName("data_rc_udp_rx_thread");

    ROLL_CTL_IP_ADDR(&ip_addr_rc);

    /*
     *  Create a new UDP connection handle
     */
    conn = netconn_new(NETCONN_UDP);
    LWIP_ERROR("data_udp_receive_thread: invalid conn", (conn != NULL), return RDY_RESET;);

    /*
     * Bind sensor address to a udp port
     */
    err = netconn_bind(conn, &ip_addr_rc, ROLL_CTL_LISTEN_PORT);

    if (err == ERR_OK) {
        while(1) {
            data_udp_rx_serve(conn);
        }
        return RDY_OK;
    } else {
        return RDY_RESET;
    }
}
//
///*!
// * Stack area for the data_udp_receive_thread.
// */
//WORKING_AREA(wa_data_udp_receive_thread, DATA_UDP_SEND_THREAD_STACK_SIZE);
//
///*!
// * data_udp_rx  thread.
// */
//msg_t data_udp_receive_thread(void *p) {
//  void * arg __attribute__ ((unused)) = p;
//
//  struct netconn *conn;
//
//  chRegSetThreadName("data_udp_receive_thread");
//
//  chThdSleepSeconds(2);
//
////  IP4_ADDR(&ip_addr_fc, 192,168,0,91);
//  IP4_ADDR(&ip_addr_fc, 10,0,0,2);
//  /* Create a new UDP connection handle */
//  conn = netconn_new(NETCONN_UDP);
//  LWIP_ERROR("data_udp_receive_thread: invalid conn", (conn != NULL), return RDY_RESET;);
//
//  netconn_bind(conn, &ip_addr_fc, DATA_UDP_RX_THREAD_PORT);
//
//  while(1) {
//    //netconn_connect(conn,  &ip_addr_fc, DATA_UDP_RX_THREAD_PORT );
//    data_udp_rx_serve(conn);
//  }
//  return RDY_OK;
//}

void sendResponsePacket( char  payload[]) {
   struct     netconn    *conn;
   char                   msg[DATA_UDP_MSG_SIZE] ;
   struct     netbuf     *buf;
   char*                  data;
	struct ip_addr addr;
	addr.addr = PSAS_UDP_TARGET;
   conn       = netconn_new( NETCONN_UDP );
   netconn_bind(conn, NULL, 35001 ); //local port

   netconn_connect(conn, &addr , DATA_UDP_REPLY_PORT );
   buf     =  netbuf_new();
   data    =  netbuf_alloc(buf, sizeof(msg));
   sprintf(msg, "%s", payload);
   memcpy (data, msg, sizeof (msg));
   netconn_send(conn, buf);
   netbuf_delete(buf); // De-allocate packet buffer
	netconn_disconnect(conn);
}

#endif

//! @}

