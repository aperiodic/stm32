/*!
 * \file data_udp.c

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

/*! \brief Sensor datapath node.
 * \defgroup dataudp Data Sensor UDP PSAS Experiment
 * @{
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/ip_addr.h"

#include "fc_net.h"
#include "device_net.h"
#include "data_udp.h"

	enum { remote_ip, local_ip };

#define LWIP_NETCONN 1
#if LWIP_NETCONN

WORKING_AREA(wa_data_udp_send_thread, DATA_UDP_SEND_THREAD_STACK_SIZE);


static uint8_t ip_addr_1(ip_addr_t addr) {
	return (addr.addr & 0xff);
}

static uint8_t ip_addr_2(ip_addr_t addr) {
	return (addr.addr & 0xff00) >> 8;
}

static uint8_t ip_addr_3(ip_addr_t addr) {
	return (addr.addr & 0xff0000) >> 16;
}

static uint8_t ip_addr_4(ip_addr_t addr) {
	return (addr.addr & 0xff000000) >> 24;
}

static void print_ip(ip_addr_t addr, uint16_t port) {
	BaseSequentialStream *chp   =  (BaseSequentialStream *)&SD1;

	chprintf(chp, "my_ip: %u.%u.%u.%u: %u\r\n", ip_addr_1(addr), ip_addr_2(addr), ip_addr_3(addr), ip_addr_4(addr), port);

}

msg_t data_udp_send_thread(void *p) {
	void * arg __attribute__ ((unused)) = p;
	BaseSequentialStream *chp   =  (BaseSequentialStream *)&SD1;

	err_t                 err;
	uint8_t               count = 0;

	struct     netconn    *conn;
	struct     netbuf     *buf;

	char*                  data;
	char                   msg[DATA_UDP_MSG_SIZE] ;

	ip_addr_t              ip_addr_rnet;
	ip_addr_t              ip_addr_fc;
	ip_addr_t            my_ip;

		uint16_t             my_port;
	RNET_A_IP_ADDR(&ip_addr_rnet);
	IP_PSAS_FC(&ip_addr_fc);

	chRegSetThreadName("data_udp_send_thread");

	conn   = netconn_new( NETCONN_UDP );

	chThdSleepMilliseconds(1000);
	chprintf(chp, "Start udp send thread\n\r");

	/* Bind to the local address, or to ANY address */
	//	netconn_bind(conn, NULL, DATA_UDP_TX_THREAD_PORT ); //local port, NULL is bind to ALL ADDRESSES! (IP_ADDR_ANY)
	err    = netconn_bind(conn, &ip_addr_rnet, RNET_A_TX_PORT ); //local port
	netconn_getaddr(conn, &my_ip, &my_port,  local_ip);
		print_ip(my_ip, my_port);

	if (err == ERR_OK) {
		/* Connect to specific address or a broadcast address */
		/*
		 * \todo Understand why a UDP needs a connect...
		 *   This may be a LwIP thing that chooses between tcp_/udp_/raw_ connections internally.
		 *
		 */
		//	netconn_connect(conn, IP_ADDR_BROADCAST, DATA_UDP_TX_THREAD_PORT );
		err = netconn_connect(conn, &ip_addr_fc, FC_LISTEN_PORT_RNET_A );
		if(err == ERR_OK) {
			for( ;; ){
				buf     =  netbuf_new();
				data    =  netbuf_alloc(buf, sizeof(msg));
				sprintf(msg, "rnet tx: %d", count++);
				memcpy (data, msg, sizeof (msg));
				err = netconn_send(conn, buf);
				chprintf(chp, "rnet sent: index: %d. Error: %d\r\n", count, err);

				netbuf_delete(buf); // De-allocate packet buffer
				chThdSleepMilliseconds(500);
			}
			return RDY_OK;
		} else {
			return RDY_RESET;
		}
	} else {
		return RDY_RESET;
	}
}

static void data_udp_rx_serve(struct netconn *conn) {
	BaseSequentialStream *chp   =  (BaseSequentialStream *)&SD1;

	static uint8_t       count  = 0;

	struct netbuf        *inbuf;

	char                 *buf;

	uint16_t             buflen = 0;
	uint16_t             i      = 0;

	ip_addr_t            my_ip;

	uint16_t             my_port;
	err_t                err;

	/*
	 * Read the data from the port, blocking if nothing yet there.
	 * We assume the request (the part we care about) is in one netbuf
	 */
	err = netconn_recv(conn, &inbuf);
	netconn_getaddr(conn, &my_ip, &my_port,  local_ip);
	print_ip(my_ip, my_port);

	if (err == ERR_OK) {
		netbuf_data(inbuf, (void **)&buf, &buflen);
		chprintf(chp, "\r\nsensor rx:%d:->", count++);
		for(i=0; i<buflen; ++i) {
			chprintf(chp, "%c", buf[i]);
		}
		chprintf(chp, "\r\n");
	}
	netconn_close(conn);

	/* Delete the buffer (netconn_recv gives us ownership,
	 * so we have to make sure to deallocate the buffer)
	 */
	netbuf_delete(inbuf);
}

/*!
 * Stack area for the data_udp_receive_thread.
 */
WORKING_AREA(wa_data_udp_receive_thread, DATA_UDP_SEND_THREAD_STACK_SIZE);

/*!
 * data_udp_rx  thread.
 */
msg_t data_udp_receive_thread(void *p) {
	void * arg __attribute__ ((unused)) = p;

	err_t             err;

	struct netconn    *conn;

	ip_addr_t        ip_addr_rnet;

	chRegSetThreadName("data_udp_receive_thread");

	RNET_A_IP_ADDR(&ip_addr_rnet);

	/*
	 *  Create a new UDP connection handle
	 */
	conn = netconn_new(NETCONN_UDP);
	LWIP_ERROR("data_udp_receive_thread: invalid conn", (conn != NULL), return RDY_RESET;);

	/*
	 * Bind sensor address to a udp port
	 */
	err = netconn_bind(conn, &ip_addr_rnet, RNET_A_LISTEN_PORT);

	if (err == ERR_OK) {
		while(1) {
			data_udp_rx_serve(conn);
		}
		return RDY_OK;
	} else {
		return RDY_RESET;
	}
}

#endif

//! @}

