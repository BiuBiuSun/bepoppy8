/*
 * Copyright (C) 2016 Wilco Vlenterie, Anand Sundaresan.
 * Contact: Anand Sundaresan <nomail@donotmailme.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * \file rtcm2ivy.c
 * \brief RTCM3 GPS packets to Ivy for DGPS and RTK
 *
 * This communicates with an RTCM3 GPS receiver like an
 * ublox M8P. This then forwards the Observed messages
 * over the Ivy bus to inject them for DGPS and RTK positioning.
 */

#include "std.h"
#include "serial_port.h"
/** Used variables **/
struct SerialPort *serial_port;

#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <rtcm3.h>
#include <CRC24Q.h>
#include <math/pprz_geodetic_float.h>




/** Used variables **/
struct SerialPort *serial_port;

/* ubx structure definitions*/
msg_state_t msg_state;
msg_callbacks_node_t rtcm3_1005_node;
msg_callbacks_node_t rtcm3_1077_node;
msg_callbacks_node_t rtcm3_1087_node;

msg_callbacks_node_t ubx_nav_svin_node;

msg_callbacks_node_t ubx_ack_ack_node;
msg_callbacks_node_t ubx_ack_nak_node;

/** Default values **/
uint8_t ac_id         = 0;
uint32_t msg_cnt      = 0;
char *serial_device   = "/dev/ttyACM0";
uint32_t serial_baud  = B9600;

/** Debugging options */
bool verbose          = FALSE;
bool logger           = FALSE;

#define printf_debug    if(verbose == TRUE) printf

FILE * 	pFile;

/** Ivy Bus default */
#ifdef __APPLE__
char *ivy_bus                   = "224.255.255.255";
#else
char *ivy_bus                   = "127.255.255.255"; // 192.168.1.255   127.255.255.255
#endif

/*
 * Read bytes from the uBlox UART connection
 * This is a wrapper functions used in the librtcm3 library
 */
static uint32_t uart_read(unsigned char (*buff)[], uint32_t n)
{
	int ret = read(serial_port->fd, buff, n);
	if(ret > 0)
		return ret;
	else
		return 0;
}

static void ivy_send_message(uint8_t packet_id, uint8_t len, uint8_t msg[]) {
	char gps_packet[4146], number[10];    // 1024 + 6 = max msg_len --> *4 for int representation in string (255,) + 25 ivy_msg description + 1 null character = 4146
	uint8_t i;
	snprintf(gps_packet, 4146, "rtcm2ivy RTCM_INJECT %d %d", packet_id, msg[0]);
	for(i = 1; i < len; i++) {
		if(i==124)
		{
			snprintf(number, 10, ",5,5,%d", msg[i]); 	// Temporary fix for IVYBUS message size of 128 bytes
		}else{
			snprintf(number, 10, ",%d", msg[i]);
		}
		strcat(gps_packet, number);
	}
	printf_debug("%s\n\n", gps_packet);
	IvySendMsg("%s", gps_packet);
	if(logger == TRUE)
	{
		pFile = fopen("./RTCM3_log.txt","a");
		fprintf(pFile,"%s\n", gps_packet);
		fclose(pFile);
	}
	printf_debug("Ivy send: %s\n", gps_packet);
}

/*
 * Callback for the 1005 message to send it trough RTCM_INJECT
 */
struct EcefCoor_f posEcef;
struct LlaCoor_f  posLla;

static void rtcm3_1005_callback(uint8_t len, uint8_t msg[])
{
	if(len > 0) {
		if (crc24q(msg, len - 3) == RTCMgetbitu(msg, (len - 3) * 8, 24)) {
			ivy_send_message(RTCM3_MSG_1005, len, msg);
			msg_cnt++;
			u16 StaId      = RTCMgetbitu(msg, 24 + 12, 12);
			u8 ItRef       = RTCMgetbitu(msg, 24 + 24, 6);
			u8 indGPS      = RTCMgetbitu(msg, 24 + 30, 1);
			u8 indGlonass  = RTCMgetbitu(msg, 24 + 31, 1);
			u8 indGalileo  = RTCMgetbitu(msg, 24 + 32, 1);
			u8 indRefS     = RTCMgetbitu(msg, 24 + 33, 1);
			posEcef.x      = RTCMgetbits_38(msg, 24 + 34) * 0.0001;
			posEcef.y      = RTCMgetbits_38(msg, 24 + 74) * 0.0001;
			posEcef.z      = RTCMgetbits_38(msg, 24 + 114) * 0.0001;
			lla_of_ecef_f(&posLla, &posEcef);
			printf_debug("Lat: %f, Lon: %f, Alt: %f\n", posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, posLla.alt);
			// Send spoof gpsd message to GCS to plot groundstation position
			IvySendMsg("%s %s %s %f %f %f %f %f %f %f %f %f %f %f %d %f", "ground", "FLIGHT_PARAM", "GCS", 0.0, 0.0, 0.0, posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, 0.0, 0.0, posLla.alt, 0.0, 0.0, 0.0, 0,  0.0);
			// Send UBX_RTK_GROUNDSTATION message to GCS for RTK info
			IvySendMsg("%s %s %s %i %i %i %i %i %i %f %f %f", "ground", "UBX_RTK_GROUNDSTATION", "GCS", StaId, ItRef, indGPS, indGlonass, indGalileo, indRefS, posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, posLla.alt);
		}else{
			printf("Skipping 1005 message (CRC check failed)\n");
		}
	}
	printf_debug("Parsed 1005 callback\n");
}

/*
 * Callback for the 1077 message to send it trough RTCM_INJECT
 */
static void rtcm3_1077_callback(uint8_t len, uint8_t msg[])
{
	if(len > 0) {
		if (crc24q(msg, len - 3) == RTCMgetbitu(msg, (len - 3) * 8, 24)) {
			ivy_send_message(RTCM3_MSG_1077, len, msg);
			msg_cnt++;
		}else{
			ivy_send_message(RTCM3_MSG_1077, len, msg);
			printf("Skipping 1077 message (CRC check failed)\n");
		}
	}
	printf_debug("Parsed 1077 callback\n");
}

/*
 * Callback for the 1087 message to send it trough RTCM_INJECT
 */
static void rtcm3_1087_callback(uint8_t len, uint8_t msg[])
{
	if(len > 0) {
		if (crc24q(msg, len - 3) == RTCMgetbitu(msg, (len - 3) * 8, 24)) {
			ivy_send_message(RTCM3_MSG_1087, len, msg);
			msg_cnt++;
		}else{
			printf("Skipping 1087 message (CRC check failed)\n");
		}
	}
	printf_debug("Parsed 1087 callback\n");
}


/*
 * Callback for UBX survey-in message
 */

static void ubx_navsvin_callback(uint8_t len, uint8_t msg[])

{
   if (len>0) {

	   u32 iTow      = UBX_NAV_SVIN_ITOW(msg);
	   u32 dur       = UBX_NAV_SVIN_dur (msg);
	   float meanAcc = (float) 0.1 * UBX_NAV_SVIN_meanACC(msg);
	   u8 valid      = UBX_NAV_SVIN_Valid(msg);
	   u8 active     = UBX_NAV_SVIN_Active(msg);
	   printf ("iTow: %u \t dur: %u \t meaAcc: %f \t valid: %u \t active: %u \n", iTow, dur, meanAcc,valid,active);
   }
}


static void ubx_ackack_callback(uint8_t len, uint8_t msg[])

{
   if (len>0) {
	   u8 ack_ClsId = UBX_ACK_ACK_ClsID(msg);
	   u8 ack_MsgId = UBX_ACK_ACK_MsgID(msg);
	   printf("Acknowledge: Success! \t Msg_class: %0x \t Msg_id: %0x\n", ack_ClsId, ack_MsgId);
   }
}

static void ubx_acknak_callback(uint8_t len, uint8_t msg[])

{
   if (len>0) {
	   u8 nak_ClsId = UBX_ACK_NAK_ClsID(msg);
	   u8 nak_MsgId = UBX_ACK_NAK_MsgID(msg);
	   printf("Acknowledge: Failed! \t Msg_class: %u \t Msg_id: %u \n", nak_ClsId, nak_MsgId);

   }
}

/**
 * Parse the tty data when bytes are available
 */
static gboolean parse_device_data(GIOChannel *chan, GIOCondition cond, gpointer data)
{
	unsigned char buff[1000];
	int c;
	c = uart_read(&buff, 1);
	if(c > 0) 									// Have we read anything?
	{
		if(msg_state.msg_class == RTCM_CLASS) 		// Are we already reading a RTCM message?
		{
			rtcm3_process(&msg_state, buff[0]); 		// If so continue reading RTCM
		}else if(msg_state.msg_class == UBX_CLASS) 	// Are we already reading a UBX message?
		{
			ubx_process(&msg_state, buff[0]); 			// If so continue reading UBX
		}else{
			msg_state.state = UNINIT; 					// Not reading anything yet
			rtcm3_process(&msg_state, buff[0]); 		// Try to process preamble as RTCM
			if(msg_state.msg_class != RTCM_CLASS)		// If it wasn't a RTCM preamble
			{
				ubx_process(&msg_state, buff[0]);			// Check for UBX preamble
			}
		}
	}
	return TRUE;
}

/** Print the program help */
void print_usage(int argc __attribute__((unused)), char ** argv) {
	static const char *usage =
			"Usage: %s [options]\n"
			" Options :\n"
			"   -h, --help                Display this help\n"
			"   -v, --verbose             Verbosity enabled\n"
			"   -l, --logger              Save RTCM3 messages to log\n\n"

			"   -d <device>               The GPS device(default: /dev/ttyACM0)\n"
			"   -b <baud_rate>            The device baud rate(default: B9600)\n\n";
	fprintf(stderr, usage, argv[0]);
}


int main(int argc, char** argv)
{

	// Parse the options from cmdline
	char c;
	while ((c = getopt (argc, argv, "hvld:b:i:")) != EOF) {
		switch (c) {
		case 'h':
			print_usage(argc, argv);
			exit(EXIT_SUCCESS);
			break;
		case 'v':
			verbose = TRUE;
			break;
		case 'l':
			logger = TRUE;
			break;
		case 'd':
			serial_device = optarg;
			break;
		case 'b':
			serial_baud = atoi(optarg);
			break;
		case 'i':
			ac_id = atoi(optarg);
			break;
		case '?':
			if (optopt == 'd' || optopt == 'b' || optopt == 'i')
				fprintf (stderr, "Option -%c requires an argument.\n", optopt);
			else if (isprint (optopt))
				fprintf (stderr, "Unknown option `-%c'.\n", optopt);
			else
				fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
			print_usage(argc, argv);
			exit(EXIT_FAILURE);
		default:
			abort();
		}
	}

	// Create the Ivy Client
	GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
	IvyInit("Paparazzi server", "Paparazzi server READY", 0, 0, 0, 0);
	IvyStart(ivy_bus);

	// Start the tty device
	printf_debug("Opening tty device %s...\n", serial_device);
	serial_port = serial_port_new();
	int ret = serial_port_open_raw(serial_port, serial_device, serial_baud);
	if (ret != 0) {
		fprintf(stderr, "Error opening %s code %d\n", serial_device, ret);
		serial_port_free(serial_port);
		exit(EXIT_FAILURE);
	}

	// Setup RTCM3 callbacks
	printf_debug("Setup RTCM3 callbacks...\n");
	msg_state_init(&msg_state);
	register_callback(&msg_state, RTCM3_MSG_1005, &rtcm3_1005_callback, &rtcm3_1005_node);
	register_callback(&msg_state, RTCM3_MSG_1077, &rtcm3_1077_callback, &rtcm3_1077_node);
	register_callback(&msg_state, RTCM3_MSG_1087, &rtcm3_1087_callback, &rtcm3_1087_node);

	register_callback(&msg_state, UBX_NAV_SVIN, &ubx_navsvin_callback, &ubx_nav_svin_node);
	register_callback(&msg_state, UBX_ACK_ACK_ID, &ubx_ackack_callback, &ubx_ack_ack_node);
	register_callback(&msg_state, UBX_ACK_NAK_ID, &ubx_acknak_callback, &ubx_ack_nak_node);

	// Add IO watch for tty connection
	printf_debug("Adding IO watch...\n");
	GIOChannel *sk = g_io_channel_unix_new(serial_port->fd);
	g_io_add_watch(sk, G_IO_IN, parse_device_data, NULL);


	// Configuring the chip for TMODE3 (svin parameters)
	//	uint16_t ubx_flags;
	uint8_t lla = 1; // 0 - ECEF
	uint8_t mode =1; // 1- suvery in, 0 - disabled , 2- Fixed mode
	uint16_t ubx_flags = (lla<<8)+mode; // Fix me : mind the reserve bytes get shifted
	uint32_t svinacclimit= 10 * 25000; // in  mm
	uint32_t svinmindur= 10;

	UbxSend_CFG_TMODE3(0,0, ubx_flags,0,0,0,0,0,0,0,0,svinmindur,svinacclimit,0,0);

	// Run the main loop
	printf_debug("Started rtcm2ivy for aircraft id %d!\n", ac_id);
	g_main_loop_run(ml);

	return 0;
}

