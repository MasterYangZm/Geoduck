// Part of flagger mobile
// orig 11/15/2004 
// flagger/decoy adaptation  2/2/2018
// update for ublox8 7/1/2018
// Author: GAStephens
//
// This file contains the Ublox UBX protocol definitions.
// Copyright (C) 2004 - 2018 Traqmate, LLC.
//
#include <atmel_start.h>
#include <string.h>
#include "ubx8.h"
#include "gpsutilities.h"

volatile bool gpsPulseHigh = false;
uint8_t ubxfullset = 0;				// 7 when all 3 messages received
gpsType currentGPS;					// where we are
gpsType previousGPS[GPS_SAMPLES];   // last twenty gps samples samples - used for dead reckoning
uint8_t lastgps = GPS_SAMPLES-1;	// index into previousGPS of last collected gps sample

extern struct io_descriptor *io_USART_0;
extern void gpsUart_Flush(void);
extern volatile uint8_t gps_usart_buffer[2][UBXRXSIZE];	// Double buffer
extern volatile uint8_t gps_next_buffer;					// Always points to buffer being filled. process the other one.

typedef struct
{
	uint16_t	messageid;
	uint8_t		rate;
} messagelisttype;

uint8_t ubxbuff[UBXRXSIZE];     // buffer to collect UBX message (minus wrapper)

// **** TEST
#if 0
char USART_getch_timeout(struct io_descriptor *const io_descr) {
	int const timeOutVal = 500;			// in ms
	uint32_t readtimer = debounceTimer;

	do {
		if (usart_sync_is_rx_not_empty(USART_0)) {
			io_read(io_USART_0, &c, 1);
			return (c);
		} // if
	} while ((debounceTimer - readtimer) < timeOutVal);
	return (NULL);
}
#endif

uint16_t waitForUBX()
{
	uint8_t c;
  
	do
	{
		// get characters until timeout or ACKACK received
		do
		{
			// look for synch character
//			usart_serial_getchar(GPS_UART_SERIAL, &c);
			if (usart_sync_is_rx_not_empty(&USART_0))
				io_read(io_USART_0, &c, 1);
		} while (UBXSYNCH1 != c);
		
		// got first synch char so look for the rest of the message
		io_read(io_USART_0, &c, 1);
		
		if (UBXSYNCH2 == c)	{
			io_read(io_USART_0, &c, 1);
			
			if (0x05 == c) {
				// upper byte ACKACK
				io_read(io_USART_0, &c, 1);
				
				if (0x01 == c) {
					// lower byte ACKACK
					return (ACKACK);
				}
			} // if
		} // if
	} while (1);
} // waitForUBX

void UBXBaudChange()
{
	uint16_t	i;
	uint8_t		packet[UBXTXSIZE];

	// configure protocols and baudrates on port
	i = 0;

	packet[UBXPAYLOAD+i++] = 1;										// 0 port number 1
	packet[UBXPAYLOAD+i++] = 0;										// 1 reserved
	packet[UBXPAYLOAD+i++] = 0;										// 2 reserved
	packet[UBXPAYLOAD+i++] = 0;										// 3 reserved
	packet[UBXPAYLOAD+i++] = 0xC0;									// 4 11000000 8 bits
	packet[UBXPAYLOAD+i++] = 0x08;									// 5 00001000 1 stop bit, no parity       
	packet[UBXPAYLOAD+i++] = 0;										// 6 bitmask second byte
	packet[UBXPAYLOAD+i++] = 0;										// 7 bitmask top byte

//  **** runtime barfs on this statement for some reason
	*(uint32_t *) (packet + UBXPAYLOAD + i) = (uint32_t) UBXBAUD;	// 8 57 Kbps
	i += 4;
//	packet[UBXPAYLOAD+i++] = (uint8_t) (UBXBAUD & 0x000000FF);			// 8 lsb
//	packet[UBXPAYLOAD+i++] = (uint8_t) ((UBXBAUD & 0x0000FF00) >> 8);		// 9 next
//	packet[UBXPAYLOAD+i++] = (uint8_t) ((UBXBAUD & 0x00FF0000) >> 16);	// 10 next
//	packet[UBXPAYLOAD+i++] = (uint8_t) ((UBXBAUD & 0xFF000000) >> 24);	// 11 msb

	packet[UBXPAYLOAD+i++] = 0x01;									// 12 bitmask ublox input enabled
	packet[UBXPAYLOAD+i++] = 0x00;									// 13 bitmask top byte
	packet[UBXPAYLOAD+i++] = 0x01;									// 14 bitmask ublox output enabled
	packet[UBXPAYLOAD+i++] = 0x00;									// 15 bitmask top byte
	packet[UBXPAYLOAD+i++] = 0;										// 16 reserved
	packet[UBXPAYLOAD+i++] = 0;										// 17 reserved
	packet[UBXPAYLOAD+i++] = 0;										// 18 reserved
	packet[UBXPAYLOAD+i++] = 0;										// 19 reserved

	// send configuration message
	UBXSendPacket(CFGPRT, i, packet);

	// wait for transmission to finish
	while (usart_sync_is_tx_empty(&USART_0) != 1);
	
	// breathe a moment
	delay_ms(20);
} // UBXBaudChange

void UBXInit()
{
	uint16_t	i;
	uint8_t		msgcnt;
	uint8_t		packet[UBXTXSIZE];

#ifdef UBLOX6
#define NUMUBLOXMSGS	3
#endif

#ifdef UBLOX8
#define NUMUBLOXMSGS	1
#endif

	messagelisttype messagelist[NUMUBLOXMSGS] =
	{
#ifdef UBLOX6
		{ NAVVELNED, 1},
		{ NAVSOL, 1 },
		{ NAVPOSLLH, 1 }
#endif

#ifdef UBLOX8
		{ NAVPVT, 1 }
#endif
	};

	// Clear out any junk in the input buffer
	while(usart_sync_is_rx_not_empty(&USART_0)) {
		uint8_t c;
		io_read(io_USART_0, &c, 1);
	}

	// turn UBX messages on/off and set the reporting rate
	for (msgcnt = 0; msgcnt < NUMUBLOXMSGS; msgcnt++)
	{
		do
		{
			// repeat until we get it right
			i = 0;
  
			packet[UBXPAYLOAD+i++] = (messagelist[msgcnt].messageid >> 8) & 0xFF;	// class
			packet[UBXPAYLOAD+i++] = messagelist[msgcnt].messageid & 0xFF;			// id
			packet[UBXPAYLOAD+i++] = messagelist[msgcnt].rate;						// rate (usually 1 per solution)
#ifdef UBLOX8
			packet[UBXPAYLOAD+i++] = messagelist[msgcnt].rate;						// rate ports 2-6
			packet[UBXPAYLOAD+i++] = messagelist[msgcnt].rate;						// rate
			packet[UBXPAYLOAD+i++] = messagelist[msgcnt].rate;						// rate
			packet[UBXPAYLOAD+i++] = messagelist[msgcnt].rate;						// rate
			packet[UBXPAYLOAD+i++] = messagelist[msgcnt].rate;						// rate
#endif
  
			// send configuration message
			UBXSendPacket(CFGMSG, i, packet);
      
			// wait for transmission to finish
			while (usart_sync_is_tx_empty(&USART_0) != 1);

			// wait for a UBX message
			i = waitForUBX();

			delay_ms(50);															// pause a moment
		} while (i != ACKACK);
	} // for

	// set the measuring rate
	do
	{
		// repeat until we get it right
		i = 0;

		packet[UBXPAYLOAD+i++] = GPS_MS & 0xFF;				// lower byte of GPS measurement rate in ms
		packet[UBXPAYLOAD+i++] = (GPS_MS & 0xFF00) >> 8;	// upper byte of GPS measurement rate in ms
		packet[UBXPAYLOAD+i++] = 1;							// 1 report per measurement cycle
		packet[UBXPAYLOAD+i++] = 0;
		packet[UBXPAYLOAD+i++] = 0;							// align to UTC time
		packet[UBXPAYLOAD+i++] = 0;

		// send configuration message
		UBXSendPacket(CFGRATE, i, packet);

			// wait for transmission to finish
			while (usart_sync_is_tx_empty(&USART_0) != 1);
		
		// wait for a UBX message
		i = waitForUBX();

		delay_ms(50);										// pause a moment
	} while (i != ACKACK);

	// set the pulse parameters NEW TP5 IMPLEMENTATION 2018/4/9 gas
	do
	{
		// repeat until we get it right
		i = 0;
		packet[UBXPAYLOAD+i++] = 0;														// 0 which time pulse? 0=primary
		packet[UBXPAYLOAD+i++] = 0;														// 1 message version
		packet[UBXPAYLOAD+i++] = 0;														// 2 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 3 reserved
		packet[UBXPAYLOAD+i++] = 50;													// 4 50 ns cable delay (Ublox supplied typical value)
		packet[UBXPAYLOAD+i++] = 0;														// 5
		packet[UBXPAYLOAD+i++] = 820 & 0xFF;											// 6 rf group delay (Ublox supplied value)
		packet[UBXPAYLOAD+i++] = 820 >> 8;												// 7
		packet[UBXPAYLOAD+i++] = SAMPS_PER_SEC;											// 8 frequency of timepulse, general
		packet[UBXPAYLOAD+i++] = 0;														// 9
		packet[UBXPAYLOAD+i++] = 0;														// 10
		packet[UBXPAYLOAD+i++] = 0;														// 11
		packet[UBXPAYLOAD+i++] = SAMPS_PER_SEC;											// 12 frequency of timepulse when locked
		packet[UBXPAYLOAD+i++] = 0;														// 13
		packet[UBXPAYLOAD+i++] = 0;														// 14
		packet[UBXPAYLOAD+i++] = 0;														// 15
		*((uint32_t *) &(packet[UBXPAYLOAD+i])) = (uint32_t) GPS_PULSE_WID;				// 16 - 19 pulse length general
		i += 4;
		*((uint32_t *) &(packet[UBXPAYLOAD+i])) = (uint32_t) GPS_LOCK_PULSE_WID;		// 20 - 23 pulse length when locked
		i += 4;
		*((uint32_t *) &(packet[UBXPAYLOAD+i])) = (uint32_t) (PULSE_DELAY * 1000000L);	// 24 - 27 user pulse delay
		i += 4;
		
		/*
		 * config flags most sig first
		 * 0 gridutcgps utc time
		 * 1 polarity = rising
		 * 1 aligntotow
		 * 1 islength
		 * 1 isfreq
		 * 0 lockedotherset
		 * 1 lockgpsfreq
		 * 1 active
		 * 01111011 = 0x7B
		 */
		
		packet[UBXPAYLOAD+i++] = 0x7D;													// 28 configuration flags
		packet[UBXPAYLOAD+i++] = 0;														// 29
		packet[UBXPAYLOAD+i++] = 0;														// 30
		packet[UBXPAYLOAD+i++] = 0;														// 31

		// send configuration message
		UBXSendPacket(CFGTP5, i, packet);

			// wait for transmission to finish
			while (usart_sync_is_tx_empty(&USART_0) != 1);
		
		// wait for a UBX message
		i = waitForUBX();

		delay_ms(50);																	// pause a moment
	} while (i != ACKACK);

	do
	{
		// repeat until we get it right
		i = 0;
		
		/*
		 * config flags most sig first
		 * 1 auto recovery from short
		 * 1 enable short circuit protection
		 * 1 enable open circuit detection
		 * 1 enable short circuit detection
		 * 1 enable antenna supply voltage
		 * 00011111 = 0x1F
		 */
		
#if EXTANT
	    *((uint16_t *) (packet + UBXPAYLOAD)) = (uint16_t) 0x001F;						// 0 enable external antenna power and protection
#else
		*((uint16_t *) (packet + UBXPAYLOAD)) = (uint16_t) 0x0000;						// 0 disable external antenna power
#endif
		*((uint16_t *) (packet + UBXPAYLOAD + 2)) = (uint16_t) 0x8000;					// 2 antenna config bitfield pins

		// send configuration message
		UBXSendPacket(CFGANT, i, packet);

			// wait for transmission to finish
			while (usart_sync_is_tx_empty(&USART_0) != 1);
	
		// wait for a UBX message
		i = waitForUBX();

		delay_ms(50);																	// pause a moment
	} while (i != ACKACK);

#ifdef UBLOX8
	// turn on auto sbas
	do
	{
		// repeat until we get it right
		i = 0;

		packet[UBXPAYLOAD+i++] = 1;														// 0 sbas enabled
		packet[UBXPAYLOAD+i++] = 0x07;													// 1 use it all
		packet[UBXPAYLOAD+i++] = 32;													// 2 max sbas channels
		packet[UBXPAYLOAD+i++] = 0xFF;													// 3 scanmode
		packet[UBXPAYLOAD+i++] = 0xFF;													// 4 scanmode
		packet[UBXPAYLOAD+i++] = 0xFF;													// 5 scanmode
		packet[UBXPAYLOAD+i++] = 0xFF;													// 6 scanmode
		packet[UBXPAYLOAD+i++] = 0x7F;													// 7 scanmode

		// send configuration message
		UBXSendPacket(CFGSBAS, i, packet);

			// wait for transmission to finish
			while (usart_sync_is_tx_empty(&USART_0) != 1);
		
		// wait for a UBX message
		i = waitForUBX();

		delay_ms(50);																    // pause a moment
	} while (i != ACKACK);

	// configure heading filtering
	do
	{
		// repeat until we get it right
		i = 0;

#ifdef HDGFILTER
		packet[UBXPAYLOAD+i++] = 0;														// 0  version
		packet[UBXPAYLOAD+i++] = 0;														// 1  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 2  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 3  reserved
		packet[UBXPAYLOAD+i++] = 0x00;													// 4  odo/cog flags - 0 = disable filters, 1 = enable odo filter, 2 = enable cog filter
		packet[UBXPAYLOAD+i++] = 3;														// 5  odo filter = swimming
		packet[UBXPAYLOAD+i++] = 0;														// 6  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 7  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 8  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 9  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 10 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 11 reserved    
		packet[UBXPAYLOAD+i++] = 40;													// 12 cog max speed for filter (x10)
		packet[UBXPAYLOAD+i++] = 2;														// 13 cog max pos acc (2 meter)
		packet[UBXPAYLOAD+i++] = 0;														// 14 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 15 reserved
		packet[UBXPAYLOAD+i++] = 0x7F;													// 16 velocity LPF gain (0..255)
		packet[UBXPAYLOAD+i++] = 0x0F;													// 17 cog LPF gain (0..255)
		packet[UBXPAYLOAD+i++] = 0;														// 18 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 19 reserved
#else
		packet[UBXPAYLOAD+i++] = 0;														// 0  version
		packet[UBXPAYLOAD+i++] = 0;														// 1  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 2  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 3  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 4  odo/cog flags - 0 = disable filters, 1 = enable odo filter, 2 = enable cog filter
		packet[UBXPAYLOAD+i++] = 3;														// 5  odo filter = swimming
		packet[UBXPAYLOAD+i++] = 0;														// 6  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 7  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 8  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 9  reserved
		packet[UBXPAYLOAD+i++] = 0;														// 10 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 11 reserved    
		packet[UBXPAYLOAD+i++] = 40;													// 12 cog max speed for filter (x10)
		packet[UBXPAYLOAD+i++] = 2;														// 13 cog max pos acc (2 meter)
		packet[UBXPAYLOAD+i++] = 0;														// 14 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 15 reserved
		packet[UBXPAYLOAD+i++] = 0x7F;													// 16 velocity LPF gain (0..255)
		packet[UBXPAYLOAD+i++] = 0x0F;													// 17 cog LPF gain (0..255)
		packet[UBXPAYLOAD+i++] = 0;														// 18 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 19 reserved
#endif

		// send configuration message
		UBXSendPacket(CFGODO, i, packet);
    
			// wait for transmission to finish
			while (usart_sync_is_tx_empty(&USART_0) != 1);
		
		// wait for a UBX message
		i = waitForUBX();

		delay_ms(50);																	// pause a moment
	} while (i != ACKACK);
#endif

	// set the nav engine settings NEW NAV5 IMPLEMENTATION 2018/4/9 gas
	do
	{
		// repeat until we get it right
		i = 0;
		packet[UBXPAYLOAD+i++] = 0x05;													// 0 parameter bitmask lower byte, change dynamic platform & fixmode
		packet[UBXPAYLOAD+i++] = 0x00;													// 1 parameter bitmask upper byte
		packet[UBXPAYLOAD+i++] = DYNPLAT;												// 2  dynamic platform,
		packet[UBXPAYLOAD+i++] = 2;														// 3 fixing mode - 3d only
		*(int32_t *) (packet + UBXPAYLOAD + i) = 50000L;								// 4 fixed alt - 2d mode only
		i += 4;
		*(int32_t *) (packet + UBXPAYLOAD + i) = 0L;									// 8 fixed alt variance
		i += 4;
		packet[UBXPAYLOAD+i++] = 0;														// 12 min elevation
		packet[UBXPAYLOAD+i++] = 0;														// 13 dead reckoning max time
		*(uint16_t *) (packet + UBXPAYLOAD + i) = (uint16_t) 250;						// 14 position dop mask
		i += 2;
		*(uint16_t *) (packet + UBXPAYLOAD + i) = (uint16_t) 250;						// 16 time dop mask
		i += 2;
		*(uint16_t *) (packet + UBXPAYLOAD + i) = (uint16_t) 100;						// 18 position accuracy mask
		i += 2;
		*(uint16_t *) (packet + UBXPAYLOAD + i) = (uint16_t) 300;						// 20 time accuracy mask
		i += 2;
		packet[UBXPAYLOAD+i++] = 0;														// 22 static threshold in cm/s, 0 = disable
		packet[UBXPAYLOAD+i++] = 0;														// 23 dgnss timeout
		packet[UBXPAYLOAD+i++] = 0;														// 24 cnothreshold sat number
		packet[UBXPAYLOAD+i++] = 0;														// 25 cnothreshold for fix
		packet[UBXPAYLOAD+i++] = 0;														// 26 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 27 reserved
		packet[UBXPAYLOAD+i++] = 0;														// 28 static hold dist threshold in meters *** TRY FOR DUCKS
		packet[UBXPAYLOAD+i++] = 0;														// 29 static hold
		packet[UBXPAYLOAD+i++] = 0;														// 30 utc standard to use, 0 = automatic
		packet[UBXPAYLOAD+i++] = 0;														// 31 reserved 1
		packet[UBXPAYLOAD+i++] = 0;														// 32 reserved 2
		packet[UBXPAYLOAD+i++] = 0;														// 33 reserved 3
		packet[UBXPAYLOAD+i++] = 0;														// 34 reserved 4
		packet[UBXPAYLOAD+i++] = 0;														// 35 reserved 5
        
		// send configuration message
		UBXSendPacket(CFGNAV5, i, packet);

			// wait for transmission to finish
			while (usart_sync_is_tx_empty(&USART_0) != 1);
		
		// wait for a UBX message
		i = waitForUBX();

		delay_ms(50);																	// pause a moment
	} while (i != ACKACK);  
} // UBXInit

void UBXSendPacket(uint16_t ubxcmd, uint16_t datalen, uint8_t *txbuff)
{
	int16_t	i;
	uint8_t	cka, ckb;

	// first synch char
	txbuff[0] = UBXSYNCH1;
	// second synch char
	txbuff[1] = UBXSYNCH2;
	// class
	txbuff[2] = ubxcmd >> 8;
	// message id
	txbuff[3] = ubxcmd & 0xFF;
	// length lsb
	txbuff[4] = datalen & 0xFF;
	// length msb
	txbuff[5] = datalen >> 8;

	// calculate checksum
	cka = ckb = 0;
	
	for (i = 2; i<(datalen+UBXPAYLOAD); i++)
	{
		cka = cka + txbuff[i];
		ckb = ckb + cka;
	}
	
	// checksum a
	txbuff[datalen+UBXPAYLOAD] = cka;
	// checksum b
	txbuff[datalen+UBXPAYLOAD+1] = ckb;

	// send out the packet
//	usart_sync_s(GPS_UART_SERIAL, txbuff, datalen + UBXPAYLOAD + 2);
	io_write(io_USART_0, (uint8_t *) txbuff, datalen + UBXPAYLOAD + 2);
//	io_write(io_USART_0, (uint8_t *)"Hello World!", 12);
}

// the following routine will parse a ubx message based on the class and message id
// void parseUBX(gpsType *gps, uint8_t * ubxbuff) {  changed for efficiency
void parseUBX(void)
{
	uint8_t		*ubxptr = ubxbuff;	// pointer into ublox message
	gpsType		*gps = &currentGPS;
	int16_t		i;					// counter index
	int32_t		templong;			// scratchpad
	uint16_t	classmsg;			// class and message

	classmsg = (*ubxptr << 8) + *(ubxptr+1);

	// switch on the class/message type
	switch(classmsg)
	{
		case NAVPOSLLH:
		{
			// position in LLH format (degrees)
			int32_t temp;
			ubxptr += 6;			// skip message id and time

			// fill in longitude
			temp = *((int32_t *) ubxptr);
			gps->lon = (double) temp / 10000000.0;
			ubxptr += 4;

			// fill in latitude
			temp = *((int32_t *) ubxptr);
			gps->lat = (double) temp / 10000000.0;
			ubxptr += 4;

			// use mean sea level for alt so skip ht above elipsoid
			ubxptr += 4;

			// fill in altitude
			templong = *(int32_t *) ubxptr;
			ubxptr += 4;

			// convert from mm to meters
			gps->alt = (int16_t) (templong /= 1000L);

			ubxfullset |= 0x04;

		} // NAVPOSLLH
		break;

		case NAVSOL:
		{
			// GPS solution info and time
			ubxptr += 2;			// skip over message id
      
			// get fix information
			 gps->fix = (0x03 == ubxptr[10]);

			// get DOP
			gps->dop = (ubxptr[44] + ((ubxptr[45] & 0xFF) << 8));
  
			// get number of SVs
			 gps->numSVs = ubxptr[47]; // position + 2 chars for message id

			// skip if solution not valid
			if (0x0D != (0x0D & *(ubxptr + 11)))  // 1101 = mask for GPSOK, WEEK OK, TIME OK
			{
				gps->numSVs = 0;
			}
			else
			{
				// good solution
				// fill in time of week in ms
				gps->time = *(int32_t *) ubxptr;
				ubxptr += 4;

#if 0
				// save first good date as starting date
				ptr++;
				
				if (currentGPS.fix && 0xFFFFFFFF == sessstarttime && 0L != *((uint32_t *) ptr) && 0xFFFFFFFF != *((uint32_t *) ptr))
					sessstarttime = *((uint32_t *) ptr);
  
				// keep last good date as ending date
				if (currentGPS.fix && 0L != *((uint32_t *) ptr) && 0xFFFFFFFF != *((uint32_t *) ptr))
					sessendtime = *((uint32_t *) ptr);
#endif

				// skip over fraction
				ubxptr += 4;

				// fill in week number
				gps->weeks = *(int16_t *) ubxptr;
				ubxptr += 2;

#if 0
				// save first good week as starting week
				ptr++;
				
				if (currentGPS.fix && 0xFFFF == sessstartweeks && 0 != *((uint16_t *) ptr) && 0xFFFF != *((uint16_t *) ptr))
					sessstartweeks = *((uint16_t *) ptr);
  
				// keep last good week as ending week
				if (currentGPS.fix && 0 != *((uint16_t *) ptr) && 0xFFFF != *((uint16_t *) ptr))
					sessendweeks = *((uint16_t *) ptr);
#endif
			} // if 

			// mark that we got a navsol even if we didn't use it
			ubxfullset |= 0x02;
		} // NAVSOL
		break;

		case NAVVELNED:
		{
			// GPS velocities
			ubxptr += 22;			// skip over message id and time, n, e, d vel, 3d speed

			// skip if solution not valid
			if (!(gps->fix))
			{
				// 1101 = mask for GPSOK, WEEK OK, TIME OK
				gps->numSVs = 0;
				gps->vel = 0;
				gps->hdg = 0;
			} // if
			else
			{      
				// get ground speed in cm/s
				templong = *(uint32_t *) ubxptr;
				// convert to mph
				gps->vel = (uint16_t) ((float) templong / 5280.0 / (12.0*2.54) * 3600.0);

				ubxptr += 4;
				// get heading in degrees
				templong = *(int32_t *) ubxptr;
				gps->hdg = (int) NORMALIZE_DEG((templong / 10000));
			} // else
			
			ubxfullset |= 0x01;
		} // NAVVELNED
		break;

#ifdef UBLOX8
		case NAVPVT:
		{
			// Ublox 7 and 8 only - universal single message
			ubxptr += 2;  // skip over message id

			// get fix information
			gps->fix = (0x03 == ubxptr[20]);
			// get DOP
			gps->dop = *((uint16_t *) (76 + ubxptr));

			// skip if solution not valid
			if (!(gps->fix))
			{
				// 1101 = mask for GPSOK, WEEK OK, TIME OK
				gps->numSVs = 0;
				gps->vel = 0;
			} // if
			else
			{
				// good solution
				gps->numSVs = ubxptr[23];
				// fill in time of week in ms
				gps->time = *(int32_t *) ubxptr;

#if 0
				// save first good date as starting date
				ptr++;
				
				if (currentGPS.fix && 0xFFFFFFFF == sessstarttime && 0L != *((uint32_t *) ptr) && 0xFFFFFFFF != *((uint32_t *) ptr))
					sessstarttime = *((uint32_t *) ptr);

				// keep last good date as ending date
				if (currentGPS.fix && 0L != *((uint32_t *) ptr) && 0xFFFFFFFF != *((uint32_t *) ptr))
					sessendtime = *((uint32_t *) ptr);
#endif

				gps->year = *(uint16_t *) (4 + ubxptr);
				gps->month = *(uint8_t *) (6 + ubxptr);
				gps->day = *(uint8_t *) (7 + ubxptr);
				gps->hour = *(uint8_t *) (8 + ubxptr);
				gps->min = *(uint8_t *) (9 + ubxptr);
				gps->sec = *(uint8_t *) (10 + ubxptr);
				gps->msec = (uint16_t) ((*(uint32_t *) ubxptr) / 1000000L);			// convert from nanoseconds to milliseconds
				gps->lon = ((double) *(int32_t *) (24 + ubxptr)) / 10000000.0;
				gps->lat = ((double) *(int32_t *) (28 + ubxptr)) / 10000000.0;
				gps->alt = (int16_t) ((*(int32_t *) (36 + ubxptr)) / 1000L);	    // convert from mm to meters
				gps->vel = (uint16_t) *((uint32_t *) (60 + ubxptr)) / 10L; 			// convert from mm/s to cm/s
				gps->hdg = NORMALIZE_DEG((*(int32_t *) (64 + ubxptr)) / 100000L);	// convert from 1e5 scaling to degrees and normalize

				// fill in GPS week number (since 1908) - not furnished in NAV-PVT
				gps->weeks = doweeks(gps->day, gps->month, gps->year);
				
#if 0
				// save first good week as starting week
				ptr++;
				if (currentGPS.fix && 0xFFFF == sessstartweeks && 0 != *((uint16_t *) ptr) && 0xFFFF != *((uint16_t *) ptr))
					sessstartweeks = *((uint16_t *) ptr);

				// keep last good week as ending week
				if (currentGPS.fix && 0 != *((uint16_t *) ptr) && 0xFFFF != *((uint16_t *) ptr))
					sessendweeks = *((uint16_t *) ptr);
#endif

			} // else

			// NAV-PVT contains everything so mark that we got a complete set of data
			ubxfullset = 0x07;
		} // NAVPVT
		break;
#endif // UBLOX8

		default:
			// breakpoint trap
			i = 1;
			break;
	} // switch

	// dead reckoning
	if (0x07 == ubxfullset)
	{
		// got all 3 messages
		// clear message list (can also be done at gpspulse)
		ubxfullset = 0;

		// if no gpslock then interpolate
		if (!currentGPS.fix)
		{
#if 0
			// no interpolation      
			// copy the last one for a start
			*gps = previousGPS[lastgps];
    
			// adjust the time by the GPS sample rate
			// NOTE: does not compensate for event of data loss at week boundary
			gps->time = gps->time + (1000 / GPS_RATE);

			// don't interpolate if not enough good history
			if (0L != previousGPS[(lastgps+GPS_SAMPLES-1) % GPS_SAMPLES].time)
			{  
				// interpolate the lat and lon. factor in gps recording freq
				gps->lat = gps->lat + (previousGPS[lastgps].lat - previousGPS[(lastgps+GPS_SAMPLES-1) % GPS_SAMPLES].lat);
				gps->lon = gps->lon + (previousGPS[lastgps].lon - previousGPS[(lastgps+GPS_SAMPLES-1) % GPS_SAMPLES].lon);
			} // if
#endif
		} // if
    
		// store latest gps for historical purposes
		lastgps = (lastgps + 1) % GPS_SAMPLES;		// add to circular buffer
		previousGPS[lastgps] = *gps;				// overwrite with latest
	} // if
} // parseUBX

// UBX binary protocol
void checkUbloxGPS()
{
	static enum ubx_state
	{
		FIND_START1,			// looking for first synch char
        FIND_START2,			// looking for second synch char
        GET_CLASS,				// found synch so get the class and verify
        GET_MSGID,				// get the message id
        GET_LEN1,				// looking for LSB of payload length
        GET_LEN2,				// looking for MSB of payload length
        CAPTURE_PACKET,			// capturing data in buffer
        GET_CKA,				// verify checksum A
        GET_CKB					// verify checksum B
	} state = FIND_START1;

	static uint8_t ubxcount;	// where we are in the message
	static uint16_t len;        // length of buffer payload
	static uint8_t cka, ckb;    // A and B checksums
	uint8_t tempch;

	for (int icnt = 0; icnt < UBXRXSIZE; icnt++)
	{
		tempch = gps_usart_buffer[!gps_next_buffer][icnt];
		
		switch (state)
		{
			case FIND_START1:
				if (UBXSYNCH1 == tempch) state = FIND_START2;
				break;
			
			case FIND_START2:
				if (UBXSYNCH2 == tempch)
					state = GET_CLASS;
				else
					state = FIND_START1;
				break;
				
			case GET_CLASS:
				cka = ckb = 0;
				ubxcount = 0;
				ubxbuff[ubxcount] = tempch;
				cka = cka + tempch;
				ckb = ckb + cka;
				state = GET_MSGID;
				break;
				
			case GET_MSGID:
				ubxbuff[++ubxcount] = tempch;
				cka = cka + tempch;
				ckb = ckb + cka;
				state = GET_LEN1;
				break;
				
			case GET_LEN1:
				len = tempch;
				cka = cka + tempch;
				ckb = ckb + cka;
				state = GET_LEN2;
				break;
				
			case GET_LEN2:
				len += tempch << 8;
				
				if (len > (UBXRXSIZE-2))
				{
					// bad synch, try again
					state = FIND_START1;
				}
				else
				{
					cka = cka + tempch;
					ckb = ckb + cka;
					
					if (0 == len)
						state = GET_CKA;
					else
						state = CAPTURE_PACKET;
				} // else
				break;
				
			case CAPTURE_PACKET:
				ubxbuff[++ubxcount] = tempch;
				cka = cka + tempch;
				ckb = ckb + cka;
				if (ubxcount == (len+1)) state = GET_CKA;
				break;
				
			case GET_CKA:
				if (cka == tempch)
					state = GET_CKB;
				else
					state = FIND_START1;
				break;
				
			case GET_CKB:
				state = FIND_START1;
				
				if (ckb == tempch)
				{
					parseUBX();
					return;								// do one UBX message at a time to allow other processing
				} // if
				break;
		} // switch
	} // while
} // checkUbloxGPS
