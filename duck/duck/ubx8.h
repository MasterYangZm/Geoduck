//*****************************************************************************           
//                                                                                        
// These are the definitions for the UBX Messaging Protocol
// GAStephens 11-15-2004
// update for Ublox 6 2-2-2019, removed byte packing - no longer Traqmate compatible
// update for Ublox 8 7-1-2018
//
//*****************************************************************************

#ifndef UBX_H
#define UBX_H

#define GPS_SAMPLES					3

#define UBLOX8								// which gps chip are we using
#undef UBLOX6

#define USE_INTERPOLATION	0				// set to 1 to fill in missing GPS samples
#define SAMPS_PER_SEC       1				// GPS samples per second

// 0 = portable, 2 = stationery, 3 = pedestrian, 4 = car, 5 = sea, 6 = airborne <1g, 7 = airborne <2g, 8 = airborne <4g
#define DYNPLAT				4				// dynamic platform setting
#define EXTANT				1				// set to 1 to enable power to external antenna, 0 for passive antenna

#define GPS_RATE			SAMPS_PER_SEC	// GPS reporting rate in Hertz
#define GPS_MS				(1000/GPS_RATE)	// number of ms between GPS measurements
#define GPS_GOOD			3				// must have 3 satellites to be "in coverage"

// the pulse triggers the data processing isr on rising edge but also drives the gps module pps led
#define GPS_PULSE_WID		10				// GPS pulse width in microseconds (or 10 works)
#define GPS_LOCK_PULSE_WID	GPS_PULSE_WID   // GPS pulse width in microseconds time locked to GNSS
#define PULSE_DELAY			0				// delay pulse by this amount

#define UBXINITBAUD			9600L			// Starting baud rate in bps for Ublox GPS chip
#define UBXBAUD				57600L			// Baud rate to change Ublox for polling in bps

// ubx buffer sizes
#define	UBXTXSIZE			64				// max length of UBX transmit packet
#define UBXMAXRXSIZE		220				// max length of UBX receive packet NAV-SVINFO
#define UBXRXSIZE			160				// practical length of UBX receive packets (must be enough for all messages received in a sampling period)

#define GPSRXSIZE			UBXRXSIZE
#define GPSTXSIZE			UBXTXSIZE

// Useful Macros
#define MIN(x,y)			(((x)<(y))?(x):(y))
#define MAX(x,y)			(((x)>(y))? (x):(y))
#define ABS(a)				((a >= 0)? (a) : -(a))
#define FLOATABS(a)			((a >= 0.0)? (a) : -(a))
#define AVG(q,r)			(((q)+(r))/2)
#define tohex(c)			(c<10? (c+'0'):(c+'A'-10))
#define SWAP16(x)			( (((x) >> 8) & 0x00FF) | (((x) << 8) & 0xFF00) )
#define SWAP32(x)			( (((x) >> 24) & 0x000000FF) | (((x) >> 8) & 0x0000FF00) | \
							  (((x) << 8) & 0x00FF0000) | (((x) << 24) & 0xFF000000) )

typedef struct
{
	uint32_t time;			// 4 in milliseconds since midnight on Sunday
	uint16_t weeks;			// 2 gps number of weeks, week 0 started Jan 6, 1980
	double lat;				// 4 decimal degrees
	double lon;				// 4 decimal degrees
	int16_t alt;			// 2 altitude in meter (- = below sea level)
	uint16_t vel;			// 2 ground speed in cm/s
    int16_t hdg;			// 2 heading in degrees
    int16_t dop;			// 2 dillution of precision * 100
    int8_t numSVs;			// 1 number of satellites
    bool fix;				// 1 true if we have a good gps position
	
#ifdef UBLOX8
	uint16_t year;			// 2 4 digit year
	uint8_t month;			// 1
	uint8_t day;			// 1
	uint8_t	hour;			// 1
	uint8_t min;			// 1
	uint8_t sec;			// 1
	uint16_t msec;			// 1 milliseconds
#endif
} gpsType;				  	// 32 total 

// use these structures to access the individual sample groups within a flash page
typedef struct
{
    uint16_t x;				// 2
    uint16_t y;				// 2
    uint16_t z;				// 2
} acceltype;				// total 6

typedef struct
{							// ***** DO NOT CHANGE STRUCTURE. CODE DEPENDENT!! *****
	gpsType gps;			// 22
	acceltype accel[10];	// 6*10 = 60
} samptype10;				// 82 total bytes

typedef struct
{							// ***** DO NOT CHANGE STRUCTURE. CODE DEPENDENT!! *****
	gpsType gps;			// 20
	acceltype accel[5];		// 6*5 = 30
} samptype5;				// 52 total bytes

#define UBXPAYLOAD			6		// payload starts on 6th byte

// constants
#define UBXSYNCH1			0xB5	// first message synch byte
#define UBXSYNCH2			0x62	// second message synch byte

// class definitions
#define NAV					0x01	// navigational
#define RXM					0x02	// reciever manager
#define INF					0x04	// informative
#define UBXACK				0x05	// ack - nak
#define CFG					0x06	// configuration
#define UPD					0x09	// sw updates
#define MON					0x0A	// monitor
#define AID					0x0B	// navigation aiding
#define USR					0x4F	// mask, actually 0x4x, user messages
#define TIM					0x0D	// timing

// commands
#define NAVPOSECEF			0x0101  // position in ECEF format
#define NAVPOSLLH			0x0102  // position in LLH format
#define NAVPOSUTM			0x0108  // position in UTM format
#define NAVDOP				0x0104  // dillution of precision
#define NAVSTATUS			0x0103  // navigation status
#define NAVSOL				0x0106  // multipart message, solution, time, accuracy
#define NAVPVT				0x0107  // new multipart message - all info
#define NAVVELECEF			0x0111  // velocity solution in ECEF
#define NAVVELNED			0x0112  // velocity solution in NED
#define NAVTIMEGPS			0x0120  // GPS time
#define NAVTIMEUTC			0x0121  // UTC time
#define NAVCLOCK			0x0122  // clock solution
#define NAVSVINFO			0x0130  // space vehicle information
#define NAVDGPS				0x0131  // DGPS corrected solution
#define NAVSBAS				0x0132  // status of SBAS subsystem
#define NAVEKFSTATUS		0x0140  // dead reckoning status info
#define NAVAOPSTATUS		0x0160  // assistnow autonomous status

#define ACKACK				0x0501  // acknowledgement
#define ACKNAK				0x0500  // negative acknowledgement

#define CFGPRT				0x0600  // get/set port configuration
#define CFGMSG				0x0601  // get/set message configuration
#define CFGNMEA				0x0617  // get/set NMEA protocol config
#define CFGRATE				0x0608  // get/set navigational solution rate
#define CFGCFG				0x0609  // clear, save, and load configurations
#define CFGTP				0x0607  // get/set timepulse configuration
#define CFGNAV				0x0603  // get/set nav engine config
#define CFGDAT				0x0606  // get/set datum
#define CFGINF				0x0602  // get/set informational message config
#define CFGRST				0x0604  // reset receiver
#define CFGRXM				0x0611  // get/set receiver sensitivity config
#define CFGANT				0x0613  // get/set antenna control settings
#define CFGFXN				0x060E  // get/set FixNow config
#define CFGSBAS				0x0616  // get/set SBAS config
#define CFGTM				0x0610  // get/set timemark config
#define CFGEKF				0x0612  // get/set dead reckoning config
#define CFGNAV2				0x061A  // get/set nav engine config (antaris 4)
#define CFGODO				0x061E  // get/set odometer settings (heading filtering)
#define CFGNAVX5			0x0623  // get/set nav engine config, expert (ublox 5)
#define CFGNAV5				0x0624  // get/set nav engine modes (ublox 5)
#define CFGUDOC				0x0625  // undocumented message for Ublox5 V4.00
#define CFGESFGWT			0x0629  // get/set gyro+wheel tick sol
#define CFGITFM				0x0639  // jamming interference monitor config
#define CFGPM				0x0632  // power mgt config
#define CFGPM2				0x0638  // extended power mfg config
#define CFGTP5				0x0631  // get/set timepulse parameters

#define MONEXCEPT			0x0A05  // exception dump
#define MONHW				0x0A09  // hw status
#define MONIO				0x0A02  // I/O subsystem status
#define MONIPC				0x0A03  // IPC sybsystem status
#define MONMSGPP			0x0A06  // message parse/process status
#define MONRXBUF			0x0A07  // receive buffer status
#define MONSCHD				0x0A01  // system scheduler status
#define MONTXBUF			0x0A08  // transmit buffer status
#define MONVER				0x0A04  // receiver and sw version

#define INFERROR			0x0400  // ascii error string
#define INFWARNING			0x0401  // ascii warning string
#define INFNOTICE			0x0402  // ascii information string
#define INFTEST				0x0403  // ascii test output string
#define INFDEBUG			0x0404  // ascii string denoting debug output

// function prototypes
void UBXInit(void);
void UBXBaudChange(void);
void UBXSendPacket(uint16_t, uint16_t, uint8_t *);
void checkUbloxGPS(void);
void parseUBX(void);

#endif
