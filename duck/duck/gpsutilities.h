#ifndef GPSUTILITIES_H
#define GPSUTILITIES_H

#define GMTOFFSET   -5   // -4 = EST, -5 = EDT

// This is the control type used for formattime
#define SHORTDANDT  0
#define LONGDANDT   1
#define SHORTTIME   2
#define HDGSPEED    3
#define GPSCSVLINE  4

#define MAXTIME		(7L*24L*60L*60L*1000L)    // 604,800,000 maximum time value
#ifndef PI
#define PI			3.14159265358979323846
#endif

#define DEGREES_TO_RADIANS(x) ((double) x * PI / 180.0)
#define RADIANS_TO_DEGREES(x) ((double) x * 180.0 / PI)
#define D2R(x) DEGREES_TO_RADIANS(x)
#define R2D(x) RADIANS_TO_DEGREES(x)

#define NORMALIZE_DEG(x) ((x + 360) % 360)

#define CIRCUMKM	40455.0                             // circumference of earth in km
#define CIRCUMKFT	(CIRCUMKM * 39.0/12.0)              // circumference of earth in kilofeet
#define LATFT		(1000.0/360.0*CIRCUMKFT)            // ft per degree of latitude
#define LONFT(B)	(LATFT*abs(cos((B)/57.29577951)))	// ft per degree of longitude, B must be current latitude in degrees

//float	distance(double, double, double, double, char);
int32_t distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, char unit);
int16_t bearing(double, double, double, double);
int32_t deg2rad(int32_t deg);
int32_t rad2deg(int32_t rad);

char		*f2a(float, uint8_t, uint8_t);
void		dodate(int16_t *, int16_t *, int16_t *, int16_t);
uint16_t	doweeks(uint8_t , uint8_t , uint16_t );
void		formattime (uint32_t, uint16_t, uint32_t, int8_t, char, char *);
void		formatgps(gpsType *, char *, char , uint32_t);

#endif
