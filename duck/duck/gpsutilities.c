// generally useful math and formatting functions
// portions copyright 2004 - 2018 Traqmate, LLC
// 8/3/2004 created for traqmate displayunit
// 2/8/2018 converted for flagger
// Author: GAStephens
//
// formats and uploads the contents of the dataflash
// part of traqmate.c

#include <atmel_start.h>
#include <math.h>
#include <string.h>
#include "ubx8.h"
#include "gpsutilities.h"

// gps is a structure with all the relevant gps data
// msec is the tenth of second number
// textline is a buffer large enough to hold the expected formatted string
// linenum is 0 if all data is required or 1,2,3 for respective 20 int8_t  lines
// linenum is 4 for a complete .CSV recording line
//
void formatgps(gpsType *gps, char *textline, char units, uint32_t linenum)
{
	int16_t speed;

	if (units == 'M' || units == 'm')
	{
		// Metric
		// Convert from cm/s to km / hour
		speed = (uint16_t) ((float) gps->vel * 3600.0/100000.0);
	}
	else
	{
		// Standard english
		// Convert from cm/s to miles / hour
		speed = (uint16_t) ((float) gps->vel * (3600.0/25.4/5280.0));
	}

	if (linenum <= LONGDANDT) formattime(gps->time, gps->weeks, LONGDANDT, 0, ' ', textline);	// GMT offset 0

	if (linenum == SHORTDANDT)
	{
		strcat(textline, " ");
		textline += strlen(textline);
	}

	if (linenum == SHORTDANDT || linenum == SHORTTIME)
	{
		// Location
		sprintf(textline, "%3.6f,%3.6f,", (float) gps->lat, (float) gps->lon);
		textline += strlen(textline);

		// Do altitude in proper units
		if (units == 'M' || units == 'm')
			sprintf(textline, "%dm", (int16_t) gps->alt);						// Metric
		else
			sprintf(textline, "%d'", (int16_t) (((int32_t) gps->alt*39L)/12L));	// English

		textline += strlen(textline);
	}

	if (linenum == SHORTDANDT)
	{
		strcat(textline, " ");
		textline += strlen(textline);
	}

	if (linenum == SHORTDANDT || linenum == HDGSPEED)
	{
		// Do not write heading if we are not moving
		if (speed != 0)
			sprintf(textline, "H%03d, ", gps->hdg / 100);	// Heading
		else
			sprintf(textline, "H---, ");					// No heading

		textline += strlen(textline);
		sprintf(textline, "V%02d, ", speed);	// Velocities

		textline += strlen(textline);
		sprintf(textline, "S%d, ", (int16_t) gps->numSVs);	// Number of satellites

		textline += strlen(textline);
		sprintf(textline, "D%d", (int16_t) gps->dop);		// Dilution of precision
	}

	// Date,Time,Lat,Lon,Alt(ft),Vel(mph),Hdg(deg),Sats,DOP*100,Bat Voltage
	// 2018-04-06,14:48:13,+LLL.FFFFFF,+LLL.FFFFFF,AAA,VVV,SSS,DDD,B.BB

	if (linenum == GPSCSVLINE)
	{
		// .CSV
		formattime(gps->time, gps->weeks, LONGDANDT, -4, ',', textline);	// GMT offset -4 = EDT
		textline += strlen(textline);

		// Location
		sprintf(textline, ",%3.6f,%3.6f", (float) gps->lat / 10000000.0, (float) gps->lon / 10000000.0);
		textline += strlen(textline);

		// Do altitude in proper units
		if (units == 'M' || units == 'm')
			sprintf(textline, ",%d", (int16_t ) gps->alt);							// Metric
		else
			sprintf(textline, ",%d", (int16_t )(((int32_t) gps->alt * 39L)/12L));	// English

		textline += strlen(textline);

		// Velocity
		sprintf(textline, ",%d", speed);
		textline += strlen(textline);

		// Heading
		if (speed != 0)
			sprintf(textline, ",%03d", gps->hdg / 100);	// Heading
		else
			sprintf(textline, ",000");					// No heading

		textline += strlen(textline);

		// Number of satellites
		sprintf(textline, ",%d", (int16_t) gps->numSVs);
		textline += strlen(textline);

		// Dilution of precision
		sprintf(textline, ",%d", (int16_t) gps->dop);
		textline += strlen(textline);
	}
}

// Time and weeks are gps format
// textline is a buffer large enough to hold the expected formatted string
void formattime(uint32_t time, uint16_t weeks, uint32_t whattodo, int8_t gmtoffset, char separator, char *textline)
{
	int16_t day, mon, dow;
	int16_t year, days;
	int16_t hour, min, sec;
	uint32_t secs;
	uint16_t msec;

	if (weeks == 0)
	{
		// Bad GPS time
		day = mon = year = hour = min = sec = 0;
	}
	else
	{
		int32_t  newtime;

		// Add on offset in milliseconds
		newtime = (int32_t) time +  ((int32_t) gmtoffset * (60L * 60L * 1000L));

		// Handle week boundaries
		if (newtime < 0)
		{
			// Moved into last week
			weeks--;
			newtime += MAXTIME;
		}
		else if (newtime > MAXTIME)
		{
			// Moved into next week
			weeks++;
			newtime -= MAXTIME;
		}

		time = (uint32_t) newtime;

		// Get seconds since Sunday midnight
		secs = time/1000;
		dow = secs/60/60/24;
		hour = (secs/3600) % 24;
		min = (secs/60) % 60;
		sec = secs % 60;
		msec = (uint16_t) (time % 1000L);

		// Number days from GPS start Jan 6, 1980 to Jan 1, 2002 = 8030
		days = (weeks*7) + dow - 8030;	// Days since Jan 1, 2002
		dodate (&day, &mon, &year, days);
	}

	switch (whattodo)
	{
		case SHORTDANDT:
			sprintf(textline, "%02d/%02d", mon, day);												// Date
			sprintf(textline + strlen(textline), "%02d:%02d:%02d", hour, min, sec);					// Time
			break;

		case LONGDANDT:
			sprintf(textline, "%04d-%02d-%02d%c", year, mon, day, separator);						// Date
			sprintf(textline + strlen(textline), "%02d:%02d:%02d.%02d", hour, min, sec, msec/10);	// Time
			break;

		case SHORTTIME:
			sprintf(textline, "%02d:%02d", hour, min);												// Time
			break;
	}
}

// Day 1 = Jan 1, 2002
// returns day of month, month (jan = 1), and year
void dodate(int16_t *day, int16_t *mon, int16_t *year, int16_t days)
{
	uint8_t calendar[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

	*year = 2002;

	while (days > 365 )
	{
		// Fix leap year
		if (!(*year%4) && days == 366) break;
		days -= ((*year % 4)? 365 : 366);
		++*year;
	}

	if (!(*year%4)) calendar[1] = 29;	// Fix leap year

	*mon = 0;

	while (days > calendar[*mon])
	{
		days -= calendar[*mon];
		++*mon;
	}

	++*mon;
	*day = days;
}

// Converts day mon year to gps epoch weeks (since jan 6, 1980)
uint16_t doweeks(uint8_t day, uint8_t mon, uint16_t year)
{
	uint8_t calendar[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	uint32_t daycount = 360;							// Days in 1980 starting Jan 6th.

	if ((year%4) && (mon > 2)) daycount++;				// If leap year and already past feb add a day
	daycount += day;									// Add on the days into this month

	// add on the days this year up to last month
	mon--;
	for (; mon > 0; mon--) daycount += calendar[mon-1];	// Calendar is zero indexed, mon is 1 indexed

	--year;												// Start with last year

	// Add on the days in the previous years
	while (year > 1980)
	{
		if (year%4)						  // Fix leap year
			daycount += 365;
		else										// Evenly divisible by 4 so leap year
			daycount += 366;
		--year;
	}

	return (daycount/7);								// Figure out how many weeks that has been
}

/*----------------------------------------------------------------------------*/
/* f2a() - convert x to string with l places left of decimal point and r on   */
/*         the right. Returns a pointer to the string in an internal buffer.  */
/*         Written by Morris Dovey (mrdovey@iedu.com)                         */
/*----------------------------------------------------------------------------*/
char *f2a(float x, uint8_t l, uint8_t r)
{
	static uint8_t buffer[16];
	uint8_t        digits = 0, *p, q = r, sign = x < 0;
	float          fract, round;
	uint32_t       whole;

	if (sign) x = -x;
	for (round = 0.5; q--; round /= 10);
	x += round;
	whole = x;
	fract = x - whole;
	do ++digits; while (whole /= 10);
	whole = x;
	digits += sign;
	if (l > digits) digits = l;
	p = buffer + digits;
	do *--p = whole % 10 + '0'; while (whole /= 10);
	if (sign) *--p = '-';
	while (p != buffer) *--p = ' ';
	p = buffer + digits;
	*p++ = '.';

	while (r--)
	{
		fract *= 10;
		whole = fract;
		fract -= whole;
		*p++ = whole + '0';
	}

	*p = '\0';
	return (char *) buffer;
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::                                                                         :*/
/*::  This routine calculates the distance between two points (given the     :*/
/*::  latitude/longitude of those points). It is being used to calculate     :*/
/*::  the distance between two locations using GeoDataSource(TM) products.   :*/
/*::                                                                         :*/
/*::  Definitions:                                                           :*/
/*::    South latitudes are negative, east longitudes are positive           :*/
/*::                                                                         :*/
/*::  Passed to function:                                                    :*/
/*::    lat1, lon1 = Latitude and Longitude of point 1 (in decimal degrees)  :*/
/*::    lat2, lon2 = Latitude and Longitude of point 2 (in decimal degrees)  :*/
/*::    unit = the unit you desire for results                               :*/
/*::           where: 'M' is statute miles (default)                         :*/
/*::                  'K' is kilometers                                      :*/
/*::                  'N' is nautical miles                                  :*/
/*::                                                                         :*/
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
#define PI 3.14159265358979323846

int32_t distance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, char unit)
{
	int32_t theta, dist;

	theta = lon1 - lon2;
	dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
	dist = acos(dist);
	dist = rad2deg(dist);
	dist = dist*60*1.1515;

	switch(unit)
	{
		case 'M':
		case 'm':
			break;

		case 'K':
		case 'k':
			dist = dist*1.609344;
			break;

		case 'N':
		case 'n':
			dist = dist*0.8684;
			break;
	}

	return (dist);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
int32_t deg2rad(int32_t deg)
{
	return (deg*PI/180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
int32_t rad2deg(int32_t rad)
{
	return (rad*180/PI);
}
