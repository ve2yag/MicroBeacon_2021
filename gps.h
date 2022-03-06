/*
 * nmea.h
 *
 *  Created on: 3 dec. 2017
 *      Author: remib
 */

#ifndef NMEA_H_
#define NMEA_H_

#include <stdint.h>

// NAV MODE
//----------------
#define NAV_PORTABLE   0x00   // (For portable device)
#define NAV_TIMING     0x02   //Stationary Mode (For timing purpose)
#define NAV_PEDESTRIAN 0x03   //Pedestrian Mode    
#define NAV_AUTOMOTIVE 0x04   //Automotive Mode
#define NAV_SEAMODE    0x05   //Sea Mode           
#define NAV_AIRBORNE1G 0x06   //Airborne < 1G Mode (Max vertical velocity 100 m/s)
#define NAV_AIRBORNE2G 0x07   //Airborne < 2G Mode (Max vertical velocity 250 m/s)
#define NAV_AIRBORNE4G 0x08   //Airborne < 4G Mode (Max vertical velocity 500 m/s)

typedef struct nav_t {
	uint8_t hour,minute,second, month, day;
	float latitude, longitude;
	float altitude;			// In meter
	uint8_t sat_for_fix, sat_in_sky;
	uint16_t speed;			// In km/h
	uint16_t course, year;
	char fix;				// -1=timeout, 0=no fix, >=1 fix
} nav_t;

extern nav_t nav;

int natoi(char *s, int n);
unsigned int htoi (char *ptr);
float fix_position(float Position);

char GPS_init(uint8_t navMode);    // Initialize GPS, 0=success, -1=error
char GPS_setnav(uint8_t navMode);  // Select navagation mode, 0=success, -1=error
void GPS_powerdown();              // Put GPS in sleep mode, keep data.
char GPS_powerup();                // Power up GPS and turn antenna bias ON

int  GPS_poll();				           // Process NMEA string, return TRUE if more data to process

#endif /* NMEA_H_ */
