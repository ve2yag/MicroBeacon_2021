/*
 * nmea.cpp
 *
 *  Created on: 3 dec. 2017
 *      Author: remib
 */
#include <Arduino.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <math.h>

#include "gps.h"

nav_t  nav;		// Navigation data

/* UBX PROTOCOL */
// CLASS
#define CLASS_NAV 0x01
#define CLASS_RXM 0x02
#define CLASS_INF 0x04
#define CLASS_ACK 0x05
#define CLASS_CFG 0x06
#define CLASS_MON 0x0A
#define CLASS_AIB 0x0B
#define CLASS_TIM 0x0D
#define CLASS_ESF 0x10

// CFG ID
#define CFG_ANT  0x13
#define CFG_MSG  0x01
#define CFG_PRT  0x00
#define CFG_RATE 0x08
#define CFG_NAV5 0x24

// RXM ID
#define RXM_PMREQ 0x41

char aprs_lat[6];	// Extract latitude from NMEA to aprs beacon

/******************************************************
 * FixPosition
 * 
 * Correct latitude and longitude from atof() or nmea
 ******************************************************/
float fix_position(float Position) {
	float Minutes, Seconds;

	Position = Position / 100;
	Minutes = trunc(Position);
	Seconds = fmod(Position, 1);

	return Minutes + Seconds * 5 / 3;
}


/******************************************************
 * natoi
 * 
 * Convert ASCII to integer, with digit count limit.
 ******************************************************/
int natoi(char *s, int n) {
    int x = 0;
    while(isdigit(s[0]) && n--) {
        x = x * 10 + (s[0] - '0');
        s++;
    }
    return x;
}


/******************************************************
 * htoi
 * 
 * Convert ASCII hex to integer.
 ******************************************************/
unsigned int htoi (char *ptr) {
	unsigned int value = 0;
	char ch = *ptr;

    while (ch == ' ' || ch == '\t')
        ch = *(++ptr);

    for (;;) {

        if (ch >= '0' && ch <= '9')
            value = (value << 4) + (ch - '0');
        else if (ch >= 'A' && ch <= 'F')
            value = (value << 4) + (ch - 'A' + 10);
        else if (ch >= 'a' && ch <= 'f')
            value = (value << 4) + (ch - 'a' + 10);
        else
            return value;
        ch = *(++ptr);
    }
}


/******************************************************
 * next_field
 * 
 * Return point to next comma separated field.
 ******************************************************/
char *next_field(char *str) {
	while(*str!=0) if(*(str++)==',') break;
	return str;
}

/******************************************************
 * next_fields
 * 
 * Skip more than 1 field.
 ******************************************************/
char *next_fields(char *str, int n) {
	while(n--) str = next_field(str);
	return str;
}


/******************************************************
 * skip_char
 * 
 * Increase pointer of n caracter or up to end of string.
 ******************************************************/
char *skip_char(char *str, int n) {
	while(*str!=0 && n--) str++;
	return str;
}


/******************************************************
 * nmea_valid
 * 
 * Test NMEA checksum of string.
 ******************************************************/
int nmea_valid(char *str) {
	unsigned char checksum=0;
	if(*(str++)!='$') return 0;
	while(*str!='*' && *str!=0) checksum^=*(str++);
	return (checksum==htoi(++str));
}


/******************************************************
 * nmea_parse
 * 
 * Convert NMEA string to struct.
 ******************************************************/
int nmea_parse(char *str) {
	uint8_t i;

	/* TEST STRING AND SKIP $ */
	if(nmea_valid(str)==0) return 0;
	str++;
	
	/* GPGGA PARSER */
	if(strncmp(str, "GPGGA", 5)==0) {
		str = next_field(str);
		nav.hour    = natoi(str, 2);
		str = skip_char(str, 2);
		nav.minute  = natoi(str, 2);
		str = skip_char(str, 2);
		nav.second  = natoi(str, 2);
		str = next_field(str);
		for(i=0; i<4; i++) aprs_lat[i] = str[i];	  // APRS hack to keep latitude as string.
		for(i=4; i<6; i++) aprs_lat[i] = str[i+1];
		nav.latitude  = fix_position(atof(str));
		str = next_field(str);
		if(*str=='S') nav.latitude = -nav.latitude;
		str = next_field(str);
		nav.longitude  = fix_position(atof(str));
		str = next_field(str);
		if(*str=='W') nav.longitude = -nav.longitude;
		str = next_field(str);
		nav.fix = atoi(str);
		str = next_field(str);
		nav.sat_for_fix = atoi(str);
		str = next_fields(str, 2);
		nav.altitude = atof(str);
		return 1;
	}
	
	/* GPRMC PARSER */
	if(strncmp(str, "GPRMC", 5)==0) {
		str = next_fields(str, 7);
		nav.speed = (uint16_t)(atof(str)*185/100);
		str = next_field(str);
		nav.course = atoi(str);
		str = next_field(str);
		nav.day    = natoi(str, 2);
		str = skip_char(str, 2);
		nav.month  = natoi(str, 2);
		str = skip_char(str, 2);
		nav.year   = natoi(str, 2) + 2000;
		return 1;
	}
	
	/* GPGSV PARSER */
	if(strncmp(str, "GPGSV", 5)==0) {
		str = next_fields(str, 3);
		nav.sat_in_sky = atoi(str);
		return 1;
	}
}
 
 
/***************************************************************************
 * getUBX - Get UBX packet with timeout, Sync caracter is removed.
 * 
 * Return: 0 on success, -1 on timeout or error.
 **************************************************************************/
uint8_t getUBX(uint8_t *UBXmsg, uint8_t maxLength) {
    uint8_t incoming_char, index = 0, seq = 0;
    unsigned long ackWait = millis();

    while (1) {
        if (Serial.available()) {

            /* GET SYNC OR PACKET CONTENT */
            incoming_char = Serial.read();
            switch(seq) {
                case 0:   if(incoming_char==0xB5) seq++; break;
                case 1:   if(incoming_char==0x62) seq++; else seq--; break;
                case 2:   UBXmsg[index++] = incoming_char; break;
            }

            /* BREAK WHEN BUFFER ARE FULL */
            if(index==maxLength) break;
        }

        /* END OF PACKET (CLASS+ID+SIZE+PAYLOAD+CKSUM) */
        if(index>=4) {
          if (index >=  6 + (UBXmsg[2] + (UBXmsg[3]*256)) ) break;
        }
        
        /* CHECK FOR TIMEOUT */
        if ((millis() - ackWait) > 1500) return -1;

    }

    /* COMPUTER CHECKSUM OF ACK */
    uint8_t CK_A = 0, CK_B = 0;
    for (uint8_t i = 0; i < (index-2); i++) {
        CK_A = CK_A + UBXmsg[i];
        CK_B = CK_B + CK_A;
    }

    /* CHECK CHECKSUM, SUCCESS? */
    if (CK_A == UBXmsg[index-2] && CK_B == UBXmsg[index-1]) return 0;

    /* FAILURE */
    return -1;
}


/***************************************************************************
 * sendUBX - UBlox send string                                             *
 * 
 * Message must have no sync caracter and checksum. Only class, ID 
 * and payload.
 ***************************************************************************/
void sendUBX(uint8_t *UBXmsg) {
    uint8_t CK_A = 0, CK_B = 0;

    /* UBX HEADER */ 
    Serial.write(0xB5); 
    Serial.write(0x62); 

    /* SET LENGHT OF MESSAGE, CLASS+ID+SIZE(2)+PAYLOAD */
    uint8_t msgLength = 4 + UBXmsg[2] + (UBXmsg[3]*256);

    /* SEND MESSAGE CLASS (2 BYTES) AND PAYLOAD SIZE (2 BYTES) AND PAYLOAD */
    for(int i = 0; i < msgLength; i++) {
        Serial.write(UBXmsg[i]);
        CK_A = CK_A + UBXmsg[i];
        CK_B = CK_B + CK_A;
    }

    /* CHECKSUM AND PACKET END */
    Serial.write(CK_A); 
    Serial.write(CK_B); 
    Serial.write('\r');
    Serial.write('\n');
 }

/***************************************************************************
 * sendUBX_ack - UBlox send command and wait ack                           *
 * 
 * Return 0 on success, -1 on error.
 ***************************************************************************/
int sendUBX_ack(uint8_t *UBXmsg) {
    char status;
    uint8_t pkt[10];

    for(char i=0; i<3; i++) {

        /* SEND UBX COMMAND AND WAIT FOR ACK */
        sendUBX(UBXmsg);
        status = getUBX(pkt, 10);

        /* IF PACKET IS VALID AND ACK(0x05,0x01) AND CLASS/ID IS CORRECT, SUCCESS */
        if(status==0 && pkt[0] == 0x05 && pkt[1] == 0x01 && pkt[4] == UBXmsg[0] && pkt[5] == UBXmsg[1]) return 0;
        
    }

    return -1;
}

/*************************************************************************
* SET GPS NAV MODE                                                       *
* Return: 0 on success, -1 on error.
**************************************************************************/
char GPS_setnav(uint8_t navMode) {
    char status,i;
    uint8_t ubx_buf[42];  // Class, ID, size and checksum + payload size
    uint8_t getNav[] =      {CLASS_CFG, CFG_NAV5, 0x00, 0x00 };

    for(i=0; i<3; i++) {
      
        /* POLL NAV5 */
        memcpy(ubx_buf, getNav, sizeof(getNav));
        sendUBX(ubx_buf);  

        /* GET NAV5 FROM GPS */
        if(getUBX(ubx_buf, 42) == 0) {

            /* GET ANSWER, IF NAV MODE IS NOT THE SAME, RESEND THEM */
            if(ubx_buf[0]==CLASS_CFG && ubx_buf[1]==CFG_NAV5 && ubx_buf[2]==0x24) {
                if(ubx_buf[6]==navMode) return 0;     // Same mode, exit.
                ubx_buf[6] = navMode;
                return sendUBX_ack(ubx_buf);
            } else {

                /* RECEIVE UNKNOWN PACKET */
                return -1;
            }
        } 
    }

    return -1;
}

/*************************************************************************
* SHUTDOWN GPS - SEND UBX RXM-PMREQ                                                          *
**************************************************************************/
void GPS_powerdown() {
    uint8_t setPowerdown[] = {CLASS_RXM, RXM_PMREQ, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
    sendUBX(setPowerdown);
    Serial.end();
}

/*************************************************************************
* POWER UP GPS - SEND 0xFF AND RECONFIGURE ANTENNA                                                          *
**************************************************************************/
char GPS_powerup() {
    // Configure ANTOFF on PIN22 (pin17) connect to mosfet-P with 100k pull-up and 10ohm serie to antenna bias
    uint8_t setAnt[] = {CLASS_CFG, CFG_ANT, 0x04, 0x00, 0x01, 0x00, 0x16, 0x80};
    Serial.begin(9600); delay(50);
    Serial.write(0xFF); delay(50);
    Serial.write(0xFF); delay(50);
    return sendUBX_ack(&setAnt[0]);  //Send UBX Packet
}
/*************************************************************************
* INITIALIZE NMEA DECODER                                                *
**************************************************************************/
char GPS_init(uint8_t navMode) {
    Serial.begin(9600);
    char status = GPS_powerup();       
    if(status!=0) return status;    // Init serial port, wake-up GPS and power ON antenna Bias
    delay(50);
    return GPS_setnav(navMode);     // Set navigation mode
}

/*************************************************************************
* PROCESS NMEA DATA, RETURN TRUE IF MORE DATA TO PROCESS.                *
*************************************************************************/
int GPS_poll() {
	static char buf[83];
	static uint8_t index;
	static uint32_t NmeaTimeout;

    /* NMEA TIMEOUT */
    if(millis() > NmeaTimeout) {
		nav.fix = -1;		// No GPS detected
		NmeaTimeout = millis() + 10000;
    }

    /* DECODE NMEA STRING */
    while(Serial.available()) {
		char newchar = Serial.read();
		if(newchar=='$') { NmeaTimeout = millis() + 10000; index=0; }
		if(newchar=='\r' || newchar=='\n') { buf[index]=0; return nmea_parse(buf); }
		if(index>=82) continue;
		buf[index++] = newchar;
	}
	return 0;
}
