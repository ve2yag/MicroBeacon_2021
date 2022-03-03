/***************************************************************************
 * Micro beacon 2021
 * 
 * Derived from MicroFlight for Nanikana HAB flight
 * 
 * Atmega328PB running external crystal at 11.0592 MHz (MiniCore)
 * 
 * Beacon are send in ASCII format, and now converted back to AX25.
 * 
 * Install: 
 * MiniCore card (328PB SMD support)
 * 
 * Les NEO-6M n'ont pas de power switch pour l'antenne GPS, donc le LNA est toujours a ON, 11ma environ.
 * 
 * GPS sensor: UBlox module (Position, speed, altitude, course) 
 * SPI device: Lora radio.
 * 
 * Lora mode: 433.775 Mhz 125BW12SF45CR
 * Send Lora_aprs packet. (Position to track on aprs.fi)
 * 
 * 
 * Pinout:
 * PD0  - 0  - Serial RXD from GPS 
 * PD1  - 1  - Serial TXD to GPS
 * PD2  - 2  - Lora CS
 * PD3  - 3  - Lora DIO0
 * PB3  - 11 - Lora MOSI
 * PB4  - 12 - Lora MISO
 * PB5  - 13 - Lora SCLK
 * PC3  - 17 - Lora RESET
 * ADC7 - A7 - Battery voltage (A0 for DIP28 prototype)
 * 
 * 
 ***************************************************************************/
#include <stdio.h>
#include <math.h>
#include <string.h>

/* AVR SPECIFIC HEADER */
#include <avr/sleep.h>
#include <avr/pgmspace.h>

/* ARDUINO STANDARD LIBRARY */
#include <SPI.h>

/* LOCAL HEADER */
#include "project.h"

/* LORA RADIO MODULE */
SX1278 lora(SX1278_BW_125_00_KHZ, SX1278_SF_12, SX1278_CR_4_5);

/* BOARD BATTERY VOLTAGE */
uint16_t voltage;

/* RADIO BUFFER (NOT SUPPORT MAX SIZE) */
#define MAX_RADIO_BUFSIZE 128
uint8_t radio_buffer[MAX_RADIO_BUFSIZE];
uint8_t radio_bufsize;

/* MICROBEACON USE ASCII BEACON, THIS IS FOR CONVERTING TO AX25 FORMAT */
uint8_t ax25_buffer[MAX_RADIO_BUFSIZE];


/*****************************************************************
 * void Base91Encode(uint8_t *buf, uint8_t size)
 *
 * Encode base91 value to string. Size must be 2 ou 3.
 * Return string size. (size)
 ****************************************************************/
int Base91Encode(uint8_t *buf, uint32_t value, uint8_t size) {

	if(size==2 && value>8280) value=0;
	if(size==3 && value>753570) value=0;
	if(size==4 && value>68574960) value=0;

	if(size==3) {
		*(buf++)  = 33 + (value / (91*91));
		value = (value % (91*91));
	}
	*(buf++) = 33 + (value / 91);
	*(buf++) = 33 + (value % 91);

	return size;
}


/******************************************************************************
 * Watch clear channel, 50ms slottime, persistance 63.
 *****************************************************************************/
void WaitClearChannel() {
    uint32_t t;

    do {
        t = millis() + CHANNEL_SLOTTIME;
        do {
            if(lora.rxBusy()) t = millis() + CHANNEL_SLOTTIME;
        } while(millis() < t);
    } while(random(0,256) > CHANNEL_PERSIST);
}


/******************************************
 * Send_lora_aprs()
 *****************************************/
void Send_lora_aprs() {
    char c, *dest, *p;

    /* SET LORA_APRS HEADER */
    memset(radio_buffer, 0, MAX_RADIO_BUFSIZE);
    radio_buffer[0] = '<'; 
    radio_buffer[1] = 0xFF; 
    radio_buffer[2] = 0x01; 
    p = (char*)&radio_buffer[3];		
    
    /* SOURCE CALL */
    strcpy(p, MYCALL);
    strcat_P(p, PSTR(">"));
    
    /* DEST CALL */
    dest = strrchr(p, 0);
    memcpy(dest, aprs_lat, 6);
 
    /* SET MESSAGE (DESTINATION) */	
    uint8_t message = 1;			// En route
    if((message&1)==0) dest[2]+=32; 	// Insert message bit (inverted)
    if((message&2)==0) dest[1]+=32; 
    if((message&4)==0) dest[0]+=32;

    /* HEMISPHERE AND LONGITUDE OFFSET */
    uint8_t long_deg = abs(nav.longitude); 
    if(nav.latitude>0) dest[3]+=32;				// Insert NORTH hemisphere bit 
    if(long_deg<10 || long_deg>99) dest[4]+=32;	// Set longitude offset bit if <10 or >99 degree
    if(nav.longitude<0) dest[5]+=32;			// Insert EAST hemisphere bit  

	  /* ADD PATH OR DEST SSID FOR DIGIPEATING */
#if BCN_PATH==1
    if(strlen_P(path)) {
        strcat_P(p, PSTR(",")); 
        strcat_P(p, path);
    }
#else
        strcat_P(p, PSTR("-1")); 
#endif
    
    /* ADDRESS END AND MIC-E PACKET TYPE */
    strcat_P(p, PSTR(":`"));           
    p = strrchr(p, 0);

    /* Longitude degree byte (match with logitude offset bit) */
    if(long_deg<10) c = long_deg+118;      
      else 
        if(long_deg<100) c = long_deg+28; 
	  else
	    if(long_deg<110) c = long_deg+8; 
	      else
	        c = long_deg-72;
    *(p++) = c;
    
    /* Set longitude minute byte */
    float minutes = 60.0 * (fabs(nav.longitude) - long_deg);
    if(minutes<10) c = (uint8_t)minutes+88; else c = (uint8_t)minutes+28;
    *(p++) = c;

    /* Set longitude hundred of minute */
    *(p++) = (uint8_t)((minutes - (uint8_t)minutes) * 100) + 28;	

    /* Set hundreds/tens of knots byte */
    uint16_t knot = nav.speed * 100L / 185L;
    if(knot<200) 
    	c = (knot/10)+108;
    else	
    	c = (knot/10)+28;
    *(p++) = c;
    
    /* Set units of knots/hundreds of degrees */
    *(p++) = 32 + ((knot%10)*10) + (nav.course/100);
	
    /* Set tens/units of degrees */
    *(p++) = 28 + (nav.course%100); 

    /* LORA RUNNER SYMBOL */
    *(p++) = BCN_SYMBOL_ID;
    *(p++) = BCN_SYMBOL_TABLE;	

    /* ADD BASE-91 ALTITUDE FROM 10KM UNDER SEA (APRS PROTOCOL) */
#if BCN_ALTITUDE==1
    Base91Encode((uint8_t*)p, 10000L+(uint32_t)nav.altitude, 3);
    strcat_P(p, PSTR("}"));           
#endif

    /* SEND TEXT (OPTIONAL) */
#if BCN_COMMENT==1
    strcat_P(p, PSTR(BCN_COMMENT_TEXT));           
#endif    
}


/******************************************
 * setup()
 *****************************************/
void setup() {
	
    /* LIFT ALL SPI SELECT PIN TO HIGH */
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    /* CONFIGURE WATCHDOG FOR 1HZ INTERRUPT */
    Watchdog_setup();

    /* BATTERY VOLTAGE SENSOR */
    analogReference(INTERNAL);

    /* CONFIGURE GPS */
    GPS_init(NAV_PORTABLE);

    /* CONFIGURE LORA RADIO  */
    lora.begin(LORA_CS, LORA_RESET, LORA_DIO);
    lora.setFrequency((FREQ * 1000000.0)+FREQ_ERR);   // APRS freq
    lora.setPpmError(PPM_ERR);
    lora.setPower(LORA_POWER);  // dbm (max 20)
    lora.setSyncword(0x34);
    delay(150);
}


/******************************************
 * loop()
 *****************************************/
void loop() {
    static uint32_t beacon_to, status_to;
    uint8_t ax25_size;

    /* SEND BEACON */
    if(millis()>beacon_to && nav.fix>=1) {
	    beacon_to = millis() + 60000L * BCN_POSITION_INTERVAL;
        Send_lora_aprs();
#if LORAPRS_OE_STYLE==1
        WaitClearChannel();
        lora.tx(radio_buffer, strlen((char*)radio_buffer));
#else 
        ax25_size = EncodeAX25((char*)&radio_buffer[3], ax25_buffer);
        WaitClearChannel();
        lora.tx(ax25_buffer, ax25_size);
#endif
        while(lora.txBusy()) GPS_poll();
        delay(1);
    }

    /* SEND STATUS */
    if(millis() > (status_to + 10000L)) {
        status_to = millis() + 60000L * BCN_STATUS_INTERVAL;
        radio_buffer[0] = '<'; 
        radio_buffer[1] = 0xFF; 
        radio_buffer[2] = 0x01;
        char *p = (char*) &radio_buffer[3];
        strcpy(p, MYCALL);
        strcat_P(p, PSTR(">"));
        strcat(p, BCN_DEST);
        if(strlen(BCN_PATH)) {
            strcat_P(p, PSTR(",")); 
            strcat(p, BCN_PATH);
        }
        strcat_P(p, PSTR(":"));           
        if(nav.fix>=1) {
            voltage = (float)analogRead(BATT_VOLT) * BAT_CAL / 1023.0;
            sprintf(strchr(p, 0),">%umV Experimental beacon", voltage); 
        } else {
            sprintf(strchr(p, 0),">Not fix sat(%u/%u)", (unsigned int)nav.sat_for_fix, (unsigned int)nav.sat_in_sky); 
        }

#if LORAPRS_OE_STYLE==1
        WaitClearChannel();
        lora.tx(radio_buffer, strlen((char*)radio_buffer));
#else 
        ax25_size = EncodeAX25((char*)&radio_buffer[3], ax25_buffer);
        WaitClearChannel();
        lora.tx(ax25_buffer, ax25_size);
#endif
	      while(lora.txBusy()) GPS_poll();
	      delay(1);
    }    
	
    /* PROCESS GPS DATA UNTIL SERIAL BUFFER ARE EMPTY */
    GPS_poll(); 
	    
    /* RESET WATCHDOG */
    wdt_flag = 0;
}
