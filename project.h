
#include <arduino.h>

#include "gps.h"
#include "sx1278.h"
#include "watchdog.h"
#include "ax25_util.h"

/*
 * When using L as primary table symbol, here symbol ID icon:
 * 'a' is red losange 
 * '>' is car 
 * 'A' is white box 
 * 'H' is orange box 
 * 'O' is baloon 
 * 'W' is green circle 
 * '[' is person 
 * '_' is blue circle
 * 'n' is red triangle 
 * 's' is boat 
 * 'k' SUV 
 * 'u' is Truck 
 * 'v' is Van
 * 'z' is Red house
 */

/* LORA APRS PROTOCOL OE is ASCII MODE, ELSE USE STANDARD AX25 BINARY PACKET */
#define LORAPRS_OE_STYLE 0

/* BEACON CONFIG */
#define MYCALL   "VE2YAG-4"
#define BCN_DEST "APZDG2"
#define BCN_PATH "WIDE1-1"			// Set to "" to disable it.
#define BCN_POSITION_INTERVAL 1		/* in minute */
#define BCN_STATUS_INTERVAL   15	/* in minute */
#define BCN_SYMBOL_TABLE 'L'  /* L displayed for overlay */
#define BCN_SYMBOL_ID    'k'  /* k is SUV */
#define BCN_PATH         0    /* Set to 1 if use PATH field, else use -1 SSID digipeating(-7 bytes) */
#define BCN_ALTITUDE     0    /* Set to 1 if transmit altitude (+4 bytes) */
#define BCN_COMMENT      0    /* Set to 1 for comment transmit */
#define BCN_COMMENT_TEXT "LorAPRS beacon test."

/* BATTERY ADC CALIBRATION */
#define BAT_CAL 4400L   // (float)batt_volt * BAT_CAL / 1023  In millivolts

/* LORA RADIO PARAMETER */
#define FREQ 433.775      // TX freq in MHz
#define FREQ_ERR -21000   // Freq error in Hz
#define LORA_POWER 20     // Power of radio (dbm)
#define PPM_ERR lround(0.95 * (FREQ_ERR/FREQ))  // 25khz offset, 0.95*ppm = 25Khz / 433.3 = 57.65 * 0.95 = 55

/* RADIO CHANNEL CONFIG */
#define CHANNEL_SLOTTIME 100
#define CHANNEL_PERSIST 63

/* PIN DEF */
#define RXD_GPS    0	// UBlox GPS
#define TXD_GPS    1
#define LORA_CS    2	// Lora radio module
#define LORA_DIO   3	// Lora radio module
#define DS_SENSOR  5	// DS18B20 external temp sensor		
#define LORA_MOSI  11	// Shared with SD, Camera buffer chip and Lora radio module
#define LORA_MISO  12
#define LORA_SCLK  13
#define LORA_RESET 17 
#define I2C_SDA    18	// BMP180 sensor 
#define I2C_SCL    19
#define BATT_VOLT  A7 	// Cell voltage, 15k/47k 5v = 1.22v (internal 1.2v ref)

/* TELEMETRY */
//extern float voltage;

/* SET WHEN BOARD ARE UNDER SLEEP MODE */
extern bool sleep_flag;
