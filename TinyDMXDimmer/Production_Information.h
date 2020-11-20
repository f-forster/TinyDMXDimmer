/*
 * Production_Information.h
 *
 * Implementation of Production specific Data
 *
 * Created: 10.08.2016 11:15:43
 * Author: Franz
 */ 


#ifndef PRODUCTION_INFORMATION_H_
#define PRODUCTION_INFORMATION_H_

#define EEM_PRODUCTION_INFO_DATA	__attribute__ ((section (".ee_production_information_section")))


typedef struct
{
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
} tTimestamp;

typedef struct
{
	tTimestamp	testTimestamp;		// Timestamp when the Device was tested
	uint16_t	temperatureOffsetValue;
	uint16_t	testResult;			// Result of Device Test
	uint16_t	testVersion;		// Version of Device Test
	uint8_t		testComment[32];		// Text Comment
} tProductionInformation;


tProductionInformation eeProductionInformation EEM_PRODUCTION_INFO_DATA = {
							// Preset Values for Production Information, HEX-Notation
	{
		0x5555,				// Year
		0xAA,				// Month
		0x55,				// Day
		0xAA,				// Hour
		0x55,				// Minute
		0xAA				// Second
	},
	280,					// Temperature Offset
	0x5555,					// Test Result
	0xAAAA,					// Test Version
	"<No Test Comment>"		// Test Comment
};


#endif 