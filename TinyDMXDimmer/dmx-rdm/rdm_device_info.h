/*
 * RDM_Device_Information.h
 *
 * Definition of DMX RDM Data for this Device
 * 
 * Created: 09.08.2016 22:29:53
 *  Author: Franz
 */ 



#ifndef RDM_DEVICE_INFORMATION_H_
#define RDM_DEVICE_INFORMATION_H_

#include "rdm_protocol.h"


#define PGM_DEVICE_INFO_TEXT __attribute__ ((section (".device_information_text_section")))
#define PGM_DEVICE_INFO_DATA __attribute__ ((section (".device_information_section")))


// Max 32 letters each text
const uint8_t aStrManufacturerLabel[] PGM_DEVICE_INFO_TEXT = "ABCDE Systems";					// @ MANUFACTURER_LABEL;			Label of Manufacturer, see 10.5.4
const uint8_t aStrDeviceLabel[] PGM_DEVICE_INFO_TEXT = "Hallo";								// @ DEVICE_LABEL;					Label of Device (Name), see 10.5.5
const uint8_t aStrDeviceModelDescription[] PGM_DEVICE_INFO_TEXT = "Hallo";					// @ DEVICE_MODEL_DESCRIPTION;		Device Model description, see 10.5.3
const uint8_t aStrSoftwareVersionLabel[] PGM_DEVICE_INFO_TEXT = "Hallo";					// @ SOFTWARE_VERSION_LABEL			Label of Software version
const uint8_t aStrSensorDescription[] PGM_DEVICE_INFO_TEXT = "Hallo";						// @ SENSOR_DEFINITION;				See 10.7.1
const uint8_t aStrDmxPersonalityDescription[] PGM_DEVICE_INFO_TEXT = "Hallo";				// @ DMX_PERSONALITY_DESCRIPTION;	Personality Description, see 10.6.2
const uint8_t aStrWrongRequest [] PGM_DEVICE_INFO_TEXT = "Requested not known!";


const tDevice_Information aDeviceInformation PGM_DEVICE_INFO_DATA = { // Location: 0x0F00 (Word Address)
													
	0x0001,									// uint16_t		Device Model ID
	PRODUCT_CATEGORY_FIXTURE_FIXED,			// uint16_t		Product Category
	PRODUCT_DETAIL_LED,						// uint16_t		Product Detail ID 
	0x01,									// uint8_t		Sensor Count
	0x04,									// uint8_t		Slot Count

	0x00000001,								// uint32_t		Software Version ID

	NUM_DMX_RECV_VALUES,					// uint16_t		DMX Footprint
	0x0101									// uint16_t		DMX Personality

};


const tSensor_Information aTempSensors[] PGM_DEVICE_INFO_DATA = {
	{
		SENS_TEMPERATURE,						// uint8_t		Sensor Type
		UNITS_CENTIGRADE,						// uint8_t		Sensor Unit
		PREFIX_NONE,							// uint8_t		Sensor Prefix
		-20,									// int16_t		Sensor Range Min Value (signed)
		80,										// int16_t		Sensor Range Max Value (signed)
		0,										// int16_t		Sensor Normal Min Value (signed)
		50,										// int16_t		Sensor Normal Max Value (signed)
		0										// uint8_t		Sensor Recorded Value Support
	},
	{
		SENS_TEMPERATURE,						// uint8_t		Sensor Type
		UNITS_CENTIGRADE,						// uint8_t		Sensor Unit
		PREFIX_NONE,							// uint8_t		Sensor Prefix
		-20,									// int16_t		Sensor Range Min Value (signed)
		80,										// int16_t		Sensor Range Max Value (signed)
		0,										// int16_t		Sensor Normal Min Value (signed)
		50,										// int16_t		Sensor Normal Max Value (signed)
		0										// uint8_t		Sensor Recorded Value Support
	}
};


const tSlot_Information aDeviceSlots[] PGM_DEVICE_INFO_DATA = {
	{
		ST_PRIMARY,				// uint8_t 		Slot Type
		SD_COLOR_ADD_RED		// uint16_t		Slot Label ID
	},
	{
		ST_PRIMARY,				// uint8_t 		Slot Type
		SD_COLOR_ADD_GREEN		// uint16_t		Slot Label ID
	},
	{
		ST_PRIMARY,				// uint8_t 		Slot Type
		SD_COLOR_ADD_BLUE		// uint16_t		Slot Label ID
	},
	{
		ST_PRIMARY,				// uint8_t 		Slot Type
		0x8000					// uint16_t		Slot Label ID
	}
};



#endif /* RDM_DEVICE_INFORMATION_H_ */