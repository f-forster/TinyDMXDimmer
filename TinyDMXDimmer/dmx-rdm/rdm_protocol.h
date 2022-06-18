/*
 * dmxrdm_responder.h
 * 
 * Implementation of DMX RDM Responder Data Structure and Constants
 *
 * 
 * Created: 09.08.2016 10:48:45
 *  Author: Franz
 */ 


#ifndef DMXRDM_RESPONDER_H_
#define DMXRDM_RESPONDER_H_

#define RDM_INPUT_BUFFER_SIZE		72			// TODO: evtl anpassen wenn fertig
#define RDM_OUTPUT_BUFFER_SIZE		72


// ----------------------------------------------------------------------------
// Defined Parameters (ABSTRACT!)
// ----------------------------------------------------------------------------

// Appendix A -----------------------------------------------------------------

// START Codes (Slot 0)
#define SC_RDM							0xCC
#define SC_DMX512						0

// RDM Protocol Data Structure ID (Slot 1)
#define SC_SUB_MESSAGE					0x01

//Broadcast Device UID's
#define BROADCAST_ALL_DEVICES			0xFFFFFFFFFFFF
#define SUB_DEVICE_ALL_CALL				0xFFFF

// Table A-1: Command Class Defines
#define DISCOVERY_COMMAND				0x10
#define DISCOVERY_COMMAND_RESPONSE		0x11
#define GET_COMMAND						0x20
#define GET_COMMAND_RESPONSE			0x21
#define SET_COMMAND						0x30
#define SET_COMMAND_RESPONSE			0x31

// Table A-2: Response Type Definitions
#define RESPONSE_TYPE_ACK				0x00
#define RESPONSE_TYPE_ACK_TIMER			0x01
#define RESPONSE_TYPE_NACK_REASON		0x02
#define RESPONSE_TYPE_ACK_OVERFLOW		0x03

// Table A-3: RDM Parameter ID Defines
#define DISC_UNIQUE_BRANCH				0x0001
#define DISC_MUTE						0x0002
#define DISC_UN_MUTE					0x0003

#define DEVICE_INFO						0x0060
#define PRODUCT_DETAIL_ID_LIST			0x0070
#define DEVICE_MODEL_DESCRIPTION		0x0080
#define MANUFACTURER_LABEL				0x0081
#define DEVICE_LABEL					0x0082
#define SOFTWARE_VERSION_LABEL			0x00C0

#define DMX_PERSONALITY					0x00E0
#define DMX_PERSONALITY_DESCRIPTION		0x00E1
#define DMX_START_ADDRESS				0x00F0
#define SLOT_INFO						0x0120
#define SLOT_DESCRIPTION				0x0121

#define SENSOR_DEFINITION				0x0200
#define SENSOR_VALUE					0x0201

#define DEVICE_HOURS					0x0400
#define DEVICE_POWER_CYCLES				0x0405

#define IDENTIFY_DEVICE					0x1000
#define RESET_DEVICE					0x1001

// Table A-5: Product Category Defines
#define PRODUCT_CATEGORY_NOT_DECLARED	0x0000
#define PRODUCT_CATEGORY_FIXTURE_FIXED	0x0101

// Table A-6: Product Detail Defines
#define PRODUCT_DETAIL_NOT_DECLARED		0x0000
#define PRODUCT_DETAIL_LED				0x0004

// Table A-12: Sensor Type Defines
#define SENS_TEMPERATURE				0x00

// Table A-13: Sensor Unit Defines
#define UNITS_NONE						0x00
#define UNITS_CENTIGRADE				0x01

// Table A-14: Sensor Unit Prefix Defines
#define PREFIX_NONE						0x00

// Table A-17: Response NACK Reason Code Defines
#define NR_UNKNOWN_PID					0x0000 // Unknown request
#define NR_FORMAT_ERROR					0x0001 // not formatted correctly
#define NR_UNSUPPORTED_COMMAND_CLASS	0x0005 // Not valid for Command Class attempted
#define NR_DATA_OUT_OF_RANGE			0x0006 // Parameter out of range
#define NR_PACKET_SIZE_UNSUPPORTED		0x0008 // Incoming message exceeds buffer
#define NR_SUB_DEVICE_OUT_OF_RANGE		0x0009 // Sub-Device unknown

// Appendix C -----------------------------------------------------------------

// Table C-1: Slot Type
#define ST_PRIMARY						0x00

// Table C-2: Slot ID Definitions
#define SD_INTENSITY					0x0001 // Intensity
#define SD_INTENSITY_MASTER				0x0002 // Intensity Master

#define SD_COLOR_ADD_RED				0x0205 // Additive Color Mixer - Red
#define SD_COLOR_ADD_GREEN				0x0206 // Additive Color Mixer - Green
#define SD_COLOR_ADD_BLUE				0x0207 // Additive Color Mixer - Blue


// MISC
#define DMX_512_MAX_CHANNEL_NR			512



// ----------------------------------------------------------------------------
// Device Data Structure
// ----------------------------------------------------------------------------

typedef  union
{
	struct __attribute__((__packed__))
	{
		uint16_t manufacturerID;
		uint32_t deviceID;
	};
	uint8_t bytes[6];
} tuRdmUID;

// size of base structure _without_ ParameterData
#define RDM_PACKET_BASE_SIZE		24
#define RDM_INPUT_BUFFER_LAST		RDM_INPUT_BUFFER_SIZE-1
#define RDM_OUTPUT_BUFFER_LAST		RDM_OUTPUT_BUFFER_SIZE-1

typedef union
{
	struct __attribute__((__packed__))
	{
		// Reversed because of Byte ordering
		uint8_t		ParameterData[RDM_INPUT_BUFFER_SIZE - RDM_PACKET_BASE_SIZE];
		uint8_t		ParameterDataLength;
		uint16_t	ParameterID;
		uint8_t		CommandClass;
		uint16_t	SubDevice;
		uint8_t		MessageCount;
		uint8_t		PortID_ResponseType;
		uint8_t		TransactionNumber;
		tuRdmUID	SourceUID;
		tuRdmUID	DestinationUID;
		uint8_t		MessageLength;
		uint8_t		SubStartCode;
		uint8_t		StartCode;
	};
	uint8_t		bytes[RDM_INPUT_BUFFER_SIZE];
} tuRdmInputPacket;

typedef union
{
	struct __attribute__((__packed__))
	{
		// Reversed because of Byte ordering
		uint8_t		ParameterData[RDM_OUTPUT_BUFFER_SIZE - RDM_PACKET_BASE_SIZE];
		uint8_t		ParameterDataLength;
		uint16_t	ParameterID;
		uint8_t		CommandClass;
		uint16_t	SubDevice;
		uint8_t		MessageCount;
		uint8_t		PortID_ResponseType;
		uint8_t		TransactionNumber;
		tuRdmUID	SourceUID;
		tuRdmUID	DestinationUID;
		uint8_t		MessageLength;
		uint8_t		SubStartCode;
		uint8_t		StartCode;
	};
	uint8_t		bytes[RDM_OUTPUT_BUFFER_SIZE];
} tuRdmOutputPacket;


typedef struct // Size: 49 Bytes
{
	// Hardware Parameters				
	uint16_t			deviceModelID;					// @ DEVICE_INFO;					Device Model ID, determined by Manufacturer, see 10.5.1
	uint16_t			productCategory;				// @ DEVICE_INFO;					Device Category, see 10.5.1 and table A-5
	uint16_t			productDetailIDList;			// @ PRODUCT_DETAIL_ID_LIST;		Detaild produt info, see Table A-6, CURRENTLY only one Detail used!
	uint8_t				sensorCount;					// @ DEVICE_INFO;					Number of Sensors, see 10.5.1
	uint8_t				slotCount;

	// Software Parameters
	uint32_t			softwareVersionID;				// @ DEVICE_INFO;					ID of Software Version, see 10.5.1
	
	// DMX Parameters
	uint16_t			dmxFootprint;					// @ DEVICE_INFO;					Number of DMX512 Slots, see 10.5.1
	uint16_t			dmxPersonality;					// @ DEVICE_INFO, DMX_PERSONALITY;	High-Byte: current Personality; Low-Byte: available Personalities, see 10.5.1
	
} tDevice_Information;



typedef struct {
	// Sensor Parameters, see 10.7.1
	uint8_t				sensorType;						// @ SENSOR_DEFINITION;				See Table A-12
	uint8_t				sensorUnit;						// @ SENSOR_DEFINITION;				See Table A-13
	uint8_t				sensorPrefix;					// @ SENSOR_DEFINITION;				See Table A-14
	int16_t				sensorRangeMinValue;			// @ SENSOR_DEFINITION;				See 10.7.1
	int16_t				sensorRangeMaxValue;			// @ SENSOR_DEFINITION;				See 10.7.1
	int16_t				sensorNormalMinValue;			// @ SENSOR_DEFINITION;				See 10.7.1
	int16_t				sensorNormalMaxValue;			// @ SENSOR_DEFINITION;				See 10.7.1
	uint8_t				sensorRecordedValueSupport;		// @ SENSOR_DEFINITION;				See 10.7.1

} tSensor_Information;

typedef struct {
	uint8_t				slotType;		// @ SLOT_INFO;						Information about DMX512 Slots, see 10.6.4 and Table C-1, C-2
	uint16_t			slotLabelID;
} tSlot_Information;


#endif /* DMXRDM-RESPONDER_H_ */




