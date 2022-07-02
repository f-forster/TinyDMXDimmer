/*
 * tinydmx.c
 *
 * Created: 18.11.2020 00:29:42
 *  Author: Franz Forster
 */ 

#include "tinydmx.h"
#include "rdm_protocol.h"
#include "rdm_device_info.h"


#define WRD_LOW(x)   ((x) & 0xFF)
#define WRD_HIGH(x)  (((x)>>8) & 0xFF)

// DMX receiver state machine
#define DMX_WAIT_FOR_RESET			1
#define DMX_VERIFY_RESET_LENGTH		2
#define DMX_RESET_LENGTH_OK			3
#define DMX_RECEIVE_RESET			4
#define DMX_RECEIVE_DMX_VALUES		5
#define	DMX_RECEIVE_RDM_DATA		6
#define RECV_RDM_COMPLETE			7



// DMX receiver errors
#define NO_ERROR					0
#define ERROR_UART_OVF0				2
#define ERROR_UART_FE0				3
#define ERROR_TIMEOUT_MAB			4
#define ERROR_TIMEOUT_DATA			5
#define ERROR_UNKNOWN_STARTCODE		6
#define ERROR_DMX_DATA_OVERRUN		7

#define ERROR_RDM_UNKNOWN_SUB_STARTCODE		8
#define ERROR_RDM_WRONG_MESSAGE_CHECKSUM	9
#define ERROR_RDM_DATA_OVERRUN				10

// RDM response message types
#define RDM_NORMAL_RESPONSE			0
#define RDM_NO_RESPONSE				1
#define RDM_RESPONSE_NO_RESET		2


// Device definition from rdm_device_info.h
extern const uint8_t aStrManufacturerLabel[];
extern const uint8_t aStrDeviceLabel[];
extern const uint8_t aStrDeviceModelDescription[];
extern const uint8_t aStrSoftwareVersionLabel[];
extern const uint8_t aStrSensorDescription[];
extern const uint8_t aStrDmxPersonalityDescription[];
extern const uint8_t aStrWrongRequest [];
extern const tDevice_Information aDeviceInfo;
extern const tSensor_Information aTempSensors[];
extern const tSlot_Information aDeviceSlots[];


static			tuRdmUID *rdmUID;
static			uint16_t dmx512ListenAddr;

static volatile	uint8_t dmxrdmReceiverState = 0;
static volatile	uint8_t dmxrdmDataProcessingState = 0;
static			uint8_t dmxValueRxBuffer[NUM_DMX_RECV_VALUES];


static			uint8_t rxPacketIndex = 0;
static			uint16_t rdmPacketRxChecksum = 0;
static			tuRdmInputPacket rdmPacketRxBuffer;
// static			uint8_t txPacketIndex = 0;
static			uint16_t rdmPacketTxChecksum = 0;
static			tuRdmOutputPacket rdmPacketTxBuffer;




void		_RDMResetRxBuffer(void);
void		_RDMPutRxBuffer_Byte(uint8_t data);

void		_HandleDMXData(uint8_t data, uint16_t dataCounter);
uint8_t		_HandleRDMData(uint8_t data, uint16_t dataCounter);

void		_RDMResetTxBuffer(void);
void		_RDMPutTxParam_Byte(uint8_t data);
void		_RDMPutTxParam_Word(uint16_t data);
void		_RDMPutTxParam_DWord(uint32_t data);
void		_RDMPutTxParam_PGMStr(const uint8_t *aStr, uint8_t strLength);


void InitTinyDMX(tuRdmUID *ownUID, uint16_t dmxAddr)
{
	if (dmxAddr < 1 || dmxAddr > DMX_512_MAX_CHANNEL_NR - NUM_DMX_RECV_VALUES) {
		// Invalid StartAddress
		dmx512ListenAddr = 1;
	} else {
		dmx512ListenAddr = dmxAddr;
	}
	rdmUID = ownUID;
}



uint8_t GetSatus(void)
{
	return dmxrdmDataProcessingState;
}



uint8_t* GetDMXValues(void)
{
	dmxrdmDataProcessingState &= ~(DMX_DATA_READY_FOR_PROCESS);
	return dmxValueRxBuffer;
}



uint8_t HandleUsartRx(uint8_t usartRXErrors, uint8_t usartRxData)
{
	static	uint16_t uartReceiverCounter = 0;
	uint8_t result = NO_ERROR;
	
	if (usartRXErrors & (USART_FRAMING_ERROR)) {
		if (usartRxData == 0) {
			// Break detected ? Start time measurement
			dmxrdmReceiverState = DMX_VERIFY_RESET_LENGTH;
			td_StartTimer();
			uartReceiverCounter = 0;
		}
		else
		{
			dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
			return ERROR_UART_FE0;
		}
	}
	else if (usartRXErrors & (USART_DATA_OVERRUN_ERROR))
	{
		dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
		return ERROR_UART_OVF0;
	}
	else
	{
		if (dmxrdmReceiverState == DMX_RECEIVE_RESET)
		{
			// Check if Startcode was Received
			if (usartRxData == SC_DMX512)
			{
				if (dmxrdmDataProcessingState & DMX_DATA_READY_FOR_PROCESS) {
					// There is still DMX-Data which has to be processed
					dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
					return ERROR_DMX_DATA_OVERRUN;
				} else {
					// DMX512 Startcode
					dmxrdmReceiverState = DMX_RECEIVE_DMX_VALUES;
				}
			}
			else if (usartRxData == SC_RDM)
			{
				if (dmxrdmDataProcessingState & RDM_DATA_READY_FOR_PROCESS) {
					// There is still RDM-Data which has to be processed
					dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
					return ERROR_RDM_DATA_OVERRUN;
				} else {
					// RDM Startcode
					dmxrdmReceiverState = DMX_RECEIVE_RDM_DATA;
					_RDMResetRxBuffer();
					_RDMPutRxBuffer_Byte(usartRxData);
				}
			}
			else
			{
				// Unknown Startcode
				dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
				return ERROR_UNKNOWN_STARTCODE;
			}
		}
		else if (dmxrdmReceiverState == DMX_RECEIVE_DMX_VALUES)
		{
			_HandleDMXData(usartRxData, uartReceiverCounter);
		}
		else if (dmxrdmReceiverState == DMX_RECEIVE_RDM_DATA)
		{
			result = _HandleRDMData(usartRxData, uartReceiverCounter);
		}
		++uartReceiverCounter;
	}
	return result;
}



void MinResetLengthReached(void)
{
	if (dmxrdmReceiverState == DMX_VERIFY_RESET_LENGTH) {
		dmxrdmReceiverState = DMX_RESET_LENGTH_OK;
		
	}
}



void ResetPinChanged(void)
{
	if (dmxrdmReceiverState == DMX_RESET_LENGTH_OK) {
		// Reset-pulse is ok -> ready to receive Data
		dmxrdmReceiverState = DMX_RECEIVE_RESET;
		
	} else if (dmxrdmReceiverState == DMX_VERIFY_RESET_LENGTH) {
		// Reset-pulse was too short
		dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
	}
}



void _HandleDMXData(uint8_t data, uint16_t incommingDataCounter)
{
	if (incommingDataCounter >= dmx512ListenAddr && incommingDataCounter < (dmx512ListenAddr + NUM_DMX_RECV_VALUES))
	{
		dmxValueRxBuffer[incommingDataCounter - dmx512ListenAddr] = data;
		if (incommingDataCounter == (dmx512ListenAddr + NUM_DMX_RECV_VALUES - 1)) {
			// last value received
			dmxrdmDataProcessingState |= DMX_DATA_READY_FOR_PROCESS;
			dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
		}
	}
}



uint8_t _HandleRDMData(uint8_t data, uint16_t dataCounter)
{
	static uint8_t rdmReceivedMessageLength;
	static uint16_t rdmReceivedChecksum;
	
	if (dataCounter <= 8) 
	{
		if (dataCounter == 1)
		{
			if (data != SC_SUB_MESSAGE)
			{
				dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
				return ERROR_RDM_UNKNOWN_SUB_STARTCODE;
			}
		}
		else if (dataCounter == 2)
		{	
			// TODO: implement broadcast address match (for discovery commands!)
			rdmReceivedMessageLength = data;
			if (rdmReceivedMessageLength > RDM_INPUT_BUFFER_SIZE) 
			{
				dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
				return ERROR_RDM_DATA_OVERRUN;
			}
		}
		else if (rdmUID->bytes[dataCounter-3] != data)
		{
			// No Device Address Match
			dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
			return NO_ERROR;
		}
		// store byte when no error occoured
		_RDMPutRxBuffer_Byte(data);
	} 
	else 
	{		
		if (dataCounter < rdmReceivedMessageLength) 
		{
			_RDMPutRxBuffer_Byte(data);
		} 
		else if (dataCounter == rdmReceivedMessageLength) 
		{
			// Checksum high byte received
			rdmReceivedChecksum = (data<<8) & 0xFF00;
		} 
		else if (dataCounter == rdmReceivedMessageLength + 1) 
		{
			// checksum low byte received
			rdmReceivedChecksum |= data & 0xFF;
			dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
			if (rdmReceivedChecksum == rdmPacketRxChecksum) 
			{
				// Checksum ok
				dmxrdmReceiverState |= RDM_DATA_READY_FOR_PROCESS;
				return NO_ERROR;
			} 
			else 
			{
				return ERROR_RDM_WRONG_MESSAGE_CHECKSUM;
			}
		}
	}
	return NO_ERROR;	
}



void _RDMResetRxBuffer(void)
{
	rxPacketIndex = 0;
	rdmPacketRxChecksum = 0;
}



void _RDMPutRxBuffer_Byte(uint8_t data)
{
	rdmPacketRxBuffer.bytes[RDM_INPUT_BUFFER_LAST-rxPacketIndex] = data;
	rxPacketIndex++;
	rdmPacketRxChecksum += data;
}



void _RDMResetTxBuffer(void)
{
	rdmPacketTxChecksum = 0;
	rdmPacketTxBuffer.ParameterDataLength = 0;
	rdmPacketTxBuffer.MessageLength = RDM_PACKET_BASE_SIZE;
}

void _RDMPutTxParam_Byte(uint8_t data)
{
	uint8_t paramLen = rdmPacketTxBuffer.ParameterDataLength;
	rdmPacketTxBuffer.ParameterData[RDM_OUTPUT_PARAM_BUFFER_SIZE-1-(paramLen)] = data;
	++paramLen;
	rdmPacketTxChecksum += data;
	rdmPacketTxBuffer.ParameterDataLength = paramLen;
	rdmPacketTxBuffer.MessageLength = paramLen + RDM_PACKET_BASE_SIZE;
}

void _RDMPutTxParam_Word(uint16_t data)
{
	_RDMPutTxParam_Byte(WRD_LOW(data));
	_RDMPutTxParam_Byte(WRD_HIGH(data));
}

void _RDMPutTxParam_DWord(uint32_t data)
{
	_RDMPutTxParam_Byte((data & 0xFF));
	_RDMPutTxParam_Byte(((data>>8) & 0xFF));
	_RDMPutTxParam_Byte(((data>>16) & 0xFF));
	_RDMPutTxParam_Byte(((data>>24) & 0xFF));
}

void _RDMPutTxParam_PGMStr(const uint8_t *aStr, uint8_t strLength)
{
	uint8_t stringIt = 0;
	for (stringIt = 0; stringIt < strLength; strLength++) {
		_RDMPutTxParam_Byte(td_ReadPGM_Byte(&(aStr[stringIt++])));
	}
}



uint8_t EvaluateRDMRequest(void)
{
	static uint8_t deviceIdentifyState = 0;
	uint16_t dataIndex = 0;
	uint8_t sensorIndex = 0;
	uint16_t tmpValue = 0;
	
	uint8_t responseType = RDM_NORMAL_RESPONSE;

	_RDMResetTxBuffer();
	
	rdmPacketTxBuffer.StartCode	= SC_RDM;
	rdmPacketTxBuffer.SubStartCode = SC_SUB_MESSAGE;
	rdmPacketTxBuffer.DestinationUID = rdmPacketRxBuffer.SourceUID;
	rdmPacketTxBuffer.SourceUID = *rdmUID;
	rdmPacketTxBuffer.TransactionNumber = rdmPacketRxBuffer.TransactionNumber;
	rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_ACK;
	rdmPacketTxBuffer.MessageCount = 0;
	rdmPacketTxBuffer.SubDevice = 0;
	rdmPacketTxBuffer.CommandClass = 0; // is overwritten below
	rdmPacketTxBuffer.ParameterID = rdmPacketRxBuffer.ParameterID;
	
	if (rdmPacketRxBuffer.CommandClass == DISCOVERY_COMMAND) 
	{
		rdmPacketTxBuffer.CommandClass = DISCOVERY_COMMAND_RESPONSE;
		
		switch (rdmPacketRxBuffer.ParameterID)
		{
			case DISC_UNIQUE_BRANCH:
				
			break;
			case DISC_MUTE:
				
			break;
			case DISC_UN_MUTE:
				
			break;
			default:
				rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
				_RDMPutTxParam_Word(NR_UNKNOWN_PID);
			break;
		}
	}
	
	

	if (rdmPacketRxBuffer.CommandClass == GET_COMMAND)
	{
		// Set Command Class
		rdmPacketTxBuffer.CommandClass = GET_COMMAND_RESPONSE;

		// All Commands supporting GET_COMMAND
		switch (rdmPacketRxBuffer.ParameterID)
		{
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DEVICE_INFO:
			// RDM Protocol Version (16 Bit)
			// Device Model ID (16 Bit)
			// Product Category (16 Bit)
			// Software Version ID (32 Bit)
			// DMX 512 Footprint (16 Bit)
			// DMX 512 Personality (16 Bit)
			// DMX 512 Start Address (16 Bit)
			// Subdevice Count (16 Bit)
			// Sensor Count (8 Bit)
			// Size: 0x13
			_RDMPutTxParam_Word(RDM_PROTOCOL_VERSION);
			_RDMPutTxParam_Word(td_ReadPGM_Word(&(aDeviceInfo.deviceModelID)));
			_RDMPutTxParam_Word(td_ReadPGM_Word(&(aDeviceInfo.productCategory)));
			_RDMPutTxParam_DWord(td_ReadPGM_DWord(&(aDeviceInfo.softwareVersionID)));
			_RDMPutTxParam_Word(td_ReadPGM_Word(&(aDeviceInfo.dmxFootprint)));
			_RDMPutTxParam_Word(td_ReadPGM_Word(&(aDeviceInfo.dmxPersonality)));
			_RDMPutTxParam_Word(dmx512ListenAddr);
			_RDMPutTxParam_Word(0);
			_RDMPutTxParam_Byte(td_ReadPGM_Byte(&(aDeviceInfo.sensorCount)));

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case PRODUCT_DETAIL_ID_LIST:
			// Product Detail ID (16 Bit)
			// Size: 0x02
			_RDMPutTxParam_Word(td_ReadPGM_Word(&(aDeviceInfo.productDetailIDList)));

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DEVICE_MODEL_DESCRIPTION:
			// Device Model description (Str)
			// Size: variable
			_RDMPutTxParam_PGMStr(aStrDeviceModelDescription, sizeof(aStrDeviceModelDescription));

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case MANUFACTURER_LABEL:
			// Manufacturer Label (Str)
			// Size: variable
			_RDMPutTxParam_PGMStr(aStrManufacturerLabel, sizeof(aStrManufacturerLabel));

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DEVICE_LABEL:
			// Device Label (Str)
			// Size: variable
			_RDMPutTxParam_PGMStr(aStrDeviceLabel, sizeof(aStrDeviceLabel));

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case SOFTWARE_VERSION_LABEL:
			// Software Version Label (Str)
			// Size: variable
			_RDMPutTxParam_PGMStr(aStrSoftwareVersionLabel, sizeof(aStrSoftwareVersionLabel));
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DMX_PERSONALITY:
			// Current Personality (8 Bit)
			// Number of Personalities (8 Bit)
			// Size: 0x02
			_RDMPutTxParam_Word(td_ReadPGM_Word(&(aDeviceInfo.dmxPersonality)));

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DMX_PERSONALITY_DESCRIPTION:
			// PD: Number of Personality requested
			// Personality Requestet (8 Bit)
			// Number of DMX 512 Slots (16 Bit)
			// Variable Text Description (Str)
			// Size: variable
			
			if (rdmPacketRxBuffer.ParameterData[0] == 1) {
				_RDMPutTxParam_Byte(1);
				_RDMPutTxParam_Word(td_ReadPGM_Word(&(aDeviceInfo.dmxFootprint)));
				_RDMPutTxParam_PGMStr(aStrDmxPersonalityDescription, sizeof(aStrDmxPersonalityDescription));
			} else {
				rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
				_RDMPutTxParam_Word(NR_DATA_OUT_OF_RANGE);
			}
			
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DMX_START_ADDRESS:
			// DMX512 Stard Address (16 Bit)
			// Size: 0x02
			_RDMPutTxParam_Word(dmx512ListenAddr);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case SLOT_INFO:
			// Slot Nuber (0 - 3) (16 Bit)
			// Slot Type (8 Bit)
			// Slot Label ID (16 Bit)
			// Size: 5 * 4 = 20 Byte

			for(sensorIndex = 0; sensorIndex < DEVICE_SLOT_NUM; ++sensorIndex) {
				_RDMPutTxParam_Word(sensorIndex);
				_RDMPutTxParam_Byte(td_ReadPGM_Byte(&(aDeviceSlots[sensorIndex].slotType)));
				_RDMPutTxParam_Word(td_ReadPGM_Word(&(aDeviceSlots[sensorIndex].slotLabelID)));
			}
			
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case SENSOR_DEFINITION:
			// PD: Number of Sensor requested
			// Sensor Requested (8 Bit)
			// Sensor Type (8 Bit)
			// Sensor Unit (8 Bit)
			// Sensor Prefix (8 Bit)
			// Range Min Value (16 Bit)
			// Range Max Value (16 Bit)
			// Normal Min Value (16 Bit)
			// Normal Max Value (16 Bit)
			// Recorded Value Support (8 Bit)
			// Description (Str)
			// Size: 0x0D - 0x2D
			
			sensorIndex = rdmPacketRxBuffer.ParameterData[0];
			if (sensorIndex < DEVICE_SENSOR_NUM) {
				_RDMPutTxParam_Byte(sensorIndex);
				_RDMPutTxParam_Byte(td_ReadPGM_Byte(&(aTempSensors[sensorIndex].sensorType)));
				_RDMPutTxParam_Byte(td_ReadPGM_Byte(&(aTempSensors[sensorIndex].sensorUnit)));
				_RDMPutTxParam_Byte(td_ReadPGM_Byte(&(aTempSensors[sensorIndex].sensorPrefix)));
				_RDMPutTxParam_Word(td_ReadPGM_Word((const uint16_t*)&(aTempSensors[sensorIndex].sensorRangeMinValue))); // cast
				_RDMPutTxParam_Word(td_ReadPGM_Word((const uint16_t*)&(aTempSensors[sensorIndex].sensorRangeMaxValue)));
				_RDMPutTxParam_Word(td_ReadPGM_Word((const uint16_t*)&(aTempSensors[sensorIndex].sensorNormalMinValue)));
				_RDMPutTxParam_Word(td_ReadPGM_Word((const uint16_t*)&(aTempSensors[sensorIndex].sensorNormalMaxValue)));
				_RDMPutTxParam_Byte(td_ReadPGM_Byte(&(aTempSensors[sensorIndex].sensorRecordedValueSupport)));
				// _RDMPutTxParam_PGMStr(  aStrSensorDescription, sizeof(aStrSensorDescription));
			}
			else {
				rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
				_RDMPutTxParam_Word(NR_DATA_OUT_OF_RANGE);
			}
			
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case SENSOR_VALUE:
			// Sensor Requested (8 Bit)
			// Present Value (16 Bit)
			// Highest Value (16 Bit) (Std 0)
			// Lowest Value (16 Bit) (Std 0)
			// Recorded Value (16 Bit) (Std 0)
			// Size: 0x09
			sensorIndex = rdmPacketRxBuffer.ParameterData[0];
			if (sensorIndex < td_ReadPGM_Byte(&(aDeviceInfo.sensorCount))) {
				_RDMPutTxParam_Byte(sensorIndex);
				_RDMPutTxParam_Word((uint16_t)td_ReadSensor(sensorIndex));
				_RDMPutTxParam_Word(0x0000);
				_RDMPutTxParam_Word(0x0000);
				_RDMPutTxParam_Word(0x0000);
			} else {
				rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
				_RDMPutTxParam_Word(NR_DATA_OUT_OF_RANGE);
			}

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case IDENTIFY_DEVICE:
			// Identify Device State (8 Bit)
			// Size 0x01
			_RDMPutTxParam_Byte(deviceIdentifyState);
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case 0x8000: // TODO: Implement user def test data respond
			// Device Production Test Data
			// Size ???

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			// Exception, unknown PID
			default:
			rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
			_RDMPutTxParam_Word(NR_UNKNOWN_PID);
			break;

		} // End switch PID
	}
	else if (rdmPacketRxBuffer.CommandClass == SET_COMMAND)
	{
		// Set Command Class
		rdmPacketTxBuffer.CommandClass = SET_COMMAND_RESPONSE;

		switch (rdmPacketRxBuffer.ParameterID)
		{
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DMX_START_ADDRESS:
			// Empty response
			// -> Put DMX512 Address in EEPROM
			tmpValue = (rdmPacketRxBuffer.ParameterData[0] << 8) | (rdmPacketRxBuffer.ParameterData[0] & 0xFF);
			if (tmpValue > 0 && tmpValue < (DMX_512_MAX_CHANNEL_NR - NUM_DMX_RECV_VALUES)) {
				dmx512ListenAddr = tmpValue;
				td_SetDMXAddr(tmpValue);
			} else {
				rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
				_RDMPutTxParam_Word(NR_DATA_OUT_OF_RANGE);
			}
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case IDENTIFY_DEVICE:
			// Empty Response
			// Set Identify State to recomended state
			if (rdmPacketRxBuffer.ParameterData[0] > 0) {
				deviceIdentifyState = 1;
			} else {
				deviceIdentifyState = 0;
			}
			td_IdentifyDevice(deviceIdentifyState);
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case RESET_DEVICE:
			// Empty Response
			// -> Invoke Reset (0x01 Warm Reset/0xFF Cold Reset)
			if (rdmPacketRxBuffer.ParameterData[0] == 0xFF) {
				td_ResetDevice();
			}
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			// For other Commands Set is not allowed
			case DEVICE_INFO:
			case PRODUCT_DETAIL_ID_LIST:
			case DEVICE_MODEL_DESCRIPTION:
			case MANUFACTURER_LABEL:
			case DEVICE_LABEL:
			case SOFTWARE_VERSION_LABEL:
			case DMX_PERSONALITY:
			case DMX_PERSONALITY_DESCRIPTION:
			case SLOT_INFO:
			case SLOT_DESCRIPTION:
			case SENSOR_DEFINITION:
			case SENSOR_VALUE:
			case DEVICE_HOURS:
			case DEVICE_POWER_CYCLES:
			// ERROR - Command Class Set not Allowed
			rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
			_RDMPutTxParam_Word(NR_UNSUPPORTED_COMMAND_CLASS);
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			// Exception, unknown PID
			default:
			rdmPacketTxBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
			_RDMPutTxParam_Word(NR_UNKNOWN_PID);
			break;

		} // End switch PID
	} // End if/else Get/Set Command
	
	
	return 	NO_ERROR;
}



uint8_t* GetRDMResponse(uint8_t *responseLen)
{
	uint8_t paramLen = rdmPacketTxBuffer.ParameterDataLength;
	rdmPacketTxBuffer.ParameterData[RDM_OUTPUT_PARAM_BUFFER_SIZE-1-(paramLen++)] = WRD_LOW(rdmPacketTxChecksum);
	rdmPacketTxBuffer.ParameterData[RDM_OUTPUT_PARAM_BUFFER_SIZE-1-(paramLen++)] = WRD_HIGH(rdmPacketTxChecksum);
	rdmPacketTxBuffer.ParameterDataLength = paramLen;
	rdmPacketTxBuffer.MessageLength = paramLen + RDM_PACKET_BASE_SIZE;
	
	*responseLen = rdmPacketTxBuffer.MessageLength;
	return rdmPacketTxBuffer.bytes;
}