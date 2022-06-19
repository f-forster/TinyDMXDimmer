/*
 * tinydmx.c
 *
 * Created: 18.11.2020 00:29:42
 *  Author: Franz Forster
 */ 

#include <avr/pgmspace.h>
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



extern const uint8_t aStrManufacturerLabel[];
extern const uint8_t aStrDeviceLabel[];
extern const uint8_t aStrDeviceModelDescription[];
extern const uint8_t aStrSoftwareVersionLabel[];
extern const uint8_t aStrSensorDescription[];
extern const uint8_t aStrDmxPersonalityDescription[];
extern const uint8_t aStrWrongRequest [];



static			tuRdmUID *rdmUID;
static			uint16_t dmx512ListenAddr;

static volatile	uint8_t dmxrdmReceiverState = 0;
static volatile	uint8_t dmxrdmDataProcessingState = 0;
static			uint8_t dmxReceiveBuffer[NUM_DMX_RECV_VALUES];

static			uint16_t rdmPacketReceiveChecksum = 0;
static			tuRdmInputPacket rdmReceiveBuffer;
static			uint16_t rdmPacketTransmitChecksum = 0;
static			tuRdmOutputPacket rdmTransmitBuffer;

static			pCallback_Function _StartResetPulseTimerCb;



uint8_t		_ReadPGM_Byte (const uint8_t* addr);
uint16_t	_ReadPGM_Word (const uint16_t* addr);
uint32_t	_ReadPGM_DWord (const uint32_t* addr);

void		_RDMInputBufferReset(void);
void		_RDMInputBufferPush(uint8_t data, uint16_t dataCounter);
uint16_t	_RDMInputBufferGetChecksum(void);

void		_HandleDMXData(uint8_t data, uint16_t dataCounter);
uint8_t		_HandleRDMData(uint8_t data, uint16_t dataCounter);

uint8_t		_evaluateRDMReqest();




void InitTinyDMX(tuRdmUID *ownUID, uint16_t dmxAddr, pCallback_Function startResetPulseTimerCallback)
{
	if (dmxAddr < 1 || dmxAddr > DMX_512_MAX_CHANNEL_NR - NUM_DMX_RECV_VALUES) {
		// Invalid StartAddress
		dmx512ListenAddr = 1;
	} else {
		dmx512ListenAddr = dmxAddr;
	}
	rdmUID = ownUID;
	_StartResetPulseTimerCb = startResetPulseTimerCallback;
}



uint8_t GetSatus(void)
{
	return dmxrdmDataProcessingState;
}



uint8_t* GetDMXValues(void)
{
	dmxrdmDataProcessingState &= ~(DMX_DATA_READY_FOR_PROCESS);
	return dmxReceiveBuffer;
}



// Wrapper for pgm_read functions
uint8_t _ReadPGM_Byte (const uint8_t* addr)
{
	return pgm_read_byte(addr);
}

uint16_t _ReadPGM_Word (const uint16_t* addr)
{
	return pgm_read_word(addr);
}

uint32_t _ReadPGM_DWord (const uint32_t* addr)
{
	return pgm_read_dword(addr);
}



uint8_t HandleUsartRx(uint8_t usartRXErrors, uint8_t usartRxData)
{
	static	uint16_t uartReceiverCounter = 0;
	uint8_t result = NO_ERROR;
	
	if (usartRXErrors & (USART_FRAMING_ERROR)) {
		if (usartRxData == 0) {
			// Break detected ? Start time measurement
			dmxrdmReceiverState = DMX_VERIFY_RESET_LENGTH;
			_StartResetPulseTimerCb();
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
					_RDMInputBufferReset();
					_RDMInputBufferPush(usartRxData, uartReceiverCounter);
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
		dmxReceiveBuffer[incommingDataCounter - dmx512ListenAddr] = data;
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
		_RDMInputBufferPush(data, dataCounter);
	} 
	else 
	{		
		if (dataCounter < rdmReceivedMessageLength) 
		{
			_RDMInputBufferPush(data, dataCounter);
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
			if (rdmReceivedChecksum == _RDMInputBufferGetChecksum()) 
			{
				// Checksum ok
				// dmxrdmReceiverState |= RDM_DATA_READY_FOR_PROCESS;
				return _evaluateRDMReqest();
			} 
			else 
			{
				dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
				return ERROR_RDM_WRONG_MESSAGE_CHECKSUM;
			}
		}
	}
	return NO_ERROR;	
}



void _RDMInputBufferReset(void)
{
	rdmPacketReceiveChecksum = 0;
}



void _RDMInputBufferPush(uint8_t data, uint16_t dataCounter)
{
	rdmReceiveBuffer.bytes[RDM_INPUT_BUFFER_LAST-dataCounter] = data;
	rdmPacketReceiveChecksum += data;
}



void _RDMOutputBufferPush_Byte(uint8_t data, uint16_t *dataCounter)
{
	rdmReceiveBuffer.bytes[RDM_OUTPUT_BUFFER_LAST-(*dataCounter)] = data;
	*dataCounter += 1;
	rdmPacketReceiveChecksum += data;
}

void _RDMOutputBufferPush_Word(uint16_t data, uint16_t *dataCounter)
{
	_RDMOutputBufferPush_Byte(WRD_LOW(data), dataCounter);
	_RDMOutputBufferPush_Byte(WRD_HIGH(data), dataCounter);
}

void _RDMOutputBufferPush_DWord(uint32_t data, uint16_t *dataCounter)
{
	_RDMOutputBufferPush_Byte((data & 0xFF), dataCounter);
	_RDMOutputBufferPush_Byte(((data>>8) & 0xFF), dataCounter);
	_RDMOutputBufferPush_Byte(((data>>16) & 0xFF), dataCounter);
	_RDMOutputBufferPush_Byte(((data>>24) & 0xFF), dataCounter);
}

void _RDMOutputBufferPush_PGMStr(const uint8_t *aStr, uint8_t strLength, uint16_t *dataCounter)
{
	uint8_t stringIt = 0;
	for (stringIt = 0; stringIt < strLength; strLength++) {
		_RDMOutputBufferPush_Byte(pgm_read_byte(&(aStr[stringIt++])), dataCounter);
	}
}


uint16_t _RDMInputBufferGetChecksum(void)
{
	return rdmPacketReceiveChecksum;
}



uint8_t _evaluateRDMReqest(void)
{
	uint16_t dataIndex = 0;


	if (rdmReceiveBuffer.CommandClass == GET_COMMAND)
	{
		// Set Command Class
		rdmTransmitBuffer.CommandClass = GET_COMMAND_RESPONSE;

		// All Commands supporting GET_COMMAND
		switch (rdmReceiveBuffer.ParameterID)
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
			_RDMOutputBufferPush_Word(RDM_PROTOCOL_VERSION, &dataIndex);
			_RDMOutputBufferPush_Word(_ReadPGM_Word(&(aDeviceInfo->deviceModelID)), &dataIndex);
			_RDMOutputBufferPush_Word(_ReadPGM_Word(&(aDeviceInfo->productCategory)), &dataIndex);
			_RDMOutputBufferPush_DWord(_ReadPGM_DWord(&(aDeviceInfo->softwareVersionID)), &dataIndex);
			_RDMOutputBufferPush_Word(_ReadPGM_Word(&(aDeviceInfo->dmxFootprint)), &dataIndex);
			_RDMOutputBufferPush_Word(_ReadPGM_Word(&(aDeviceInfo->dmxPersonality)), &dataIndex);
			_RDMOutputBufferPush_Word(dmx512ListenAddr, &dataIndex);
			_RDMOutputBufferPush_Word(0, &dataIndex);
			_RDMOutputBufferPush_Byte(_ReadPGM_Byte(&(aDeviceInfo->sensorCount)), &dataIndex);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case PRODUCT_DETAIL_ID_LIST:
			// Product Detail ID (16 Bit)
			// Size: 0x02
			_RDMOutputBufferPush_Word(_ReadPGM_Word(&(aDeviceInfo->productDetailIDList)), &dataIndex);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DEVICE_MODEL_DESCRIPTION:
			// Device Model description (Str)
			// Size: variable
			_RDMOutputBufferPush_PGMStr(aStrDeviceModelDescription, sizeof(aStrDeviceModelDescription), &dataIndex);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case MANUFACTURER_LABEL:
			// Manufacturer Label (Str)
			// Size: variable
			_RDMOutputBufferPush_PGMStr(aStrManufacturerLabel, sizeof(aStrManufacturerLabel), &dataIndex);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DEVICE_LABEL:
			// Device Label (Str)
			// Size: variable
			_RDMOutputBufferPush_PGMStr(aStrDeviceLabel, sizeof(aStrDeviceLabel), &dataIndex);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case SOFTWARE_VERSION_LABEL:
			// Software Version Label (Str)
			// Size: variable
			_RDMOutputBufferPush_PGMStr(aStrSoftwareVersionLabel, sizeof(aStrSoftwareVersionLabel), &dataIndex);
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DMX_PERSONALITY:
			// Current Personality (8 Bit)
			// Number of Personalities (8 Bit)
			// Size: 0x02
			_RDMOutputBufferPush_Word(_ReadPGM_Word(&(aDeviceInfo->dmxPersonality)), &dataIndex);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DMX_PERSONALITY_DESCRIPTION:
			// PD: Number of Personality requested
			// Personality Requestet (8 Bit)
			// Number of DMX 512 Slots (16 Bit)
			// Variable Text Description (Str)
			// Size: variable
			/*
			if (rdmReceiveBuffer.ParameterData[RDM_INPUT_DATA_BUFFER_SIZE-2] == 1) {
				_RDMOutputBufferPush_Byte(  ReadPGM_Byte((const uint8_t*)&(aDeviceInformation.dmxPersonality)));
				_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInfo->dmxFootprint)));
				_RDMOutputBufferPush_PGMStr(  aStrDmxPersonalityDescription, sizeof(aStrDmxPersonalityDescription));
				} else {
				_RDMOutputBufferPush_PGMStr(  aStrWrongRequest, sizeof(aStrWrongRequest));
			}
			*/
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DMX_START_ADDRESS:
			// DMX512 Stard Address (16 Bit)
			// Size: 0x02
			_RDMOutputBufferPush_Word(dmx512ListenAddr, &dataIndex);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case SLOT_INFO:
			// Slot Nuber (0 - 3) (16 Bit)
			// Slot Type (8 Bit)
			// Slot Label ID (16 Bit)
			// Size: 5 * 4 = 20 Byte
			/*
			// TBD
			_RDMOutputBufferPush_Word(  0x0000);
			_RDMOutputBufferPush_Byte(  ReadPGM_Byte(&(aDeviceInformation.slot0Type)));
			_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInformation.slot0LabelID)));
			_RDMOutputBufferPush_Word(  0x0000);
			_RDMOutputBufferPush_Byte(  ReadPGM_Byte(&(aDeviceInformation.slot1Type)));
			_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInformation.slot1LabelID)));
			_RDMOutputBufferPush_Word(  0x0000);
			_RDMOutputBufferPush_Byte(  ReadPGM_Byte(&(aDeviceInformation.slot2Type)));
			_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInformation.slot2LabelID)));
			_RDMOutputBufferPush_Word(  0x0000);
			_RDMOutputBufferPush_Byte(  ReadPGM_Byte(&(aDeviceInformation.slot3Type)));
			_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInformation.slot3LabelID)));
			*/
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
			/*
			// TBD
			if (rdmReceiveBuffer.ParameterData[RDM_INPUT_DATA_BUFFER_SIZE-1] == 1) {
				_RDMOutputBufferPush_Byte(  0x01);
				_RDMOutputBufferPush_Byte(  ReadPGM_Byte(&(aDeviceInformation.sensorType)));
				_RDMOutputBufferPush_Byte(  ReadPGM_Byte(&(aDeviceInformation.sensorUnit)));
				_RDMOutputBufferPush_Byte(  ReadPGM_Byte(&(aDeviceInformation.sensorPrefix)));
				_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInformation.sensorRangeMinValue))); // cast
				_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInformation.sensorRangeMaxValue)));
				_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInformation.sensorNormalMinValue)));
				_RDMOutputBufferPush_Word(  _ReadPGM_Word(&(aDeviceInformation.sensorNormalMaxValue)));
				_RDMOutputBufferPush_Byte(  ReadPGM_Byte(&(aDeviceInformation.sensorRecordedValueSupport)));
				_RDMOutputBufferPush_PGMStr(  aStrSensorDescription, sizeof(aStrSensorDescription));
			}
			else {
				_RDMOutputBufferPush_PGMStr(  aStrWrongRequest, sizeof(aStrWrongRequest));
			}
			*/
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case SENSOR_VALUE:
			// Sensor Requested (8 Bit)
			// Present Value (16 Bit)
			// Highest Value (16 Bit) (Std 0)
			// Lowest Value (16 Bit) (Std 0)
			// Recorded Value (16 Bit) (Std 0)
			// Size: 0x09
			_RDMOutputBufferPush_Byte(0x01, &dataIndex);
			// TODO: Add callback ReadRamByteToTransmitBuffer(ReadCurrentTemperature());
			_RDMOutputBufferPush_Word(0x0000, &dataIndex);
			_RDMOutputBufferPush_Word(0x0000, &dataIndex);
			_RDMOutputBufferPush_Word(0x0000, &dataIndex);

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DEVICE_HOURS:
			// Device Hours (32 Bit)
			// Size: 0x04
			// TODO: report device hours

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DEVICE_POWER_CYCLES:
			// Device Power Cycles (32 Bit)
			// Size: 0x04
			// TODO: report power clycles

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case IDENTIFY_DEVICE:
			// Identify Device State (8 Bit)
			// Size 0x01

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case 0x8000: // TODO: Implement user def test data respond
			// Device Production Test Data
			// Size ???

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			// Exception, unknown PID
			default:
			rdmTransmitBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
			_RDMOutputBufferPush_Word(NR_UNKNOWN_PID, &dataIndex);
			// GPIOR1_rdmResponseMDBLength = 2;
			break;

		} // End switch PID
	}
	else if (rdmReceiveBuffer.CommandClass == SET_COMMAND)
	{
		// Set Command Class
		rdmTransmitBuffer.CommandClass = SET_COMMAND_RESPONSE;

		switch (rdmReceiveBuffer.ParameterID)
		{
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case DMX_START_ADDRESS:
			// Empty response
			// -> Put DMX512 Address in EEPROM

			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case IDENTIFY_DEVICE:
			// Empty Response
			// Set Identify State to recomended state
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			case RESET_DEVICE:
			// Empty Response
			// -> Invoke Reset (0x01 Warm Reset/0xFF Cold Reset)

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
			rdmTransmitBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
			_RDMOutputBufferPush_Word(NR_UNSUPPORTED_COMMAND_CLASS, &dataIndex);
			// GPIOR1_rdmResponseMDBLength = 2;
			break;
			// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
			// Exception, unknown PID
			default:
			rdmTransmitBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
			_RDMOutputBufferPush_Word(NR_UNKNOWN_PID, &dataIndex);
			// GPIOR1_rdmResponseMDBLength = 2;
			break;

		} // End switch PID
	} // End if/else Get/Set Command
	return 	NO_ERROR;
}
