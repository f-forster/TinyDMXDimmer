/*
 *
 * Created: 16.11.2020 20:02:55
 * Author : Franz Forster
 */ 

// TODO: propper Configuration of Brown-out-Detect!

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "Production_Information.h"
#include "dmxrdm_responder.h"
#include "RDM_Device_Information.h"
#include "Gamma_Table.h"

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

// PWM-Registers
#define PWMR	OCR1A
#define PWMW	OCR2A
#define PWMG	OCR2B
#define PWMB	OCR1B

// Mux Enable
#define TOCCER	(1<<TOCC7OE)
#define TOCCEW	(1<<TOCC5OE)
#define TOCCEG	(1<<TOCC4OE)
#define TOCCEB	(1<<TOCC6OE)




// Constant Values
#define RDM_INPUT_DATA_BUFFER_SIZE		72			// TODO: evtl anpassen wenn fertig
#define RDM_OUTPUT_DATA_BUFFER_SIZE		72

#define RDM_INPUT_BUFFER_LAST		RDM_INPUT_DATA_BUFFER_SIZE+23
#define RDM_OUTPUT_BUFFER_LAST		RDM_OUTPUT_DATA_BUFFER_SIZE+23


// DMX Receiver Status Codes
#define DMX_WAIT_FOR_RESET			1
#define DMX_VERIFY_RESET_LENGTH		2
#define DMX_RESET_LENGTH_OK			3
#define DMX_RECEIVE_RESET			4
#define DMX_RECEIVE_DMX_VALUES		5
#define	DMX_RECEIVE_RDM_DATA		6
#define RECV_RDM_COMPLETE			7



// DMX Receiver Error Codes
#define ERROR_OVERHEAT				1
#define ERROR_UART_OVF0				2
#define ERROR_UART_FE0				3
#define ERROR_TIMEOUT_MAB			4
#define ERROR_TIMEOUT_DATA			5
#define ERROR_UNKNOWN_STARTCODE		6
#define ERROR_DMX_DATA_OVERRUN		7

#define ERROR_RDM_UNKNOWN_SUB_STARTCODE		8
#define ERROR_RDM_WRONG_MESSAGE_CHECKSUM	9
#define ERROR_RDM_DATA_OVERRUN				10


// DMX RDM Process Data Flags
#define DMX_DATA_READY_FOR_PROCESS			(1<<0)
#define RDM_DATA_READY_FOR_PROCESS			(1<<1)
#define DMX_DATA_PROCESSING					(1<<2)

#define GPIOR1_rdmResponseMDBLength			GPIOR1



#define low(x)   ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)

#define EEM_OPERATION_INFO_DATA		__attribute__ ((section (".ee_operation_information_section")))
#define EEM_16_CONTROL_BIT_FIELD	0x99FF
#define EEM_32_CONTROL_BIT_FIELD	0x99FF99FF

// ----------------------------------------------------------------------------
// Prototypes
// ----------------------------------------------------------------------------

void			InitIO();					// Init function for I/O functionality
void			InitTempSens();				// Init ADC an so on
void			InitTimer();				// Init function for timer registers
void			InitInterrupts();			// Init function for interrupt functionality
void			InitDMX();					// Init function for everything with DMX

void			ReportError(const uint8_t);	// Process an Error-Ringbuffer
void			ProcessDMXValues();			// Write received DMX-Values into PWM-Registers
void			ProcessRDMMessage();		// Evaluate received RDM Message and send Response

void			ReadStrToTransmitBuffer( const uint8_t aSrcStr[], uint8_t srcStrSize);
void			ReadByteToTransmitBuffer( const uint8_t *aSrcByte);
void			ReadRamByteToTransmitBuffer( uint8_t SrcByte);
void			ReadWordToTransmitBuffer( const uint16_t *aSrcWord);
void			ReadSignedWordToTransmitBuffer( const int16_t *aSrcWord);
void			ReadRamWordToTransmitBuffer( uint16_t srcWord);
void			ReadDWordToTransmitBuffer( const uint32_t *aSrcDWord);
void			ReadRamDWordToTransmitBuffer( uint32_t SrcDWord);
int16_t			ReadCurrentTemperature();

ISR (USART0_RX_vect);		// UART Receive ISR
ISR (TIMER0_OVF_vect);		// Timer0 Overflow ISR
ISR (PCINT0_vect);
ISR(TIMER0_COMPA_vect);

// ----------------------------------------------------------------------------
// Typedefs
// ----------------------------------------------------------------------------

typedef struct
{
	// PageSize of 4 Bytes
	// Todo: Check beginning of operationinformation section (page size)
	uint16_t DmxStartAddress;
	uint16_t dummy1;
	
	uint32_t DmxDeviceHours;
	uint32_t DmxPowerCycles;
	
	uint16_t DmxStartAddressControlField;
	uint16_t dummy2;
	uint32_t DmxDeviceHoursControlField;
	uint32_t DmxPowerCyclesControlField;
	
} tOperationInformation;

typedef union
{
	struct
	{
		// Reversed because of Byte ordering
		uint8_t		ParameterData[RDM_INPUT_DATA_BUFFER_SIZE];
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
	uint8_t		Bytes[RDM_INPUT_DATA_BUFFER_SIZE+24];
} tuRdmInputPacket;

typedef union
{
	struct
	{
		// Reversed because of Byte ordering
		uint8_t		ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE];
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
	uint8_t		Bytes[RDM_OUTPUT_DATA_BUFFER_SIZE+24];
} tuRdmOutputPacket;


// ----------------------------------------------------------------------------
// Declarations
// ----------------------------------------------------------------------------

// Unspecified
extern const	uint16_t aPwmValueTable[] PROGMEM;
extern			tProductionInformation eeProductionInformation EEM_PRODUCTION_INFO_DATA;

// Temporary!
volatile		uint8_t lastError = 0; //Temp, TODO: anständigen ringpuffer...


// DMX-specific declarations:   -----------------------------------------------

volatile		uint8_t dmxrdmReceiverStatus = 0;
volatile		uint8_t dmxrdmDataProcessingStatus = 0;
volatile		uint16_t dmxStartAddress = 1;
volatile		uint8_t  dmxReceiveBuffer[4]; // Prepared for White Channel

// RDM Stuff
extern const	tDevice_Information aDeviceInformation PGM_DEVICE_INFO_DATA;

tOperationInformation eeOperationInformation EEM_OPERATION_INFO_DATA = {
	1, // DMX START ADRESSE
	0,
	0,
	0,0,0,0,0 // TODO: and propper checksums...
};

volatile		tuRdmUID rdmUID;

volatile		tuRdmOutputPacket rdmTransmitBuffer;
volatile		tuRdmInputPacket rdmReceiveBuffer;

// Timer Stuff
volatile		uint32_t dmxDeviceHours;

// ------------------------------------------------------------------------------------------------------------------------
// main
// ------------------------------------------------------------------------------------------------------------------------

int main(void)
{
	
	InitIO();
	InitTimer();
	InitInterrupts();
	InitDMX();
	//TODO: main zam ramma
	
	UCSR0B |= (1<<RXEN0);		// RXEN


	sei();

	while(1) {
		


		
		// 		//LÖSCHEN TEMP
		// 		uint8_t temp = eeprom_read_word(&eeProductionInformation.testResult);
		// 		temp = pgm_read_dword(&aDeviceInformation.rdmUID.UID.manufacturerID);
		// 		// ? TEMPORARY
		
		
		asm volatile ("nop");
		
		if (dmxrdmDataProcessingStatus & RDM_DATA_READY_FOR_PROCESS) {
			// Transmit Buffer is free to use
			ProcessRDMMessage();
		}
		if (dmxrdmDataProcessingStatus & DMX_DATA_READY_FOR_PROCESS) {
			
			ProcessDMXValues();
			if (lastError) asm volatile ("nop"); // temp
		}
	}
}

// ------------------------------------------------------------------------------------------------------------------------

void ProcessDMXValues()
{	// Getestet und funktioniert!
	// 1 Durchlauf dauert ca. 15us
	dmxrdmDataProcessingStatus |= DMX_DATA_PROCESSING;
	
	if (dmxReceiveBuffer[0] == 0) {
		TOCPMCOE &= ~TOCCER;												// Disable Output Mux (pull pin low)
		} else {
		PWMR = pgm_read_word(&aPwmValueTable[dmxReceiveBuffer[0]]);			// Load with PWM-Value, not inverted
		TOCPMCOE |= TOCCER;													// Enable Output Mux (pin attached to timer)
	}
	if (dmxReceiveBuffer[1] == 0) {
		TOCPMCOE &= ~TOCCEG;												// Disable Output Mux (pull pin low)
		} else {
		PWMG = 0x3FFF- pgm_read_word(&aPwmValueTable[dmxReceiveBuffer[1]]);			// Load with PWM-Value, inverted
		TOCPMCOE |= TOCCEG;													// Enable Output Mux (pin attached to timer)
	}
	if (dmxReceiveBuffer[2] == 0) {
		TOCPMCOE &= ~TOCCEB;												// Disable Output Mux (pull pin low)
		} else {
		PWMB = 0x3FFF- pgm_read_word(&aPwmValueTable[dmxReceiveBuffer[2]]);	// Load with PWM-Value, inverted
		TOCPMCOE |= TOCCEB;													// Enable Output Mux (pin attached to timer)
	}

	// White Channel
	if (dmxReceiveBuffer[3] == 0) {
		TOCPMCOE &= ~TOCCEW;												// Disable Output Mux (pull pin low)
		} else {
		PWMW = pgm_read_word(&aPwmValueTable[dmxReceiveBuffer[3]]);			// Load with PWM-Value, not inverted
		TOCPMCOE |= TOCCEW;													// Enable Output Mux (pin attached to timer)
	}
	
	dmxrdmDataProcessingStatus &= ~DMX_DATA_READY_FOR_PROCESS;						// Reset Data Ready
	dmxrdmDataProcessingStatus &= ~DMX_DATA_PROCESSING;								// Finished Processing
}

// ------------------------------------------------------------------------------------------------------------------------


void ProcessRDMMessage()
{
	cli();
	
	// --------------------------------------------------
	// Definitions:
	// uint8_t			GPIOR1_rdmResponseMDBLength = 0; // is now declared as register

	// Input and Output Packet:
	

	// --------------------------------------------------
	// Composing Response
	rdmTransmitBuffer.StartCode = SC_RDM;
	rdmTransmitBuffer.SubStartCode = SC_SUB_MESSAGE;
	rdmTransmitBuffer.DestinationUID = rdmReceiveBuffer.SourceUID;
	rdmTransmitBuffer.SourceUID = rdmUID;
	rdmTransmitBuffer.TransactionNumber = rdmReceiveBuffer.TransactionNumber;
	rdmTransmitBuffer.PortID_ResponseType = RESPONSE_TYPE_ACK;
	rdmTransmitBuffer.MessageCount = 0;
	rdmTransmitBuffer.SubDevice = 0;
	rdmTransmitBuffer.ParameterID = rdmReceiveBuffer.ParameterID;
	
	// --------------------------------------------------
	// Message Check:
	if (rdmReceiveBuffer.PortID_ResponseType != 0x01 || rdmReceiveBuffer.SubDevice != 0)
	{
		// Wrong Port ID
		rdmTransmitBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
		rdmTransmitBuffer.ParameterData[0] = high(NR_SUB_DEVICE_OUT_OF_RANGE);
		rdmTransmitBuffer.ParameterData[1] = low(NR_SUB_DEVICE_OUT_OF_RANGE);
		GPIOR1_rdmResponseMDBLength = 2;
	}
	else
	{
		// --------------------------------------------------
		if (rdmReceiveBuffer.CommandClass == DISCOVERY_COMMAND)
		{
			// Set Command Class
			rdmTransmitBuffer.CommandClass = DISCOVERY_COMMAND_RESPONSE;
			
			
		}
		else
		{
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
					ReadWordToTransmitBuffer(  &(aDeviceInformation.rdmProtocolVersion));
					ReadWordToTransmitBuffer(  &(aDeviceInformation.deviceModelID));
					ReadWordToTransmitBuffer(  &(aDeviceInformation.productCategory));
					ReadDWordToTransmitBuffer(  &(aDeviceInformation.softwareVersionID));
					ReadWordToTransmitBuffer(  &(aDeviceInformation.dmxFootprint));
					ReadWordToTransmitBuffer(  &(aDeviceInformation.dmxPersonality));
					ReadRamWordToTransmitBuffer(  dmxStartAddress);
					ReadWordToTransmitBuffer(  &(aDeviceInformation.subDeviceCount));
					ReadByteToTransmitBuffer(  &(aDeviceInformation.sensorCount));
					
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case PRODUCT_DETAIL_ID_LIST:
					// Product Detail ID (16 Bit)
					// Size: 0x02
					ReadWordToTransmitBuffer(  &(aDeviceInformation.productDetailIDList));
					
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case DEVICE_MODEL_DESCRIPTION:
					// Device Model description (Str)
					// Size: variable
					ReadStrToTransmitBuffer(  aStrDeviceModelDescription, sizeof(aStrDeviceModelDescription));

					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case MANUFACTURER_LABEL:
					// Manufacturer Label (Str)
					// Size: variable
					ReadStrToTransmitBuffer(  aStrManufacturerLabel, sizeof(aStrManufacturerLabel));

					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case DEVICE_LABEL:
					// Device Label (Str)
					// Size: variable
					ReadStrToTransmitBuffer(  aStrDeviceLabel, sizeof(aStrDeviceLabel));
					
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case SOFTWARE_VERSION_LABEL:
					// Software Version Label (Str)
					// Size: variable
					ReadStrToTransmitBuffer(  aStrSoftwareVersionLabel, sizeof(aStrSoftwareVersionLabel));
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case DMX_PERSONALITY:
					// Current Personality (8 Bit)
					// Number of Personalities (8 Bit)
					// Size: 0x02
					ReadWordToTransmitBuffer(  &(aDeviceInformation.dmxPersonality));
					
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case DMX_PERSONALITY_DESCRIPTION:
					// PD: Number of Personality requested
					// Personality Requestet (8 Bit)
					// Number of DMX 512 Slots (16 Bit)
					// Variable Text Description (Str)
					// Size: variable
					if (rdmReceiveBuffer.ParameterData[RDM_INPUT_DATA_BUFFER_SIZE-2] == 1) {
						ReadByteToTransmitBuffer(  (const uint8_t*)&(aDeviceInformation.dmxPersonality));
						ReadWordToTransmitBuffer(  &(aDeviceInformation.dmxFootprint));
						ReadStrToTransmitBuffer(  aStrDmxPersonalityDescription, sizeof(aStrDmxPersonalityDescription));
						} else {
						ReadStrToTransmitBuffer(  aStrWrongRequest, sizeof(aStrWrongRequest));
					}
					
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case DMX_START_ADDRESS:
					// DMX512 Stard Address (16 Bit)
					// Size: 0x02
					ReadRamWordToTransmitBuffer(  dmxStartAddress);
					
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case SLOT_INFO:
					// Slot Nuber (0 - 3) (16 Bit)
					// Slot Type (8 Bit)
					// Slot Label ID (16 Bit)
					// Size: 5 * 4 = 20 Byte
					ReadRamWordToTransmitBuffer(  0x0000);
					ReadByteToTransmitBuffer(  &(aDeviceInformation.slot0Type));
					ReadWordToTransmitBuffer(  &(aDeviceInformation.slot0LabelID));
					ReadRamWordToTransmitBuffer(  0x0000);
					ReadByteToTransmitBuffer(  &(aDeviceInformation.slot1Type));
					ReadWordToTransmitBuffer(  &(aDeviceInformation.slot1LabelID));
					ReadRamWordToTransmitBuffer(  0x0000);
					ReadByteToTransmitBuffer(  &(aDeviceInformation.slot2Type));
					ReadWordToTransmitBuffer(  &(aDeviceInformation.slot2LabelID));
					ReadRamWordToTransmitBuffer(  0x0000);
					ReadByteToTransmitBuffer(  &(aDeviceInformation.slot3Type));
					ReadWordToTransmitBuffer(  &(aDeviceInformation.slot3LabelID));
					
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					case SLOT_DESCRIPTION:
					// PD: Number of Slot requested
					// Slot Requested (16 Bit)
					// Slot Description Label (Str)
					// Size: variable
					ReadRamWordToTransmitBuffer(  rdmReceiveBuffer.ParameterData[RDM_INPUT_DATA_BUFFER_SIZE-2]);
					switch (rdmReceiveBuffer.ParameterData[RDM_INPUT_DATA_BUFFER_SIZE-2])
					{
						case 0:
						ReadStrToTransmitBuffer(  aStrSlot0Descreption, sizeof(aStrSlot0Descreption));
						break;
						case 1:
						ReadStrToTransmitBuffer(  aStrSlot1Descreption, sizeof(aStrSlot1Descreption));
						break;
						case 2:
						ReadStrToTransmitBuffer(  aStrSlot2Descreption, sizeof(aStrSlot2Descreption));
						break;
						case 3:
						ReadStrToTransmitBuffer(  aStrSlot3Descreption, sizeof(aStrSlot3Descreption));
						break;
						default:
						ReadStrToTransmitBuffer(  aStrWrongRequest, sizeof(aStrWrongRequest));
						break;
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
					if (rdmReceiveBuffer.ParameterData[RDM_INPUT_DATA_BUFFER_SIZE-1] == 1) {
						ReadRamByteToTransmitBuffer(  0x01);
						ReadByteToTransmitBuffer(  &(aDeviceInformation.sensorType));
						ReadByteToTransmitBuffer(  &(aDeviceInformation.sensorUnit));
						ReadByteToTransmitBuffer(  &(aDeviceInformation.sensorPrefix));
						ReadSignedWordToTransmitBuffer(  &(aDeviceInformation.sensorRangeMinValue));
						ReadSignedWordToTransmitBuffer(  &(aDeviceInformation.sensorRangeMaxValue));
						ReadSignedWordToTransmitBuffer(  &(aDeviceInformation.sensorNormalMinValue));
						ReadSignedWordToTransmitBuffer(  &(aDeviceInformation.sensorNormalMaxValue));
						ReadByteToTransmitBuffer(  &(aDeviceInformation.sensorRecordedValueSupport));
						ReadStrToTransmitBuffer(  aStrSensorDescription, sizeof(aStrSensorDescription));
					}
					else {
						ReadStrToTransmitBuffer(  aStrWrongRequest, sizeof(aStrWrongRequest));
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
					ReadRamByteToTransmitBuffer(  0x01);
					ReadRamByteToTransmitBuffer(ReadCurrentTemperature()); // TODO: Testen...
					ReadRamWordToTransmitBuffer(  0x0000);
					ReadRamWordToTransmitBuffer(  0x0000);
					ReadRamWordToTransmitBuffer(  0x0000);
					
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
					// For other Commands Get is not allowed
					case RESET_DEVICE:
					// ERROR - Command Class Get not Allowed
					rdmTransmitBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
					rdmTransmitBuffer.ParameterData[0] = high(NR_UNSUPPORTED_COMMAND_CLASS);
					rdmTransmitBuffer.ParameterData[1] = low(NR_UNSUPPORTED_COMMAND_CLASS);
					GPIOR1_rdmResponseMDBLength = 2;
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					// Exception, unknown PID
					default:
					rdmTransmitBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
					rdmTransmitBuffer.ParameterData[0] = high(NR_UNKNOWN_PID);
					rdmTransmitBuffer.ParameterData[1] = low(NR_UNKNOWN_PID);
					GPIOR1_rdmResponseMDBLength = 2;
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
					rdmTransmitBuffer.ParameterData[0] = high(NR_UNSUPPORTED_COMMAND_CLASS);
					rdmTransmitBuffer.ParameterData[1] = low(NR_UNSUPPORTED_COMMAND_CLASS);
					GPIOR1_rdmResponseMDBLength = 2;
					break;
					// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
					// Exception, unknown PID
					default:
					rdmTransmitBuffer.PortID_ResponseType = RESPONSE_TYPE_NACK_REASON;
					rdmTransmitBuffer.ParameterData[0] = high(NR_UNKNOWN_PID);
					rdmTransmitBuffer.ParameterData[1] = low(NR_UNKNOWN_PID);
					GPIOR1_rdmResponseMDBLength = 2;
					break;
					
				} // End switch PID
			} // End if/else Get/Set Command
		} // End if/else Disc. Cmd
	} // End if/else sub device id
	
	// Write Lengths
	rdmTransmitBuffer.ParameterDataLength = GPIOR1_rdmResponseMDBLength;
	rdmTransmitBuffer.MessageLength = GPIOR1_rdmResponseMDBLength + 24;
	// TODO: Write Lengths & Calculate Checksum
	
	// --------------------------------------------------
	// Now transmit the message
	
	// Finished
	// --------------------------------------------------
	dmxrdmDataProcessingStatus &= ~RDM_DATA_READY_FOR_PROCESS;						// Reset Data Ready
	sei();
}


void ReadStrToTransmitBuffer( const uint8_t aSrcStr[], uint8_t srcStrSize)
{
	uint8_t stringReadIterator = 0;
	uint8_t stringWriteIterator = GPIOR1_rdmResponseMDBLength;
	GPIOR1_rdmResponseMDBLength += srcStrSize-1;		// Delete Null-termination
	while (stringReadIterator <= srcStrSize-1) {
		rdmTransmitBuffer.ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE - 1 - stringWriteIterator++] =
		pgm_read_byte(&(aSrcStr[stringReadIterator++]));
	}
}

void ReadByteToTransmitBuffer( const uint8_t *aSrcByte)
{
	GPIOR1_rdmResponseMDBLength += 1;	// sizeof Byte
	rdmTransmitBuffer.ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE - GPIOR1_rdmResponseMDBLength] = pgm_read_byte(aSrcByte);
}

void ReadRamByteToTransmitBuffer( uint8_t SrcByte)
{
	GPIOR1_rdmResponseMDBLength += 1;	// sizeof Byte
	rdmTransmitBuffer.ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE - GPIOR1_rdmResponseMDBLength] = SrcByte;
}

void ReadWordToTransmitBuffer( const uint16_t *aSrcWord)
{
	GPIOR1_rdmResponseMDBLength += 2;	// sizeof Word
	*((uint16_t*)(&(rdmTransmitBuffer.ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE - GPIOR1_rdmResponseMDBLength]))) = pgm_read_word(aSrcWord);
}

void ReadSignedWordToTransmitBuffer( const int16_t *aSrcWord)
{
	GPIOR1_rdmResponseMDBLength += 2;	// sizeof Word
	*((uint16_t*)(&(rdmTransmitBuffer.ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE - GPIOR1_rdmResponseMDBLength]))) = pgm_read_word(aSrcWord);
}

void ReadRamWordToTransmitBuffer( uint16_t srcWord)
{
	GPIOR1_rdmResponseMDBLength += 2;	// sizeof Word
	*((uint16_t*)(&(rdmTransmitBuffer.ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE - GPIOR1_rdmResponseMDBLength]))) = srcWord;
}

void ReadDWordToTransmitBuffer( const uint32_t *aSrcDWord)
{
	GPIOR1_rdmResponseMDBLength += 4;	// sizeof DWord
	*((uint32_t*)(&(rdmTransmitBuffer.ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE - GPIOR1_rdmResponseMDBLength]))) = pgm_read_dword(aSrcDWord);
}

void ReadRamDWordToTransmitBuffer( uint32_t SrcDWord)
{
	GPIOR1_rdmResponseMDBLength += 4;	// sizeof DWord
	*((uint32_t*)(&(rdmTransmitBuffer.ParameterData[RDM_OUTPUT_DATA_BUFFER_SIZE - GPIOR1_rdmResponseMDBLength]))) = SrcDWord;
}




// ------------------------------------------------------------------------------------------------------------------------

ISR (USART0_RX_vect)
{
	uint8_t  uartReceivedData = 1;
	uint8_t	 uartReceiveStatus = 0;
	static	uint16_t uartReceiverCounter = 0;
	static	uint8_t  rdmReceivedMessageLength = 0;
	static	uint16_t rdmCalcInputChecksum;
	// -----------------------------------
	// Local copys of staus
	uint8_t l_dmxrdmReceiverStatus = dmxrdmReceiverStatus;
	uint8_t l_dmxrdmDataProcessingStatus = dmxrdmDataProcessingStatus;
	
	uartReceiveStatus = UCSR0A;		// Stats must be read before Data (p. 171)
	uartReceivedData = UDR0;
	// TODO: make proof against data Overrun!
	
	if (uartReceiveStatus & (1<<FE0)) {
		if (uartReceivedData == 0) {
			// Break detected ? Start time measurement
			
			l_dmxrdmReceiverStatus = DMX_VERIFY_RESET_LENGTH;
			
			// Loading Timer with data to verify minimum Reset-Pulse length of 88us
			OCR0A = 41;					// = 45µs until Interrupt occurs TODO: Anpassen wenn sich Code VOR diesen Zeilen ändert
			TCNT0 = 0;					// Timer counter value reset
			TIFR0 |= (1<<OCF0A);		// Reset Timer0 compare interrupt flag
			TIMSK0 |= (1<<OCIE0A);		// Timer0 compare interrupt enable
			GIFR |= (1<<PCIF0);			// Enable Pin-Change Detector for rising edge at the end of Reset-Pulse
			GIMSK |= (1<<PCIE0);		// Reset Pin-Change interrupt flag
			
			uartReceiverCounter = 0;	// Counter for incoming bytes

			// TODO: Timer0 auf durchlaufen umbauen
			// TODO: Laufzeitanalyse der RECV-ISR

		}
		else
		{
			ReportError(ERROR_UART_FE0);
			l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
		}
	}
	else if (uartReceiveStatus & (1<<DOR0))
	{
		ReportError(ERROR_UART_OVF0);
		l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
	}
	else
	{
		if (l_dmxrdmReceiverStatus == DMX_RECEIVE_RESET)
		{
			// Check if Startcode was Received
			if (uartReceivedData == SC_DMX512)
			{
				if (l_dmxrdmDataProcessingStatus & DMX_DATA_PROCESSING) {
					// There is still DMX-Data which has to be processed
					ReportError(ERROR_DMX_DATA_OVERRUN);
					l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
					} else {
					// DMX512 Startcode
					l_dmxrdmReceiverStatus = DMX_RECEIVE_DMX_VALUES;		// Set Status
					++uartReceiverCounter;
				}
			}
			else if (uartReceivedData == SC_RDM)
			{
				if (l_dmxrdmDataProcessingStatus & RDM_DATA_READY_FOR_PROCESS) {
					// There is still RDM-Data which has to be processed
					ReportError(ERROR_RDM_DATA_OVERRUN);
					l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
					} else {
					// RDM Startcode
					l_dmxrdmReceiverStatus = DMX_RECEIVE_RDM_DATA;			// Set Status
					rdmReceiveBuffer.Bytes[RDM_INPUT_BUFFER_LAST] = SC_RDM;	// Write SC_RDM in Receive-Buffer
					++uartReceiverCounter;
					rdmCalcInputChecksum = SC_RDM;							// Reset checksum calculation
					rdmReceivedMessageLength = 0;
				}
			}
			else
			{
				// Unknown Startcode
				ReportError(ERROR_UNKNOWN_STARTCODE);
				l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
			}
		}
		else if (l_dmxrdmReceiverStatus == DMX_RECEIVE_DMX_VALUES)
		{
			if (uartReceiverCounter >= dmxStartAddress)
			{
				if (uartReceiverCounter == dmxStartAddress) {
					dmxReceiveBuffer[0] = uartReceivedData;
					} else if (uartReceiverCounter == (dmxStartAddress+1)) {
					dmxReceiveBuffer[1] = uartReceivedData;
					} else if (uartReceiverCounter == (dmxStartAddress+2)) {
					dmxReceiveBuffer[2] = uartReceivedData;
					} else if (uartReceiverCounter == (dmxStartAddress+3)) {
					dmxReceiveBuffer[3] = uartReceivedData;					// Prepared for White Channel
					l_dmxrdmDataProcessingStatus |= DMX_DATA_READY_FOR_PROCESS;
					l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
				}
			}
			++uartReceiverCounter;
		}
		else if (l_dmxrdmReceiverStatus == DMX_RECEIVE_RDM_DATA)
		{
			if (uartReceiverCounter <= 8) {
				if (uartReceiverCounter == 1)
				{
					if (uartReceivedData != SC_SUB_MESSAGE)
					{
						ReportError(ERROR_RDM_UNKNOWN_SUB_STARTCODE);
						l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
						} else {
						rdmReceiveBuffer.Bytes[RDM_INPUT_BUFFER_LAST-uartReceiverCounter] = uartReceivedData;
						rdmCalcInputChecksum += uartReceivedData;
					}
				}
				else if (uartReceiverCounter == 2)
				{	// TODO: implement overrun protection here
					// TODO: implement broadcast address match (for discovery commands!)
					rdmReceivedMessageLength = uartReceivedData;
					rdmReceiveBuffer.Bytes[RDM_INPUT_BUFFER_LAST-uartReceiverCounter] = uartReceivedData;
					rdmCalcInputChecksum += uartReceivedData;
				}
				else if (rdmUID.UID_byte[uartReceiverCounter-3] != uartReceivedData)
				{
					// No Device Address Match
					l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
					asm volatile ("nop");
					} else {
					rdmReceiveBuffer.Bytes[RDM_INPUT_BUFFER_LAST-uartReceiverCounter] = uartReceivedData;
					rdmCalcInputChecksum += uartReceivedData;
				}
				} else {
				// TODO: implement overrun protection here
				rdmReceiveBuffer.Bytes[RDM_INPUT_BUFFER_LAST-uartReceiverCounter] = uartReceivedData;
				
				if (uartReceiverCounter < rdmReceivedMessageLength) {
					// Only Calculate Checksum if no Checksum received
					rdmCalcInputChecksum += uartReceivedData;
					} else if (uartReceiverCounter == rdmReceivedMessageLength + 1) {
					// Last Byte of RDM-Message received
					if (rdmReceiveBuffer.Bytes[RDM_INPUT_BUFFER_LAST-uartReceiverCounter+1] == high(rdmCalcInputChecksum) &&
					rdmReceiveBuffer.Bytes[RDM_INPUT_BUFFER_LAST-uartReceiverCounter] == low(rdmCalcInputChecksum)) {
						// Checksum ok
						l_dmxrdmDataProcessingStatus |= RDM_DATA_READY_FOR_PROCESS;
						} else {
						ReportError(ERROR_RDM_WRONG_MESSAGE_CHECKSUM);
					}
					l_dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
				}
			}
			++uartReceiverCounter;
		}

	}
	
	// Put back local copys
	dmxrdmReceiverStatus = l_dmxrdmReceiverStatus;
	dmxrdmDataProcessingStatus = l_dmxrdmDataProcessingStatus;
}

// ------------------------------------------------------------------------------------------------------------------------

ISR(TIMER0_COMPA_vect)
{
	if (dmxrdmReceiverStatus == DMX_VERIFY_RESET_LENGTH) {
		dmxrdmReceiverStatus = DMX_RESET_LENGTH_OK;
		
	}
	TIMSK0 &= ~(1<<OCIE0A);
}

// ------------------------------------------------------------------------------------------------------------------------

ISR (TIMER0_OVF_vect)
{
	
	
}

// ------------------------------------------------------------------------------------------------------------------------

ISR (PCINT0_vect)
{
	if (dmxrdmReceiverStatus == DMX_RESET_LENGTH_OK) {
		// Reset-pulse is ok -> ready to receive Data
		dmxrdmReceiverStatus = DMX_RECEIVE_RESET;
		
		} else if (dmxrdmReceiverStatus == DMX_VERIFY_RESET_LENGTH) {
		// Reset-pulse was too short
		dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
		
	}
	GIMSK &= ~(1<<PCIE0);
	
}

// ------------------------------------------------------------------------------------------------------------------------

void ReportError(const uint8_t errorForReport){
	lastError = errorForReport;
}

// ------------------------------------------------------------------------------------------------------------------------

void InitIO()
{
	// All outputs off
	PORTA = 0;
	PORTB = 0;
	
	// Set as Outputs
	DDRA = (1<<PINA4) | (1<<PINA5) | (1<<PINA6) | (1<<PINA7);  // DMX RE/TE | PWM PIN 1 | PWM PIN 2 |  PWM PIN 3
	DDRB = (1<<PINB2); // PWM PIN 4
	
	// TODO: Pullup bei unbenutzen pins einschalten
}


int16_t ReadCurrentTemperature()
{
	ADCSRA &= ~((1<<ADATE) | (1<<ADIE));
	ADCSRA |= (1<<ADEN) | (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	uint16_t tempTempterature = (ADCL | (ADCH << 8));
	
	return tempTempterature - eeprom_read_word(&(eeProductionInformation.temperatureOffsetValue));
	// Todo: zugriff auf temp offset über RDM
}


void InitTempSens()
{
	ADMUXA = (1<<MUX2) | (1<<MUX3);				// Select Temperature Sensor
	ADMUXB = (1<<REFS0);						// Select 1.1V internal Reference, no external connection
	_delay_ms(1);
	ReadCurrentTemperature();					// discard first measurement
}

void InitTimer ()
{
	// Timer1 Init
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<COM1B0) | (1<<WGM11);								// Fast PWM + non inverting mode
	ICR1 = 0x3FFF;													// Max value of PWM
	TCCR1B |=  (1<<CS10) | (1<<WGM13) | (1<<WGM12);					// Prescaler 1/61 (p. 116)
	
	
	// Timer2 Init
	TCCR2A |= (1<<COM2A1) | (1<<COM2B1) | (1<<COM2B0) | (1<<WGM21);		// Fast PWM + non inverting mode //Test
	ICR2 = 0x3FFF;													// Max value of PWM
	TCCR2B |=  (1<<CS20) | (1<<WGM23) | (1<<WGM22);					// Prescaler 1/61 (p. 116) // Neu: ???????
	
	// PWMR = PWMB = PWMG = 0;		//TODO: dürfen nicht alle auf 0 gesetzt werden (invertierung)
	
	// Timer Preload -> phase difference of Timer1 and Timer2 (1FFF=TimerTop/2)
	TCNT1 = 0;
	TCNT2 = 0x1FFF;
	
	
	// Output Mux
	// Timer/Counter Output Compare Pin Mux Selection Registers (p. 116)
	TOCPMSA1 = (1<<TOCC4S1) | (1<<TOCC5S1) | (1<<TOCC6S0) | (1<<TOCC7S0);
	// Timer/Counter Output Compare Pin Mux Channel Output Enable
	// TOCPMCOE = TOCCEB | TOCCEG | TOCCER | TOCCEW;	// erst in Process DMX
	
	// Timer0
	TCCR0B = (1<<CS01) ;			// Prescaler 1/8 -> 1 counterstep = 1µs -> 45 Counterstep + 41µs (Zeit bis FE0) = min. Reset Länge; ~3920 Überläufe = Timeout (1s)
	
}

// ------------------------------------------------------------------------------------------------------------------------

void InitInterrupts()
{
	// Mask Pin-Change-Interrupt2 = PCINT on PA2
	PCMSK0 |= (1<<PCINT2);			// PCINT is enabled ind GIMSK if Reset-Pulse is detected
	
	// Timer2 Overflow
	TIMSK0 |= (1<<TOIE0);			// Timer 0 OVF ISR enable (Timeout calculation)
}

// ------------------------------------------------------------------------------------------------------------------------

void InitDMX()
{
	// USART0 Init
	UCSR0B |= (1<<RXCIE0);			// Enable receive interrupt
	UCSR0C |= (1<<USBS0);			// Two stop bits
	UBRR0L = 1;						// with U2X0 (UCSR0A) = 0 -> 250k BAUD, hardcoded because of fixed DMX BAUD
	// Mapping, Async Mode, 8 Data Bits is left default
	
	
	// DMX-RDM UID
	rdmUID.UID.deviceID = pgm_read_dword(&(aDeviceInformation.rdmUID.UID.deviceID));
	rdmUID.UID.manufacturerID = pgm_read_word(&(aDeviceInformation.rdmUID.UID.manufacturerID));
	
	// DMX Start-Address
	dmxStartAddress = eeprom_read_word(&(eeOperationInformation.DmxStartAddress));
	if (dmxStartAddress < 1 || dmxStartAddress > 512 - 3) {		// TODO: Weiß Kanal irgendwann berücksichtigen
		// Invalid StartAddress
		dmxStartAddress = 1;
		
	}
	// TODO: Testen !!!
	// Increment Power-Cycle counter
	uint32_t tempDmxPowerCycles = eeprom_read_dword(&(eeOperationInformation.DmxPowerCycles));
	// uint32_t readDummy = eeprom_read_dword(&(eeOperationInformation.DmxPowerCyclesControlField));
	
	++tempDmxPowerCycles;
	eeprom_write_dword(&(eeOperationInformation.DmxPowerCycles), tempDmxPowerCycles);
	
	
	// Load Device Hours
	dmxDeviceHours = eeprom_read_dword(&(eeOperationInformation.DmxDeviceHours));

}

