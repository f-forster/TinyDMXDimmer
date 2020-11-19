/*
 *
 * Created: 16.11.2020 20:02:55
 * Author : Franz Forster
 */ 

// TODO: propper Configuration of Brown-out-Detect!

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "dmx-rdm/tinydmx.h"
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










// #define GPIOR1_rdmResponseMDBLength			GPIOR1



#define low(x)   ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)

// #define EEM_OPERATION_INFO_DATA		__attribute__ ((section (".ee_operation_information_section")))


// ----------------------------------------------------------------------------
// Prototypes
// ----------------------------------------------------------------------------

void			InitDevice();				
		
			
void			ReportError(const uint8_t);	// Process an Error-Ringbuffer
void			ProcessDMXValues();			// Write received DMX-Values into PWM-Registers
void			ProcessRDMMessage();		// Evaluate received RDM Message and send Response

/*
void			ReadStrToTransmitBuffer( const uint8_t aSrcStr[], uint8_t srcStrSize);
void			ReadByteToTransmitBuffer( const uint8_t *aSrcByte);
void			ReadRamByteToTransmitBuffer( uint8_t SrcByte);
void			ReadWordToTransmitBuffer( const uint16_t *aSrcWord);
void			ReadSignedWordToTransmitBuffer( const int16_t *aSrcWord);
void			ReadRamWordToTransmitBuffer( uint16_t srcWord);
void			ReadDWordToTransmitBuffer( const uint32_t *aSrcDWord);
void			ReadRamDWordToTransmitBuffer( uint32_t SrcDWord);
*/

int16_t			ReadCurrentTemperature();

ISR (USART0_RX_vect);		// UART Receive ISR
ISR (TIMER0_OVF_vect);		// Timer0 Overflow ISR
ISR (PCINT0_vect);
ISR (TIMER0_COMPA_vect);

// ----------------------------------------------------------------------------
// Typedefs
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
// Declarations
// ----------------------------------------------------------------------------

// Unspecified
extern const	uint16_t aPwmValueTable[] PROGMEM;


// ------------------------------------------------------------------------------------------------------------------------
// main
// ------------------------------------------------------------------------------------------------------------------------

int main(void)
{
	tRdmUID thisRdmUID;
	uint16_t dmxAddr;
	
	InitDevice();

	thisRdmUID.deviceID = 0xbc9a7856;
	thisRdmUID.manufacturerID = 0x3412;
	
	// DMX Start-Address
	dmxAddr = 5;


	InitTinyDMX(&thisRdmUID, dmxAddr);





	sei();

	while(1) {
		

		asm volatile ("nop");
		
		if (GetSatus() & RDM_DATA_READY_FOR_PROCESS) {
			// Transmit Buffer is free to use
			ProcessRDMMessage();
		}
		if (GetSatus() & DMX_DATA_READY_FOR_PROCESS) {
			
			ProcessDMXValues();
			asm volatile ("nop"); // temp
		}
	}
}

// ------------------------------------------------------------------------------------------------------------------------

void ProcessDMXValues()
{	
	tRgbwColor* outputColor = (tRgbwColor*)GetDMXValues();;
	
	if (outputColor->r == 0) {
		TOCPMCOE &= ~TOCCER;												// Disable Output Mux (pull pin low)
		} else {
		PWMR = pgm_read_word(&aPwmValueTable[outputColor->r]);				// not inverted
		TOCPMCOE |= TOCCER;													// Enable Output Mux (pin attached to timer)
	}
	if (outputColor->g == 0) {
		TOCPMCOE &= ~TOCCEG;												// Disable Output Mux (pull pin low)
		} else {
		PWMG = 0x3FFF- pgm_read_word(&aPwmValueTable[outputColor->g]);		// inverted
		TOCPMCOE |= TOCCEG;													// Enable Output Mux (pin attached to timer)
	}
	if (outputColor->b == 0) {
		TOCPMCOE &= ~TOCCEB;												// Disable Output Mux (pull pin low)
		} else {
		PWMB = 0x3FFF- pgm_read_word(&aPwmValueTable[outputColor->b]);		// inverted
		TOCPMCOE |= TOCCEB;													// Enable Output Mux (pin attached to timer)
	}
	if (outputColor->w == 0) {
		TOCPMCOE &= ~TOCCEW;												// Disable Output Mux (pull pin low)
		} else {
		PWMW = pgm_read_word(&aPwmValueTable[outputColor->w]);				// not inverted
		TOCPMCOE |= TOCCEW;													// Enable Output Mux (pin attached to timer)
	}
}

// ------------------------------------------------------------------------------------------------------------------------


void ProcessRDMMessage()
{
	
	
	/*
	
	cli();
	
	// --------------------------------------------------
	// Definitions:
	uint8_t			GPIOR1_rdmResponseMDBLength = 0;

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
	
	
	
	
	
	*/
}

/*
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

*/


// ------------------------------------------------------------------------------------------------------------------------

ISR (USART0_RX_vect)
{
	HandleUsartRx();
}

// ------------------------------------------------------------------------------------------------------------------------

ISR(TIMER0_COMPA_vect)
{
	/*
	if (dmxrdmReceiverStatus == DMX_VERIFY_RESET_LENGTH) {
		dmxrdmReceiverStatus = DMX_RESET_LENGTH_OK;
		
	}
	TIMSK0 &= ~(1<<OCIE0A);
	*/
}

// ------------------------------------------------------------------------------------------------------------------------

ISR (TIMER0_OVF_vect)
{
	
	
}

// ------------------------------------------------------------------------------------------------------------------------

ISR (PCINT0_vect)
{
	/*
	if (dmxrdmReceiverStatus == DMX_RESET_LENGTH_OK) {
		// Reset-pulse is ok -> ready to receive Data
		dmxrdmReceiverStatus = DMX_RECEIVE_RESET;
		
		} else if (dmxrdmReceiverStatus == DMX_VERIFY_RESET_LENGTH) {
		// Reset-pulse was too short
		dmxrdmReceiverStatus = DMX_WAIT_FOR_RESET;
		
	}
	*/
	GIMSK &= ~(1<<PCIE0);
	
}



int16_t ReadCurrentTemperature()
{
	ADCSRA &= ~((1<<ADATE) | (1<<ADIE));
	ADCSRA |= (1<<ADEN) | (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	uint16_t tempTempterature = (ADCL | (ADCH << 8));
	
	return tempTempterature; // offset?
	// Todo: zugriff auf temp offset über RDM
}


void InitDevice(void)
{
	/************************************************************************/
	/* IOs                                                                  */
	/************************************************************************/
	PORTA = 0;	// All outputs off
	PORTB = 0;
	
	DDRA = (1<<PINA4) | (1<<PINA5) | (1<<PINA6) | (1<<PINA7);  // DMX RE/TE | PWM PIN 1 | PWM PIN 2 |  PWM PIN 3
	DDRB = (1<<PINB2); // PWM PIN 4
	// TODO: enable pullups on unused pins
	
	/************************************************************************/
	/* Temperatur Sensor                                                    */
	/************************************************************************/
	ADMUXA = (1<<MUX2) | (1<<MUX3);				// Select Temperature Sensor
	ADMUXB = (1<<REFS0);						// Select 1.1V internal Reference, no external connection
	_delay_ms(1);
	ReadCurrentTemperature();					// discard first measurement
	
	/************************************************************************/
	/* Timers                                                               */
	/************************************************************************/
	// Timer1 Init
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<COM1B0) | (1<<WGM11);	// Fast PWM + non inverting mode
	ICR1 = 0x3FFF;													// Max value of PWM
	TCCR1B |=  (1<<CS10) | (1<<WGM13) | (1<<WGM12);					// Prescaler 1/61 (p. 116)
	
	// Timer2 Init
	TCCR2A |= (1<<COM2A1) | (1<<COM2B1) | (1<<COM2B0) | (1<<WGM21);	// Fast PWM + non inverting mode //Test
	ICR2 = 0x3FFF;													// Max value of PWM
	TCCR2B |=  (1<<CS20) | (1<<WGM23) | (1<<WGM22);					// Prescaler 1/61 (p. 116)
	
	TCNT1 = 0;			// Timer preload -> phase difference btw. Timer1 and Timer2 (1FFF=TimerTop/2)
	TCNT2 = 0x1FFF;
	
	TOCPMSA1 = (1<<TOCC4S1) | (1<<TOCC5S1) | (1<<TOCC6S0) | (1<<TOCC7S0); // Timer/Counter Output Compare Pin Mux Selection Registers (p. 116)
	
	// Timer0
	TCCR0B = (1<<CS01) ;		// Prescaler 1/8 -> 1 counterstep = 1µs -> 45 Counterstep + 41µs (Zeit bis FE0) = min. Reset Länge; ~3920 Überläufe = Timeout (1s)
	
	
	/************************************************************************/
	/* Interrupts                                                           */
	/************************************************************************/
	PCMSK0 |= (1<<PCINT2);			// Pin-Change-Interrupt2 on PA2
	TIMSK0 |= (1<<TOIE0);			// Timer 0 OVF ISR enable (Timeout calculation)
	
	
	/************************************************************************/
	/* DMX Bus                                                              */
	/************************************************************************/
	UCSR0B |= (1<<RXCIE0);		// enable USART0 receive interrupt
	UCSR0C |= (1<<USBS0);		// Two stop bits
	UBRR0L = 1;					// with U2X0 (UCSR0A) = 0 -> 250k BAUD, hardcoded because of fixed DMX BAUD
	// Mapping, Async Mode, 8 Data Bits is left default
	UCSR0B |= (1<<RXEN0);		// Enable reception
}




