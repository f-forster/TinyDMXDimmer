/*
 * tinydmx.c
 *
 * Created: 18.11.2020 00:29:42
 *  Author: Franz Forster
 */ 

#include "tinydmx.h"
#include "rdm_protocol.h"

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






// static volatile		uint8_t dmxrdmReceiverStatus = 0;
// static volatile		uint8_t dmxrdmDataProcessingStatus = 0;
// static volatile		uint16_t dmxStartAddress = 1;
// static volatile		uint8_t  dmxReceiveBuffer[4]; // Prepared for White Channel

// RDM Stuff
// extern const	tDevice_Information aDeviceInformation PGM_DEVICE_INFO_DATA;



// new:
static tRdmUID *rdmUID;
static uint16_t dmx512ListenAddr;





void InitTinyDMX(tRdmUID *ownUID, uint16_t dmxAddr)
{
	if (dmxAddr < 1 || dmxAddr > DMX_512_MAX_CHANNEL_NR - NUM_DMX_RECV_VALUES) {
		// Invalid StartAddress
		dmx512ListenAddr = 1;
	}
	rdmUID = ownUID;
}

uint8_t GetSatus(void)
{
	return 0;
}

uint8_t * GetDMXValues(void)
{
	return 0;
}

void HandleUsartRx(void)
{/*
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
	*/
}
