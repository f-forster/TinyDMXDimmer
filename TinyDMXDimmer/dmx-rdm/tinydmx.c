/*
 * tinydmx.c
 *
 * Created: 18.11.2020 00:29:42
 *  Author: Franz Forster
 */ 

#include <avr/pgmspace.h>
#include "tinydmx.h"
#include "rdm_protocol.h"



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



static			tuRdmUID *rdmUID;
static			uint16_t dmx512ListenAddr;

static volatile	uint8_t dmxrdmReceiverState = 0;
static volatile	uint8_t dmxrdmDataProcessingState = 0;
static			uint8_t dmxReceiveBuffer[NUM_DMX_RECV_VALUES];

static			uint16_t rdmPacketReceiveChecksum = 0;
static			tuRdmInputPacket rdmReceiveBuffer;

static			pCallback_Function _StartResetPulseTimerCb;



uint8_t		_ReadPGM_Byte (const uint8_t* addr);
uint16_t	_ReadPGM_Word (const uint16_t* addr);
uint32_t	_ReadPGM_DWord (const uint32_t* addr);

void		_RDMInputBufferReset(void);
void		_RDMInputBufferPush(uint8_t data, uint16_t dataCounter);
uint16_t	_RDMInputBufferGetChecksum(void);

void		_HandleDMXData(uint8_t data, uint16_t dataCounter);
uint8_t		_HandleRDMData(uint8_t data, uint16_t dataCounter);





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
			if (rdmReceivedChecksum == _RDMInputBufferGetChecksum()) 
			{
				// Checksum ok
				dmxrdmReceiverState |= RDM_DATA_READY_FOR_PROCESS;
			} 
			else 
			{
				dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
				return ERROR_RDM_WRONG_MESSAGE_CHECKSUM;
			}
			dmxrdmReceiverState = DMX_WAIT_FOR_RESET;
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



uint16_t _RDMInputBufferGetChecksum(void)
{
	return rdmPacketReceiveChecksum;
}


