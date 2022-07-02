/*
 * tinydmx.h
 *
 * Created: 18.11.2020 00:29:26
 *  Author: Franz Forster
 */ 


#ifndef TINYDMX_H_
#define TINYDMX_H_

#include <inttypes.h>
#include "rdm_protocol.h"

// User adjustable
#define NUM_DMX_RECV_VALUES				4

// Constant Values
// DMX RDM status flags
#define DMX_DATA_READY_FOR_PROCESS		(1<<0)
#define RDM_DATA_READY_FOR_PROCESS		(1<<1)


#define USART_NO_ERROR					0
#define	USART_FRAMING_ERROR				(1<<0)
#define	USART_DATA_OVERRUN_ERROR		(1<<1)


typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t w;
} tRgbwColor;



void		InitTinyDMX(tuRdmUID *ownUID, uint16_t dmxAddr);

uint8_t		GetSatus(void);
uint8_t*	GetDMXValues(void);
uint8_t		EvaluateRDMRequest(void);
uint8_t*	GetRDMResponse(uint8_t *responseLen);

uint8_t		HandleUsartRx(uint8_t usartRXErrors, uint8_t usartRxData);
void		MinResetLengthReached(void);
void		ResetPinChanged(void);


// platform dependent functions that must be defined
extern void		td_StartTimer(void);
extern void		td_SetDMXAddr(uint16_t addr);
extern int16_t	td_ReadSensor(uint8_t sensor);
extern void		td_IdentifyDevice(uint8_t state);
extern void		td_ResetDevice(void);


extern uint8_t	td_ReadPGM_Byte (const uint8_t* addr);
extern uint16_t td_ReadPGM_Word (const uint16_t* addr);
extern uint32_t td_ReadPGM_DWord (const uint32_t* addr);


#endif /* TINYDMX_H_ */