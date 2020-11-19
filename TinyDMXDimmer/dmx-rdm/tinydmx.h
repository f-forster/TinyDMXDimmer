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

#define NUM_DMX_RECV_VALUES				4

// Constant Values
#define RDM_INPUT_DATA_BUFFER_SIZE		72			// TODO: evtl anpassen wenn fertig
#define RDM_OUTPUT_DATA_BUFFER_SIZE		72

#define RDM_INPUT_BUFFER_LAST		RDM_INPUT_DATA_BUFFER_SIZE+23
#define RDM_OUTPUT_BUFFER_LAST		RDM_OUTPUT_DATA_BUFFER_SIZE+23

// DMX RDM status flags
#define DMX_DATA_READY_FOR_PROCESS		(1<<0)
#define RDM_DATA_READY_FOR_PROCESS		(1<<1)


typedef void (*pCallback_Function)(void);

typedef struct {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t w;
} tRgbwColor;



void InitTinyDMX(tRdmUID *ownUID, uint16_t dmxAddr);

uint8_t GetSatus(void);

uint8_t *GetDMXValues(void);

void HandleUsartRx(void);



#endif /* TINYDMX_H_ */