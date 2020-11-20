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




// ----------------------------------------------------------------------------
// Prototypes
// ----------------------------------------------------------------------------

void		InitDevice(void);				
void		ProcessDMXValues(void);
void		StartResetPulseTimer(void);
int16_t		ReadCurrentTemperature();

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

extern const	uint16_t aPwmValueTable[] PROGMEM;


// ------------------------------------------------------------------------------------------------------------------------
// main
// ------------------------------------------------------------------------------------------------------------------------

int main(void)
{
	tuRdmUID thisRdmUID;
	uint16_t dmxAddr;
	
	thisRdmUID.deviceID = 0xbc9a7856;
	thisRdmUID.manufacturerID = 0x3412;
	dmxAddr = 1;
	
	InitDevice();
	InitTinyDMX(&thisRdmUID, dmxAddr, StartResetPulseTimer);

	sei();

	while(1) {
		
		asm volatile ("nop");
		if (GetSatus() & DMX_DATA_READY_FOR_PROCESS) {
			ProcessDMXValues();
			asm volatile ("nop");
		}
	}
}

// ------------------------------------------------------------------------------------------------------------------------

void ProcessDMXValues()
{	
	tRgbwColor* outputColor = (tRgbwColor*)GetDMXValues();
	
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



ISR (USART0_RX_vect)
{
	uint8_t  uartReceivedData = 1;
	uint8_t	 uartReceiveStatus = 0;
	uartReceiveStatus = UCSR0A;		// Stats must be read before Data (p. 171)
	uartReceivedData = UDR0;
	if (uartReceiveStatus & (1<<FE0)) {
		// framing error
		HandleUsartRx(USART_FRAMING_ERROR, uartReceivedData);
	} else if (uartReceiveStatus & (1<<DOR0)) {
		HandleUsartRx(USART_DATA_OVERRUN_ERROR, uartReceivedData);
	} else {
		HandleUsartRx(USART_NO_ERROR, uartReceivedData);
	}
}



ISR(TIMER0_COMPA_vect)
{
	MinResetLengthReached();
	TIMSK0 &= ~(1<<OCIE0A);
}



ISR (TIMER0_OVF_vect)
{
	
	
}



ISR (PCINT0_vect)
{
	ResetPinChanged();
	GIMSK &= ~(1<<PCIE0);
}

void StartResetPulseTimer(void)
{
	// Loading Timer with data to verify minimum Reset-Pulse length of 88us
	OCR0A = 41;					// = 45µs until Interrupt occurs TODO: Anpassen wenn sich Code VOR diesen Zeilen ändert
	TCNT0 = 0;					// Timer counter value reset
	TIFR0 |= (1<<OCF0A);		// Reset Timer0 compare interrupt flag
	TIMSK0 |= (1<<OCIE0A);		// Timer0 compare interrupt enable
	GIFR |= (1<<PCIF0);			// Enable Pin-Change Detector for rising edge at the end of Reset-Pulse
	GIMSK |= (1<<PCIE0);		// Reset Pin-Change interrupt flag
}



int16_t ReadCurrentTemperature(void)
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




