#include <atmel_start.h>
#include <driver_init.h>
#include <hpl_sercom_config.h>

#include "geoduck.h"
#include "ubx8.h"
#include "gpsisr.h"

extern volatile bool gpsPulseHigh;
extern uint8_t  ubxfullset;
extern struct io_descriptor *io_USART_0;

// Settings for the gps uart
// data buckets
volatile uint8_t    gps_usart_buffer[2][UBXRXSIZE];	// Double buffer
volatile uint8_t    gps_next_buffer = 0;				// Next GPS message to be filled (open slot)

void gpsUart_Flush(void) {
	// Clear out any junk in the input buffer
	while(usart_sync_is_rx_not_empty(&USART_0)) {
		uint8_t c;

		//		uart_read(GPS_UART_SERIAL, &c);
		io_read(io_USART_0, &c, 1);
	} // while
}

// Setup the gps uart and program the Ublox chip for most efficient operation
void gpsUart_Setup(void)
{

/* should already be set to these
	usart_sync_set_baud_rate(&USART_0, UBXINITBAUD);
	usart_sync_set_parity(&USART_0, USART_PARITY_EVEN);
	usart_sync_set_stopbits(&USART_0, USART_STOP_BITS_ONE);
	usart_sync_set_mode(&USART_0, USART_MODE_ASYNCHRONOUS);
	delay_ms(60);		// Allows ublox time to cold start
	*/

	// Change baud rate and program the ublox gps chip

#if FASTGPSCOMM
	UBXBaudChange();	// Change ublox from 9600 bps to 57kbps
	delay_ms(60);		// Breathe a moment

	// set baud
	usart_sync_disable(&USART_0);
	usart_sync_set_baud_rate(&USART_0, CALC_BAUD_VALUE(UBXBAUD));
	delay_ms(100);
	usart_sync_enable(&USART_0);

	delay_ms(60);		// Breathe a moment
#endif

// **** TEST
//io_read(io_USART_0, tmpbuf, 10);

	UBXInit();			// Program ublox parameters and select messages

	// Clear out any junk in the input buffer
	gpsUart_Flush();

	// PPS interrupt configuration
	ext_irq_register(PIN_PB02, gpsPPS_ISR);
//	pio_handler_set(PIOA, ID_PIOA, PIO_PA1, PIO_IT_RISE_EDGE, gpsPPS_ISR);
//	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
//	pio_enable_interrupt(PIOA, PIO_PA1);
}

// Pulse Per Sample Interrupt
// This executes when a gps sample is ready to be processed
// Sets flag for main loop and then starts receive for another input buffer
void gpsPPS_ISR(void)
{
	// Set a global flag to indicate a GPS sample cycle
	gpsPulseHigh = true;
	ubxfullset = 0;

	// *** flash the green LED
	gpio_toggle_pin_level(LED_GREEN);

	// Flip the buffers
	gps_next_buffer = !gps_next_buffer;

	// Clear the new buffer
	for (int i = 0; i < UBXRXSIZE; i++ ) gps_usart_buffer[gps_next_buffer][i] = 0xFF;

	// set flag to process message
}

