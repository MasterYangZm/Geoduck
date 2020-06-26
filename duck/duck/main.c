#include <stdio.h>
#include <string.h>
#include <atmel_start.h>
#include <driver_init.h>
#include "geoduck.h"
#include "lcd.h"
#include "ubx8.h"

void TIMER_0_task1_cb(const struct timer_task *const timer_task);

void gpsUart_Setup(void);
void lcdInit(void);
void lcdWriteChar(uint8_t);
bool readswitch(uint8_t, bool *);

struct timer_task TIMER_0_task1;
struct timer_task	TIMER_0_task2;

extern volatile bool gpsPulseHigh;
extern gpsType currentGPS;

extern struct pwm_descriptor PWM_SERVO;		// servo pwm
extern struct pwm_descriptor PWM_MOTOR1;		// motor pwm

struct io_descriptor *io_USART_0;			// handle for GPS uart i/o

uint32_t ledTimer;
uint32_t debounceTimer;		// free running timer
uint8_t  buffer[50];
char textline[40];				// string used for output to USB

int main(void) {
	atmel_start_init();		// sets all the gpio and usb

	// init timers
	TIMER_0_task1.interval = 1;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_start(&TIMER_0);

	lcdInit();			// turn and initialize LCD
	// greeting
	lcdWriteText("GeoDuck!\n1.0");

	ledTimer = LED_INTERVAL;

	// get the USART handle
	usart_sync_get_io_descriptor(&USART_0, &io_USART_0);
	usart_sync_enable(&USART_0);

// **** TEST
//io_read(io_USART_0, buffer, 10);

	// fire up the GPS
//	gpsUart_Setup();

	// turn on servo power
	gpio_set_pin_level(SERVO_PWREN, true);

	// initialize servo pwm
	pwm_set_parameters(&PWM_SERVO, 20000, 1600);
	pwm_enable(&PWM_SERVO);
	delay_ms(500);

	pwm_set_parameters(&PWM_SERVO, 20000, 1200);
	pwm_enable(&PWM_SERVO);
	delay_ms(500);

	pwm_set_parameters(&PWM_SERVO, 20000, 1400);
	pwm_enable(&PWM_SERVO);

	// turn on motor power - must set all three to HIGH
	gpio_set_pin_level(MOTOR_PWREN, true);
	gpio_set_pin_level(MOTOR_SLEEP, true);
	gpio_set_pin_level(MOTOR_MODE, true);
	delay_ms(100);

	// this manual pulsing works
	int i;
	gpio_set_pin_level(MOTOR_IN2, false);
	for (i=0; i<500; i++) {
		gpio_set_pin_level(MOTOR_IN1, true);
//		gpio_set_pin_level(MOTOR_IN2, false);
		delay_us(1);
		gpio_set_pin_level(MOTOR_IN1, false);
//		gpio_set_pin_level(MOTOR_IN2, true);
		delay_us(800);
	};

#if 0
	// currently the pwm on motor1 does not work

	// initialize motor pwm
	pwm_set_parameters(&PWM_MOTOR1, 20000, 1600);
	pwm_enable(&PWM_MOTOR1);
	delay_ms(500);
#endif

	// clear screen to get ready for application
	lcdClear();

	// turn on tail light
	gpio_set_pin_level(TAIL_LED, true);

	while (1)	{
		// check for user input
		for (int i=0; i<NUM_BUTTONS; i++) {
			bool newButtonState;

			if (readswitch(i, &newButtonState)) {
				if (!newButtonState)
					lcdWriteCharPos(('0'+i), 1, 1+2*i);
				else
					lcdWriteCharPos(' ', 1, 1+2*i);
			} // if
		} // for

		if (ledTimer == 0) {
			ledTimer = LED_INTERVAL;
			gpio_toggle_pin_level(LED_RED);
		}	// if
	} // while
}	// main

// timer task runs every ms
void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
	debounceTimer++;
	if (ledTimer != 0) --ledTimer;
}

