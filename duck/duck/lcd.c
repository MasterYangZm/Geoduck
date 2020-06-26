#include <atmel_start.h>
#include "lcd.h"

/*
 * lcd.c
 *
 * Created: 4/9/2020 2:00:52 PM
 *  Author: Glenn
 */

void lcdBacklight(bool onoff)	{
	gpio_set_pin_level(LCD_PWREN,	onoff);
}

void lcdOn(bool onoff)		{		// turns lcd on
	gpio_set_pin_level(LCD_BACKLIGHT_PWM, onoff);
	delay_ms(50);
}

void lcdLoadNibble(char c, bool upper) {
	if (upper)
		c = c >> 4;
	gpio_set_pin_level(LCD_DB4, c & 0x01);
	c = c >> 1;
	gpio_set_pin_level(LCD_DB5, c & 0x01);
	c = c >> 1;
	gpio_set_pin_level(LCD_DB6, c & 0x01);
	c = c >> 1;
	gpio_set_pin_level(LCD_DB7, c & 0x01);
}

void lcdPulse() {
	gpio_set_pin_level(LCD_E, LCDENABLE);	// E = 1;
	gpio_toggle_pin_level(LED_GREEN);
	delay_us(1); //enable pulse width >= 300ns
	gpio_set_pin_level(LCD_E, LCDDISABLE);		//	E = 0; //Clock enable: falling edge
	gpio_toggle_pin_level(LED_GREEN);
}

void lcdCommand(uint8_t b) {
	lcdLoadNibble(b, UPPERBYTE);									// P1 = i; //put half the data on output Port;
	gpio_set_pin_level(LCD_RS, LCDCOMMAND);	//	D_I =0; //D/I=LOW : send instruction
	gpio_set_pin_level(LCD_RW, LCDWRITE);	//	R_W =0; //R/W=LOW : Write
	delay_us(50);
	lcdPulse(); //Send lower 4 bits
	lcdLoadNibble(b, LOWERBYTE);									// P1 = i; //put half the data on output Port
	delay_us(50);
	lcdPulse(); //Send upper 4 bits
	delay_ms(1);
}

void lcdWriteChar(uint8_t c) {
	lcdLoadNibble(c, UPPERBYTE);        // i; //put data on output port
	gpio_set_pin_level(LCD_RS, LCDDATA);	//	D_I =1; //D/I=HIGH : send data
	gpio_set_pin_level(LCD_RW, LCDWRITE);	//	R_W =0; //R/W=LOW : Write
	delay_us(50);
	lcdPulse(); //Clock lower 4 bits
	lcdLoadNibble(c, LOWERBYTE);         // i; //put data on output Port
	delay_us(50);
	lcdPulse(); //Clock upper 4 bits
	delay_ms(1);
}

void lcdClear() {
	lcdCommand(LCD_CLEARDISPLAY);		// clear display
	delay_ms(1);				// needs extra time
}

void lcdCR() {				// returns active character position to beginning of line
	lcdCommand(LCD_RETURNHOME);
	delay_ms(1);				// needs extra time
}

void lcdGoToPos(uint8_t line, uint8_t pos) {			// move cursor to line and character position
	lcdCommand(LCD_SETDDRAMADDR + pos + LINELEN*line);
	delay_ms(1);				// needs extra time
}

void lcdGoToLine(uint8_t line)	{				// moves cursor to beginning of selected line (0-1)
	lcdGoToPos(line, 0);
}

void lcdWriteText(char *s) {							// write a string at current location
	for(int i=0; s[i] != '\0'; i++)
		if ('\n' == s[i])
			lcdGoToLine(1);
		else
			lcdWriteChar(s[i]);
}

void lcdWriteLine(char *s, uint8_t line) {		// lines are 0 and 1
	lcdGoToLine(line);
	lcdWriteText(s);
}

void lcdWriteCharPos(uint8_t c, uint8_t line, uint8_t pos) {		// lines are 0 and 1, character position is 0-7
	lcdGoToPos(line, pos);
	lcdWriteChar(c);
}

void lcdWriteTextPos(char *s, uint8_t line, uint8_t pos) {		// lines are 0 and 1, character position is 0-7
	lcdGoToPos(line, pos);
	lcdWriteText(s);
}

void lcdInit() {
/*
	#define LCD_BACKLIGHT_PWM GPIO(GPIO_PORTA, 14)
	#define LCD_CONTRAST_PWM GPIO(GPIO_PORTA, 15)
	#define LCD_PWREN GPIO(GPIO_PORTA, 28)
	#define LCD_RS GPIO(GPIO_PORTB, 7)
	#define LCD_E GPIO(GPIO_PORTB, 8)
	#define LCD_RW GPIO(GPIO_PORTB, 9)
	#define LCD_DB4 GPIO(GPIO_PORTB, 12)
	#define LCD_DB5 GPIO(GPIO_PORTB, 13)
	#define LCD_DB6 GPIO(GPIO_PORTB, 14)
	#define LCD_DB7 GPIO(GPIO_PORTB, 15)
*/
	// turn on the LCD
	gpio_set_pin_level(LCD_CONTRAST_PWM, false);	// currently set by potentiometer
	lcdBacklight(true);
	lcdOn(true);

	// wake this mofo up
	gpio_set_pin_level(LCD_RS, LCDCOMMAND);	//	D_I =0; //D/I=LOW : send instruction
	gpio_set_pin_level(LCD_RW, LCDWRITE);	//	R_W =0; //R/W=LOW : Write
	delay_ms(5);
	lcdLoadNibble(0x30, UPPERBYTE);									// P1 = i; //put half the data on output Port;
	lcdPulse(); //Send lower 4 bits				// we are in 8 bit mode here so toggle 3 times to get attention
	delay_ms(5);
	lcdPulse(); //Send lower 4 bits
	delay_ms(5);
	lcdPulse(); //Send lower 4 bits
	delay_ms(5);
	lcdLoadNibble(LCD_FUNCTIONSET | LCD_4BITMODE, UPPERBYTE);				// now set to 4 bit mode
	delay_ms(5);
	lcdPulse();

	lcdCommand(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE);		// 4 bit, 2 line
	lcdCommand(LCD_CURSORSHIFT);		// don't display cursor or shift
	lcdCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON);		// turns display on, no cursor
	lcdCommand(LCD_ENTRYMODESET | LCD_ENTRYLEFT);		// entry Mode. move left to right
	lcdClear();
}
