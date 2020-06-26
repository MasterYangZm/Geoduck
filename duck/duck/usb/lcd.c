/*
 * lcd.c
 *
 * Created: 4/9/2020 2:00:52 PM
 *  Author: Glenn

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

 /**********************************************************/
 void lcdCommand(char i)
 {
	 P1 = i; //put data on output Port
	 D_I =0; //D/I=LOW : send instruction
	 R_W =0; //R/W=LOW : Write
	 lcdNybble(); //Send lower 4 bits
	 i = i<<4; //Shift over by 4 bits
	 P1 = i; //put data on output Port
	 lcdNybble(); //Send upper 4 bits
 }

 /**********************************************************/
 void lcdWrite(char i)
 {
	 P1 = i; //put data on output Port
	 D_I =1; //D/I=HIGH : send data
	 R_W =0; //R/W=LOW : Write
	 lcdNybble(); //Clock lower 4 bits
	 i = i<<4; //Shift over by 4 bits
	 P1 = i; //put data on output Port
	 lcdNybble(); //Clock upper 4 bits
 }

 /**********************************************************/
 void lcdNybble()
 {
	 E = 1;
	 delay_ms(1); //enable pulse width >= 300ns
	 E = 0; //Clock enable: falling edge
 }

 /**********************************************************/
 void lcdInit()
 {


	 P1 = 0;
	 P3 = 0;
	 delay_ms(100); //Wait >40 msec after power is applied
	 P1 = 0x30; //put 0x30 on the output port
	 delay_ms(30); //must wait 5ms, busy flag not available
	 Nybble(); //command 0x30 = Wake up
	 delay_ms(10); //must wait 160us, busy flag not available
	 Nybble(); //command 0x30 = Wake up #2
	 delay_ms(10); //must wait 160us, busy flag not available
	 Nybble(); //command 0x30 = Wake up #3
	 delay_ms(10); //can check busy flag now instead of delay
	 P1= 0x20; //put 0x20 on the output port
	 lcdNybble(); //Function set: 4-bit interface
	 lcdCommand(0x28); //Function set: 4-bit/2-line
	 lcdCommand(0x10); //Set cursor
	 lcdCommand(0x0F); //Display ON; Blinking cursor
	 lcdCommand(0x06); //Entry Mode set
 }
 /**********************************************************/