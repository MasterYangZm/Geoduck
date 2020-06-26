#define FASTGPSCOMM		0			// set to one for GPS speed change to 57600
#define VERBOSE				1

#define USB_PRINT(x)	cdcdf_acm_write(x, strlen(x));

#define LCD_BUTTON_L	0
#define LCD_BUTTON_M	1
#define LCD_BUTTON_R	2
#define GO_BUTTON		3
#define NUM_BUTTONS		4

#define LED_INTERVAL	1000

// function to get the register value based on the usart baud rate
#define CALC_BAUD_VALUE(desired_baud) (65536 - ((65536 * 16.0f * desired_baud) / CONF_GCLK_SERCOM0_CORE_FREQUENCY))