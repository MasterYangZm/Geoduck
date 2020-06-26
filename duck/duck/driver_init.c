/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_adc_base.h>
#include <hpl_rtc_base.h>

struct spi_m_sync_descriptor SPI_1;
struct spi_m_sync_descriptor SPI_0;
struct timer_descriptor      TIMER_0;

struct adc_sync_descriptor ADC_0;

struct usart_sync_descriptor USART_0;

struct i2c_m_sync_desc I2C_0;

struct i2c_m_sync_desc I2C_1;

struct pwm_descriptor PWM_SERVO;

struct pwm_descriptor PWM_MOTOR1;

void ADC_0_PORT_init(void)
{

	// Disable digital pin circuitry
	gpio_set_pin_direction(MOTOR_VREF, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(MOTOR_VREF, PINMUX_PA02B_ADC_AIN0);

	// Disable digital pin circuitry
	gpio_set_pin_direction(VBAT_MON, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(VBAT_MON, PINMUX_PA03B_ADC_AIN1);

	// Disable digital pin circuitry
	gpio_set_pin_direction(MOTOR_IMON, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(MOTOR_IMON, PINMUX_PA04B_ADC_AIN4);
}

void ADC_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);
}

void ADC_0_init(void)
{
	ADC_0_CLOCK_init();
	ADC_0_PORT_init();
	adc_sync_init(&ADC_0, ADC, (void *)NULL);
}

void EXTERNAL_IRQ_0_init(void)
{
	_gclk_enable_channel(EIC_GCLK_ID, CONF_GCLK_EIC_SRC);

	// Set pin direction to input
	gpio_set_pin_direction(NAV_PPS, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(NAV_PPS,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(NAV_PPS, PINMUX_PB02A_EIC_EXTINT2);

	ext_irq_init();
}

void USART_0_PORT_init(void)
{

	gpio_set_pin_function(NAV_TxD, PINMUX_PA10C_SERCOM0_PAD2);

	gpio_set_pin_function(NAV_RxD, PINMUX_PA11C_SERCOM0_PAD3);
}

void USART_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
}

void USART_0_init(void)
{
	USART_0_CLOCK_init();
	usart_sync_init(&USART_0, SERCOM0, (void *)NULL);
	USART_0_PORT_init();
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(NAV_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(NAV_SDA, PINMUX_PA08D_SERCOM2_PAD0);

	gpio_set_pin_pull_mode(NAV_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(NAV_SCL, PINMUX_PA09D_SERCOM2_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
	_gclk_enable_channel(SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC);
	_gclk_enable_channel(SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM2);
	I2C_0_PORT_init();
}

void I2C_1_PORT_init(void)
{

	gpio_set_pin_pull_mode(SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SDA, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_pull_mode(SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SCL, PINMUX_PA23C_SERCOM3_PAD1);
}

void I2C_1_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM3);
	_gclk_enable_channel(SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC);
	_gclk_enable_channel(SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC);
}

void I2C_1_init(void)
{
	I2C_1_CLOCK_init();
	i2c_m_sync_init(&I2C_1, SERCOM3);
	I2C_1_PORT_init();
}

void SPI_1_PORT_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(MISO, PINMUX_PA12D_SERCOM4_PAD0);

	gpio_set_pin_level(MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOSI, PINMUX_PB10D_SERCOM4_PAD2);

	gpio_set_pin_level(SCK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SCK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SCK, PINMUX_PB11D_SERCOM4_PAD3);
}

void SPI_1_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
	_gclk_enable_channel(SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC);
}

void SPI_1_init(void)
{
	SPI_1_CLOCK_init();
	spi_m_sync_init(&SPI_1, SERCOM4);
	SPI_1_PORT_init();
}

void SPI_0_PORT_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(MOTOR_MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(MOTOR_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(MOTOR_MISO, PINMUX_PB30D_SERCOM5_PAD0);

	gpio_set_pin_level(MOTOR_MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOTOR_MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOTOR_MOSI, PINMUX_PB00D_SERCOM5_PAD2);

	gpio_set_pin_level(MOTOR_SCK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOTOR_SCK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOTOR_SCK, PINMUX_PB01D_SERCOM5_PAD3);
}

void SPI_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM5);
	_gclk_enable_channel(SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_init(&SPI_0, SERCOM5);
	SPI_0_PORT_init();
}

void delay_driver_init(void)
{
	delay_init(SysTick);
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBA, RTC);
	_gclk_enable_channel(RTC_GCLK_ID, CONF_GCLK_RTC_SRC);
	timer_init(&TIMER_0, RTC, _rtc_get_timer());
}

void PWM_SERVO_PORT_init(void)
{

	gpio_set_pin_function(SERVO_PWM, PINMUX_PA07E_TCC1_WO1);
}

void PWM_SERVO_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TCC1);
	_gclk_enable_channel(TCC1_GCLK_ID, CONF_GCLK_TCC1_SRC);
}

void PWM_SERVO_init(void)
{
	PWM_SERVO_CLOCK_init();
	PWM_SERVO_PORT_init();
	pwm_init(&PWM_SERVO, TCC1, _tcc_get_pwm());
}

void PWM_MOTOR1_PORT_init(void)
{
#if 0
	gpio_set_pin_function(MOTOR_IN1, PINMUX_PA16E_TCC2_WO0);	// Set pin direction to input ???
	gpio_set_pin_function(MOTOR_IN2, PINMUX_PA17E_TCC2_WO1);
#endif
	gpio_set_pin_direction(MOTOR_IN1, GPIO_DIRECTION_OUT);

	gpio_set_pin_pull_mode(MOTOR_IN1,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(MOTOR_IN1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA17

	// Set pin direction to input
	gpio_set_pin_direction(MOTOR_IN2, GPIO_DIRECTION_OUT);

	gpio_set_pin_pull_mode(MOTOR_IN2,
	// <y> Pull configuration
	// <id> pad_pull_config
	// <GPIO_PULL_OFF"> Off
	// <GPIO_PULL_UP"> Pull-up
	// <GPIO_PULL_DOWN"> Pull-down
	GPIO_PULL_OFF);

	gpio_set_pin_function(MOTOR_IN2, GPIO_PIN_FUNCTION_OFF);
}

void PWM_MOTOR1_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TCC2);
	_gclk_enable_channel(TCC2_GCLK_ID, CONF_GCLK_TCC1_SRC);
}

void PWM_MOTOR1_init(void)
{
	PWM_MOTOR1_CLOCK_init();
	PWM_MOTOR1_PORT_init();
	pwm_init(&PWM_MOTOR1, TCC2, _tcc_get_pwm());
}

void USB_0_PORT_init(void)
{

	gpio_set_pin_direction(USB_DM,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(USB_DM,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(USB_DM,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(USB_DM,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA24G_USB_DM"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      PINMUX_PA24G_USB_DM);

	gpio_set_pin_direction(USB_DP,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(USB_DP,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(USB_DP,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(USB_DP,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA25G_USB_DP"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      PINMUX_PA25G_USB_DP);
}

/* The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
 * for low speed and full speed operation. */
#if (CONF_GCLK_USB_FREQUENCY > (48000000 + 48000000 / 400)) || (CONF_GCLK_USB_FREQUENCY < (48000000 - 48000000 / 400))
#warning USB clock should be 48MHz ~ 0.25% clock, check your configuration!
#endif

void USB_0_CLOCK_init(void)
{

	_pm_enable_bus_clock(PM_BUS_APBB, USB);
	_pm_enable_bus_clock(PM_BUS_AHB, USB);
	_gclk_enable_channel(USB_GCLK_ID, CONF_GCLK_USB_SRC);
}

void USB_0_init(void)
{
	USB_0_CLOCK_init();
	usb_d_init();
	USB_0_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA05

	// Set pin direction to input
	gpio_set_pin_direction(BUTTON, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(BUTTON,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(BUTTON, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA06

	gpio_set_pin_level(TAIL_LED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(TAIL_LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(TAIL_LED, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA13

	gpio_set_pin_level(uSD_CS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(uSD_CS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(uSD_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA14

	gpio_set_pin_level(LCD_BACKLIGHT_PWM,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_BACKLIGHT_PWM, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_BACKLIGHT_PWM, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA15

	gpio_set_pin_level(LCD_CONTRAST_PWM,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_CONTRAST_PWM, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_CONTRAST_PWM, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA18

	gpio_set_pin_level(MOTOR_SLEEP,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOTOR_SLEEP, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOTOR_SLEEP, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA19

	gpio_set_pin_level(MOTOR_MODE,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOTOR_MODE, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOTOR_MODE, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA20

	gpio_set_pin_level(LED_GREEN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_GREEN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_GREEN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21

	gpio_set_pin_level(uSD_CD,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(uSD_CD, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(uSD_CD, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA27

	gpio_set_pin_level(uSD_PWREN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(uSD_PWREN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(uSD_PWREN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA28

	gpio_set_pin_level(LCD_PWREN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_PWREN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_PWREN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB03

	gpio_set_pin_level(LED_RED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_RED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_RED, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB04

	// Set pin direction to input
	gpio_set_pin_direction(UI_BUTTON1, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(UI_BUTTON1,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(UI_BUTTON1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB05

	// Set pin direction to input
	gpio_set_pin_direction(UI_BUTTON2, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(UI_BUTTON2,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(UI_BUTTON2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB06

	// Set pin direction to input
	gpio_set_pin_direction(UI_BUTTON3, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(UI_BUTTON3,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(UI_BUTTON3, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB07

	gpio_set_pin_level(LCD_RS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_RS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_RS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08

	gpio_set_pin_level(LCD_E,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_E, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_E, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB09

	gpio_set_pin_level(LCD_RW,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_RW, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_RW, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB12

	gpio_set_pin_level(LCD_DB4,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_DB4, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_DB4, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB13

	gpio_set_pin_level(LCD_DB5,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_DB5, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_DB5, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB14

	gpio_set_pin_level(LCD_DB6,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_DB6, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_DB6, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB15

	gpio_set_pin_level(LCD_DB7,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LCD_DB7, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LCD_DB7, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB16

	// Set pin direction to input
	gpio_set_pin_direction(MOTOR_FAULT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(MOTOR_FAULT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(MOTOR_FAULT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB17

	// Set pin direction to input
	gpio_set_pin_direction(MOTOR_WDFLT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(MOTOR_WDFLT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(MOTOR_WDFLT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22

	gpio_set_pin_level(MOTOR_PWREN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOTOR_PWREN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOTOR_PWREN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB23

	gpio_set_pin_level(SERVO_PWREN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SERVO_PWREN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SERVO_PWREN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB31

	gpio_set_pin_level(MOTOR_CS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(MOTOR_CS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(MOTOR_CS, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();
	EXTERNAL_IRQ_0_init();

	USART_0_init();

	I2C_0_init();

	I2C_1_init();

	SPI_1_init();

	SPI_0_init();

	delay_driver_init();

	TIMER_0_init();

	PWM_SERVO_init();

	PWM_MOTOR1_init();

	USB_0_init();
}
