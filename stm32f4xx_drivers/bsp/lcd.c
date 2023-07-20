#include "lcd.h"

static void write_4_bits(uint8_t value);
static void lcd_enable(void);

void lcd_send_command(uint8_t cmd)
{
	// RS = 0 for LCD command
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	// RnW = 0 for write
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	// Higher nibbles first
	write_4_bits(cmd >> 4);

	// Lower nibbles second
	write_4_bits(cmd & 0x0F);
}

void lcd_send_char(uint8_t data)
{
	// RS = 1 for LCD user Data
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	// RnW = 0 for write
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	// Higher nibble first
	write_4_bits(cmd >> 4);

	// Lower nibble second
	write_4_bits(cmd & 0x0F);
}

void lcd_init(void)
{
	//1. configure the GPIO pins used for LCD connections
	GPIO_Handle_t lcd_signal;

	lcd_signal.gpiox                               = LCD_GPIO_PORT;
	lcd_signal.gpio_config.gpio_pin_mode           = GPIO_MODE_OUT;
	lcd_signal.gpio_config.gpio_pin_op_type        = GPIO_OP_TYPE_PP;
	lcd_signal.gpio_config.gpio_pin_pu_pd_control  = GPIO_NO_PUPD;
	lcd_signal.gpio_config.gpio_pin_speed          = GPIO_SPEED_FAST;
	lcd_signal.gpio_config.gpio_pin_number         = LCD_GPIO_RS;
	GPIO_Init(&lcd_signal);

	lcd_signal.gpio_config.gpio_pin_number         = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.gpio_config.gpio_pin_number         = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	lcd_signal.gpio_config.gpio_pin_number         = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.gpio_config.gpio_pin_number         = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.gpio_config.gpio_pin_number         = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.gpio_config.gpio_pin_number         = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//2. LCD Initialisation

	mdelay(40); // milliseconds delay

	// RS = 0, For LCD command
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	// RnW = 0, Writing to LCD
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(0x3); // (MSB) 0 0 1 1 (LSB)

	mdelay(5);

	write_4_bits(0x3);

	udelay(150);

	write_4_bits(0x3);

	write_4_bits(0x2);

}

// writes 4 bits of data/command on to D4,D5,D6,D7 lines
static void write_4_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x1) );
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x1) );

	lcd_enable();
}

static void lcd_enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100); // execution time > 37 microseconds (Datasheet page: 24)
}

