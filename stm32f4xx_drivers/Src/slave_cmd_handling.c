#include <string.h>
#include "stm32f407xx.h"

// command codes
#define COMMAND_LED_CTRL                      0x50
#define COMMAND_SENSOR_READ                   0x51
#define COMMAND_LED_READ                      0x52
#define COMMAND_PRINT                         0x53
#define COMMAND_ID_READ                       0x54

#define LED_ON                                1
#define LED_OFF                               0

// arduino analogue pins
#define ANALOG_PIN0                           0
#define ANALOG_PIN1                           1
#define ANALOG_PIN2                           2
#define ANALOG_PIN3                           3
#define ANALOG_PIN4                           4

// arduino led
#define LED_PIN_9                             9

// PB14 --> SPI2_MISO
// PB15 --> SPI2_MOSI
// PB13 --> SPI2_SCLK
// PB12 --> SPI2_NSS
// ALT function mode : 5

void delay(void) {
	for (uint32_t i=0; i<500000/2; i++);
}

void SPI2_GPIOInits(void) {
	GPIO_Handle_t spi_pins;

	spi_pins.GPIOx = GPIOB;
	spi_pins.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_ALTFN;
	spi_pins.GPIO_PinConfig.gpio_pin_alt_fun = 5;
	spi_pins.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	spi_pins.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;
	spi_pins.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;

	// SCLK
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_13;
	GPIO_Init(&spi_pins);

	// MOSI
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_15;
	GPIO_Init(&spi_pins);

	// MISO
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_14;
	GPIO_Init(&spi_pins);

	// NSS
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	GPIO_Init(&spi_pins);
}

void SPI2_Inits(void) {
	SPI_Handle_t spi2_handle;

	spi2_handle.SPIx = SPI2;
	spi2_handle.SPIConfig.spi_bus_config = SPI_BUS_CONFIG_FD;
	spi2_handle.SPIConfig.spi_device_mode = SPI_DEVICE_MODE_MASTER;
	spi2_handle.SPIConfig.spi_sclk_speed = SPI_SCLK_SPEED_DIV8; // generate sclk of 2MHz
	spi2_handle.SPIConfig.spi_dff = SPI_DFF_8BITS;
	spi2_handle.SPIConfig.spi_cpol = SPI_CPOL_LOW;
	spi2_handle.SPIConfig.spi_cpha = SPI_CPHA_LOW;
	spi2_handle.SPIConfig.spi_ssm = SPI_SSM_DI; // Harware slave managment enabled for NSS pin

	SPI_Init(&spi2_handle);

}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t gpio_button;

	gpio_button.GPIOx = GPIOA;
	gpio_button.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_0;
	gpio_button.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IN;
	gpio_button.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_button.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;

	GPIO_Init(&gpio_button);

}

int main(void) {

	uint8_t dummy_byte = 0xff;

	GPIO_ButtonInit();

	// initialise the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// initialise/configure SPI2 peripheral
	SPI2_Inits();

	// this makes the NSS signal high internally, avoids the Master Mode fault
	// when SSM is disabled, SSI has no use!!
	// SPI_SSIConfig(SPI2, ENABLE);

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e. when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1) {

		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// first send length information to slave device
		// Arduino sketch expects 1 byte of length information followed by data
		uint8_t data_len = strlen(user_data);
		SPI_SendData(SPI2, &data_len, 1);

		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// confitm the SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// disable after a data communication
		SPI_PeripheralControl(SPI2, DISABLE);

	}

	return 0;
}
