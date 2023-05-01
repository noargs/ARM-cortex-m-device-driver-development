#include <string.h>
#include "stm32f407xx.h"

// PB14 --> SPI2_MISO (not used)
// PB15 --> SPI2_MOSI
// PB13 --> SPI2_SCLK
// PB12 --> SPI2_NSS  (not used)
// ALT function mode : 5

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
//	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_14;
//	GPIO_Init(&spi_pins);

	// NSS
//	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
//	GPIO_Init(&spi_pins);
}

void SPI2_Inits(void) {
	SPI_Handle_t spi2_handle;

	spi2_handle.SPIx = SPI2;
	spi2_handle.SPIConfig.spi_bus_config = SPI_BUS_CONFIG_FD;
	spi2_handle.SPIConfig.spi_device_mode = SPI_DEVICE_MODE_MASTER;
	spi2_handle.SPIConfig.spi_sclk_speed = SPI_SCLK_SPEED_DIV2; // generate sclk of 8 MHz
	spi2_handle.SPIConfig.spi_dff = SPI_DFF_8BITS;
	spi2_handle.SPIConfig.spi_cpol = SPI_CPOL_LOW;
	spi2_handle.SPIConfig.spi_cpha = SPI_CPHA_LOW;
	spi2_handle.SPIConfig.spi_ssm = SPI_SSM_EN; // software slave managment enabled for NSS pin

	SPI_Init(&spi2_handle);

}

int main(void) {

	char user_data[] = "Hello world";

	char user_data[] = "An Arduino Uno board is best suited for beginners who has just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiast who require lot of I/O pins for their projects";

	// initialise the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// initialise/configure SPI2 peripheral
	SPI2_Inits();

	// this makes the NSS signal high internally, avoids the Master Mode fault
	SPI_SSIConfig(SPI2, ENABLE);

	// enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);


	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	// confitm the SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
