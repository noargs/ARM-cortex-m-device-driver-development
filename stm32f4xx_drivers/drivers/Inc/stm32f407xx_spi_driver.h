#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

// Configuration structure for SPIx peripherals
typedef struct
{
	uint8_t spi_device_mode;
	uint8_t spi_bus_config;
	uint8_t spi_sclk_speed;
	uint8_t spi_dff;
	uint8_t spi_cpol;
	uint8_t spi_cpha;
	uint8_t spi_ssm;
}SPI_Config_t;

// Handle structure for SPIx peripheral
typedef struct
{
	SPI_RegDef_t *SPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
