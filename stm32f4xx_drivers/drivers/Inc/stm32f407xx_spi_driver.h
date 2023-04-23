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


//                              [APIs supported by this driver]
//              For more information about the APIs check the function definitions


// Peripheral clock setup
void SPI_PCLK_Ctrl (SPI_RegDef_t *spix, uint8_t en_or_di);

// Initialise and De-intialise GPIOs
void SPI_Init (SPI_Handle_t *spi_handle);
void SPI_DeInit (SPI_RegDef_t *spix);

// Data send and receive


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
