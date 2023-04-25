#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

// Configuration structure for SPIx peripherals
typedef struct
{
	uint8_t spi_device_mode;       /* possible values from @SPI_Device_Mode */
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

/*
 * @SPI_Device_Mode
 */
#define SPI_DEVICE_MODE_SLAVE                0
#define SPI_DEVICE_MODE_MASTER               1


/*
 * @SPI_Bus_Config
 */
#define SPI_BUS_CONFIG_FD                     1     // Full-Duplex
#define SPI_BUS_CONFIG_HD                     2     // Half-Duplex
// #define SPI_BUS_CONFIG_SIMPLEX_TXONLY         3     // Simplex TX
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY         3     // Simplex RX


/*
 * @SPI_Sclk_Speed - Reference Manual SPI_CR1 Page:917
 */
#define SPI_SCLK_SPEED_DIV2                   0
#define SPI_SCLK_SPEED_DIV4                   1
#define SPI_SCLK_SPEED_DIV8                   2
#define SPI_SCLK_SPEED_DIV16                  3
#define SPI_SCLK_SPEED_DIV32                  4
#define SPI_SCLK_SPEED_DIV64                  5
#define SPI_SCLK_SPEED_DIV128                 6
#define SPI_SCLK_SPEED_DIV256                 7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS                         0 // default DataFrame
#define SPI_DFF_16BITS                        1

/*
 * @SPI_CPOL - Clock Polarity
 */
#define SPI_CPOL_LOW                          0
#define SPI_CPOL_HIGH                         1

/*
 * @SPI_CPOH - Clock Phase
 */
#define SPI_CPHA_LOW                          0
#define SPI_CPHA_HIGH                         1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI                            0 // Software Slave Managment
#define SPI_SSM_EN                            1 // default Hardware slave managment

//                              [APIs supported by this driver]
//              For more information about the APIs check the function definitions


// Peripheral clock setup
void SPI_PCLK_Ctrl (SPI_RegDef_t *spix, uint8_t enable_or_disable);

// Initialise and De-intialise GPIOs
void SPI_Init (SPI_Handle_t *spi_handle);
void SPI_DeInit (SPI_RegDef_t *spix);

// Data send and receive
void SPI_Send(SPI_RegDef_t *spix, uint8_t *tx_buffer, uint32_t len);
void SPI_Receive(SPI_RegDef_t *spix, uint8_t *rx_buffer, uint32_t len);

// IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t irq_number, uint8_t enable_or_disable);
void SPI_IRQPriorityConfig(uint8_t irq_number, uint32_t irq_priority);
void SPI_IRQHandling(SPI_Handle_t *spix_handle);

// other peripheral control APIs



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
