#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

// Configuration structure for SPIx peripherals
typedef struct
{
	uint8_t spi_device_mode;       /* possible values from @SPI_Device_Mode */
	uint8_t spi_bus_config;        /* possible values from @SPI_Bus_Config */
	uint8_t spi_sclk_speed;        /* possible values from @SPI_Sclk_Speed */
	uint8_t spi_dff;               /* possible values from @SPI_DFF */
	uint8_t spi_cpol;              /* possible values from @SPI_CPOL */
	uint8_t spi_cpha;              /* possible values from @SPI_CPOH */
	uint8_t spi_ssm;               /* possible values from @SPI_SSM */
}SPI_Config_t;

// Handle structure for SPIx peripheral
typedef struct
{
	SPI_RegDef_t *SPIx;
	SPI_Config_t SPIConfig;
	uint8_t      *tx_buffer;
	uint8_t      *rx_buffer;
	uint32_t     tx_len;
	uint32_t     rx_len;
	uint8_t      tx_state;
	uint8_t      rx_state;
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
#define SPI_SSM_DI                            0 // default Hardware slave managment
#define SPI_SSM_EN                            1 // Software Slave Managment

// SPI related status flags definitions
#define SPI_TXE_FLAG                           (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG                          (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG                          (1 << SPI_SR_BSY)

// Possible SPI application states
#define SPI_READY                              0
#define SPI_BUSY_IN_RX                         1
#define SPI_BUSY_IN_TX                         2

// Possible SPI Application events
#define SPI_EVENT_TX_COMPLETE                  1
#define SPI_EVENT_RX_COMPLETE                  2
#define SPI_EVENT_OVR_ERR                      3
#define SPI_EVENT_CRC_ERR                      4


//                              [APIs supported by this driver]
//              For more information about the APIs check the function definitions


// Peripheral clock setup
void SPI_PCLK_Ctrl (SPI_RegDef_t *spix, uint8_t enable_or_disable);

// Initialise and De-intialise GPIOs
void SPI_Init (SPI_Handle_t *spi_handle);
void SPI_DeInit (SPI_RegDef_t *spix);

// Data send and receive
void SPI_SendData(SPI_RegDef_t *spix, uint8_t *tx_buffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *spix, uint8_t *rx_buffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *spi_handle, uint8_t *tx_buffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *spi_handle, uint8_t *rx_buffer, uint32_t len);

// IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t irq_number, uint8_t enable_or_disable);
void SPI_IRQPriorityConfig(uint8_t irq_number, uint32_t irq_priority);
void SPI_IRQHandling(SPI_Handle_t *spi_handle);

// other peripheral control APIs
void SPI_PeripheralControl(SPI_RegDef_t *spix, uint8_t enable_or_disable);
void SPI_SSIConfig(SPI_RegDef_t *spix, uint8_t enable_or_disable);
void SPI_SSOEConfig(SPI_RegDef_t *spix, uint8_t enable_or_disable);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *spix, uint32_t flag_name);
void SPI_ClearOVRFlag(SPI_RegDef_t *spix);
void SPI_CloseTransmission(SPI_Handle_t *spi_handle);
void SPI_CloseReception(SPI_Handle_t *spi_handle);

// Application callback
void SPI_ApplicationEventCallback(SPI_Handle_t *spi_handle, uint8_t APPLICATION_EVENT);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
