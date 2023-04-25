#include "stm32f407xx_spi_driver.h"

// Peripheral clock setup
void SPI_PCLK_Ctrl(SPI_RegDef_t *spix, uint8_t enable_or_disable) {
	if (enable_or_disable == ENABLE) {
		if (spix == SPI1) {
			SPI1_PCLK_EN();
		} else if (spix == SPI2) {
			SPI2_PCLK_EN();
		} else if (spix == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (spix == SPI1) {
			SPI1_PCLK_DI();
		} else if (spix == SPI2) {
			SPI2_PCLK_DI();
		} else if (spix == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

// Initialise and De-intialise GPIOs
void SPI_Init(SPI_Handle_t *spi_handle) {
	// configure the SPI_CR1 register
	uint32_t temp_register = 0;

	//1. configure the device mode
	temp_register |= spi_handle->SPIConfig.spi_device_mode << SPI_CR1_MSTR; // 2 -> MSTR bit RM page:916

	//2. bus configuration
	if (spi_handle->SPIConfig.spi_bus_config == SPI_BUS_CONFIG_FD) {
		// bidi mode should be cleared - 15 -> BIDIMODE, 0 -> 2-line unidirection data mode
		temp_register &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(spi_handle->SPIConfig.spi_bus_config == SPI_BUS_CONFIG_HD) {
		// bidi mode should be set
		temp_register |= (1 << SPI_CR1_BIDIMODE);
	} else if (spi_handle->SPIConfig.spi_bus_config == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// bidi mode should be cleared
		temp_register &= ~(1 << SPI_CR1_BIDIMODE);

		// RXONLY bit must be set
		temp_register |= (1 << SPI_CR1_RXONLY);
	}

	// 3. configure the SPI Serial clock speed (baud rate)
	temp_register |= spi_handle->SPIConfig.spi_sclk_speed << SPI_CR1_BR;

	// 4. configure the Data frame format (DFF)
	temp_register |= spi_handle->SPIConfig.spi_dff << SPI_CR1_DFF;

	// 5. configure the CPOL - Clock polarity
	temp_register |= spi_handle->SPIConfig.spi_cpol << SPI_CR1_CPOL;

	// 6. configure the CPHA
	temp_register |= spi_handle->SPIConfig.spi_cpha << SPI_CR1_CPHA;

	spi_handle->SPIx->CR1 = temp_register;

}

void SPI_DeInit(SPI_RegDef_t *spix) {
	if (spix == SPI1) {
		SPI1_REG_RESET();
	} else if (spix == SPI2) {
		SPI2_REG_RESET();
	} else if (spix == SPI3) {
		SPI3_REG_RESET();
	}
}

// Data send and receive
void SPI_Send(SPI_RegDef_t *spix, uint8_t *tx_buffer, uint32_t len) {}

void SPI_Receive(SPI_RegDef_t *spix, uint8_t *rx_buffer, uint32_t len) {}

// IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t irq_number, uint8_t enable_or_disable) { }

void SPI_IRQPriorityConfig(uint8_t irq_number, uint32_t irq_priority) { }

void SPI_IRQHandling(SPI_Handle_t *spix_handle){ }
