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
