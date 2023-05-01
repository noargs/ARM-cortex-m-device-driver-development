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

	// peripheral clock enable
	SPI_PCLK_Ctrl(spi_handle->SPIx, ENABLE);

	// first configure the SPI_CR1 register
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


void SPI_PeripheralControl(SPI_RegDef_t *spix, uint8_t enable_or_disable){
	if (enable_or_disable == ENABLE) {
		spix->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		spix->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *spix, uint8_t enable_or_disable) {
	if (enable_or_disable == ENABLE) {
		spix->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		spix->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *spix, uint8_t enable_or_disable) {
	if (enable_or_disable == ENABLE) {
		spix->CR1 |= (1 << SPI_CR2_SSOE);
	} else {
		spix->CR1 &= ~(1 << SPI_CR2_SSOE);
	}
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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *spix, uint32_t flag_name) {
	if (spix->SR & flag_name){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

// Data send and receive - blocking API
void SPI_SendData(SPI_RegDef_t *spix, uint8_t *tx_buffer, uint32_t len) {
	while(len > 0) {
		// 1. wait until TXE is empty ( Tx buffer empty : 1 Reference Manual Page:919 )
	    // while(!(spix->SR & (1 << 1)));
		while( SPI_GetFlagStatus(spix, SPI_TXE_FLAG) == FLAG_RESET );

		//2. check the DFF bit in CR1
		if (spix->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bit DFF
			// 1. load the data into the DR
			spix->DR = *((uint16_t*)tx_buffer);
			len--;
			len--;
			(uint16_t*)tx_buffer++;
		} else {
			// 8 bit DFF
			spix->DR = *tx_buffer;
			len--;
			tx_buffer++;
		}
	}
}

void SPI_Receive(SPI_RegDef_t *spix, uint8_t *rx_buffer, uint32_t len) {}

// IRQ configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t irq_number, uint8_t enable_or_disable) { }

void SPI_IRQPriorityConfig(uint8_t irq_number, uint32_t irq_priority) { }

void SPI_IRQHandling(SPI_Handle_t *spix_handle){ }
