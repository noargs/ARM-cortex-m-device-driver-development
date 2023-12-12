#include "stm32f407xx_usart_driver.h"

void USART_Init(USART_Handle_t *usart_handle)
{
  uint32_t temp_reg = 0;

  //                    -[ configuration of CR1 ]-
  // enable the clock for given USART peripheral
  USART_PCLKControl(usart_handle->usartx, ENABLE);

  // enable USART Tx and Rx engines according to the @usart_mode configuration item
  if (usart_handle->usart_config.usart_mode == USART_MODE_ONLY_RX)
  {
	// enable the `receiver` bit field
	temp_reg |= (1 << USART_CR1_RE);
  } else if (usart_handle->usart_config.usart_mode == USART_MODE_ONLY_TX)
  {
	// enable the `transmitter` bit field
	temp_reg |= (1 << USART_CR1_TE);
  } else if (usart_handle->usart_config.usart_mode == USART_MODE_TXRX)
  {
	// enable the both `receiver` and `transmitter` bit fields
	temp_reg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
  }

  // configure the @word_length configuration item
  temp_reg |= usart_handle->usart_config.usart_word_length << USART_CR1_M;

  // configure @usart_parity_control bit field
  if (usart_handle->usart_config.usart_parity_control == USART_PARITY_EN_EVEN)
  {
	// enable parity control
	temp_reg |= (1 << USART_CR1_PCE);

	// enable EVEN parity
	// not required as EVEN parity will be selected by default upon enabling the parity control
  } else if (usart_handle->usart_config.usart_parity_control == USART_PARITY_EN_ODD)
  {
	// enable the parity control
	temp_reg |= (1 << USART_CR1_PCE);

	// enable the ODD parity
	temp_reg |= (1 << USART_CR1_PS);
  }

  // program the CR1 register
  usart_handle->usartx->CR1 = temp_reg;

  //                    -[ configuration of CR2 ]-

  temp_reg = 0;

  // configure the @usart_no_of_stop_bits inserted during USART frame transmission
  temp_reg |= usart_handle->usart_config.usart_no_of_stop_bits << USART_CR2_STOP;

  // program the CR2 register
  usart_handle->usartx->CR2 = temp_reg;

  //                    -[ configuration of CR3 ]-

  temp_reg = 0;

  // configure USART hardware flow control @usart_hw_flow_control
  if (usart_handle->usart_config.usart_hw_flow_control == USART_HW_FLOW_CONTROL_CTS)
  {
	// enable CTS flow control
	temp_reg |= (1 << USART_CR3_CTSE);
  } else if (usart_handle->usart_config.usart_hw_flow_control == USART_HW_FLOW_CONTROL_RTS)
  {
	// enable RTS flow control
	temp_reg |= (1 << USART_CR3_RTSE);
  } else if (usart_handle->usart_config.usart_hw_flow_control == USART_HW_FLOW_CONTROL_CTS_RTS)
  {
	// enable both CTS and RTS flow control
	temp_reg |= ((1 << USART_CR3_RTSE) | (1 << USART_CR3_CTSE));
  }

  // program the CR3 register
  usart_handle->usartx->CR3 = temp_reg;

  //                    -[ configuration of BRR (Baudrate register) ]-

  // configure the baudrate
  USART_SetBaudRate(usart_handle->usartx, usart_handle->usart_config.usart_baud);
}


void USART_SendData(USART_Handle_t *usart_handle, uint8_t *tx_buffer, uint32_t len)
{
  uint16_t *data;

  // loop over until `len` number of bytes are transferred
  for (uint32_t i = 0; i < len; i++)
  {
	// wait until TXE flag is set in the SR
	while (! USART_GetFlagStatus(usart_handle->usartx, USART_FLAG_TXE));

	// check the @word_length item for 9bit or 8bit in a frame
	if (usart_handle->usart_config.usart_word_length == USART_WORDLEN_9BITS)
	{
	  // 9bit configuration: load the DR with 2 bytes masking the bits other than first 9 bits
	  data = (uint16_t*)tx_buffer;
	  usart_handle->usartx->DR = (*data & (uint16_t)0x01FF);

	  // check the @usart_parity_control
	  if (usart_handle->usart_config.usart_parity_control == USART_PARITY_DISABLE)
	  {
		// no parity is used in this transfer. hence 9 bits of user data will be sent
		// increment tx_buffer twice
		tx_buffer++;
		tx_buffer++;
	  }
	  else
	  {
		// parity bit is used in this transfer. hence 8 bits of user data will be sent
		// 9th bit will be replaced by parity bit by the hardware
		tx_buffer++;
	  }
	}
	else
	{
	  // 8 bit data transfer
	  usart_handle->usartx->DR = (*tx_buffer & (uint8_t)0xFF);

	  // increment the buffer address
	  tx_buffer++;
	}
  }

  // wait till TC flag is set in the SR
  while(! USART_GetFlagStatus(usart_handle->usartx, USART_FLAG_TC));
}


void USART_ReceiveData(USART_Handle_t *usart_handle, uint8_t *rx_buffer, uint32_t len)
{
  // loop over until `len` number of byters are received
  for (uint32_t i = 0; i < len; i++)
  {
	// wait until RXNE flag is set in the SR
	while (! USART_GetFlagStatus(usart_handle->usartx, USART_FLAG_RXNE));

	// check the usart @word_length to decide whether to receive 9 bit data frame or 8 bit
	if (usart_handle->usart_config.usart_word_length == USART_WORDLEN_9BITS)
	{
	  // receive 9bit data in a frame

	  // check are we using @usart_parity_control or not
	  if (usart_handle->usart_config.usart_parity_control == USART_PARITY_DISABLE)
	  {
		// no parity is used. so, all 9bits will be of user data

		// read only first 9 bits, hence mask the DR with 0x01FF
		*((uint16_t*)rx_buffer) = (usart_handle->usartx->DR & (uint16_t)0x01FF);

		// increment the rx_buffer twice
		rx_buffer++;
		rx_buffer++;
	  }
	  else
	  {
		// parity is used, so 8bits will be of user data and 1 is parity bit
		*rx_buffer = (usart_handle->usartx->DR & (uint8_t)0xFF);

		// increment the buffer
		rx_buffer++;
	  }
	}
	else
	{
	  // receiving 8 bit data in a frame

	  // check if we are using @usart_parity_control or not
	  if (usart_handle->usart_config.usart_parity_control == USART_PARITY_DISABLE)
	  {
		// no parity is used, hence all 8 bits will be of user data

		// read 8 bits from DR
		*rx_buffer = (uint8_t)(usart_handle->usartx->DR & (uint8_t)0xFF);
	  }
	  else
	  {
		// parity is use, hence 7bits will be of user data and 1 bit is parity
		// read only 7 bits, hence mask the DR with 0x7F
		*rx_buffer = (uint8_t) (usart_handle->usartx->DR & (uint8_t)0x7F);
	  }

	  // increment the rx_buffer
	  rx_buffer++;
	}
  }

}


uint8_t USART_SendDataIT(USART_Handle_t *usart_handle, uint8_t *tx_buffer, uint32_t len)
{
  uint8_t tx_state = usart_handle->tx_busy_state;

  if (tx_state != USART_BUSY_IN_TX)
  {
	usart_handle->tx_len = len;
	usart_handle->tx_buffer = tx_buffer;
	usart_handle->tx_busy_state = USART_BUSY_IN_TX;

	// enable interrupt for TXE
    usart_handle->usartx->CR1 |= (1 << USART_CR1_TXEIE);

	// enable interrupt for TC
    usart_handle->usartx->CR1 |= (1 << USART_CR1_TCIE);
  }

  return tx_state;
}


uint8_t USART_ReceiveDataIT(USART_Handle_t *usart_handle, uint8_t *rx_buffer, uint32_t len)
{
  uint8_t rx_state = usart_handle->tx_busy_state;

  if (rx_state != USART_BUSY_IN_RX)
  {
	usart_handle->rx_len = len;
	usart_handle->rx_buffer = rx_buffer;
	usart_handle->rx_busy_state = USART_BUSY_IN_RX;

	(void)usart_handle->usartx->DR;

	// enable interrupt for RXNE
	usart_handle->usartx->CR1 |= (1 << USART_CR1_RXNEIE);
  }

  return rx_state;
}


void USART_SetBaudRate(USART_RegDef_t *usartx, uint32_t baud_rate)
{
  // APB clock
  uint32_t pclkx;
  uint32_t usartdiv;

  // Mantissa and Fraction values
  uint32_t mantissa, fraction;

  uint32_t temp_reg = 0;

  // APB bus clock into pclkx
  if (usartx == USART1 || usartx == USART6)
  {
	// USART1 and USART6 are hanging on APB2 bus
	pclkx = RCC_GetPCLK2Value();
  }
  else
  {
	pclkx = RCC_GetPCLK1Value();
  }

  // check OVER8 configuration bit
  if (usartx->CR1 & (1 << USART_CR1_OVER8))
  {
	// OVER8 = 1, oversampling by 8
	usartdiv = ((25 * pclkx) / (2 * baud_rate));
  }
  else
  {
	// oversampling by 16
	usartdiv = ((25 * pclkx) / (4 * baud_rate));
  }

  // calculate the Mantissa part
  mantissa = usartdiv/100;

  // Mantissa part in DIV_Mantissa[11:0], USART_BRR [Reference Manual page:1010]
  temp_reg |= mantissa << 4;

  // Extract the fraction part
  fraction = (usartdiv - (mantissa * 100));

  // calculate the final fractional
  if (usartx->CR1 & (1 << USART_CR1_OVER8))
  {
	// OVER8=1, oversampling by 8
	// When OVER8=1, the DIV_Fraction3 bit (i.e. Bit-3) is not considered and must be kept cleared.
	fraction = (((fraction * 8) + 50) / 100) & ((uint8_t)0x07);
  }
  else
  {
	// oversampling by 16
	fraction = (((fraction * 16) + 50) / 100) & ((uint8_t)0x0F);
  }

  // place the fractional part in DIV_Fraction[3:0], USART_BRR [Reference Manual page:1010]
  temp_reg |= fraction;

  // temp_reg to USART_BRR register
  usartx->BRR = temp_reg;

}


void USART_PeripheralControl(USART_RegDef_t *usartx, uint8_t ENABLE_OR_DISABLE)
{
  if (ENABLE_OR_DISABLE == ENABLE)
  {
	// UE `USART Enable` disabled by default [Reference Manual page:1010]
	usartx->CR1 |= (1 << 13);
  } else
  {
	usartx->CR1 &= ~(1 << 13);
  }
}


void USART_PCLKControl(USART_RegDef_t *usartx, uint8_t ENABLE_OR_DISABLE)
{
  if (ENABLE_OR_DISABLE == ENABLE)
  {
	if (usartx == USART1)
	{
	  USART1_PCLK_EN();
	} else if (usartx == USART2)
	{
	  USART2_PCLK_EN();
	} else if (usartx == USART3)
	{
	  USART3_PCLK_EN();
	} else if (usartx == UART4)
	{
	  UART4_PCLK_EN();
	} else if (usartx == UART5)
	{
	  UART5_PCLK_EN();
	} else if (usartx == USART6)
	{
	  USART6_PCLK_EN();
	}
  } else
  {
	if (usartx == USART1)
	{
	  USART1_PCLK_DI();
	} else if (usartx == USART2)
	{
	  USART2_PCLK_DI();
	} else if (usartx == USART3)
	{
	  USART3_PCLK_DI();
	} else if (usartx == UART4)
	{
	  UART4_PCLK_DI();
	} else if (usartx == UART5)
	{
	  UART5_PCLK_DI();
	} else if (usartx == USART6)
	{
	  USART6_PCLK_DI();
	}
  }
}


uint8_t USART_GetFlagStatus(USART_RegDef_t *usartx, uint8_t status_flag_name)
{
  if (usartx->SR & status_flag_name)
  {
	return SET;
  }
  return RESET;
}


void USART_IRQHandling(USART_Handle_t *usart_handle)
{
  uint32_t temp1, temp2, temp3;
  uint16_t *data;

  //                    -[ check TC flag ]-
  temp1 = usart_handle->usartx->SR & (1 << USART_SR_TC);

  // check the state of TCEIE bit
  temp2 = usart_handle->usartx->CR1 & (1 << USART_CR1_TCIE);

  if (temp1 && temp2)
  {
	// interrupt based on TC
	// close transmission and call application callback if `tx_len` is zero
	if (usart_handle->tx_busy_state == USART_BUSY_IN_TX)
	{
	  // check `tx_len` if zero then close the data transmission
	  if (! usart_handle->tx_len)
	  {
		// clear the TC flag
		usart_handle->usartx->SR &= ~(1 << USART_SR_TC);

		// clear the TCIE control bit

		// reset the application state
		usart_handle->tx_busy_state = USART_READY;

		// reset buffer address to NULL
		usart_handle->tx_buffer = NULL;

		// reset the length to zero
		usart_handle->tx_len = 0;

		// Call the application callback with event USART_EVENT_TX_COMPLETE
		USART_ApplicationEventCallback(usart_handle, USART_EVENT_TX_COMPLETE);
	  }
	}
  }

  //                    -[ check TXE flag ]-

  // check the state of TXE bit in the SR
  temp1 = usart_handle->usartx->SR & (1 << USART_SR_TXE);

  // check the state of TXEIE bit in the CR1
  temp2 = usart_handle->usartx->CR1 & (1 << USART_CR1_TXEIE);

  if (temp1 && temp2)
  {
	// interrupt due to TXE
	if (usart_handle->tx_busy_state == USART_BUSY_IN_TX)
	{
	  // keep sending data until `tx_len` reaches zero
	  if (usart_handle->tx_len > 0)
	  {
		if (usart_handle->usart_config.usart_word_length == USART_WORDLEN_9BITS)
		{
		  // 9 bit, load the DR with 2bytes masking the bits other than first 9 bits
		  data = (uint16_t*)usart_handle->tx_buffer;

		  // load only first 9 bits, hence maske with 0x01FF
		  usart_handle->usartx->DR = (*data & (uint16_t)0x01FF);

		  // check for `usart_parity_control`
		  if (usart_handle->usart_config.usart_parity_control == USART_PARITY_DISABLE)
		  {
			// no parity in this transfer, 9bits of user data will be set
			usart_handle->tx_buffer++;
			usart_handle->tx_buffer++;
			usart_handle->tx_len -= 2;
		  }
		  else
		  {
			// parity bit is used in this transfer. 8bits of user data will be sent
			// 9bit will be replaced by parity bit by the hardware
			  usart_handle->tx_buffer++;
			  usart_handle->tx_len -= 1;

		  }
		}
		else
		{
		  // 8bit data transfer
		  usart_handle->usartx->DR = (*usart_handle->tx_buffer & (uint8_t)0xFF);

		  usart_handle->tx_buffer++;
          usart_handle->tx_len -= 1;
		}
	  }
	  if (usart_handle->tx_len == 0)
	  {
		// `tx_len` is zero
		// clear the TXEIE bit (disable interrupt for TXE flag)
		usart_handle->usartx->CR1 &= ~(1 << USART_CR1_TXEIE);
	  }
	}
  }

  //                    -[ check RXNE flag ]-

  temp1 = usart_handle->usartx->SR & (1 << USART_SR_RXNE);
  temp2 = usart_handle->usartx->CR1 & (1 << USART_CR1_RXNEIE);

  if (temp1 && temp2)
  {
	// interrupt because of RXNE
	// interrupt because of TXE
	if (usart_handle->rx_busy_state == USART_BUSY_IN_RX)
	{
	  // TXE is set so send data
	  if (usart_handle->rx_len > 0)
	  {
		// check `usart_word_length` to decide to receive 9bit or 8bit data in a frame
		if (usart_handle->usart_config.usart_word_length == USART_WORDLEN_9BITS)
		{
		  // receive 9 bit data in a frame
		  // check `usart_parity_control` on or off
		  if (usart_handle->usart_config.usart_parity_control == USART_PARITY_DISABLE)
		  {
			// no parity, all 9bits will be user data
			// read only first 9bits so mask the rest
			*((uint16_t*)usart_handle->rx_buffer) = (usart_handle->usartx->DR & (uint16_t)0x01FF);

			usart_handle->rx_buffer++;
			usart_handle->rx_buffer++;
            usart_handle->rx_len -= 2;
		  }
		  else
		  {
			// parity is used, 8bits will be user data and 1 bit is parity
			*usart_handle->rx_buffer = (usart_handle->usartx->DR & (uint8_t)0xFF);

			usart_handle->rx_buffer++;
			usart_handle->rx_len -= 1;
		  }
		}
		else
		{
		  // receive 8bit data in a frame
		  // check `usart_parity_control` on or of
		  if (usart_handle->usart_config.usart_parity_control == USART_PARITY_DISABLE)
		  {
			// no parity is used, all 8 bits will be user data
			// read 8 bits from DR
			*usart_handle->rx_buffer = (uint8_t)(usart_handle->usartx->DR & (uint8_t)0xFF);
		  }
		  else
		  {
			// parity is used, 7 bits will be of user data and 1 bit for parity
			// read only 7bits, hence mask the DR with 0x7F
			*usart_handle->rx_buffer = (uint8_t)(usart_handle->usartx->DR & (uint8_t)0x7F);
		  }

		  usart_handle->rx_buffer++;
		  usart_handle->rx_len -= 1;
		}
	  }

	  if (! usart_handle->rx_len)
	  {
		// disable the RXNE
		usart_handle->usartx->CR1 &= ~(1 << USART_CR1_RXNEIE);
		usart_handle->rx_busy_state = USART_READY;
		USART_ApplicationEventCallback(usart_handle, USART_EVENT_RX_COMPLETE);
	  }
	}
  }

  //                    -[ check CTS flag ]-
  // Note: CTS feature is not applicable for UART4 and UART5

  // check the status of CTS bit in the SR
  temp1 = usart_handle->usartx->SR & (1 << USART_SR_CTS);

  // check the sate of CTSE bit in CR1
  temp2 = usart_handle->usartx->CR3 & (1 << USART_CR3_CTSE);

  // check the state of CTSIE bit in CR3 (this is no available for UART4 and UART5)
  temp3 = usart_handle->usartx->CR3 & (1 << USART_CR3_CTSIE);

  if (temp1 && temp2)
  {
	// clear the CTS flag in SR
	usart_handle->usartx->SR &= ~(1 << USART_SR_CTS);

	// interrupt is because of CTS
	USART_ApplicationEventCallback(usart_handle, USART_EVENT_CTS);
  }

  //                    -[ check IDLE detection flag ]-

  // check the status IDLE flag bit in the SR
  temp1 = usart_handle->usartx->SR & (1 << USART_SR_IDLE);

  // check the state of IDLEIE bit in CR1
  temp2 = usart_handle->usartx->CR1 & (1 << USART_CR1_IDLEIE);

  if (temp2 && temp2)
  {
	// clear the IDLE flag, Refer to RM to understand the clear sequence
	temp1 = usart_handle->usartx->SR &= ~(1 << USART_SR_IDLE);

	// interrupt is because of idle
	USART_ApplicationEventCallback(usart_handle, USART_EVENT_IDLE);
  }

  //                    -[ check Overrun detection flag ]-

  // check the status of ORE flag in the SR
  temp1 = usart_handle->usartx->SR & USART_SR_ORE;

  // check the status of RXNEIE bit in the CR1
  temp2 = usart_handle->usartx->CR1 & USART_CR1_RXNEIE;

  if (temp1 && temp2)
  {
	// need not to clear ORE flag here, instead given an api for the application to clear the ORE flag


	// this interrupt is because of Overrun error
	USART_ApplicationEventCallback(usart_handle, USART_ERR_ORE);
  }

  //                    -[ check Error flag ]-

  // Noise flag, Overrun error and Framming error in multibuffer communication
  // we dont discuss multibuffer communication in this course. please refer to the RM
  // the below code will get executed in only if multibuffer mode is used

  temp2 = usart_handle->usartx->CR3 & (1 << USART_CR3_EIE);

  if (temp2)
  {
	temp1 = usart_handle->usartx->SR;
	if (temp1 & (1 << USART_SR_FE))
	{
	  // this bit is set by hw when de-synchronisation, excessive noise or a break character
	  // is detected. it is cleared by software sequence (reading USART_SR register
	  // followed by reading the USART_DR register)
	  USART_ApplicationEventCallback(usart_handle, USART_ERR_FE);
	}

	if (temp1 & (1 << USART_SR_NE))
	{
	  // this bit is set by hw when noise is detected on a received frame. it is cleared
	  // by a software sequence (an read to the USART_SR register followed by read to the
	  // USART_DR register
	  USART_ApplicationEventCallback(usart_handle, USART_ERR_NE);
	}

	if (temp1 & (1 << USART_SR_ORE))
	{
	  USART_ApplicationEventCallback(usart_handle, USART_ERR_ORE);
	}
  }

}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *usart_handle, uint8_t EVENT)
{

}

void USART_IRQInterruptConfig(uint8_t irq_number, uint8_t enable_or_disable)
{
  if(enable_or_disable == ENABLE)
  {
	if(irq_number <= 31)
	{
	  // program ISER0 register
	  *NVIC_ISER0 |= (1 << irq_number);
	} else if(irq_number > 31 && irq_number < 64)
	{
	  // program ISER1 regsiter
	  *NVIC_ISER1 |= (1 << (irq_number % 32));
	} else if(irq_number >= 64 && irq_number < 96)
	{
	  // program ISER2 register
	  *NVIC_ISER2 |= (1 << (irq_number % 64));
	}
  } else
  {
	if(irq_number <= 31)
	{
	  // program ICER0 register
	  *NVIC_ICER0 |= (1 << irq_number);
	} else if(irq_number > 31 && irq_number < 64)
	{
	  // program ICER1 regsiter
	  *NVIC_ICER1 |= (1 << (irq_number % 32));
	} else if(irq_number >= 64 && irq_number < 96)
	{
	  // program ICER2 register
	  *NVIC_ICER2 |= (1 << (irq_number % 64));
	}
  }
}


void USART_IRQPriorityConfig(uint8_t irq_number, uint32_t irq_priority)
{
  uint8_t iprx = irq_number / 4;
  uint8_t iprx_section = irq_number % 4;
  uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
  *(NVIC_PR_BASE_ADDR + iprx) |= (irq_priority << shift_amount);
}































