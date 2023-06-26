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
	temp_reg = (1 << USART_CR1_RE);
  } else if (usart_handle->usart_config.usart_mode == USART_MODE_ONLY_TX)
  {
	// enable the `transmitter` bit field
	temp_reg = (1 << USART_CR1_TE);
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
  usart_handle->usartx->CR1 |= temp_reg;

  //                    -[ configuration of CR2 ]-

  temp_reg = 0;

  // configure the @usart_no_of_stop_bits inserted during USART frame transmission
  temp_reg |= usart_handle->usart_config.usart_no_of_stop_bits << USART_CR2_STOP;

  // program the CR2 register
  usart_handle->usartx->CR2 |= temp_reg;

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
  usart_handle->usartx->CR3 |= temp_reg;

  //                    -[ configuration of BRR (Baudrate register) ]-

  temp_reg = 0;

  // configure the baudrate


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

