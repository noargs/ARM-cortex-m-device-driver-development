#include "stm32f407xx_usart_driver.h"




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

