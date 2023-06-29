#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

char msg[1024] = "UART Tx testing, https://github.com/noargs\n\r";

USART_Handle_t usart2_handle;

void delay(void)
{
  for(uint32_t i = 0; i < 500000/2; i++);
}

void USART2_Init(void)
{
  usart2_handle.usartx = USART2;
  usart2_handle.usart_config.usart_baud = USART_STD_BAUD_115200;
  usart2_handle.usart_config.usart_hw_flow_control = USART_HW_FLOW_CONTROL_NONE;
  usart2_handle.usart_config.usart_mode = USART_MODE_ONLY_TX;
  usart2_handle.usart_config.usart_no_of_stop_bits = USART_STOPBITS_1;
  usart2_handle.usart_config.usart_word_length = USART_WORDLEN_8BITS;
  usart2_handle.usart_config.usart_parity_control = USART_PARITY_DISABLE;
  USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
  GPIO_Handle_t usart_gpios;

  usart_gpios.GPIOx = GPIOA;
  usart_gpios.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_ALTFN;
  usart_gpios.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
  usart_gpios.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU;
  usart_gpios.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
  usart_gpios.GPIO_PinConfig.gpio_pin_alt_fun = 7;

  // USART2 TX
  usart_gpios.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_2;
//  usart_gpios.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_9;
  GPIO_Init(&usart_gpios);

  // USART2 RX
  usart_gpios.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_3;
//  usart_gpios.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_10;
  GPIO_Init(&usart_gpios);
}

void GPIO_ButtonInit(void)
{
  GPIO_Handle_t gpio_button;

  gpio_button.GPIOx = GPIOA;
  gpio_button.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_0;
  gpio_button.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IN;
  gpio_button.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
  gpio_button.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;
  GPIO_Init(&gpio_button);
}

int main(void)
{
  GPIO_ButtonInit();

  USART2_GPIOInit();

  USART2_Init();

  USART_PeripheralControl(USART2, ENABLE);

  while(1)
  {
    // wait till button is pressed
    while (! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
    delay();

    USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg));
  }

  return 0;
}
