#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

// Arduino sketch for testing this User application.
// pair_device/usart/./usart_tx_string.ino

// 3 different messages to transmit to arduino
// you can add more into array
char *msg[3] = {"https://github.com/noargs", "ARM-cortex-m-device-driver-development", "test and chill!"};

// reply from arduino will be stored
char rx_buf[1024];

USART_Handle_t usart2_handle;

// following flag indicates Reception complete
uint8_t rx_complete = RESET;

uint8_t g_data = 0;

extern void initialise_monitor_handles();

void USART2_Init(void)
{
  usart2_handle.usartx = USART2;
  usart2_handle.usart_config.usart_baud = USART_STD_BAUD_115200;
  usart2_handle.usart_config.usart_hw_flow_control = USART_HW_FLOW_CONTROL_NONE;
  usart2_handle.usart_config.usart_mode = USART_MODE_TXRX;
  usart2_handle.usart_config.usart_no_of_stop_bits = USART_STOPBITS_1;
  usart2_handle.usart_config.usart_word_length = USART_WORDLEN_8BITS;
  usart2_handle.usart_config.usart_parity_control = USART_PARITY_DISABLE;
  USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void)
{
  GPIO_Handle_t usart_gpios;
  usart_gpios.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_ALTFN;
  usart_gpios.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
  usart_gpios.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU;
  usart_gpios.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
  usart_gpios.GPIO_PinConfig.gpio_pin_alt_fun = 7;

  // USART2 TX
  usart_gpios.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_2;
  GPIO_Init(&usart_gpios);

  // USART2 RX
  usart_gpios.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_3;
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

void delay(void)
{
  for (uint32_t i=0; i < 500000/2; i++);
}


int main(void)
{
  uint32_t count = 0;
  GPIO_ButtonInit();
  initialise_monitor_handles();
  USART2_GPIOInit();
  USART2_Init();

  USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);
  USART_PeripheralControl(USART2, ENABLE);

  printf("Application is running\n");

  while (1)
  {
	while (! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
	delay();

	// next message index; make sure that `count` value doesnt cross 2
	count = count % 3;

	// First enable the `Reception` in Interrupt mode
	// following code enables the Receive Interrupt
	while (USART_ReceiveDataIT(&usart2_handle, (uint8_t*)rx_buf, strlen(msg[count])) != USART_READY);

	// send the message indexed by count in blocking mode
	USART_SendData(&usart2_handle, (uint8_t*)msg[count], strlen(msg[count]));

	printf("Transmitted: %s\n", msg[count]);

	// wait until all the bytes are received from the Arduino
	// when all the bytes are received rx_complete will be SET in application callback
	while(rx_complete != SET);

	// make sure that last byte should be null otherwise %s fails while printing
	rx_buf[strlen(msg[count]) + 1] = '\0';

	// print what we received from the Arduino
	printf("Received: %s\n", rx_buf);

	// invalidate the flag
	rx_complete = RESET;

	// move on to next message indexed in msg[]
	count++;
  }
  return 0;
}

void USART2_IRQHandler(void)
{
  USART_IRQHandling(&usart2_handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *usart_handle, uint8_t APPLICATION_EVENT)
{
  if (APPLICATION_EVENT == USART_EVENT_RX_COMPLETE)
  {
	rx_complete = SET;
  } else if (APPLICATION_EVENT == USART_EVENT_TX_COMPLETE)
  {
	;
  }
}











