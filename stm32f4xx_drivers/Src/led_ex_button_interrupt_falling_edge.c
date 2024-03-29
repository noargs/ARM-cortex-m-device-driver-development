/*
 * Connect an external button to PD5 pin and toggle the led
 * connected to PD12 whenever interrupt is triggered by the
 * button press
 *
 * Interrupt should be triggered during falling edge of
 * button press
 */

#include <string.h>
#include "stm32f407xx.h"

#define HIGH                                        1
#define LOW                                         0
#define BTN_PRESSED                                 LOW

void delay(void)
{
	// introduce ~200ms delay on 16MHz system clock
	for(uint32_t i=0; i < 500000/2; i++);
}

int main (void) {
	GPIO_Handle_t gpio_led, gpio_button;
	memset(&gpio_led, 0,sizeof(gpio_led));
	memset(&gpio_button, 0, sizeof(gpio_button));

	// this is LED gpio configuration
	gpio_led.GPIOx = GPIOD; // ((GPIO_RegDef_t*) GPIOD_BASEADDR)
	gpio_led.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_LOW;
	gpio_led.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;

	GPIO_PCLK_Ctrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_led);

	// this is Button gpio configuration
	gpio_button.GPIOx = GPIOD;
	gpio_button.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_5;
	gpio_button.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IT_FT;
	gpio_button.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_button.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	gpio_button.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU;

	GPIO_PCLK_Ctrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_button);

	// IRQ configuration to enable
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	return 0;
}

void EXTI9_5_IRQHandler (void) {
	delay(); // 200ms - wait till debouncing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_5); // clear the pending event from EXTI line
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
