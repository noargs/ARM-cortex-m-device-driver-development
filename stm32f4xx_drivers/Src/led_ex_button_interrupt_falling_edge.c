/*
 * Connect an external button to PD5 pin and toggle the led
 * connected to PD12 whenever interrupt is triggered by the
 * button press
 *
 * Interrupt should be triggered during falling edge of
 * button press
 */


#include "stm32f407xx.h"

#define HIGH                                        1
#define LOW                                         0
#define BTN_PRESSED                                 LOW

int main (void) {
	GPIO_Handle_t gpio_led, gpio_button;

	// this is LED gpio configuration
	gpio_led.GPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_LOW;
	gpio_led.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;

	GPIO_PCLK_Ctrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_led);

	// this is Button gpio configuration
	gpio_led.GPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_5;
	gpio_led.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IT_FT;
	gpio_led.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_LOW;
	gpio_led.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU;

	GPIO_PCLK_Ctrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_button);

	// IRQ configuration to enable
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	return 0;
}

void EXTI9_5_IRQHandler (void) {
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}