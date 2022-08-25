#include "stm32f407xx.h"

// press User button in stm32f4 discovery board to
// toggle the on-board led attached to PA0

#define HIGH                          1
#define BUTTON_PRESSED                HIGH

void
delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}

int
main(void) {

	GPIO_Handle_t gpio_led, gpio_button;

	gpio_led.GPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;

	GPIO_PCLK_Ctrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_led);

	gpio_button.GPIOx = GPIOA;
	gpio_button.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_0;
	gpio_button.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IN;
	gpio_button.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_button.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD; // external pull-down resistor is available on stm32f4 discovery board

	GPIO_PCLK_Ctrl(GPIOA, ENABLE);
	GPIO_Init(&gpio_button);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BUTTON_PRESSED){
			delay(); // prevent de-bouncing of the button
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}

	return 0;
}
