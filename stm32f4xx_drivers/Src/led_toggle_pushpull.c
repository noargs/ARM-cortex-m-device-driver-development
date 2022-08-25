#include "stm32f407xx.h"

void
delay(void){
	for(uint32_t i=0;i<500000;i++);
}

int
main(void) {

	// LED TOGGLE WITH PUSH-PULL
	GPIO_Handle_t gpio_led;

	gpio_led.GPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;

	GPIO_PCLK_Ctrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_led);

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
