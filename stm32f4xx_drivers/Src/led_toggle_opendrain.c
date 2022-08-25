#include "stm32f407xx.h"

// using open drain the circuit is floating
// so we have to enable internal or external
// pull up resistor, we will go far internal one
// because internal pull up resistor has value of
// 40K OHM therefore you will see very little brightness
// in led
void
delay(void){
	for(uint32_t i=0;i<500000;i++);
}

int
main(void) {

	// LED TOGGLE WITH OPEN DRAIN
	GPIO_Handle_t gpio_led;

	gpio_led.GPIOx = GPIOD;
	gpio_led.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	gpio_led.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_OD;
	gpio_led.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU;

	GPIO_PCLK_Ctrl(GPIOD, ENABLE);
	GPIO_Init(&gpio_led);

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
