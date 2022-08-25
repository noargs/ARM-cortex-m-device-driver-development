#include "stm32f407xx.h"

// connect external button to pin PB12 and external LED
// to PA8 and toggle the LED whenever external button
// is pressed
// when button is pressed BUTTON_PRESSED = 0, Low because
// one side of button is connected to GND

#define LOW                           0
#define BUTTON_PRESSED                LOW

void
delay(void){
	for(uint32_t i=0;i<500000/2;i++);
}

int
main(void) {

	GPIO_Handle_t gpio_led, gpio_button;

	gpio_led.GPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_8;
	gpio_led.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	gpio_led.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;

	GPIO_PCLK_Ctrl(GPIOA, ENABLE);
	GPIO_Init(&gpio_led);

	gpio_button.GPIOx = GPIOB;
	gpio_button.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	gpio_button.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IN;
	gpio_button.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_button.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU; // external pull-down resistor is available on stm32f4 discovery board

	GPIO_PCLK_Ctrl(GPIOB, ENABLE);
	GPIO_Init(&gpio_button);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BUTTON_PRESSED){
			delay(); // prevent de-bouncing of the button
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}

	return 0;
}

