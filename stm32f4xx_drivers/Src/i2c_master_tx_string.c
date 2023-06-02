#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define MY_ADDR                      0x61
#define SLAVE_ADDR                   0x68

I2C_Handle_t i2c_handle;
uint8_t data[] = "We are testing I2C master Tx\n";

void delay(void)
{
	for (uint32_t i=0; i < 500000/2; i++);
}


// PB6 -> SCL
// PB7 -> SDA
void I2C1_GPIOInits(void)
{
  // Configure pins (PB6, PB7) to behave as I2C pins
  GPIO_Handle_t i2c_pins;

  i2c_pins.GPIOx = GPIOB;
  i2c_pins.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_ALTFN;
  i2c_pins.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_OD;
  i2c_pins.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU;
  i2c_pins.GPIO_PinConfig.gpio_pin_alt_fun = 4;
  i2c_pins.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;

  // SCL
  i2c_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_6;
  GPIO_Init(&i2c_pins);

  // SDA
  i2c_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_7;
  GPIO_Init(&i2c_pins);


}


void I2C1_Inits(void)
{
  i2c_handle.I2Cx = I2C1;
  i2c_handle.I2C_Config.i2c_ack_control = I2C_ACK_ENABLE;
  i2c_handle.I2C_Config.i2c_device_address = MY_ADDR;
  i2c_handle.I2C_Config.i2c_fm_duty_cycle = I2C_FM_DUTY_2;
  i2c_handle.I2C_Config.i2c_scl_speed = I2C_SCL_SPEED_SM;
  I2C_Init(&i2c_handle);
}

void GPIO_ButtonInit(void)
{
  GPIO_Handle_t gpio_button, gpio_led;

  // Button configuration
  gpio_button.GPIOx = GPIOA;
  gpio_button.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_0;
  gpio_button.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IN;
  gpio_button.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
  gpio_button.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;
  GPIO_Init(&gpio_button);

  // LED configuration
  gpio_led.GPIOx = GPIOD;
  gpio_led.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
  gpio_led.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_OUT;
  gpio_led.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
  gpio_led.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_OD;
  gpio_led.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;
  GPIO_Init(&gpio_led);
}

int main(void)
{
  // configure pins (PB6, PB9) to behave as I2C pins
  I2C1_GPIOInits();

  // i2c peripheral configuration
  I2C1_Inits();

  // enable the i2c peripheral
  I2C_PeripheralControl(I2C1, ENABLE);

  while(1)
  {
	// wait till button is pressed
	while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

	// avoid button de-bouncing 200ms of delay
	delay();

	// send some data to the slave
	I2C_MasterSendData(&i2c_handle, data, strlen((char*)data), SLAVE_ADDR);
  }

}