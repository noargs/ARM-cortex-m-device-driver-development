#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles();

#define MY_ADDR                0x61
#define SLAVE_ADDR             0x68

void delay(void)
{
  for (uint32_t i=0; i < 500000/2; i++);
}

I2C_Handle_t i2c1_handle;

uint8_t receive_buffer[32];

// PB6 -> SCL
// PB7 -> SDA
void I2C1_GPIOInits(void)
{
  GPIO_Handle_t i2c_pins;
  i2c_pins.GPIOx = GPIOB;
  i2c_pins.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_ALTFN;
  i2c_pins.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_OD;
  i2c_pins.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;
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
  i2c1_handle.I2Cx = I2C1;
  i2c1_handle.I2C_Config.i2c_ack_control = I2C_ACK_ENABLE;
  i2c1_handle.I2C_Config.i2c_device_address = MY_ADDR;
  i2c1_handle.I2C_Config.i2c_fm_duty_cycle = I2C_FM_DUTY_2;
  i2c1_handle.I2C_Config.i2c_scl_speed = I2C_SCL_SPEED_SM;
  I2C_Init(&i2c1_handle);
}

void GPIO_ButtonInit(void)
{
  GPIO_Handle_t gpio_button;

  // Button configuration
  gpio_button.GPIOx = GPIOA;
  gpio_button.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_0;
  gpio_button.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IN;
  gpio_button.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
  gpio_button.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;
  GPIO_Init(&gpio_button);
}

int main(void)
{
  uint8_t command_code;
  uint8_t len;

  initialise_monitor_handles();

  printf("Application is running\n");

  GPIO_ButtonInit();

  // configure pins (PB6, PB7) to behave as I2C pins
  I2C1_GPIOInits();

  // i2c peripheral configuration
  I2C1_Inits();

  // enable the i2c peripheral
  I2C_PeripheralControl(I2C1, ENABLE);

  // PE not 0 hence enable the ACK [Reference Manual Page:861]
  // If PE=0 then ACK cannot be 1
  I2C_ManageACK(I2C1, I2C_ACK_ENABLE);

  while(1)
  {
	// wait till button is pressed
	while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
	delay();

	// query the `length information` of the actual data
	// which will be received in the next I2C transaction
	command_code = 0x51;
	I2C_MasterSendData(&i2c1_handle, &command_code, 1, SLAVE_ADDR);
	I2C_MasterReceiveData(&i2c1_handle, &len, 1, SLAVE_ADDR);

	// tell the Slave to send `actual data` whose length info was
	// queried in the previous I2C transaction.
	command_code = 0x52;
	I2C_MasterSendData(&i2c1_handle, &command_code, 1, SLAVE_ADDR);
	I2C_MasterReceiveData(&i2c1_handle, receive_buffer, len, SLAVE_ADDR);

	receive_buffer[len+1] = '\0';

	printf("Data : %s", receive_buffer);
  }

}
