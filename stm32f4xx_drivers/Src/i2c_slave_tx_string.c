#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

// (1):
// Master (Arduino) send command code 0x51 to read
// the length (1 byte) of the data from the Slave (STM32)

// (2):
// Master send command code 0x52 to read the complete
// data from the Slave (STM32)

#define SLAVE_ADDR             0x68
#define MY_ADDR                SLAVE_ADDR

I2C_Handle_t i2c1_handle;

// limit characters to 32 bytes, Arduino Wire lib limitation
uint8_t tx_buffer[32] = "STM32 Slave mode testing..";


void delay(void)
{
  for (uint32_t i=0; i < 500000/2; i++);
}


// PB8 -> SCL
// PB9 -> SDA
void I2C1_GPIOInits(void)
{
  GPIO_Handle_t i2c_pins;
  i2c_pins.GPIOx = GPIOB;
  i2c_pins.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_ALTFN;
  i2c_pins.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_OD;

  // OR GPIO_NO_PUPD with external 4.7k Ohm resistor -> 3.3 VDD (both SDA and SCL)
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
  i2c1_handle.I2Cx = I2C1;
  i2c1_handle.I2C_Config.i2c_ack_control = I2C_ACK_ENABLE;
  i2c1_handle.I2C_Config.i2c_device_address = MY_ADDR;
  i2c1_handle.I2C_Config.i2c_fm_duty_cycle = I2C_FM_DUTY_2;
  i2c1_handle.I2C_Config.i2c_scl_speed = I2C_SCL_SPEED_SM;
  I2C_Init(&i2c1_handle);
}


int main(void)
{

  // configure pins (PB8, PB9) to behave as I2C pins
  I2C1_GPIOInits();

  // i2c peripheral configuration
  I2C1_Inits();

  // i2c irq configurations for event and error
  I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
  I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

  I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

  // enable the i2c peripheral
  I2C_PeripheralControl(I2C1, ENABLE);

  // PE not 0 hence enable the ACK [Reference Manual Page:861]
  // If PE=0 then ACK cannot be 1
  I2C_ManageACK(I2C1, I2C_ACK_ENABLE);

  while(1);

}

void I2C1_EV_IRQHandler (void)
{
  I2C_EV_IRQHandling(&i2c1_handle);
}

void I2C1_ER_IRQHandler (void)
{
  I2C_ER_IRQHandling(&i2c1_handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *i2c_handle, uint8_t APPLICATION_EVENT)
{
  static uint8_t command_code = 0;
  static uint8_t count = 0;

  if (APPLICATION_EVENT == I2C_EV_DATA_REQUEST)
  {
	// Master wants some data. Slave has to send it
	if (command_code == 0x51)
	{
	  // Send the data's length information to the master
	  I2C_SlaveSendData(i2c_handle->I2Cx, strlen((char*)tx_buffer));
	} else if (command_code == 0x52)
	{
	  // Send the contents of data (tx_buffer), which length info previously sent
	  I2C_SlaveSendData(i2c_handle->I2Cx, tx_buffer[count++]);
	}
  } else if (APPLICATION_EVENT == I2C_EV_DATA_RECEIVE)
  {
	// Slave has to read the data waiting.
	command_code = I2C_SlaveReceiveData(i2c_handle->I2Cx);

  } else if (APPLICATION_EVENT == I2C_ERROR_AF)
  {
	// This only happens in Slave tx (transmission)
	// Master has sent the NACK Hence Slave should not send any more data
	command_code = 0xFF;
	count = 0;
  } else if (APPLICATION_EVENT == I2C_EV_STOP)
  {
	// This only happens in Slave rx (Reception)
	// Master has ended the I2C communication with the Slave
  }
}











