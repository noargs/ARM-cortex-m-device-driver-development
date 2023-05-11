#include <string.h>
#include <stdio.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles();

// // Arduino sketch located at [slave/spi/slave_cmd_handling]
// PB14 --> SPI2_MISO
// PB15 --> SPI2_MOSI
// PB13 --> SPI2_SCLK
// PB12 --> SPI2_NSS
// ALT function mode : 5


// command codes
#define COMMAND_LED_CTRL                      0x50
#define COMMAND_SENSOR_READ                   0x51
#define COMMAND_LED_READ                      0x52
#define COMMAND_PRINT                         0x53
#define COMMAND_ID_READ                       0x54

#define LED_ON                                1
#define LED_OFF                               0

// arduino analogue pins
#define ANALOG_PIN0                           0
#define ANALOG_PIN1                           1
#define ANALOG_PIN2                           2
#define ANALOG_PIN3                           3
#define ANALOG_PIN4                           4

// arduino led
#define LED_PIN_9                             9


void delay(void) {
	for (uint32_t i=0; i<500000/2; i++);
}

void SPI2_GPIOInits(void) {
	GPIO_Handle_t spi_pins;

	spi_pins.GPIOx = GPIOB;
	spi_pins.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_ALTFN;
	spi_pins.GPIO_PinConfig.gpio_pin_alt_fun = 5;
	spi_pins.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	spi_pins.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU;
	spi_pins.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;

	// SCLK
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_13;
	GPIO_Init(&spi_pins);

	// MOSI
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_15;
	GPIO_Init(&spi_pins);

	// MISO
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_14;
	GPIO_Init(&spi_pins);

	// NSS
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	GPIO_Init(&spi_pins);
}

void SPI2_Inits(void) {
	SPI_Handle_t spi2_handle;

	spi2_handle.SPIx = SPI2;
	spi2_handle.SPIConfig.spi_bus_config = SPI_BUS_CONFIG_FD;
	spi2_handle.SPIConfig.spi_device_mode = SPI_DEVICE_MODE_MASTER;
	spi2_handle.SPIConfig.spi_sclk_speed = SPI_SCLK_SPEED_DIV8; // generate sclk of 2MHz
	spi2_handle.SPIConfig.spi_dff = SPI_DFF_8BITS;
	spi2_handle.SPIConfig.spi_cpol = SPI_CPOL_LOW;
	spi2_handle.SPIConfig.spi_cpha = SPI_CPHA_LOW;
	spi2_handle.SPIConfig.spi_ssm = SPI_SSM_DI; // Harware slave managment enabled for NSS pin

	SPI_Init(&spi2_handle);

}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t gpio_button;

	gpio_button.GPIOx = GPIOA;
	gpio_button.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_0;
	gpio_button.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IN;
	gpio_button.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;
	gpio_button.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;

	GPIO_Init(&gpio_button);
}

uint8_t SPI_VerifyResponse(uint8_t ack_byte) {
	if(ack_byte == 0xF5) return 1; // ACK
	return 0;
}



int main(void) {

uint8_t dummy_write = 0xff;
uint8_t dummy_read;

initialise_monitor_handles();

printf("Application is running\n");

GPIO_ButtonInit();

// initialise the GPIO pins to behave as SPI2 pins
SPI2_GPIOInits();

printf("SPI2_GPIOInits() \n");



// initialise/configure SPI2 peripheral
SPI2_Inits();

// this makes the NSS signal high internally, avoids the Master Mode fault
// when SSM is disabled, SSI has no use!!
// SPI_SSIConfig(SPI2, ENABLE);

/*
 * making SSOE 1 does NSS output enable.
 * The NSS pin is automatically managed by the hardware.
 * i.e. when SPE=1, NSS will be pulled to low
 * and NSS pin will be high when SPE=0
 */
SPI_SSOEConfig(SPI2, ENABLE);

while(1) {

	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
	delay();

	// enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);


	// 1. COMMAND_LED_CTRL     <pin no(1)>     <value(1)>
	uint8_t commandcode = COMMAND_LED_CTRL;
	uint8_t ack_byte;
	uint8_t args[2];

	// send command
	SPI_SendData(SPI2, &commandcode, 1);

	// do dummy read to clear off the RXNE of master
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	// move data (ACK/NACK) out of slave shift register by sending dummy write
	SPI_SendData(SPI2, &dummy_write, 1);

	// read the ack byte received
	SPI_ReceiveData(SPI2, &ack_byte, 1);

	if (SPI_VerifyResponse(ack_byte)) {
		args[0] = LED_PIN_9;
		args[1] = LED_ON;

		// send arguments
		SPI_SendData(SPI2, args, 2);

		printf("COMMAND_LED_CTRL Executed\n");
	}


	// 2. COMMAND_SENSOR_READ      <analogue pin number(1)>
	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
	delay();

	commandcode = COMMAND_SENSOR_READ;

	// send command
	SPI_SendData(SPI2, &commandcode, 1);

	// dummy read to clear off the RXNE of master
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	// move data (ACK/NACK) out of slave shift register by sending dummy write
	SPI_SendData(SPI2, &dummy_write, 1);

	// read the ACK byte received from slave
	SPI_ReceiveData(SPI2, &ack_byte, 1);

	if (SPI_VerifyResponse(ack_byte)) {
		// send arguments
		args[0] = ANALOG_PIN0;
		SPI_SendData(SPI2, args, 1);

		// dummy read to clear off the RXNE of master
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// Slave will take sometime to read the Analog (ADC conversion on that pin)
		// therefore master should wait
		delay();

		// send dummy data to fetch the response (analog sensor read) from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		uint8_t analog_read;
		SPI_ReceiveData(SPI2, &analog_read, 1);
		printf("COMMAND_SENSOR_READ : %d\n", analog_read);
	}


	// 3. COMMAND_LED_READ               <pin no(1)>
	while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
	delay();

	commandcode = COMMAND_LED_READ;

	// send command
	SPI_SendData(SPI2, &commandcode, 1);

	// dummy read to clear off RXNE of master
	SPI_ReceiveData(SPI2, &dummy_read, 1);

	// send dummy byte to fetch the response from slave (to move data from slave shift register)
	SPI_SendData(SPI2, &dummy_write, 1);

	// read the ACK byte received from slave
	SPI_ReceiveData(SPI2, &ack_byte, 1);

	if (SPI_VerifyResponse(ack_byte)) {
		args[0] = LED_PIN_9;

		// send arguments
		SPI_SendData(SPI2, args, 1);

		// dummy read to clear off RXNE of master
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		delay();

		// send dummy bits to move the data out of shift register of slave
		SPI_SendData(SPI2, &dummy_write, 1);

		uint8_t led_status;
		SPI_ReceiveData(SPI2, &led_status, 1);
		printf("COMMAND_READ_LED : %d\n", led_status);
	}


	// 4. COMMAND_PRINT 		<len(2)>  <message(len) >
	while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );
	delay(); // 200 ms

	commandcode = COMMAND_PRINT;

	//send command
	SPI_SendData(SPI2,&commandcode,1);

	//do dummy read to clear off the RXNE
	SPI_ReceiveData(SPI2,&dummy_read,1);

	//Send some dummy byte to fetch the response from the slave
	SPI_SendData(SPI2,&dummy_write,1);

	//read the ack byte received
	SPI_ReceiveData(SPI2,&ack_byte,1);

	uint8_t message[] = "Hello ! How are you ??";
	if( SPI_VerifyResponse(ack_byte)) {
		args[0] = strlen((char*)message);

		//send arguments
		SPI_SendData(SPI2,args,1); //sending length

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		delay();

		//send message
		for(int i = 0 ; i < args[0] ; i++){
			SPI_SendData(SPI2,&message[i],1);
			SPI_ReceiveData(SPI2,&dummy_read,1);
		}

		printf("COMMAND_PRINT Executed \n");
	}


	// 5. COMMAND_ID_READ
	while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );
	delay();

	commandcode = COMMAND_ID_READ;

	//send command
	SPI_SendData(SPI2,&commandcode,1);

	//do dummy read to clear off the RXNE
	SPI_ReceiveData(SPI2,&dummy_read,1);

	//Send some dummy byte to fetch the response from the slave
	SPI_SendData(SPI2,&dummy_write,1);

	//read the ack byte received
	SPI_ReceiveData(SPI2,&ack_byte,1);

	uint8_t id[11];
	uint32_t i=0;
	if( SPI_VerifyResponse(ack_byte))
	{
		//read 10 bytes id from the slave
		for(  i = 0 ; i < 10 ; i++)
		{
			//send dummy byte to fetch data from slave
			SPI_SendData(SPI2,&dummy_write,1);
			SPI_ReceiveData(SPI2,&id[i],1);
		}
		id[10] = '\0';

		printf("COMMAND_ID : %s \n", id);

	}


	// confirm the SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	// disable after a data communication
	SPI_PeripheralControl(SPI2, DISABLE);

	printf("SPI communication closed\n");

} // end while

return 0;
}
