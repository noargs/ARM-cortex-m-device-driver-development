
/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */

/*
 * Note: Follow the instructions to this code
 * 1. Download this code on to STM32 board (Master)
 * 2. Download the `slave/spi/spi_slave_uart_read_over_spi` on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in serial monitor `tool line ending` set to `carriage return`)
 */
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


SPI_Handle_t spi2_handle;

#define MAX_LEN                       500

char receive_buffer[MAX_LEN];

volatile char read_byte;


volatile uint8_t receive_stop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t data_available = 0;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t spi_pins;

	spi_pins.GPIOx = GPIOB;
	spi_pins.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_ALTFN;
	spi_pins.GPIO_PinConfig.gpio_pin_alt_fun = 5;
	spi_pins.GPIO_PinConfig.gpio_pin_op_type = GPIO_OP_TYPE_PP;
	spi_pins.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_NO_PUPD;
	spi_pins.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_FAST;

	//SCLK
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_13;
	GPIO_Init(&spi_pins);

	//MOSI
    spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_15;
	GPIO_Init(&spi_pins);

	//MISO
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_14;
	GPIO_Init(&spi_pins);


	//NSS
	spi_pins.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_12;
	GPIO_Init(&spi_pins);


}

void SPI2_Inits(void)
{
	spi2_handle.SPIx = SPI2;
	spi2_handle.SPIConfig.spi_bus_config = SPI_BUS_CONFIG_FD;
	spi2_handle.SPIConfig.spi_device_mode = SPI_DEVICE_MODE_MASTER;
	spi2_handle.SPIConfig.spi_sclk_speed = SPI_SCLK_SPEED_DIV32;
	spi2_handle.SPIConfig.spi_dff = SPI_DFF_8BITS;
	spi2_handle.SPIConfig.spi_cpol = SPI_CPOL_LOW;
	spi2_handle.SPIConfig.spi_cpha = SPI_CPHA_LOW;
	spi2_handle.SPIConfig.spi_ssm = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&spi2_handle);
}


/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spi_init_pin;
	memset(&spi_init_pin,0,sizeof(spi_init_pin));

	//this is led gpio configuration
	spi_init_pin.GPIOx = GPIOD;
	spi_init_pin.GPIO_PinConfig.gpio_pin_number = GPIO_PIN_NO_6;
	spi_init_pin.GPIO_PinConfig.gpio_pin_mode = GPIO_MODE_IT_FT;
	spi_init_pin.GPIO_PinConfig.gpio_pin_speed = GPIO_SPEED_LOW;
	spi_init_pin.GPIO_PinConfig.gpio_pin_pu_pd_control = GPIO_PU;

	GPIO_Init(&spi_init_pin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

}


int main(void)
{

	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while(1){

		receive_stop = 0;

		while(!data_available); //wait till data available interrupt from transmitter device(slave)

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,DISABLE);

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);


		while(!receive_stop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while ( SPI_SendDataIT(&spi2_handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while ( SPI_ReceiveDataIT(&spi2_handle, (uint8_t*)&read_byte, 1) == SPI_BUSY_IN_RX );
		}


		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("Rcvd data = %s\n",receive_buffer);

		data_available = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	}

	return 0;

}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void)
{

	SPI_IRQHandling(&spi2_handle);
}


void SPI_ApplicationEventCallback(SPI_Handle_t *spi_handle, uint8_t APPLICATION_EVENT)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(receive_stop = 1) */
	if(APPLICATION_EVENT == SPI_EVENT_RX_COMPLETE)
	{
		receive_buffer[i++] = read_byte;
		if(read_byte == '\0' || ( i == MAX_LEN)){
			receive_stop = 1;
			receive_buffer[i-1] = '\0';
			i = 0;
		}
	}

}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	data_available = 1;
}
