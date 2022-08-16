/*
 * 	Output HSI clock on microcontroller pin and measure
 * 	it using oscilloscope or logic analyser
 *
 * 	Steps to output a clock on MCU pin (PA8)
 * 	1. Select the desired clock for the MCOx signal (Microcontroller Clock Output) (MCO1 21, 22 bit position) - Reference Manual - Page 228
 * 	2. Output the MCOx signal on the MCU pin (PA8) - Datasheet - Page 62 Alternate function mapping
 */

#include <stdint.h>

#define RCC_BASE_ADDR            0x40023800UL   // Reference Manual page 65
#define RCC_CFGR_REG_OFFSET      0x08UL         // Reference Manual page 228
#define RCC_CFGR_REG_ADDR        ( RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET )
#define GPIOA_BASE_ADDR          0x40020000UL   // Datasheet page 73

int main(void)
{
	// 01. Configure RCC_CFGR register (MCO1 21, 22 bit position) - Reference Manual - Page 228
	uint32_t *rcc_cfgr_reg = (uint32_t*) RCC_CFGR_REG_ADDR;
	*rcc_cfgr_reg &= ~(0x3 << 21);  // clear 21, 22 bit position

	// 01a. Configure MCO1 prescaler (100 means divide by 2) Reference Manual - Page 228
	*rcc_cfgr_reg |= (1 << 25);
	*rcc_cfgr_reg |= (1 << 26);

	// 02. Configure PA8 to AF0 mode to behave as MCO1 signal

	// a) Enable the RCC peripheral clock for GPIOA peripheral to use PA8 - Reference Manual - Page 242
	uint32_t *rcc_ahb1enr = (uint32_t*) (RCC_BASE_ADDR + 0x30);
	*rcc_ahb1enr |= (1 << 0);

	// b) Configure the mode of GPIOA pin 8 as alternate function mode
	uint32_t *gpioa_mode_reg = (uint32_t*)(GPIOA_BASE_ADDR + 0x00); // Reference Manual - Page 281
	*gpioa_mode_reg &= ~(0x3 << 16);  // bit 16, 17 for 8th pin on Port A (PA8)
	*gpioa_mode_reg |= (0x2 << 16);   // set 10 (0x2) into bit 16 to drive in alternate function mode

	// c) Configure the GPIO alternate function register to set the mode of 0 for PA8
	uint32_t *gpioa_alt_fun_high_reg = (uint32_t*)(GPIOA_BASE_ADDR + 0x24); // Reference Manual - Page 286
	*gpioa_alt_fun_high_reg &= ~(0xf << 0); // clear the bit 0-3 for Port A ~1111 ~(0xf)

	// basic logic analyser (i.e. Salea) only sample the signals upto 8Mhz
	// if your mcu clock is running above 8mhz (i.e. 16mhz) you have to use prescaler
	// by using prescaler you have to divide the HSI
	// to configure prescaler you have to consult RCC_CFGR clock configuration register - Reference Manual Page 228
	// you have to configure bits 26-24 (also called MCO1 PRE)
	// prescaler was configured above on step "01a"





    /* Loop forever */
	for(;;);
}
