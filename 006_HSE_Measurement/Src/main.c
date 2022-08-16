/*
 * 	Switch to HSE as system clock and measure it
 *
 *	Steps: (for STM32F407 Discovery board)
 *	1. Enable the HSE clock using HSEON bit (RCC_CR)
 *	2. Wait until HSE clock from the external crystal stabilises (only
 *	   if crystal is connected)(indicates if the high-speed external
 *	   oscillator is stable or not)
 *	3. Switch the system clock to HSE (RCC_CFGR)
 *	4. Do MCO1 settings to measure it
 *
 *	Steps: (for Nucleo board)
 *	1. Enable the HSEBYP bit (RCC_CR) (bypass the oscillator with an external clock)
 *	2. Enable the HSE clock using HSEON bit (RCC_CR)
 *	3. Switch the system clock to HSE
 *	4. Do MCO1 settings to measure it
 */


#include <stdint.h>

#define RCC_BASE_ADDR            0x40023800UL   // Reference Manual page 65
#define RCC_CFGR_REG_OFFSET      0x08UL         // Reference Manual page 228
#define RCC_CR_REG_OFFSET        0x00UL         // Reference Manual page 224
#define RCC_CFGR_REG_ADDR        ( RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET )
#define RCC_CR_REG_ADDR          ( RCC_BASE_ADDR + RCC_CR_REG_OFFSET )
#define GPIOA_BASE_ADDR          0x40020000UL   // Datasheet page 73

int main(void)
{
    // Steps for Discovery board
	// 1. Enable the HSE clock using HSEO bit (RCC_CR) - Reference Manual Page: 224
	uint32_t *rcc_cr_reg = (uint32_t*)RCC_CR_REG_ADDR;
	*rcc_cr_reg |= (1 << 16);

	// 2. Wait until HSE clock from the external crystal stabilises (only if crystal is connected)
	//    HSERDY: HSE clock ready flag - 0 means HSE oscillator not ready and 1 means ready
	while( !( *rcc_cr_reg & (1 << 17) ));

	// 3. Switch the system clock to HSE (RCC_CFGR) (SW0 and SW1 which is bit position 0 and 1) Reference Manual Page: 230
	//    SW: System clock switch - 01: HSE oscillator selected as system clock
	uint32_t *rcc_cfgr_reg = (uint32_t*)RCC_CFGR_REG_ADDR;
	*rcc_cfgr_reg |= (1 << 0);

	// 4. Do MCO1 settings to measure it
	// 4a. Configure the RCC_CFGR MC01 bit fields to select HSE as clock source
	//     for HSE 22 bit position has to be 1 - Reference Manual Page 229
	*rcc_cfgr_reg |= (1 << 22);

	// 4b. configure the MC01 prescaler, divisor as 4
	//     this step only applicable if your oscilloscope sample the
	//     signal up till 8mhz, whereas your mcu system clock generate signal above 8mhz (i.e. 16mhz)
	*rcc_cfgr_reg |= (1 << 25);
	*rcc_cfgr_reg |= (1 << 26);

	// 05. Configure PA8 to AF0 mode to behave as MCO1 signal

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

	for(;;);
}















