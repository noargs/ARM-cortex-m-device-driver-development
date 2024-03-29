#include <stdint.h>

#define ADC_BASE_ADDR          0x40012000UL
#define ADC_CR1_OFFSET         0x04UL
#define ADC_CR1_REG_ADDR       (ADC_BASE_ADDR + ADC_CR1_OFFSET)

#define RCC_BASE_ADDR          0x40023800UL
#define RCC_APB2ENR_OFFSET     0x44UL
#define RCC_APB2ENR_ADDR       (RCC_BASE_ADDR + RCC_APB2ENR_OFFSET)

int main(void)
{

	// enable the rcc peripheral clock
	uint32_t *p_rcc_apb2enr = (uint32_t*) RCC_APB2ENR_ADDR;
	*p_rcc_apb2enr |= (1 << 8);

	// modify the adc cr1 register
	uint32_t *p_adc_ctrl_reg = (uint32_t*) ADC_CR1_REG_ADDR;
	*p_adc_ctrl_reg |= (1 << 8);


	for(;;);
}
