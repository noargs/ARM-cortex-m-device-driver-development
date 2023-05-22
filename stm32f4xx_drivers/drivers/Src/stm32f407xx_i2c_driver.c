#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void)
{
  uint32_t pclk1, system_clock;
  uint8_t clock_source, temp, ahb_prescaler, apb1_prescaler;

  clock_source = ((RCC->CFGR >> 2) & 0x3);

  if (clock_source == 0) {
	  system_clock = 16000000;
  } else if (clock_source == 1) {
	  system_clock = 8000000;
  }

  // we will not be using PLL in this course
//  else if (clock_source == 2) {
//	  system_clock = RCC_GetPLLOutputClock();
//  }

  // find AHBP Prescaler Reference Manual page: 228
  temp = ((RCC->CFGR >> 4) & 0xF);

  if (temp < 8) {
	  ahb_prescaler = 1;
  } else {
	  ahb_prescaler = AHB_PreScaler[temp-8];
  }

  // find APB1 Prescaler Reference Manual page: 229
  temp = ((RCC->CFGR >> 10) & 0x7);

  if (temp < 4) {
	  apb1_prescaler = 1;
  } else {
	  apb1_prescaler = APB1_PreScaler[temp-4];
  }

  pclk1 = (system_clock / ahb_prescaler) / apb1_prescaler;

  return pclk1;
}

void I2C_Init (I2C_Handle_t *i2c_handle)
{
  uint32_t temp_reg = 0;
  temp_reg |= i2c_handle->I2C_Config.i2c_ack_control << 10;
}


