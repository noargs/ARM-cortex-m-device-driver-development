#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *i2cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *i2cx, uint8_t slave_addr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *i2cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *i2cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *i2cx)
{
  i2cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *i2cx, uint8_t slave_addr)
{
  slave_addr = slave_addr << 1;
  slave_addr &= ~(1); // lsb is r/w bit, must be 0 for WRITE
  i2cx->DR = slave_addr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *i2cx)
{
  uint32_t dummy_read = i2cx->SR1;
  dummy_read = i2cx->SR2;
  (void)dummy_read;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *i2cx)
{
  i2cx->CR1 |= (1 << I2C_CR1_STOP);
}

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

  // configure ACK control bit of CR1
  temp_reg |= i2c_handle->I2C_Config.i2c_ack_control << 10;

  // configure the FREQ field of CR2
  temp_reg = 0;
  temp_reg |= RCC_GetPCLK1Value() / 1000000U;
  i2c_handle->I2Cx->CR2 = (temp_reg & 0x3F);

  // program the device own address Reference Manual page:864
  temp_reg |= i2c_handle->I2C_Config.i2c_device_address << 1;
  temp_reg |= (1 << 14);
  i2c_handle->I2Cx->OAR1 = temp_reg;

  // CCR calculation
  uint16_t ccr_value = 0;
  if (i2c_handle->I2C_Config.i2c_scl_speed <= I2C_SCL_SPEED_SM) {
	  // for Standard mode
	  ccr_value = (RCC_GetPCLK1Value() / (2 * i2c_handle->I2C_Config.i2c_scl_speed));
	  temp_reg |= (ccr_value & 0xFFF);
  } else {
	  // for Fast mode
	  temp_reg |= (1 << 15);
	  temp_reg |= (i2c_handle->I2C_Config.i2c_fm_duty_cycle << 14);

	  // Duty cycle = 0 (Tlow twice the Thigh)
	  if (i2c_handle->I2C_Config.i2c_fm_duty_cycle == I2C_FM_DUTY_2) {
		  ccr_value = (RCC_GetPCLK1Value() / (3 * i2c_handle->I2C_Config.i2c_scl_speed));
	  } else {

		  // Duty cycle = 1 (Tlow 1.7 times the Thigh)
		  ccr_value = (RCC_GetPCLK1Value() / (25 * i2c_handle->I2C_Config.i2c_scl_speed));
	  }
	  temp_reg |= (ccr_value & 0xFFF);
  }

  i2c_handle->I2Cx->CCR = temp_reg;
}


void I2C_MasterSendData(I2C_Handle_t *i2c_handle, uint8_t *tx_buffer, uint32_t len, uint8_t slave_addr)
{
  // 1. Generate the START condition
  I2C_GenerateStartCondition(i2c_handle->I2Cx);

  // 2. Confirm that start generation is completed by checking the SB flag in the SR1
  //    Note: Until SB is cleared SCL will be stretched (pulled to LOW)
  while(! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_SB));

  // 3. Send the address of the slave with r/w bit set to 0 i.e. write
  //    Slave address(7-bits) + r/w(1-bit) = 8 bits
  I2C_ExecuteAddressPhase(i2c_handle->I2Cx, slave_addr);

  // 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
  while( ! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_ADDR));

  // 5. Clear the ADDR flag according to its software sequence (first SR1 then SR2)
  //    Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
  I2C_ClearADDRFlag(i2c_handle->I2Cx);

  // 6. Send the data until `len` becomes `0`
  while(len > 0)
  {
	while(! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_TXE)); // wait till TXE is set
	i2c_handle->I2Cx->DR = *tx_buffer;
	tx_buffer++;
	len--;
  }

  // 7. When `len = 0` wait for TXE=1 and BTF=1 before generating the STOP condition
  //    Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
  //    When BTF=1 SCL will be stretched (pulled to LOW)
  while(! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_TXE));

  while(! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_BTF));

  // 8. Generate STOP condition and master need not to wait for the completion of STOP condition
  //    Note: Generating STOP, automatically clears the BTF
  I2C_GenerateStopCondition(i2c_handle->I2Cx);

}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *i2cx, uint32_t flag_name)
{
  if (i2cx->SR1 & flag_name)
  {
	return FLAG_SET;
  }
  return FLAG_RESET;
}










