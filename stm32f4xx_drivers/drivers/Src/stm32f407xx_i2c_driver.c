#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *i2cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *i2cx, uint8_t slave_addr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *i2cx, uint8_t slave_addr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *i2cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *i2cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *i2cx)
{
  i2cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *i2cx, uint8_t slave_addr)
{
  slave_addr = slave_addr << 1;
  slave_addr &= ~(1); // lsb is r/w bit, must be 0 for WRITE
  i2cx->DR = slave_addr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *i2cx, uint8_t slave_addr)
{
  slave_addr = slave_addr << 1;
  slave_addr |= 1; // lsb is r/2 bit, must be 1 for READ
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

  // enable the clock for the i2cx peripheral
  I2C_PCLK_Ctrl(i2c_handle->I2Cx, ENABLE);


  // configure ACK control bit of CR1
  temp_reg |= i2c_handle->I2C_Config.i2c_ack_control << 10;
  i2c_handle->I2Cx->CR1 = temp_reg;

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

  // TRISE configuration
  if (i2c_handle->I2C_Config.i2c_scl_speed <= I2C_SCL_SPEED_SM) {
	  // mode is standard
	  temp_reg = (RCC_GetPCLK1Value() / 1000000U) + 1;
  } else {
	  // mode is fast mode
	  temp_reg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
  }
  i2c_handle->I2Cx->TRISE = (temp_reg & 0x3F);
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
  I2C_ExecuteAddressPhaseWrite(i2c_handle->I2Cx, slave_addr);

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

void I2C_PCLK_Ctrl (I2C_RegDef_t *i2cx, uint8_t enable_or_disable)
{
  if (enable_or_disable == ENABLE)
  {
	if (i2cx == I2C1)
	{
	  I2C1_PCLK_EN();
	} else if (i2cx == I2C2)
	{
	  I2C2_PCLK_EN();
	} else if (i2cx == I2C3)
	{
	  I2C3_PCLK_EN();
	}
  } else
  {
    // TODO
  }

}

void I2C_PeripheralControl(I2C_RegDef_t *i2cx, uint8_t enable_or_disable)
{
  if (enable_or_disable == ENABLE)
  {
	i2cx->CR1 |= (1 << I2C_CR1_PE);
  } else
  {
	i2cx->CR1 &= ~(1 << 0);
  }
}


void I2C_MasterReceiveData(I2C_Handle_t *i2c_handle, uint8_t *rx_buffer, uint8_t len, uint8_t slave_addr)
{
  //1. Generate the START condition
  I2C_GenerateStartCondition(i2c_handle->I2Cx);

  //2. Confirm that Start generation is completed by checking the SB flag in the SR
  //   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
  while (! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_SB));

  //3. Send the address of the slave with R/W bit set to 1 (i.e. R) (total 8 bits)
  I2C_ExecuteAddressPhaseRead(i2c_handle->I2Cx, slave_addr);

  //4. Wait until Address phase is completed by checking the ADDR flag in the SR1
  while (! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_ADDR));

  // Procedure to read only 1 byte from the Slave
  if (len == 1)
  {
	// Disable Acking
	I2C_ManageACK(i2c_handle->I2Cx, I2C_ACK_DISABLE);

	// Clear the ADDR flag
	I2C_ClearADDRFlag(i2c_handle->I2Cx);

	// Wait until RxNE becomes 1
	while (! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_RXNE));

	// Generate STOP condition
	I2C_GenerateStopCondition(i2c_handle->I2Cx);

	// Read data into the buffer
    *rx_buffer = i2c_handle->I2Cx->DR;

  }

  // Procedure to read data from Slave when len > 1
  if (len > 1)
  {
    // Clear the ADDR flag
	I2C_ClearADDRFlag(i2c_handle->I2Cx);

	// Read the data until len becomes 0
	for (uint32_t i=len; i>0; i--)
	{
	  // Wait until RxNE becomes 1
		while (! I2C_GetFlagStatus(i2c_handle->I2Cx, I2C_FLAG_RXNE));

	  if (i == 2) // if last 2 bytes are remaining
	  {
		// Disable (clear) the ACK bit
		I2C_ManageACK(i2c_handle->I2Cx, I2C_ACK_DISABLE);

		// Generate STOP condition
		I2C_GenerateStopCondition(i2c_handle->I2Cx);
	  }

	  // Read the data from DR in to buffer
	  *rx_buffer = i2c_handle->I2Cx->DR;

	  // Increment the buffer address
	  rx_buffer++;
	}
  }

  // Re-enable ACK
  if (i2c_handle->I2C_Config.i2c_ack_control == I2C_ACK_ENABLE)
  {
    I2C_ManageACK(i2c_handle->I2Cx, I2C_ACK_ENABLE);
  }

}

void I2C_ManageACK(I2C_RegDef_t *i2cx, uint8_t enable_or_disable)
{
  if (enable_or_disable == ENABLE)
  {
	// Enable the ACK
	i2cx->CR1 |= (1 << I2C_CR1_ACK);
  } else
  {
	// Disable the ACK
	i2cx->CR1 &= ~(1 << I2C_CR1_ACK);
  }
}








