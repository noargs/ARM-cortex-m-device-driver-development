#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
  uint32_t i2c_scl_speed;
  uint8_t  i2c_device_address;
  uint8_t  i2c_ack_control;
  uint8_t  i2c_fm_duty_cycle;
}I2C_Config_t;

typedef struct
{
  I2C_RegDef_t *I2Cx;
  I2C_Config_t I2C_Config;
}I2C_Handle_t;


// @i2c_scl_speed
#define I2C_SCL_SPEED_SM                  100000
#define I2C_SCL_SPEED_FM4K                400000
#define I2C_SCL_SPEED_FM2K                200000

// @i2c_ack_control (Reference Manual page: 860) - I2C_CR1
#define I2C_ACK_ENABLE                    1
#define I2C_ACK_DISABLE                   0

// @i2c_fm_duty_cycle (Reference Manual page: 870) - I2C_CCR
#define I2C_FM_DUTY_2                     0
#define I2C_FM_DUTY_16_9                  1



//                              [APIs supported by this driver]
//              For more information about the APIs check the function definitions


// Peripheral clock setup
void I2C_PCLK_Ctrl (I2C_RegDef_t *i2cx, uint8_t enable_or_disable);

// Initialise and De-intialise I2C
void I2C_Init (I2C_Handle_t *i2c_handle);
void I2C_DeInit (I2C_RegDef_t *i2cx);

// Data send and receive


// IRQ configuration and ISR handling
void I2C_IRQInterruptConfig(uint8_t irq_number, uint8_t enable_or_disable);
void I2C_IRQPriorityConfig(uint8_t irq_number, uint32_t irq_priority);

// other peripheral control APIs
void I2C_PeripheralControl(I2C_RegDef_t *i2cx, uint8_t enable_or_disable);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *i2cx, uint32_t flag_name);

// Application callback
void I2C_ApplicationEventCallback(I2C_Handle_t *i2c_handle, uint8_t APPLICATION_EVENT);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
