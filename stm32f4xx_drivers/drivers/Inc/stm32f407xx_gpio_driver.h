#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


// Configuration structure for GPIO pins
typedef struct {
	uint8_t gpio_pin_number;         /*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t gpio_pin_mode;           /*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t gpio_pin_speed;          /*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t gpio_pin_pu_pd_control;
	uint8_t gpio_pin_op_type;
	uint8_t gpio_pin_alt_fun;
}GPIO_PinConfig_t;

// Handle structure for GPIO pins
typedef struct {
	GPIO_RegDef_t    *gpiox;          /* holds the base address of GPIO port to which pin belongs */
	GPIO_PinConfig_t gpio_config;  /* holds GPIO pin configuration settings */

}GPIO_Handle_t;

// @GPIO_PIN_NUMBERS
#define GPIO_PIN_NO_0                    0
#define GPIO_PIN_NO_1                    1
#define GPIO_PIN_NO_2                    2
#define GPIO_PIN_NO_3                    3
#define GPIO_PIN_NO_4                    4
#define GPIO_PIN_NO_5                    5
#define GPIO_PIN_NO_6                    6
#define GPIO_PIN_NO_7                    7
#define GPIO_PIN_NO_8                    8
#define GPIO_PIN_NO_9                    9
#define GPIO_PIN_NO_10                   10
#define GPIO_PIN_NO_11                   11
#define GPIO_PIN_NO_12                   12
#define GPIO_PIN_NO_13                   13
#define GPIO_PIN_NO_14                   14
#define GPIO_PIN_NO_15                   15


// @GPIO_PIN_MODES
// GPIO pins possible modes - [Reference Manual page:281]
#define GPIO_MODE_IN                     0
#define GPIO_MODE_OUT                    1
#define GPIO_MODE_ALTFN                  2
#define GPIO_MODE_ANALOG                 3
#define GPIO_MODE_IT_FT                  4 // input falling-edge trigger
#define GPIO_MODE_IT_RT                  5 // input rising-edge trigger
#define GPIO_MODE_IT_RFT                 6 // input rising-falling-edge trigger


// GPIO pin possible output types
#define GPIO_OP_TYPE_PP                  0 // output type push-pull
#define GPIO_OP_TYPE_OD                  1 // output type open-drain


// @GPIO_PIN_SPEED
// GPIO pin possible output speeds
#define GPIO_SPEED_LOW                   0
#define GPIO_SPEED_MEDIUM                1
#define GPIO_SPEED_FAST                  2
#define GPIO_SPEED_HIGH                  3


// GPIO pin pull up and pull down
#define GPIO_NO_PUPD                     0 // no pull-up, pull-down
#define GPIO_PU                          1 // pull-up
#define GPIO_PD                          2 // pull-down




//                              [APIs supported by this driver]
//              For more information about the APIs check the function definitions


// Peripheral clock setup
void GPIO_PCLK_Ctrl (GPIO_RegDef_t *gpiox, uint8_t en_or_di);

// Initialise and De-intialise GPIOs
void GPIO_Init (GPIO_Handle_t *gpio_handle);
void GPIO_DeInit (GPIO_RegDef_t *gpiox);

// Data read and write
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *gpiox, uint8_t pin_number);
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *gpiox);
void GPIO_WriteToOutputPin (GPIO_RegDef_t *gpiox, uint8_t pin_number, uint8_t value);
void GPIO_WriteToOutputPort (GPIO_RegDef_t *gpiox, uint16_t value);
void GPIO_ToggleOutputPin (GPIO_RegDef_t *gpiox, uint8_t pin_number);

// IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig (uint8_t irq_number, uint8_t en_or_di);
void GPIO_IRQPriorityConfig (uint8_t irq_number, uint32_t irq_priority);
void GPIO_IRQHandling (uint8_t pin_number);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
