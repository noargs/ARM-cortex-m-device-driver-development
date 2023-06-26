#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
  uint8_t usart_mode;
  uint32_t usart_baud;
  uint8_t usart_no_of_stop_bits;
  uint8_t usart_word_length;
  uint8_t usart_parity_control;
  uint8_t usart_hw_flow_control;
}USART_Config_t;

typedef struct
{
  USART_RegDef_t *usartx;
  USART_Config_t usart_config;
}USART_Handle_t;

// @usart_application_states

// @usart_mode
#define USART_MODE_ONLY_TX                0
#define USART_MODE_ONLY_RX                1
#define USART_MODE_TXRX                   2

// @usart_baud
#define USART_STD_BAUD_1200               1200
#define USART_STD_BAUD_2400               2400
#define USART_STD_BAUD_9600               9600
#define USART_STD_BAUD_19200              19200
#define USART_STD_BAUD_38400              34800
#define USART_STD_BAUD_57600              57600
#define USART_STD_BAUD_115200             115200
#define USART_STD_BAUD_230400             230400
#define USART_STD_BAUD_460800             460800
#define USART_STD_BAUD_921600             921600
#define USART_STD_BAUD_2M                 2000000
#define USART_STD_BAUD_3M                 3000000

// @usart_parity_control
#define USART_PARITY_EN_ODD               2
#define USART_PARITY_EN_EVEN              1
#define USART_PARITY_DISABLE              0

// @usart_wordlength
#define USART_WORDLEN_8BITS               0
#define USART_WORDLEN_9BITS               1

// @usart_no_of_stop_bits
#define USART_STOPBITS_1                  0
#define USART_STOPBITS_0_5                1
#define USART_STOPBITS_2                  2
#define USART_STOPBITS_1_5                3

// @usart_hw_flow_control
#define USART_HW_FLOW_CONTROL_NONE        0
#define USART_HW_FLOW_CONTROL_CTS         1
#define USART_HW_FLOW_CONTROL_RTS         2
#define USART_HW_FLOW_CONTROL_CTS_RTS     3

// USART related Status flags (i.e. Status Register's) definitions
#define USART_FLAG_TXE                    (1 << USART_SR_TXE)
#define USART_FLAG_RXNE                   (1 << USART_SR_RXNE)
#define USART_FLAG_TC                     (1 << USART_SR_TC)

// USART application events macros (self-declared values)


//                              [APIs supported by this driver]
//              For more information about the APIs check the function definitions

// Peripheral clock setup
void USART_PCLKControl(USART_RegDef_t *usartx, uint8_t ENABLE_OR_DISABLE);

// Init and De-Init
void USART_Init(USART_Handle_t *usart_handle);
void USART_DeInit(USART_Handle_t *usart_handle);

// Data send and receive
void USART_SendData(USART_Handle_t *usart_handle, uint8_t *tx_buffer, uint32_t len);
void USART_ReceiveData(USART_Handle_t *usart_handle, uint8_t *rx_buffer, uint32_t len);
uint8_t USART_SendDataIT(USART_Handle_t *usart_handle, uint8_t *tx_buffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *usart_handle, uint8_t *rx_buffer, uint32_t len);

// IRQ configuration and ISR handling
void USART_IRQInterruptConfig(uint8_t irq_number, uint8_t ENABLE_OR_DISABLE);
void USART_IRQPriorityConfig(uint8_t irq_number, uint32_t irq_priority);
void USART_IRQHandling(USART_Handle_t *usart_handle);

// Other peripheral control APIs
uint8_t USART_GetFlagStatus(USART_RegDef_t *usartx, uint8_t status_flag_name);
void USART_ClearFlag(USART_RegDef_t *usartx, uint16_t status_flag_name);
void USART_PeripheralControl(USART_RegDef_t *usartx, uint8_t ENABLE_OR_DISABLE);

// Application callbacks
void USART_ApplicationEventCallback(USART_Handle_t *usart_handle, uint8_t APPLICATION_EVENT);

#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
