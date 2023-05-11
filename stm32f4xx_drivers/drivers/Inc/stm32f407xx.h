#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo                                        volatile

                           /************[ Processor specific details ]************/

// ARM Cortex Mx Processor NVIC ISERx register Addresses
#define NVIC_ISER0                                  ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1                                  ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2                                  ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3                                  ((__vo uint32_t*)0xE000E10C)


// ARM Cortex Mx Processor NVIC ICERx register addresses
#define NVIC_ICER0                                  ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1                                  ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2                                  ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3                                  ((__vo uint32_t*)0xE000E18C)


// ARM Cortex Mx Processor Priority Register IPR Address
#define NVIC_PR_BASE_ADDR                           ((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED                      4

// Base addresses of Flash and SRAM memories
#define FLASH_BASEADDR                              0x08000000U
#define SRAM1_BASEADDR                              0x20000000U         // first 112KB
#define SRAM2_BASEADDR                              0x20001C00U         // 112 * 1024  = 114688 to Hex 1C00, 0x2000000 + 1C00 = 0x20001C00
#define ROM_BASEADDR                                0x1FFF0000U         // System memory (RM page:75)
#define SRAM                                        SRAM1_BASEADDR


// AHBx and APBx Bus Peripheral base addresses
#define PERIPH_BASEADDR                             0x40000000U
#define APB1PERIPH_BASEADDR                         PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR                         0x40010000U
#define AHB1PERIPH_BASEADDR                         0x40020000U
#define AHB2PERIPH_BASEADDR                         0x50000000U


// Base addresses of peripherals, hanging on AHB1 bus
#define GPIOA_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR                              (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                                (AHB1PERIPH_BASEADDR + 0x3800)


// Base addresses of peripherals, hanging on APB1 bus
#define I2C1_BASEADDR                               (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                               (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                               (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR                               (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                               (APB1PERIPH_BASEADDR + 0x3C00)

#define UART4_BASEADDR                              (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                              (APB1PERIPH_BASEADDR + 0x5000)
#define USART2_BASEADDR                             (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                             (APB1PERIPH_BASEADDR + 0x4800)


// Base addresses of peripherals, hanging on APB2 bus
#define EXT1_BASEADDR                               (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR                               (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR                             (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR                             (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                             (APB2PERIPH_BASEADDR + 0x1400)


// peripheral register definition structure for GPIO [RM Page:287]
// below registers are specific to stm32f407 family
// and it may be different in other families of ST
// microcontroller
typedef struct {
	__vo uint32_t MODER;        // GPIO port mode register                             Address offset 0x00
	__vo uint32_t OTYPER;       // GPIO port output type register                      Address offset 0x04
	__vo uint32_t OSPEEDR;      // GPIO port output speed register                     Address offset 0x08
	__vo uint32_t PUPDR;        // GPIO port pull-up/pull-down register                Address offset 0x0C
	__vo uint32_t IDR;          // GPIO port input data register                       Address offset 0x10
	__vo uint32_t ODR;          // GPIO port output data register                      Address offset 0x14
	__vo uint32_t BSRR;         // GPIO port bit set/reset register                    Address offset 0x18
	__vo uint32_t LCKR;         // GPIO port configuration lock register               Address offset 0x1C
	__vo uint32_t AFR[2];       // GPIO alternate function
	                            // low AFR[0] / high AFR[1] register                   Address offset 0x20, 0x24
}GPIO_RegDef_t;


// peripheral register definition structure for RCC
typedef struct {
	__vo uint32_t CR;           // RCC clock control register                          Address offset 0x00
	__vo uint32_t PLLCFGR;      // RCC PLL configuration register                      Address offset 0x04
	__vo uint32_t CFGR;         // RCC clock configuration register                    Address offset 0x08
	__vo uint32_t CIR;          // RCC clock interrupt register                        Address offset 0x0C
	__vo uint32_t AHB1RSTR;     // RCC AHB1 Peripheral reset register                  Address offset 0x10
	__vo uint32_t AHB2RSTR;     // RCC AHB2 Peripheral reset register                  Address offset 0x14
	__vo uint32_t AHB3RSTR;     // RCC AHB3 Peripheral reset register                  Address offset 0x18
	uint32_t      RESERVED0;    // Reserved                                            Address offset 0x1C
	__vo uint32_t APB1RSTR;     // RCC APB1 Peripheral reset register                  Address offset 0x20
	__vo uint32_t APB2RSTR;     // RCC APB2 Peripheral reset register                  Address offset 0x24
	uint32_t      RESERVED1[2]; // Reserved                                            Address offset 0x28, 0x2C
	__vo uint32_t AHB1ENR;      // RCC AHB1 Peripheral clock register                  Address offset 0x30
	__vo uint32_t AHB2ENR;      // RCC AHB2 Peripheral clock register                  Address offset 0x34
	__vo uint32_t AHB3ENR;      // RCC AHB3 Peripheral clock register                  Address offset 0x38
	uint32_t      RESERVED2;    // Reserved                                            Address offset 0x3C
	__vo uint32_t APB1ENR;      // RCC APB1 Peripheral clock enable register           Address offset 0x40
	__vo uint32_t APB2ENR;      // RCC APB2 Peripheral clock enable register           Address offset 0x44
	uint32_t      RESERVED3[2]; // Reserved                                            Address offset 0x48, 0x4C
	__vo uint32_t AHB1LPENR;    // RCC AHB1 Peripheral clock enable LPM register       Address offset 0x50
	__vo uint32_t AHB2LPENR;    // RCC AHB2 Peripheral clock enable LPM register       Address offset 0x54
	__vo uint32_t AHB3LPENR;    // RCC AHB3 Peripheral clock enable LPM register       Address offset 0x58
	uint32_t      RESERVED4;    // Reserved                                            Address offset 0x5C
	__vo uint32_t APB1LPENR;    // RCC APB1 Peripheral clock enable LPM register       Address offset 0x60
	__vo uint32_t APB2LPENR;    // RCC APB2 Peripheral clock enable LPM register       Address offset 0x64
	uint32_t      RESERVED5[2]; // Reserved                                            Address offset 0x68, 0x6C
	__vo uint32_t BDCR;         // RCC Backup domain control register                  Address offset 0x70
	__vo uint32_t CSR;          // RCC clock control and status register               Address offset 0x74
	uint32_t      RESERVED6[2]; // Reserved                                            Address offset 0x78, 0x7C
	__vo uint32_t SSCGR;        // RCC spread spectrum clock generation register       Address offset 0x80
	__vo uint32_t PLLI2SCFGR;   // RCC PLLI2S configuration register                   Address offset 0x84
	__vo uint32_t PLLSAICFGR;   // RCC PLL configuration register                      Address offset 0x88
	__vo uint32_t DCKCFGR;      // RCC Dedicated clock configuration register          Address offset 0x8C
	__vo uint32_t CKGATENR;     // RCC clock GAT enable register                       Address offset 0x90
	__vo uint32_t DCKCFGR2;     // RCC Dedicated clock configuration 2 register        Address offset 0x94

}RCC_RegDef_t;


// peripheral register definition structure for EXTI - [Reference Manual Page:380]
typedef struct {
	__vo uint32_t IMR;          // Interrupt Mask register                             Address offset 0x00
	__vo uint32_t EMR;          // Event Mask register                                 Address offset 0x04
	__vo uint32_t RTSR;         // Rising Trigger Selection register                   Address offset 0x08
	__vo uint32_t FTSR;         // Falling Trigger Selection register                  Address offset 0x0C
	__vo uint32_t SWIER;        // Software Interrupt Event register                   Address offset 0x10
	__vo uint32_t PR;           // Pending register                                    Address offset 0x14
}EXTI_RegDef_t;

// which GPIO port should use EXTIO or which GPIO port should use EXTI1 etc
// decide by this register
// there are four register EXTI CR1, CR2, CR3, CR4
typedef struct {
	__vo uint32_t MEMRMP;        // SYSCFG_MEMRMP, Memory remap register               Address offset 0x00
	__vo uint32_t PMC;           // SYSCFG_PMC, Peripheral mode configuration reg.     Address offset 0x04
	__vo uint32_t EXTICR[4];     // External interrupt configuration register x 4      Address offset 0x08, 0x0C, 0x10, 0x14
	uint32_t      RESERVED1[2];  //                                                    Address offset 0x18, 0x1C
	__vo uint32_t CMPCR;         // Compensation cell control register                 Address offset 0x20
	uint32_t      RESERVED2[2];  //                                                    Address offset 0x24, 0x28
	__vo uint32_t CFGR;          //                                                    Address offset 0x2C
}SYSCFG_RegDef_t;


// peripheral register definition structure for SPI
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2DCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;


#define GPIOA                                       ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                                       ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                                       ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                                       ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                                       ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                                       ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                                       ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                                       ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI                                       ((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define GPIOJ                                       ((GPIO_RegDef_t*) GPIOJ_BASEADDR)

#define RCC                                         ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI                                        ((EXTI_RegDef_t*) EXT1_BASEADDR)

#define SYSCFG                                      ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1                                        ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                                        ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                                        ((SPI_RegDef_t*)SPI3_BASEADDR)


// Peripheral Clock enable macros `GPIOx peripheral clock enable`
#define GPIOA_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()                             (RCC->AHB1ENR |= (1 << 8))


// Peripheral Clock enable macros `I2Cx peripheral clock enable`
#define I2C1_PCLK_EN()                              (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()                              (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()                              (RCC->APB1ENR |= (1 << 23))


// Peripheral Clock enable macros `SPIx peripheral clock enable`
#define SPI1_PCLK_EN()                              (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()                              (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()                              (RCC->APB1ENR |= (1 << 15))


// Peripheral Clock enable macros `USARTx peripheral clock enable`
#define USART1_PCLK_EN()                            (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()                            (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()                            (RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()                            (RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()                            (RCC->APB1ENR |= (1 << 20))


// Peripheral Clock enable macros `SYSCFG peripheral clock enable`
#define SYSCFG_PCLK_EN()                            (RCC->APB2ENR |= (1 << 14))


// Clock disable macros for GPIOx peripherals
#define GPIOA_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()                             (RCC->AHB1ENR &= ~(1 << 8))


// Clock disable macros for I2Cx peripherals
#define I2C1_PCLK_DI()                              (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()                              (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()                              (RCC->APB1ENR &= ~(1 << 23))


// Clock disable macros for SPIx peripherals
#define SPI1_PCLK_DI()                              (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()                              (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()                              (RCC->APB1ENR &= ~(1 << 15))


// Clock disable macros for USARTx peripherals
#define USART1_PCLK_DI()                            (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()                            (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()                            (RCC->APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()                            (RCC->APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()                            (RCC->APB1ENR &= ~(1 << 20))


// Clock disable macros for SYSCFG peripherals


// Clock reset for GPIOx peripherals
#define GPIOA_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()                           do { (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)                    ((x == GPIOA) ? 0 :\
		                                             (x == GPIOB) ? 1 :\
				                                     (x == GPIOC) ? 2 :\
						                             (x == GPIOD) ? 3 :\
						                             (x == GPIOE) ? 4 :\
								                     (x == GPIOF) ? 5 :\
								                     (x == GPIOG) ? 6 :\
								                     (x == GPIOH) ? 7 : 0)

// Clock reset for SPIx peripherals
#define SPI1_REG_RESET()                             do { (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()                             do { (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()                             do { (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)


/*
 * IRQ (Interrupt Request) Number of STM32F407x MCU
 * NOTE: update these macros with valid values according
 *       to your MCU
 */
#define IRQ_NO_EXTI0                                6
#define IRQ_NO_EXTI1                                7
#define IRQ_NO_EXTI2                                8
#define IRQ_NO_EXTI3                                9
#define IRQ_NO_EXTI4                                10
#define IRQ_NO_EXTI9_5                              23
#define IRQ_NO_EXT15_I0                             40


#define IRQ_NO_SPI1                                 35
#define IRQ_NO_SPI2                                 36
#define IRQ_NO_SPI3                                 51


// possible priority levels for interrupts
#define NVIC_IRQ_PRI0                               0
#define NVIC_IRQ_PRI15                              15


// Helper macros (Generic)
#define ENABLE                                      1
#define DISABLE                                     0
#define SET                                         ENABLE
#define RESET                                       DISABLE
#define GPIO_PIN_SET                                SET
#define GPIO_PIN_RESET                              RESET
#define FLAG_RESET                                  RESET
#define FLAG_SET                                    SET


// Bit position definitions SPI_CR1
#define SPI_CR1_CPHA     				            0
#define SPI_CR1_CPOL      				            1
#define SPI_CR1_MSTR     				            2
#define SPI_CR1_BR   					            3
#define SPI_CR1_SPE     				            6
#define SPI_CR1_LSBFIRST   			 	            7
#define SPI_CR1_SSI     				            8
#define SPI_CR1_SSM      				            9
#define SPI_CR1_RXONLY      				        10
#define SPI_CR1_DFF     			 	            11
#define SPI_CR1_CRCNEXT   			 	            12
#define SPI_CR1_CRCEN   			 	            13
#define SPI_CR1_BIDIOE     			 	            14
#define SPI_CR1_BIDIMODE                            15


// Bit position definition SPI_CR2
#define SPI_CR2_RXDMAEN		 			             0
#define SPI_CR2_TXDMAEN				 	             1
#define SPI_CR2_SSOE				 	             2
#define SPI_CR2_FRF                                  4
#define SPI_CR2_ERRIE					             5
#define SPI_CR2_RXNEIE				 	             6
#define SPI_CR2_TXEIE					             7


// Bit position definition SPI_SR
#define SPI_SR_RXNE						             0
#define SPI_SR_TXE				 		             1
#define SPI_SR_CHSIDE                                2
#define SPI_SR_UDR					 	             3
#define SPI_SR_CRCERR                                4
#define SPI_SR_MODF					 	             5
#define SPI_SR_OVR					 	             6
#define SPI_SR_BSY					 	             7
#define SPI_SR_FRE					 	             8


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */






















