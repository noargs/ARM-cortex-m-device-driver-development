#include "stm32f407xx_gpio_driver.h"

// Peripheral clock setup

/*********************************************************************
 * @brief            - Enables or disable peripheral clock for
 *                     the given GPIO port
 *
 * @Param[in]        - Base address of the GPIO peripheral
 * @Param[in]        - Enable or Disable macro (i.e. ENABLE, DISABLE)
 *
 * @return           -
 *
 * @Note             -
 */
void GPIO_PCLK_Ctrl(GPIO_RegDef_t *gpiox, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		if (gpiox == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (gpiox == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (gpiox == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (gpiox == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (gpiox == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (gpiox == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (gpiox == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (gpiox == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (gpiox == GPIOI) {
			GPIOI_PCLK_EN();
		}
	} else {
		//TODO
	}

}

/*********************************************************************
 * @brief            -
 *
 * @Param[in]        -
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           -
 *
 * @Note             -
 */
void GPIO_Init(GPIO_Handle_t *gpio_handle) {

	uint32_t temp = 0;

	// 1. configure the mode of gpio pin
	if (gpio_handle->GPIO_PinConfig.gpio_pin_mode <= GPIO_MODE_ANALOG) {
		// -[none interrupt mode]-
		temp = (gpio_handle->GPIO_PinConfig.gpio_pin_mode
				<< (2 * gpio_handle->GPIO_PinConfig.gpio_pin_number));
		gpio_handle->GPIOx->MODER &= ~(0x3
				<< gpio_handle->GPIO_PinConfig.gpio_pin_number);
		gpio_handle->GPIOx->MODER |= temp;
	} else {
		// -[interrupt mode]-

		// GPIO Pin Interrupt configuration
		//1. Pin must be in input configuration
		//2. Configure the edge trigger (RT,FT,RFT)
		//3. Enable interrupt delivery from peripheral to the processor (on peripheral side)
		//4. Identify the IRQ number on which the processor accepts the interrupt from that pin
		//5. Configure the IRQ priority for the identified IRQ number (Processor side)
		//6. Enable interrupt reception on that IRQ number (Processor side)
		//7. Implement IRQ handler

		if (gpio_handle->GPIO_PinConfig.gpio_pin_mode == GPIO_MODE_IT_FT) {
			// configure the FTSR (Falling trigger selection register)
			// FTSR is the register of EXTI
			EXTI->FTSR |= (1 << gpio_handle->GPIO_PinConfig.gpio_pin_number);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << gpio_handle->GPIO_PinConfig.gpio_pin_number);
		} else if (gpio_handle->GPIO_PinConfig.gpio_pin_mode == GPIO_MODE_IT_RT) {
			// configure RTSR (Rising trigger selection register
			EXTI->RTSR |= (1 << gpio_handle->GPIO_PinConfig.gpio_pin_number);
			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << gpio_handle->GPIO_PinConfig.gpio_pin_number);
		} else if (gpio_handle->GPIO_PinConfig.gpio_pin_mode == GPIO_MODE_IT_RFT) {
			// configure both FTSR and RTSR
			EXTI->FTSR |= (1 << gpio_handle->GPIO_PinConfig.gpio_pin_number);
			EXTI->RTSR |= (1 << gpio_handle->GPIO_PinConfig.gpio_pin_number);
		}
		//2. configure the GPIO port selection in SYSCGF_EXTICR (Sys Config EXTI Control Register)
		//   Which GPIO port should handle by which EXTIx line
		uint8_t temp1 = gpio_handle->GPIO_PinConfig.gpio_pin_number / 4;
		uint8_t temp2 = gpio_handle->GPIO_PinConfig.gpio_pin_number % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(gpio_handle->GPIOx); // TODO
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable the EXTI interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << gpio_handle->GPIO_PinConfig.gpio_pin_number);
	}

	temp = 0;

	// 2. configure the speed
	temp = (gpio_handle->GPIO_PinConfig.gpio_pin_speed
			<< (2 * gpio_handle->GPIO_PinConfig.gpio_pin_number));
	gpio_handle->GPIOx->OSPEEDR &= ~(0x3
			<< gpio_handle->GPIO_PinConfig.gpio_pin_number);
	gpio_handle->GPIOx->OSPEEDR |= temp;

	temp = 0;

	// 3. configure the pu-pd settings
	temp = (gpio_handle->GPIO_PinConfig.gpio_pin_pu_pd_control
			<< (2 * gpio_handle->GPIO_PinConfig.gpio_pin_number));
	gpio_handle->GPIOx->PUPDR &= ~(0x3
			<< gpio_handle->GPIO_PinConfig.gpio_pin_number);
	gpio_handle->GPIOx->PUPDR |= temp;

	temp = 0;

	// 4. configure the optype
	temp = gpio_handle->GPIO_PinConfig.gpio_pin_op_type
			<< gpio_handle->GPIO_PinConfig.gpio_pin_number;
	gpio_handle->GPIOx->OTYPER &= ~(0x1
			<< gpio_handle->GPIO_PinConfig.gpio_pin_number);
	gpio_handle->GPIOx->OTYPER |= temp;

	temp = 0;

	// 5. configure the alt functionality
	if (gpio_handle->GPIO_PinConfig.gpio_pin_mode == GPIO_MODE_ALTFN) {
		uint8_t temp1, temp2;
		temp1 = gpio_handle->GPIO_PinConfig.gpio_pin_number / 8;
		temp2 = gpio_handle->GPIO_PinConfig.gpio_pin_number % 8;
		gpio_handle->GPIOx->AFR[temp1] &= ~(0xFF << (4 * temp2));
		gpio_handle->GPIOx->AFR[temp1] |=
				(gpio_handle->GPIO_PinConfig.gpio_pin_alt_fun << (4 * temp2));

	}
}

/*********************************************************************
 * @brief            -
 *
 * @Param[in]        -
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           -
 *
 * @Note             -
 */
void GPIO_DeInit(GPIO_RegDef_t *gpiox) {

	if (gpiox == GPIOA) {
		GPIOA_REG_RESET();
	} else if (gpiox == GPIOB) {
		GPIOB_REG_RESET();
	} else if (gpiox == GPIOC) {
		GPIOC_REG_RESET();
	} else if (gpiox == GPIOD) {
		GPIOD_REG_RESET();
	} else if (gpiox == GPIOE) {
		GPIOE_REG_RESET();
	} else if (gpiox == GPIOF) {
		GPIOF_REG_RESET();
	} else if (gpiox == GPIOG) {
		GPIOG_REG_RESET();
	} else if (gpiox == GPIOH) {
		GPIOH_REG_RESET();
	} else if (gpiox == GPIOI) {
		GPIOI_REG_RESET();
	}

}

// Data read and write

/*********************************************************************
 * @brief            -
 *
 * @Param[in]        -
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           -
 *
 * @Note             -
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *gpiox, uint8_t pin_number) {
	uint8_t value;
	// shift right by i.e. 8 times if pin number is 8
	// and then read the 0th bit (lsb) and mask all
	// other bit positions with 0x00000001 except 0th bit
	// which we want to read
	value = (uint8_t) ((gpiox->IDR >> pin_number) & 0x00000001);
	return value;

}

/*********************************************************************
 * @brief            -
 *
 * @Param[in]        -
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           -
 *
 * @Note             -
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *gpiox) {
	uint16_t value;
	value = (uint16_t) gpiox->IDR;
	return value;
}

/*********************************************************************
 * @brief            -
 *
 * @Param[in]        -
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           -
 *
 * @Note             -
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *gpiox, uint8_t pin_number,
		uint8_t value) {
	if (value == GPIO_PIN_SET) {
		// write 1 to the output data register at the bit field
		// correspond to the pin number
		gpiox->ODR |= (1 << pin_number);
	} else {
		gpiox->ODR &= ~(1 << pin_number);
	}
}

/*********************************************************************
 * @brief            -
 *
 * @Param[in]        -
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           -
 *
 * @Note             -
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *gpiox, uint16_t value) {
	gpiox->ODR = value;
}

/*********************************************************************
 * @brief            -
 *
 * @Param[in]        -
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           -
 *
 * @Note             -
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *gpiox, uint8_t pin_number) {
	gpiox->ODR ^= (1 << pin_number);
}

// IRQ Configuration and ISR handling

/*********************************************************************
 * @brief            - Processor specific whereas EXTI is on peripheral
 *                     side, consult Cortex-M4 Generic User Guide -
 *                     NVIC @ page 219 [GPIO -> EXTI -> NVIC]
 * @Param[in]        - IRQ Number
 * @Param[in]        - ENABLE, DISABLE
 *
 * @return           - void
 */
void GPIO_IRQInterruptConfig(uint8_t irq_number, uint8_t en_or_di) {
	if (en_or_di == ENABLE) {
		if (irq_number <= 31) {
			// Program ISER0 register Interrupt Set-enable register @ page 220 [Cortex M4 Generic User Guide]
			*NVIC_ISER0 |= ( 1 << irq_number );
		} else if (irq_number > 31 && irq_number < 64) {
			// Program ISER1 register
			*NVIC_ISER1 |= ( 1 << ( irq_number % 32 ) );
		} else if (irq_number >= 64 && irq_number < 96) {
			// program ISER2 register
			*NVIC_ISER2 |= ( 1 << ( irq_number % 64 ) );
		} else {
			if (irq_number <= 31) {
				// Program ICER0 register Interrupt Clear-enable register @ page 221 [Cortex M4 Generic User Guide]
				*NVIC_ICER0 |= ( 1 << irq_number );
			} else if (irq_number > 31 && irq_number < 64) {
				// Program ICER1 register
				*NVIC_ICER1 |= ( 1 << ( irq_number % 32 ) );
			} else if (irq_number >= 64 && irq_number < 96) {
				// program ICER2 register
				*NVIC_ISER2 |= ( 1 << ( irq_number % 64 ) );
			}
		}
	}
}


/*********************************************************************
 * @brief            - there are 60 Priority registers (Interrupt Priority
 *                     Registers IPR) in Cortex M4 Processor from IPR0 to IPR59
 *                     IPR59; each register i.e. IPR0 is divided into 4 sections
 *                     8 bit each (i.e. 8 bit x 4 sections = 32 bit)
 *                     each section is for 1 x IRQ Number in order to configure
 *                     the priority value, for example in IPR0 you can set priority
 *                     for 4 IRQ Numbers
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           - void
 */
void GPIO_IRQPriorityConfig(uint8_t irq_number, uint8_t irq_priority) {
	//1. find out the IPR register
	uint8_t iprx = irq_number / 4;
	uint8_t iprx_section = irq_number % 4;

	uint8_t shift_by = ( 8 * iprx_section ) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (irq_priority << shift_by );
}

/*********************************************************************
 * @brief            -
 *
 * @Param[in]        -
 * @Param[in]        -
 * @Param[in]        -
 *
 * @return           -
 *
 * @Note             -
 */
void GPIO_IRQHandling(uint8_t pin_number) {
	// clear the EXTI priority PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << pin_number )) {
		// clear the pending register, writing 1 to pending register will clear
		EXTI->PR |= ( 1 << pin_number);
	}
}
