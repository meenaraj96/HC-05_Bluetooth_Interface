/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: 06-Jul-2023
 *      Author: admin
 */
#include<stdint.h>
#include "stm32f411xx_gpio_driver.h"


void GPIO_PeripheralClockControl(GPIO_RegDef_t *pGPIOx,uint8_t En_Di)
{
	if(En_Di == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}

	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;
	// Configure the Mode of GPIO Pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//NON INTERRUPT MODE
		temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;

		pGPIOHandle->pGPIOx->MODER |=temp;
		temp=0;
	}
	else
	{
		// Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT)
		{
			// Configure the FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT)
		{
			// Configure the RTSR
			// Configure the FTSR
					EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

					EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RFT)
		{
			// Configure the RFTSR
			EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		// CONFIGURE THE GPIO PORT SELECTION IN SYSCFG_EXTICTR

      uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
      uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;
      uint8_t portcode = GPIO_BASEADDR_TO_CODE (pGPIOHandle->pGPIOx);
      SYSCFG_PCLK_EN();
      SYSCFG->EXTICR[temp1] =portcode << (temp2*4);


		// ENABLE THE EXTI Interrupt delivery using IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	// Configure Speed
	temp=0;
	temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;

	pGPIOHandle->pGPIOx->OSPEEDR |=temp;
	temp=0;
	// Configure the pupd settings

		temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;

		pGPIOHandle->pGPIOx->PUPDR|=temp;
		temp=0;
	// Configure the Optype
		temp =(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;

		pGPIOHandle->pGPIOx->OTYPER |=temp;
			temp=0;

	// Configure the alt functionality
			if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN)
				{
				uint32_t temp1 ,temp2;
				temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
				temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8;
				pGPIOHandle->pGPIOx->AFR[temp1]	&= 		~(0xF <<(4 *temp2));

				pGPIOHandle->pGPIOx->AFR[temp1]	|= 		pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4 *temp2);


				}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}




uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber)
{
 uint8_t value;
 value =(uint8_t)((pGPIOx->IDR >>PinNumber) & 0X00000001);
 return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	  uint16_t value;
	 value =(uint16_t)(pGPIOx->IDR);
	 return value;

}



void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber,uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |=  (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &=~  (1<<PinNumber);
	}

}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,uint8_t Value)
{
	pGPIOx->ODR =Value;

}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx ,uint8_t PinNumber)
{
	pGPIOx->ODR ^=(1<<PinNumber);


}


void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t En_Di )
{
	if (En_Di ==ENABLE)
	{
		if (IRQNumber < 32)  //0 to 31
		{
			//prgrm ISER0
			*NVIC_ISER0 |= (1<< IRQNumber);

		}else if (IRQNumber >= 32 && IRQNumber < 64)  //32 to 63
		{	//prgrm ISER1
			*NVIC_ISER1 |= (1<< IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96)  //64 to 95
		{
			//prgrm ISER2
			*NVIC_ISER2 |= (1<< IRQNumber % 64);

		}

	}
	else
	{
		if (IRQNumber < 32)  //0 to 31
				{
			//prgrm ICER0
			*NVIC_ICER0 |= (1<< IRQNumber);

				}else if (IRQNumber >= 32 && IRQNumber < 64)  //32 to 63
				{
					//prgrm ICER1
					*NVIC_ICER1 |= (1<< IRQNumber % 32);

				}else if(IRQNumber >= 64 && IRQNumber < 96)  //64 to 95
				{			//prgrm ICER2

					*NVIC_ICER2 |= (1<< IRQNumber % 64);

				}

	}

}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx =IRQNumber /4;
	uint8_t iprxSection = IRQNumber %4;
	uint8_t shift_amount =(8 * iprxSection) +(8-NO_OF_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDRESS +(iprx *4)) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//CLEAR THE EXTI PR REGISTER CORRESPONDING TO THE PIN NUMBER
	if(EXTI->PR & (1<<PinNumber))
	{
		EXTI->PR |= (1<<PinNumber);
	}

}



