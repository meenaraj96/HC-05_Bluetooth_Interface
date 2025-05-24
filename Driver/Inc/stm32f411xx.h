/*
 * stm32f411xx.h
 *
 *  Created on: 22-Jul-2023
 *      Author: admin
 */

#ifndef STM32F411XX_H_
#define STM32F411XX_H_
#include <stdint.h>

#define FLASH_BASEADDR 0x80000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define ROM_BASEADDR   0x1FFF0000U
#define SRAM           SRAM1_BASEADDR

#define APB1PERIPH_BASE 0x40000000U
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x50000000U

#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASE + 0x2000)

#define I2C1_BASEADDR (APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0X5800)
#define I2C3_BASEADDR (APB1PERIPH_BASE + 0X5C00)

#define SPI2_BASEADDR (APB1PERIPH_BASE + 0X3800)
#define SPI3_BASEADDR (APB1PERIPH_BASE + 0X3C00)

#define SPI1_BASEADDR (APB2PERIPH_BASE + 0X3000)
#define SPI4_BASEADDR (APB2PERIPH_BASE + 0X3400)
#define SPI5_BASEADDR (APB2PERIPH_BASE + 0X5000)
#define SPI6_BASEADDR (APB2PERIPH_BASE + 0X5400)

#define USART1_BASEADDR (APB2PERIPH_BASE + 0X1000)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0X1400)

#define USART2_BASEADDR (APB1PERIPH_BASE + 0X4400)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0X4800)

#define UART4_BASEADDR (APB1PERIPH_BASE + 0X4C00)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0X5000)
#define UART7_BASEADDR (APB1PERIPH_BASE + 0X7800)
#define UART8_BASEADDR (APB1PERIPH_BASE + 0X7C00)

#define RCC_BASEADDR (AHB1PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR (APB2PERIPH_BASE + 0X3C00)
#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0X3800)

#define __vo volatile

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET


#define FLAG_SET 1
#define FLAG_RESET 0

#define I2C_CR1_PE 0
#define I2C_CR1_NOSTRETCH 7
#define I2C_CR1_START 8
#define I2C_CR1_STOP 9
#define I2C_CR1_ACK 10
#define I2C_CR1_SWRST 15

#define I2C_CR2_FREQ 0
#define I2C_CR2_ITERREN 8
#define I2C_CR2_ITEVTEN 9
#define I2C_CR2_ITBUFEN 10

#define I2C_OAR1_ADD0 0
#define I2C_OAR1_ADD7_1 1
#define I2C_OAR1_ADD9_8 8
#define I2C_OAR1_ADDMODE 15

#define I2C_SR1_SB 0
#define I2C_SR1_ADDR 1
#define I2C_SR1_BTF 2
#define I2C_SR1_ADD10 3
#define I2C_SR1_STOPF 4
#define I2C_SR1_RXNE 6
#define I2C_SR1_TXE 7
#define I2C_SR1_BERR 8
#define I2C_SR1_ARL0 9
#define I2C_SR1_AF 10
#define I2C_SR1_OVR 11
#define I2C_SR1_TIMEOUT 14

#define I2C_SR2_MSL 0
#define I2C_SR2_BUSY 1
#define I2C_SR2_TRA 2
#define I2C_SR2_GENCALL 4
#define I2C_SR2_DUALF 7

#define I2C_CCR_CCR11_0 0
#define I2C_CCR_DUTY 14
#define I2C_CCR_FS 15

#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI5_9 23
#define IRQ_NO_EXTI10_15 40

#define IRQ_NO_USART1 37
#define IRQ_NO_USART2 38

#define NVIC_ISER0 ( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1 ( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2 ( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3 ( (__vo uint32_t*) 0xE000E10C )

#define NVIC_ICER0 ( (__vo uint32_t*) 0xE000E180 )
#define NVIC_ICER1 ( (__vo uint32_t*) 0xE000E184 )
#define NVIC_ICER2 ( (__vo uint32_t*) 0xE000E188 )
#define NVIC_ICER3 ( (__vo uint32_t*) 0xE000E18C )

#define NVIC_IPR0 ( (__vo uint32_t*) 0xE000E400 )
#define NVIC_IPR1 ( (__vo uint32_t*) 0xE000E404 )
#define NVIC_IPR2 ( (__vo uint32_t*) 0xE000E408 )
#define NVIC_IPR3 ( (__vo uint32_t*) 0xE000E40C )

#define NVIC_PR_BASE_ADDRESS ( (__vo uint32_t*) 0xE000E400 )
#define NO_OF_PR_BITS_IMPLEMENTED 4

#define NVIC_IRQ_PRIO1 1
#define NVIC_IRQ_PRIO2 2
#define NVIC_IRQ_PRIO3 3
#define NVIC_IRQ_PRIO4 4
#define NVIC_IRQ_PRIO5 5
#define NVIC_IRQ_PRIO6 6
#define NVIC_IRQ_PRIO7 7
#define NVIC_IRQ_PRIO8 8
#define NVIC_IRQ_PRIO9 9
#define NVIC_IRQ_PRIO10 10
#define NVIC_IRQ_PRIO11 11
#define NVIC_IRQ_PRIO12 12
#define NVIC_IRQ_PRIO13 13
#define NVIC_IRQ_PRIO14 14
#define NVIC_IRQ_PRIO15 15

#define USART_CR1_SBK 0
#define USART_CR1_RWU 1
#define USART_CR1_RE 2
#define USART_CR1_TE 3
#define USART_CR1_IDLEIE 4
#define USART_CR1_RXNEIE 5
#define USART_CR1_TCIE 6
#define USART_CR1_TXEIE 7
#define USART_CR1_PEIE 8
#define USART_CR1_PS 9
#define USART_CR1_PCE 10
#define USART_CR1_WAKE 11
#define USART_CR1_M 12
#define USART_CR1_UE 13
#define USART_CR1_OVER8 15

#define USART_CR2_ADD3_0 0
#define USART_CR2_LBDL 5
#define USART_CR2_LBDIE 6
#define USART_CR2_LBCL 8
#define USART_CR2_CPHA 9
#define USART_CR2_CPOL 10
#define USART_CR2_CLKEN 11
#define USART_CR2_STOP1_0 12
#define USART_CR2_LINEN 14

#define USART_CR3_EIE 0
#define USART_CR3_IREN 1
#define USART_CR3_IRLP 2
#define USART_CR3_HDSEL 3
#define USART_CR3_NACK 4
#define USART_CR3_SCEN 5
#define USART_CR3_DMAR 6
#define USART_CR3_DMAT 7
#define USART_CR3_RTSE 8
#define USART_CR3_CTSE 9
#define USART_CR3_CTSIE 10
#define USART_CR3_ONEBIT 11

#define USART_SR_PE 0
#define USART_SR_FE 1
#define USART_SR_NF 2
#define USART_SR_ORE 3
#define USART_SR_IDLE 4
#define USART_SR_RXNE 5
#define USART_SR_TC 6
#define USART_SR_TXE 7
#define USART_SR_LBD 8
#define USART_SR_CTS 9

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;

} RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

} EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;

} SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;      /*!< I2C Own address register 2, Address offset: 0x0C */
	__vo uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
	__vo uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
	__vo uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
	__vo uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
	__vo uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
	__vo uint32_t FLTR;

} I2C_RegDef_t;

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;

} USART_RegDef_t;

#define RCC ((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI ((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

#define SYSCFG_PCLK_EN() ( RCC->APB2ENR |= (1<<14) )

#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define GPIOA_PCLK_EN() ( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN() ( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN() ( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN() ( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN() ( RCC->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN() ( RCC->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN() ( RCC->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN() ( RCC->AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN() ( RCC->AHB1ENR |= (1<<8) )

#define GPIOA_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<7) )
#define GPIOI_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<8) )

#define USART1 ((USART_RegDef_t*) USART1_BASEADDR)
#define USART2 ((USART_RegDef_t*) USART2_BASEADDR)
#define USART3 ((USART_RegDef_t*) USART3_BASEADDR)
#define UART4 ((USART_RegDef_t*) UART4_BASEADDR)
#define UART5 ((USART_RegDef_t*) UART5_BASEADDR)
#define USART6 ((USART_RegDef_t*) USART6_BASEADDR)
#define UART7 ((USART_RegDef_t*) UART7_BASEADDR)
#define UART8 ((USART_RegDef_t*) UART8_BASEADDR)

#define USART1_PCLK_EN() ( RCC->APB2ENR |= (1<<4) )
#define USART2_PCLK_EN() ( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN() ( RCC->APB1ENR |= (1<<18) )
#define UART4_PCLK_EN() ( RCC->APB1ENR |= (1<<19) )
#define UART5_PCLK_EN() ( RCC->APB1ENR |= (1<<20) )
#define USART6_PCLK_EN() ( RCC->APB2ENR |= (1<<5) )
#define UART7_PCLK_EN() ( RCC->APB1ENR |= (1<<30) )
#define UART8_PCLK_EN() ( RCC->APB1ENR |= (1<<31) )

#define USART1_PCLK_DI() ( RCC->APB2ENR &= ~(1<<4) )
#define USART2_PCLK_DI() ( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI() ( RCC->APB1ENR &= ~(1<<18) )
#define UART4_PCLK_DI() ( RCC->APB1ENR &= ~(1<<19) )
#define UART5_PCLK_DI() ( RCC->APB1ENR &= ~(1<<20) )
#define USART6_PCLK_DI() ( RCC->APB2ENR &= ~(1<<5) )
#define UART7_PCLK_DI() ( RCC->APB1ENR &= ~(1<<30) )
#define UART8_PCLK_DI() ( RCC->APB1ENR &= ~(1<<31) )

#define I2C1 ((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t*) I2C3_BASEADDR)

#define I2C1_PCLK_EN() ( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN() ( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN() ( RCC->APB1ENR |= (1<<23) )

#define I2C1_PCLK_DI() ( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI() ( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI() ( RCC->APB1ENR &= ~(1<<23) )

#define GPIOA_REG_RESET() do{ 									\
								((RCC->AHB1RSTR) |= (1<<0));	\
								((RCC->AHB1RSTR) &= ~(1<<0));	\
							} while(0)

#define GPIOB_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<1)); ((RCC->AHB1RSTR) &= ~(1<<1)); } while(0)
#define GPIOC_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<2)); ((RCC->AHB1RSTR) &= ~(1<<2)); } while(0)
#define GPIOD_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<3)); ((RCC->AHB1RSTR) &= ~(1<<3)); } while(0)
#define GPIOE_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<4)); ((RCC->AHB1RSTR) &= ~(1<<4)); } while(0)
#define GPIOF_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<5)); ((RCC->AHB1RSTR) &= ~(1<<5)); } while(0)
#define GPIOG_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<6)); ((RCC->AHB1RSTR) &= ~(1<<6)); } while(0)
#define GPIOH_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<7)); ((RCC->AHB1RSTR) &= ~(1<<7)); } while(0)
#define GPIOI_REG_RESET() do { ((RCC->AHB1RSTR) |= (1<<8)); ((RCC->AHB1RSTR) &= ~(1<<8)); } while(0)


#define GPIO_BASEADDR_TO_CODE(X) ( 						\
									(X == GPIOA) ? 0 :	\
									(X == GPIOB) ? 1 : 	\
									(X == GPIOC) ? 2 :	\
									(X == GPIOD) ? 3 :	\
									(X == GPIOE) ? 4 :	\
									(X == GPIOF) ? 5 :	\
									(X == GPIOG) ? 6 :	\
									(X == GPIOH) ? 7 :	\
									(X == GPIOI) ? 8 :0 \
								)



#endif /* STM32F411XX_H_ */
