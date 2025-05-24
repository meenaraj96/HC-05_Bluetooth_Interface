/*
 * main_USART1.c
 *
 *  Created on: 21-Nov-2024
 *      Author: admin
 */

/* LED ON and OFF
 * LD2 - PORT A - PIN 5
 * USART2 global interrupt - Position - 38, Priority - 45
 * pNVIC_ISER0REG - 31 to 0
 * pNVIC_ISER1REG - 63 to 32
 * Position - 38
 * 0100 0000
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"

#define GPIOAEN (1U<<0)
#define UART2EN (1U<<17)
#define LED_PIN_NUMBER 5
#define Interrupt_Trigger (1U<<3)

#define SYS_FREQ 16000000
#define APB1_CLK SYS_FREQ
#define UART_BAUDRATE 9600

#define CR1_RE  (1U<<2)
#define CR1_TE  (1U<<3)
#define CR1_UE  (1U<<13)

#define CR2_STOP13  (1U<<13)
#define CR2_STOP12  (1U<<12)

#define CR1_M_WordLength  (1U<<12)
#define CR1_OVER8  (1U<<15)

#define SR_RXNE  (1U<<5)
#define SR_TXE  (1U<<7)

void uart2_Bluetooth_init();
char uart2_read();
void uart2_write(char ch);
//void uart2_read_sample(void);

static void uart_set_baudrate(USART_RegDef_t *USARTx,uint32_t PeriphClk,uint32_t Baudrate);
static uint16_t compute_uart_bd(uint32_t PeriphClk,uint32_t Baudrate);

int __io_putchar(char ch)
{
	uart2_write(ch);
	return ch;
}

void delay(void)
{
	for(uint32_t i=0; i<300000;i++);
}

int main(void)
{
	uint32_t temp=0;

	//LED
	RCC->AHB1ENR |=GPIOAEN;

	//clear and set MODER for LED
	temp = (0x1 << (2*LED_PIN_NUMBER));  //01: General purpose output mode

	GPIOA->MODER &= ~(0x3 << (2*LED_PIN_NUMBER) ) ; //Clear - 0.X=0 (3=>11) //create a mask value
	GPIOA->MODER |=temp; //Set

	uart2_Bluetooth_init();

	uart2_write('A');
}

void uart2_Bluetooth_init()
{
	RCC->AHB1ENR |=GPIOAEN;

		// UART2 - PA2  -> TRANSMIT - Alternate function mode(10)
		GPIOA->MODER &=~(1U <<4);
		GPIOA->MODER |= (1U <<5);
		// UART2 - PA3  -> RECEIVE - Alternate function mode(10)
		GPIOA->MODER &=~(1U <<6);
	    GPIOA->MODER |= (1U <<7);

	    ///PA2 - USART2_ TX -> ALTERNATE - AF7(0111)
		GPIOA->AFR[0] |=(1U<<8);
		GPIOA->AFR[0] |=(1U<<9);
		GPIOA->AFR[0] |=(1U<<10);
		GPIOA->AFR[0] &=~(1U<<11);

	    ///PA3 - USART2_ RX -> ALTERNATE - AF7(0111)
		GPIOA->AFR[0] |=(1U<<12);
		GPIOA->AFR[0] |=(1U<<13);
		GPIOA->AFR[0] |=(1U<<14);
		GPIOA->AFR[0] &=~(1U<<15);

	RCC->APB1ENR |= UART2EN;

		uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE); //139
		//139 - 1000 1011 => DIV_Mantissa = 8, DIV_Fraction = B(11)
		//When USART_CR1_OVER8=1, the DIV_Fraction bit is not considered and must be kept cleared.

		//TE=1: Transmitter is enabled, RE=1:Receiver is enabled and begins searching for a start bit
		USART2->CR1 = CR1_TE |CR1_RE;
		USART2->CR1 |= CR1_UE; //UE=1: USART enabled

		//USART2->CR1 &= CR1_OVER8;	//0: oversampling by 16
		//USART2->CR1 &= CR1_M_WordLength; 	//0: 1 Start bit, 8 Data bits, n Stop bit

		//USART2->CR2 &= CR2_STOP13; //00: 1 Stop bit
		//USART2->CR2 &= CR2_STOP12;

		//NVIC ENABLE
		*NVIC_ISER1 |=(1<<6); //pNVIC_ISER1REG - 63 to 32, Position - 38, 0100 0000
		GPIO_IRQConfig(IRQ_NO_USART2, ENABLE);
}

char uart2_read()
{
	//An interrupt is generated if RXNEIE=1 in the USART_CR1 register.
	//It is cleared by a read to the USART_DR register.
	//The RXNE flag can also be cleared by writing a zero to it.
	//This clearing sequence is recommended only for multibuffer communication.

	//CR1_RXNEIE: RXNE interrupt enable. This bit is set and cleared by software.
	//CR1_RXNEIE=1: An USART interrupt is generated, whenever ORE=1 or RXNE=1 in the USART_SR register

	//RXNE: Read data register not empty - RXNE=1: Received data is ready to be read
	//the content of the Receive shift register has been transferred to the RDR(Receive Data Register) - pg 968
	//get the "received data" from DR Register

	//make sure the data register is not empty
	while(!(USART2->SR  & SR_RXNE)){}

	//Read from Receive data register(RDR)
	return USART2->DR;
}

void USART2_IRQHandler(void)
{
	char key;
	*NVIC_IPR1 |=(1<<6); //pNVIC_IPR1REG - 63 to 32, Position - 38, 0100 0000

	key=uart2_read();
	//printf("Value = ",key);

	if (key=='O')
	{
		GPIOA->ODR |=  (1<<LED_PIN_NUMBER); //LED ON
		delay();
		printf("LED is ON \n \r");
	}
	else if (key=='F')
	{
		GPIOA->ODR &=~  (1<<LED_PIN_NUMBER); //LED OFF
		delay();
		printf("LED is OFF \n \r");
	}

	printf("Data Received \n");
}

void uart2_write(char ch)
{
	//while(!(*USART2_SR  & 0x0080)){}
	//*USART2_DR =(ch&0XFF) ;

	//interrupt is generated, if the TXEIE bit =1 in the USART_CR1 register
	//It is cleared by a write to the USART_DR register.

	//CR1_TXEIE: TXE interrupt enable. This bit is set and cleared by software
	//CR1_TXEIE=1: An USART interrupt is generated, whenever TXE=1 in the USART_SR register

	//TXE: Transmit data register empty - TXE=1: Data is transferred to the shift register
	//Content of the TDR(Transmit Data Register)register has been transferred into the Transmit shift register - pg 968
	//so keep the "data to transmit" in DR Register

	//Make sure the transmit data register(TDR) is empty
	while(!(USART2->SR  & SR_TXE)){}

	//write to transmit data register(TDR)
	USART2->DR =(ch & 0xFF) ; //1.X=X
}

static void uart_set_baudrate(USART_RegDef_t *USARTx,uint32_t PeriphClk,uint32_t Baudrate)
{
	USARTx->BRR = compute_uart_bd(PeriphClk,Baudrate); //139.38889
}


static uint16_t compute_uart_bd(uint32_t PeriphClk,uint32_t Baudrate)
{
	return ((PeriphClk+(Baudrate/2U))/Baudrate);  //139.38889
}



