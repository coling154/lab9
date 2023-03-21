/* p5_8.c TIM3 Measuring Period and Frequency

 * This program configures TIM2 CH1 to toggle PA5 as was done in p5_6.
 * The TIM3 CH1 is set to do Input Capture from PA6. The program waits
 * for capture flag (CC1IF) to set then calculates the period and 
 * frequency of the input signal.
 * A jumper should be used to connect PA5 to PA6.
 * 
 * This program was tested with Keil uVision v5.24a with DFP v2.11.0
 */
#include "stm32f4xx.h"
#include <stdio.h>

int period;
float frequency;
void USART2_init(void);
void USART2_write(char* freq);


int main(void) {
    int last = 0;
    int current;

    char buffer[64];
    // configure PA5 as output of TIM2 CH1
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    GPIOA->MODER |=  0x00000800;    /* set pin to alternate function */
    GPIOA->AFR[0] &= ~0x00F00000;   /* clear pin AF bits */
    GPIOA->AFR[0] |= 0x00100000;    /* set pin to AF1 for TIM2 CH1 */
    
    // configure TIM2 to wrap around at 1 Hz 
	// and toggle CH1 output when the counter value is 0
    RCC->APB1ENR |= 1;              /* enable TIM2 clock */
    TIM2->PSC = 1600 - 1;           /* divided by 1600 */
    TIM2->ARR = 2500 - 1;           /* divided by 3000 */
    TIM2->CCMR1 = 0x30;             /* set output to toggle on match */
    TIM2->CCR2 = 0;                 /* set match value */
    TIM2->CCER |= 1;                /* enable ch 1 compare mode */
    TIM2->CNT = 0;                  /* clear counter */
    TIM2->CR1 = 1;                  /* enable TIM2 */
        
    // configure PA7 as input of TIM3 CH2
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER &= ~0x0000C000;    /* clear pin mode */
    GPIOA->MODER |=  0x00008000;    /* set pin to alternate function */
    GPIOA->AFR[0] &= ~0xF0000000;   /* clear pin AF bits */
    GPIOA->AFR[0] |= 0x20000000;    /* set pin to AF2 for TIM3 CH1 */

    // configure TIM3 to do input capture with prescaler ...
    RCC->APB1ENR |= 2;              /* enable TIM3 clock */
    TIM3->PSC = 16000 - 1;          /* divided by 16000 */
    TIM3->CCMR1 = 0x4100;             /* set CH1 to capture at every edge */
    TIM3->CCER = 0xB0;              /* enable CH 1 capture both edges */
    TIM3->CR1 = 1;                  /* enable TIM3 */
		
		USART2_init();
    while (1) {
        while (!(TIM3->SR & 4)) {}  /* wait until input edge is captured */
        current = TIM3->CCR2;       /* read captured counter value */
        period = current - last;    /* calculate the period */
        last = current;
        frequency = 1000.0f / period;
        last = current;
				sprintf(buffer, " Frequency: %.2f Hz.\r\n", frequency);
        USART2_write(buffer);
    }
}
void USART2_init(void){
	RCC->APB1ENR |= 0x20000; /* Enable USART2 clock */
	/* Configure PA3 for USART2 RX */
	GPIOA->AFR[0] &= ~0xF000;
	GPIOA->AFR[0] |= 0x7000; /* alt7 for USART2 */
	GPIOA->MODER &= ~0x00C0;
	GPIOA->MODER |= 0x0080; /* enable alternate function for PA3 */
	/* Configure PA2 for USART2_TX */
	GPIOA->AFR[0] &= ~0x0F00;
	GPIOA->AFR[0] |= 0x0700; /* alt7 for USART2 */
	GPIOA->MODER &= ~0x0030;
	GPIOA->MODER |= 0x0020; /* enable alternate function for PA2 */
	USART2->BRR = 0x0683; /* 9600 baud @ 16 MHz */
	USART2->CR1 = 0x0004; /* enable Rx, 8-bit data */
	USART2->CR1 |= 0x0008; /* enable Tx, 8-bit data */
	USART2->CR2 = 0x0000; /* 1 stop bit */
	USART2->CR3 = 0x0000; /* no flow control */
	USART2->CR1 |= 0x2000; /* enable USART2 */
}
void USART2_write(char* freq){
	uint8_t i=0;
	for(i=0;freq[i] != 0;i++){
		while(!(USART2->SR & 0x0080)){}//wait for tx buffer to empty
		USART2->DR =freq[i];
	}
}
