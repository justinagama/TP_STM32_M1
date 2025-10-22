#include "stm32l476xx.h"
#include "TP_STM32_biblio.h"

int main (void)
{

	config_gpio_pb();

	UART_config(); 	USART3_IRQHandler();

	EXTI_Init(); EXTI2_IRQHandler(); EXTI3_IRQHandler();
	
	TIM2_config(); IT_TIM2_config();

	
	while(1)
	{
		
	}
	return 0;
}

