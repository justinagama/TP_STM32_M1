#include "stm32l476xx.h"
#include "TP_STM32_biblio.h"

void config_gpio_pb(void)
{
	RCC->AHB2ENR |=RCC_AHB2ENR_GPIOBEN;
	
	// concfiguration de la broche 12 en sortie
	
	GPIOB ->MODER &= ~(3UL<<24);
	GPIOB ->MODER |=(1UL<<24);
	
	GPIOB ->MODER &= ~(3UL<<13*2);
	GPIOB ->MODER |=(1UL<<13*2);
	
	GPIOB ->MODER &= ~(3UL<<14*2);
	GPIOB ->MODER |=(1UL<<14*2);
	
	GPIOB ->MODER &= ~(3UL<<15*2);
	GPIOB ->MODER |=(1UL<<15*2);
	
	// configuration de otyper 
	
	GPIOB ->OTYPER &=~(1<<12);
	
	GPIOB ->OTYPER &=~(1<<13);
	
	GPIOB ->OTYPER &=~(1<<14);
	
	GPIOB ->OTYPER &=~(1<<15);
	
	//conficuration de la vitesse 
	
	GPIOB ->OSPEEDR &=~(3UL<<24);
	GPIOB ->OSPEEDR |=(0UL<<24);
	
	GPIOB ->OSPEEDR &=~(3UL<<13*2);
	GPIOB ->OSPEEDR |=(0UL<<13*2);
	
	GPIOB ->OSPEEDR &=~(3UL<<14*2);
	GPIOB ->OSPEEDR |=(0UL<<14*2);
	
	GPIOB ->OSPEEDR &=~(3UL<<15*2);
	GPIOB ->OSPEEDR |=(0UL<<15*2);
}

/*-----------------------------------------------------------------*/


void TIM2_config(void)
{
	// activation de l'horloge du timer 2 
	
	RCC->APB1ENR1 |=(1UL<<0);
	
	// la valeur de rechargement et du prescalade 
	
	TIM2 ->PSC = 99;
	TIM2_calcul_ARR();
//	TIM2 ->ARR = (int)(((period_s*4000000)/((TIM2 ->PSC)+1))-1);

	
}

void TIM2_calcul_ARR(void)
{
    TIM2->ARR = (f_clk * (period_s/1000 )) / (TIM2->PSC + 1) - 1;
    TIM2->EGR = TIM_EGR_UG; // force la mise � jour
}

/*-----------------------------------------------------------------*/

void IT_TIM2_config(void)
{
	// autorisation des interuption pour tim2
	TIM2 ->DIER |=TIM_DIER_UIE;
	
	// activation
	TIM2 ->CR1 |=TIM_CR1_CEN;
	
	// definition de prioriter de niveau faible 7 plus le nombre est grand plus on a une prioriter bas 
	NVIC_SetPriority(TIM2_IRQn,3);
	
	// configuration de linteruption dans le coeur 
	NVIC_EnableIRQ(TIM2_IRQn);

	
}

/*-----------------------------------------------------------------*/

void TIM2_IRQHandler(void)
{

	if (TIM2 ->SR & TIM_SR_UIF)
	{
		TIM2->SR &=~TIM_SR_UIF; // mettre 0 sur le bit de debordement 
		
        // Effacer l'ancien �tat des LEDs
		
	    GPIOB->BSRR |= (15UL<< (12+16));

        // Appliquer le nouveau Cycle sur PB12-PB15
        GPIOB->BSRR |= ((cycle[index_cycle]) << 12);

        // Incr�menter l�index cycliquement
        index_cycle++;

		if (index_cycle >= length_cycle)
		{
            index_cycle = 0;
		}
				
	}

}


/*----------------------------- UART2 ----------------------------------*/

void UART_config(void)
{
	// configuration des clock 
	RCC->AHB2ENR |=RCC_AHB2ENR_GPIOCEN;
	RCC->AHB2ENR |=RCC_AHB2ENR_GPIOBEN;

	RCC->APB1ENR1 |=RCC_APB1ENR1_USART3EN;
	
	// mode A2 et 3
	GPIOC->MODER &=~(3UL<<4*2);
	GPIOB->MODER &=~(3UL<<11*2);
	
	GPIOC->MODER |=(2UL<<4*2);
	GPIOB->MODER |=(2UL<<11*2);
	
	// config de AF7 sur AFR[0] 
	GPIOC->AFR[0] &=~(15UL<<16);
	GPIOB->AFR[1] &=~(15UL<<12);
	
	GPIOC->AFR[0] |=(7UL<<16);
	GPIOB->AFR[1] |=(7UL<<12);
	
	// baud rate USART3DVI = fclk/19200 = 208 avec fclk = 4Mhz
	
	USART3->BRR = 208;
	
	//config du bit de donnee
	USART3->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1); // 8 bits
	
	// activation de bit de pariter 
	USART3 ->CR1 &=~(1UL<<10); // mise a 0 du bit de parite PCE 
	
	// activation 1 bit de stop 
	USART3 ->CR2 &=~(3UL<<12); // decalage de 00 sur le bit 12 et 13
	
	// activation de la reception 
	USART3 ->CR1 |=USART_CR1_RE;
	
	// activation de la transmission
	USART3 ->CR1 |=USART_CR1_TE;
	
	// activation de l'interuption 
	USART3 ->CR1 |=USART_CR1_RXNEIE;
	
	// activation de  USART3
	USART3 ->CR1 |= USART_CR1_UE;
	
	//Prioriter 
	NVIC_SetPriority(USART3_IRQn,1);
	
	// activation de NVIC
	NVIC_EnableIRQ(USART3_IRQn);
	
}

/*-----------------------------------------------------------------*/

void USART3_IRQHandler(void) 
{
	// ISR et RXNE
    if(USART3->ISR & USART_ISR_RXNE)
    {
        octet = (USART3->RDR);  // stocker donn�e re�ue dans variable octet

        if(index_recep == 0)
        {
			//  stop le chenillard

	        TIM2 ->CR1 &=~(TIM_CR1_CEN);

            // Premier octet = longueur du cycle

            length_cycle = octet-48;

            if(length_cycle > N)
			{
              length_cycle = N;
			}
				
					


            index_recep++; // ==1
        }
        else
        {
            // Donn�es du cycle
            cycle[index_recep - 1] = octet;  // on commence � 0
            index_recep++;

            if(index_recep > length_cycle)
            {
               index_recep = 0;
			   index_cycle = 0;							
				// signal qu'on a tout re�u
			   USART3->TDR = 1;           
              // red�marre le timer pour le chenillard
			   TIM2 ->CR1 |=TIM_CR1_CEN;

            }
        }
    }
}

/*------------------------------- 4_RF ----------------------------------*/

void EXTI_Init(void)
{
	// Configuration des horloge pour le pC et syscf
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;  
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;   
	
	// Mettre PC2/PC3 en input (MODER = 00) et pull-up (PUPDR = 01)
    GPIOC->MODER &= ~(3 << (2*2)); 
    GPIOC->MODER &= ~(3 << (3*2));   

	
    GPIOC->PUPDR &= ~(3 << (2*2)); 
	GPIOC->PUPDR &= ~(3 << (3*2));
	
	
    GPIOC->PUPDR |=  (1 << (2*2));
	GPIOC->PUPDR |= (1 << (3*2));    


	SYSCFG ->EXTICR[0] &=~(15<<8) ;
	SYSCFG ->EXTICR[0] |=(2<<8) ;

	SYSCFG ->EXTICR[0] &=~(15<<12) ;
	SYSCFG ->EXTICR[0] |=(2<<12) ;

	//EXTI ->FTSR1 |=(1<<2);
	EXTI ->FTSR1 |=	EXTI_FTSR1_FT2 ;
	//EXTI ->FTSR1 |=(1<<3);
	EXTI ->FTSR1 |=	EXTI_FTSR1_FT3 ;
	
	EXTI ->IMR1 |= EXTI_IMR1_IM2;
	EXTI ->IMR1 |= EXTI_IMR1_IM3;

	NVIC_SetPriority(EXTI2_IRQn,2);
	NVIC_SetPriority(EXTI3_IRQn,2);

    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
	
}

/*-----------------------------------------------------------------*/

void EXTI2_IRQHandler(void) 
	{
    if (EXTI->PR1 & (1 << 2)) 
			{
        EXTI->PR1 = (1 << 2); // efface le flag proprement

		if (period_s <= MAX_period_s) 
		{
            period_s *= 2;			
		    // mise a jour du ARR si je met a jour dans tim_init il ne peux pas rexecuter cette fonction on lexecute qu'une foi dans la fonction main
			TIM2_calcul_ARR();
        }
      }
  }

void EXTI3_IRQHandler(void)
{
    // V�rifie que le flag d'interruption est bien positionn� pour EXTI3
    if (EXTI->PR1 & (1 << 3))
    {
        EXTI->PR1 = (1 << 3); // Efface le flag
				
        // Divise la p�riode si elle est sup�rieure � la limite min
		if (period_s >= MIN_period_s)
		{
            period_s /= 2;
		    TIM2_calcul_ARR();				
		}
				
    }
}
