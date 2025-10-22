#ifndef _TP_STM32_BIBLIO_H_
#define _TP_STM32_BIBLIO_H_

const int N =20;
unsigned int cycle[20] = {0,8,12,4,6,2,3,1};
volatile uint8_t index_cycle = 0;
volatile uint8_t index_recep = 0;
uint32_t f_clk = 4000000;


volatile uint8_t length_cycle = 8;
volatile uint8_t octet = 0;
volatile uint16_t period_s = 240;
volatile uint16_t MAX_period_s = 960;
volatile uint16_t MIN_period_s =60;

void config_gpio_pb(void);

void TIM2_config(void);
void TIM2_IRQHandler(void);
void IT_TIM2_config(void);
void TIM2_calcul_ARR(void);


void UART_config(void);
void USART3_IRQHandler(void);

void EXTI_Init(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);

#endif