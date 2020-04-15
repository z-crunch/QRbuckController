#include <stm32f303xc.h>
#include <stdbool.h>
#include "main.h"

#define BUF_SIZE        (16)
#define ADC_RESOLUTION  (4095)

#define FCLOCK	(64000000UL)

#define FREQ_MAX	(70000)
#define FREQ_MIN	(35000)

double Voltage = 0;             // Напряжение в миливольтах
uint16_t ADC_Data[BUF_SIZE];    // Массив для необработанных данных с АЦП
bool dma_ready = false;

/* Device settings */
struct timer_t
{
	uint16_t psc;
	uint16_t arr;
};

struct timer_t timer1;

struct LPF_Struct
{
	float tau, dt, taudt;
};
struct LPF_Struct LPF_Voltage;

void timer_init_timebase()
{
	TIM1->PSC = timer1.psc;
    TIM1->ARR = timer1.arr;
}

#define T_S	(12)	// pulse length

int main(void)
{
	int i, voltage = 0;
	float pu = 0;
	
	Init_Device();
	LPF_Voltage.dt = 1.0 / 200.0;
	LPF_Voltage.tau = 5 * LPF_Voltage.dt;
	LPF_Voltage.taudt = LPF_Voltage.tau + LPF_Voltage.dt;
	
	while(1)
	{
		if(dma_ready)
		{
			for(i = 0; i < BUF_SIZE; i++)
			{
				voltage = (ADC_Data[i] * LPF_Voltage.dt + LPF_Voltage.tau * voltage) / LPF_Voltage.taudt;
				pu = (float)(voltage) / (float)(ADC_RESOLUTION);
				Output_Frequency(pu);
				dma_ready = false;
			}
		}
	}
}

void Output_Frequency(float frequency_pu)
{
	float dc;
	
	uint32_t freq = FREQ_MAX * frequency_pu;
	if(freq > FREQ_MAX)
	{
		freq = FREQ_MAX;
	}
	else if(freq < FREQ_MIN)
	{
		freq = FREQ_MIN;
	}
	uint32_t arr = FCLOCK / (timer1.psc + 1) / freq / 2;
	dc = freq * T_S / 1000000.0;
	uint32_t ccr = dc * arr;
	
	TIM1->ARR = arr;
	TIM1->CCR1 = ccr;
	TIM1->CCR2 = arr - ccr;
}

void Init_Device(void)
{
	// 64 MHz
	// Set SYSCLK to 64 MHz
    RCC->CR &= ~(RCC_CR_PLLON);
    while(RCC->CR & RCC_CR_PLLRDY);
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;
    while(RCC->CR & RCC_CR_PLLRDY);
    RCC->CFGR |= (14 << RCC_CFGR_PLLMUL_Pos);
    RCC->CFGR |= RCC_CFGR_PPRE1_2;      // APB1 36 MHz max (32 MHz)
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));
    FLASH->ACR |= FLASH_ACR_LATENCY_1;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); 
    SystemCoreClockUpdate();
	
	// GPIO
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	GPIOE->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER11_1;
	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | 
                      GPIO_OSPEEDER_OSPEEDR11;
    GPIOE->AFR[1] |= (2 << GPIO_AFRH_AFRH1_Pos) | (2 << GPIO_AFRH_AFRH3_Pos);
	
	// TIM1 PWM out
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	timer1.psc = 0;
    timer1.arr = 1000;
	timer_init_timebase();
	TIM1->CR1 |= TIM_CR1_ARPE;
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	TIM1->CR1 |= 3 << TIM_CR1_CMS_Pos;
	TIM1->CCR1 = 0;
    TIM1->CCR2 = timer1.arr;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;
	
	// ADC
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_ADC12EN | RCC_AHBENR_DMA1EN;
    ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0;
	// Режим работы линии PA0 - Analog mode (0b11)
    GPIOA->MODER |= GPIO_MODER_MODER0;
	// Настройка DMA
    DMA1_Channel1->CNDTR = BUF_SIZE;
    DMA1_Channel1->CMAR = (uint32_t)ADC_Data;
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
    DMA1_Channel1->CCR |= DMA_CCR_CIRC | DMA_CCR_EN | DMA_CCR_MINC | 
                          DMA_CCR_TCIE | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0;
	// Включаем источник опорного напряжения (ИОН)
    ADC1->CR &= ~(ADC_CR_ADVREGEN);
    ADC1->CR |= ADC_CR_ADVREGEN_0;
	TIM6->ARR = 1000;
    // OPM - режим однократного срабатывания
    TIM6->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
    TIM6->SR &= TIM_SR_UIF;
    while(!(TIM6->SR & TIM_SR_UIF));
	TIM6->CR1 &= ~TIM_CR1_CEN;
    
    // Калибровка АЦП
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);
    
    // Настройка работы АЦП: опрашивается один канал, время выборки 181,5 такт
    // Включено прерывание по окончании преобразования одного канала
    ADC1->CR |= ADC_CR_ADEN;
    ADC1->SQR1 |= ADC_SQR1_SQ1_0;
    ADC1->SMPR1 |= 6 << ADC_SMPR1_SMP1_Pos;
    ADC1->CFGR |= ADC_CFGR_CONT | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    ADC1->CR |= ADC_CR_ADSTART;
	
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->BDTR |= TIM_BDTR_MOE;
	
    // Включение прерываний DMA
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	// Глобальное разрешение прерываний
    __enable_irq();
}


void DMA1_Channel1_IRQHandler()
{
    // Пока мы забираем данные по готовности
    dma_ready = true;
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
}
