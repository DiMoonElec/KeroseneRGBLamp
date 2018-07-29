#include <stdint.h>
#include "stm32f10x.h"

#define WIN_SIZE 8 //размер окна фильтра

//массивы фильтров
uint16_t a_array[WIN_SIZE];
uint16_t b_array[WIN_SIZE];

uint8_t ab_index = 0;

//Инициализация фильтров
static void ma_init(void)
{
  ab_index = 0;
  
  for(int i=0; i<WIN_SIZE; i++)
  {
    a_array[i] = 0;
    b_array[i] = 0;
  }
}

//Добавить новое значение в фильтр
static void ma_add(uint16_t a_val, uint16_t b_val)
{
  a_array[ab_index] = a_val;
  b_array[ab_index] = b_val;
  
  ab_index++;
  if(ab_index >= WIN_SIZE)
    ab_index = 0;
}

//плучить сглаженное значение
static void ma_get(uint16_t *a, uint16_t *b)
{
  uint32_t a_tmp = 0;
  uint32_t b_tmp = 0;
  for(int i = 0; i<WIN_SIZE; i++)
  {
    a_tmp += a_array[i];
    b_tmp += b_array[i];
  }
  
  (*a) = a_tmp / WIN_SIZE;
  (*b) = b_tmp / WIN_SIZE;
}
  
void ADC_AnalogRead(uint16_t *a, uint16_t *b)
{
  ADC1->SR = 0;
  ADC1->CR2 |= ADC_CR2_JSWSTART;
  
  while(!(ADC1->SR & ADC_SR_JEOC))
    ;
  
  uint16_t _a, _b;
  _a = ADC1->JDR1;
  _b = ADC1->JDR2;
  
  ma_add(_a, _b);
  ma_get(a, b);
}

void ADC_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //Включаем тактирование порта GPIOA
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //Включаем тактирование порта ADC1
  
  //PA3 и PA4 в аналоговом режиме
  GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3 | GPIO_CRL_CNF4 | GPIO_CRL_MODE4);
  
  RCC->CFGR |= RCC_CFGR_ADCPRE_1 | RCC_CFGR_ADCPRE_0; //Предделитель АЦП
  
  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 = ADC_CR2_JEXTTRIG |
    ADC_CR2_JEXTSEL_0 | ADC_CR2_JEXTSEL_1 | ADC_CR2_JEXTSEL_2;
  
  ADC1->SMPR2 |= ADC_SMPR2_SMP3_0 | ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_2 
    | ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2;
  
  ADC1->JSQR = ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_1 //channel 3
    | ADC_JSQR_JSQ3_2 //channel 4
    | ADC_JSQR_JL_0; //nom of channels: 2
  
  ADC1->CR2 |= ADC_CR2_ADON;
  
  ma_init();
}

void ButtonInit(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //Включаем тактирование порта GPIOA
  
  GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);
  GPIOA->CRL |= (GPIO_CRL_CNF6_1); 
  GPIOA->ODR |= (1<<6);
}

int ButtonIsPress(void)
{
  if(GPIOA->IDR & (1<<6))
    return 0;
  return 1;
}



