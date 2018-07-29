#include <stdint.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "ws2812b.h"
#include "color.h"
#include "inputs.h"


#define NUM_ROW         10
#define NUM_COLUM       8

/// ���������� ����������

//���������� RGB
uint8_t rc = 0;
uint8_t gc = 0;
uint8_t bc = 0;

  
uint16_t dimmer_val; //�������� �������
uint16_t mode_val; //�������� ������

uint8_t NumMode = 1; //����� ������

/// ���������� �����
uint8_t flag_dispmode = 0;

/******************************************************************************/

#define TIMER_FPS               0
#define TIMER_DISP              1
#define TIMER_ANTIBOUNCE        2

#define NUM_SOFT_TIMERS         3
uint32_t soft_timer[NUM_SOFT_TIMERS];

//������������� ������
//TIM3
void TickTimerInit(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //�������� ������������ TIM3
  
  TIM3->PSC = 71; //������������ 71+1=72
  TIM3->ARR = 10000; //������ 10000���, ������� 100��
  TIM3->DIER = TIM_DIER_UIE; //���������� �� ����������
  
  for(int i=0; i<NUM_SOFT_TIMERS; i++)
  {
    soft_timer[i] = 0;
  }
  
  TIM3->SR &= ~(TIM_SR_UIF); //������� ���� ����������
  NVIC_EnableIRQ(TIM3_IRQn); //��������� ���������� �� �������
  TIM3->CR1 |= TIM_CR1_CEN; //�������� ������
}



//���������� TIM3
//���������� � �������� 100 ��
void TIM3_IRQHandler(void)
{
  static int i = 0;
  TIM3->SR &= ~(TIM_SR_UIF); //������� ���� ����������
  
  for(i=0; i<NUM_SOFT_TIMERS; i++)
    soft_timer[i]++;
}


uint32_t GetTimer(int n)
{
  uint32_t tmp;
  
  if(n>=NUM_SOFT_TIMERS)
    return 0;
  
  NVIC_DisableIRQ(TIM3_IRQn);
  tmp = soft_timer[n];
  NVIC_EnableIRQ(TIM3_IRQn);
  
  return tmp;
}

void ResetTimer(int n)
{
  if(n>=NUM_SOFT_TIMERS)
    return;
  
  NVIC_DisableIRQ(TIM3_IRQn);
  soft_timer[n] = 0;
  NVIC_EnableIRQ(TIM3_IRQn);
}

void SetTimer(uint32_t val, int n)
{
  if(n>=NUM_SOFT_TIMERS)
    return;
  
  NVIC_DisableIRQ(TIM3_IRQn);
  soft_timer[n] = val;
  NVIC_EnableIRQ(TIM3_IRQn);
}

/******************************************************************************/
/********************** ����������� ������� ***********************************/
/******************************************************************************/


void SingleColor(uint16_t dv, uint16_t mv) 
{
  HSV_to_RGB(((mv * 8) / 91), 255, dv/16, &rc, &gc, &bc);
  
  while(!ws2812b_is_ready())
    ;
  for(int i=0; i<WS2812B_NUM_LEDS; i++)
  {
    ws2812b_set(i, rc, gc, bc);
  }
  ws2812b_send();
}


void VGrad(uint16_t dv, uint16_t mv) //
{
  int s_index;
  int v_index;
  
  while(!ws2812b_is_ready())
    ;
  
  for(int i=0; i<10; i++)
  {
    s_index = 255-(255/8*i);
    if(s_index > 255)
      s_index = 255;
    else if(s_index < 0)
      s_index = 0;
    
    
    v_index = (dv)/(16 + i/4);
    
    if(v_index > 255)
      v_index = 255;
    else if(v_index < 0)
      v_index = 0;
    
    HSV_to_RGB(((mv * 8) / 91), (uint8_t)(s_index), v_index, &rc, &gc, &bc);
    
    for(int j=0; j<8; j++)
    {  
      ws2812b_set(j*10+i, rc, gc, bc);
    }
  }
  ws2812b_send();
}


void While(uint16_t dv, uint16_t mv)
{
  int16_t dimmer = dv/28;
  
  TColorToRGB(mv/32, &rc, &gc, &bc);
  
  rc=(rc*dimmer)/255;
  gc=(gc*dimmer)/255;
  bc=(bc*dimmer)/255;
  
  
  
  while(!ws2812b_is_ready())
    ;
  for(int i=0; i<WS2812B_NUM_LEDS; i++)
  {
    ws2812b_set(i, rc, gc, bc);
  }
  
  ws2812b_send();
}

/******************************************************************************/
/********************** ������������ ������� **********************************/
/******************************************************************************/

void DynamicHelixRainbow(uint16_t dv, uint16_t mv)
{
  static int counter = 0;
  
  counter+=((mv/64)+1);
  counter %= (360*4);
  
  uint16_t h_index;
  
  while(!ws2812b_is_ready())
    ;
  
  for(int j=0; j<NUM_COLUM; j++)
  {
    h_index = j*359/NUM_COLUM + (360-(counter/4));
    if(h_index > 359)
      h_index -= 359;
    
    HSV_to_RGB(h_index, 255, dv/16, &rc, &gc, &bc);
    
    for(int i=0; i<NUM_ROW; i++)
    {
      ws2812b_set(i*NUM_COLUM+j, rc, gc, bc);
    }
  }
  ws2812b_send();
  
}


void DynamicSingleColor(uint16_t dv, uint16_t mv)
{
  static int counter = 0;
  counter+=((mv/128) + 1);
  counter %= (360*8);
  
  HSV_to_RGB(counter/8, 255, dv/16, &rc, &gc, &bc);
  
  while(!ws2812b_is_ready())
    ;
  for(int i=0; i<WS2812B_NUM_LEDS; i++)
  {
    ws2812b_set(i, rc, gc, bc);
  }
  ws2812b_send();
  
}


void DynamicHRainbow(uint16_t dv, uint16_t mv)
{
  static int counter = 0;
  
  counter+=((mv/64) + 1);
  counter %= (360*4);
  
  
  uint16_t h_index;
  
  while(!ws2812b_is_ready())
    ;
  
  for(int i=0; i<NUM_ROW; i++)
  {
    h_index = i*359/NUM_ROW + (360 - (counter/4));
    if(h_index > 359)
      h_index -= 359;
    
    HSV_to_RGB(h_index, 255, dv/16, &rc, &gc, &bc);
    
    for(int j=0; j<NUM_COLUM; j++)
    {
      ws2812b_set(j*NUM_ROW+i, rc, gc, bc);
    }
  }
  ws2812b_send();
}


void DynamicVRainbow(uint16_t dv, uint16_t mv)
{
  static int counter = 0;
  
  counter+=((mv/64) + 1);
  counter %= (360*4);
  
  uint16_t h_index;
  
  while(!ws2812b_is_ready())
    ;
  
  for(int j=0; j<NUM_COLUM; j++)
  {
    h_index = j*359/NUM_COLUM + (360 - (counter/4));
    if(h_index > 359)
      h_index -= 359;
    
    HSV_to_RGB(h_index, 255, dv/16, &rc, &gc, &bc);
    
    for(int i=0; i<NUM_ROW; i++)
    {
      ws2812b_set(j*NUM_ROW+i, rc, gc, bc);
    }
  }
  ws2812b_send();
}


/******************************************************************************/

#define TRAIL_LEN       4
typedef struct 
{
  int8_t coord;            //������� ���������� �����
  int8_t vel;              //������������ �������� �����
  int8_t counter;          //�������, ������������ ����������
  uint16_t color;          //���� �����
  uint8_t flag_inprogress; //1-� ��������, 0-���������
  uint16_t post_delay;     //�������� ����� ���������� ����� �����
} drop;

static drop drops[NUM_COLUM];

static void DropsClear(void)
{
  for(int i=0; i<NUM_COLUM; i++)
  {
    drops[i].coord = -1;
    drops[i].vel = 0;
    drops[i].color = 0;
    drops[i].flag_inprogress = 0;
    drops[i].post_delay = 1;
  }
}

void DynamicRain(uint16_t dv, uint16_t mv)
{
  uint32_t rnd;
  
  while(!ws2812b_is_ready())
    ;
  
  for(int i=0; i<NUM_COLUM; i++)
  {
    rnd = (uint32_t)rand();
    
    if(drops[i].flag_inprogress == 0) //���� ��� ��� �����
    {
      //��������� �����
      drops[i].coord = NUM_ROW-1; //�������� ������
      drops[i].color = rnd%360; //������ ����
      drops[i].vel = rnd%10; //������ ������ ��������
      drops[i].counter = 0; //���������� �������
      
      //�������� ����� ���������� ��������� �����
      int16_t rnd_param = (3800-mv)/4;
      if(rnd_param < 0)
        drops[i].post_delay = 0;
      else
        drops[i].post_delay = rnd % rnd_param; 
      
      
      drops[i].flag_inprogress = 1;
      //��� ������� �������
      for(int j=0; j<NUM_ROW; j++)
      {
        ws2812b_set(i*NUM_ROW+j, 0, 0, 0);
      }
      
    }
    else //����� ������� �� ���������
    {
      if(drops[i].coord >= -(TRAIL_LEN+1))
      {
        if(drops[i].counter < drops[i].vel)
        {
          drops[i].counter++;
        }
        else
        {
          drops[i].counter = 0;        
          
          if(drops[i].coord >= 0) //���� ����� �� ����� �� ������� ������
          {
            int j;
            for(j=0; j<drops[i].coord; j++)
            {
              ws2812b_set(i*NUM_ROW+j, 0, 0, 0);
            }
          
            int16_t L = (dv/16);
            
            for(; j<NUM_ROW; j++)
            {
              HSV_to_RGB(drops[i].color, 255, L, &rc, &gc, &bc);
              ws2812b_set(i*NUM_ROW+j, rc, gc, bc);
              L-=((dv/16)/(TRAIL_LEN+1));
              if(L<0)
                L=0;
            }
          }
          else //��� ������ ������ �����
          {
            int j;
            uint8_t trail_residue = TRAIL_LEN+drops[i].coord+1;
            
            for(j=0; j<NUM_ROW-trail_residue; j++)
            {
              ws2812b_set(i*NUM_ROW+j, 0, 0, 0);
            }
            
            int16_t L = (dv/16);
            L -= ((TRAIL_LEN-trail_residue+1)*((dv/16)/(TRAIL_LEN+1)));
            
            for(j=0; j<trail_residue; j++)
            {
              HSV_to_RGB(drops[i].color, 255, L, &rc, &gc, &bc);
              ws2812b_set(i*NUM_ROW+j, rc, gc, bc);
              L-=((dv/16)/(TRAIL_LEN+1));
              if(L<0)
                L=0;
            }
          }
          
          drops[i].coord--;
        }
      }
      else
      {
        if(drops[i].post_delay == 0)
          drops[i].flag_inprogress = 0;
        else
          drops[i].post_delay--;

      }
    }
  }
  
  ws2812b_send();
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

void disp_mode(void)
{
  int i;
  
  while(!ws2812b_is_ready())
    ;
  ws2812b_buff_claer();
  
  
  for(i=0; i<NumMode; i++)
  {
    HSV_to_RGB(i*(300/6), 255, (dimmer_val/16), &rc, &gc, &bc);
    ws2812b_set(i, rc, gc, bc);
  }
  
  ws2812b_send();
  
}

void Display(void)
{
  switch(NumMode)
  {
  case 1:
    SingleColor(dimmer_val, mode_val);
    break;
    
  case 2:
    VGrad(dimmer_val, mode_val);
    break;
    
  case 3:
    While(dimmer_val, mode_val);
    break;
    
  case 4:
    DynamicSingleColor(dimmer_val, mode_val);
    break;
    
  case 5:
    DynamicHRainbow(dimmer_val, mode_val);
    break;
    
  case 6:
    DynamicVRainbow(dimmer_val, mode_val);
    break;
    
  case 7:
    DynamicHelixRainbow(dimmer_val, mode_val);
    break;
    
  case 8:
    DynamicRain(dimmer_val, mode_val);
    break;
    
  default:
    NumMode = 1;
    break;
  }
}

void fsm_disp(void)
{
  static uint8_t state = 0;
  
  switch(state)
  {
  case 0:
    if(GetTimer(TIMER_FPS) >= 2)
    {
      ResetTimer(TIMER_FPS);
      ADC_AnalogRead(&dimmer_val, &mode_val);
      Display();
    }
    
    if(flag_dispmode)
    {
      ResetTimer(TIMER_DISP);
      flag_dispmode = 0;
      state = 1;
    }
    break;
    
  case 1:
    if(GetTimer(TIMER_FPS) >= 2)
    {
      ResetTimer(TIMER_FPS);
      ADC_AnalogRead(&dimmer_val, &mode_val);
      disp_mode();
    }
    
    if(flag_dispmode)
    {
      ResetTimer(TIMER_DISP);
      flag_dispmode = 0;
    }
    
    if(GetTimer(TIMER_DISP) >= 100) //������ 1000��
    {
      while(!ws2812b_is_ready())
        ;
      ws2812b_buff_claer();
      ws2812b_send();
      state = 0;
    }
    break;
  }
}

void IncrMode(void)
{
  NumMode++;
  if(NumMode > 8)
    NumMode = 1;
}

void sfm_main(void)
{
  static uint8_t b_state = 0;
  
  switch(b_state)
  {
  case 0:
    if(ButtonIsPress())
    {
      ResetTimer(TIMER_ANTIBOUNCE);
      b_state = 1;
    }
    break;
    
  case 1:
    if(GetTimer(TIMER_ANTIBOUNCE) >= 5)
    {
      if(ButtonIsPress())
      {
        //������ �� ������
        IncrMode();
        flag_dispmode = 1;
        b_state = 2;
        ResetTimer(TIMER_ANTIBOUNCE);
      }
      else
      {
        b_state = 0;
      }
    }
    break;
    
  case 2:
    if(GetTimer(TIMER_ANTIBOUNCE) >= 10)
    {
      if(!ButtonIsPress())
      {
        b_state = 0;
      }
    }
    break;
  }
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

void main(void)
{
  ADC_init();
  ButtonInit();
  TickTimerInit();
  
  DropsClear();
  
  ws2812b_init();
  while(!ws2812b_is_ready())
    ;
  
  for(;;)
  {
    sfm_main();
    fsm_disp();
  }
  
  
  for(;;)
  {
    ADC_AnalogRead(&dimmer_val, &mode_val);
    switch(mode)
    {
    case 0:
      SingleColor(dimmer_val, mode_val);
      break;
      
    case 1:
      VGrad(dimmer_val, mode_val);
      break;
      
    case 2:
      While(dimmer_val, mode_val);
      break;
      
    case 3:
      HRainbow(dimmer_val, mode_val);
      break;
      
    case 4:
      VRainbow(dimmer_val, mode_val);
      break;
      
    case 5:
      HelixRainbow(dimmer_val, mode_val);
      break;
      
    default:
      mode = 0;
      break;
    }
    
    
    if(is_press == 0)
    {
      if(ButtonIsPress())
      {
        is_press = 1;
        mode++;
      }
    }
    else
    {
      if(!ButtonIsPress())
      {
        is_press = 0;
      }
    }
      
    ResetTimer(0);
    while(!(GetTimer(0) >= 2))
    {
      asm("nop");
    }
    
    /*
    for(int k=0; k<100000; k++)
      asm("nop");
    */
  }
  
#endif
}


