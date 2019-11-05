#include "stm32f30x.h" 
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"


#define xtal 								72000000									// Put here  your Xtal. It's need for delay.
#define WS2812B_TIM_ARR 		0x0059										// Tim period (ARR)
#define WRESET							(WS2812B_TIM_ARR * 45)		// Reset time
#define Low 								0x001C										// "0"
#define High 								0x0039										// "1"
#define LEDS_NUM 						13												// Num of leds in strip
#define COLRS 							3	



// Colors RGB (00000000 RRRRRRRR GGGGGGGG BBBBBBBB)
#define NONE       0x00000000
#define RED        0x00FF0000
#define RED_HALF   0x00800000
#define GREEN      0x0000FF00
#define GREEN_HALF 0x00008000
#define BLUE       0x000000FF
#define WHITE      0x00FFFFFF




uint16_t DMA_buf[LEDS_NUM+2] [COLRS] [8];



void delay_s (uint32_t time);
void delay_ms (uint32_t time);
void delay_us (uint32_t time);
void delay_ns (uint32_t time);

void LEDstrip_init(void);
void LEDstrip_set_led_state(uint16_t ledn, uint32_t color);
void setupDMAch1(void);
void setupPWM(void);
void Clear_DMA(void);
void setuptimer(void);


int main() 
{
	
	LEDstrip_init();
	
	uint16_t i;
		
	
		
	//setupPWM();				// setup PWM
	//setupDMAch1();		// setup DMA1
	setuptimer();
	for (i=0; i<LEDS_NUM; i++)
		{
			LEDstrip_set_led_state(i, RED);
		}
	while(1)
	{
	}
}





/* 																				set LED color
											Format: 0000 0000 RRRR RRRR GGGG GGGG BBBB BBBB */

void LEDstrip_set_led_state(uint16_t ledn, uint32_t color)
{
	uint32_t i;
	if (ledn >= LEDS_NUM) return;
	
	for (i=0; i<8; i++)
	{
		// GREEN ?
		if( ((color>>8) >> (7-i)) & 1 ) DMA_buf[ledn][0][i] = High;
		else DMA_buf[ledn][0][i] = Low;
		
		// RED ?
		if( ((color>>16) >> (7-i)) & 1 ) DMA_buf[ledn][1][i] = High;
		else DMA_buf[ledn][1][i] = Low;
		
		// BLUE?
		if( ((color>>0) >> (7-i)) & 1) DMA_buf[ledn][2][i] = High;
		else DMA_buf[ledn][2][i] = Low;
		
		
	}
}


void LEDstrip_init(void)																// Set all LED's low.
{
	uint32_t i,j,k;
	for (i=0; i < LEDS_NUM; i++)
	{
			for(j=0; j < COLRS; j++)
			{
					for(k=0; k<8; k++)
					{
							DMA_buf[i][j][k] = Low;
					}
			}
	}
	
	
	// RESET (>50 us)
	for(i=LEDS_NUM; i < LEDS_NUM+2; i++)
		{
			for(j=0; j<COLRS; j++)
			{
				for(k=0; k<8; k++)
				{
						DMA_buf[i][j][k] = 0;
				}
			}
		}
}




//////******************************************* INTERRUPTS ****************************************************/////////////////////////






/////********************************************* BASIC FUNCTIONS  ************************************************/////////////////////////// 
void setupDMAch1(void)
{
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_Channel_TypeDef *dma_ch = DMA1_Channel3;				// Select channel 3 DMA
			
	
	dma_ch->CCR = 0;																		// Disable channel
	
	dma_ch->CPAR 	= (unsigned int)&(TIM1->CCR2)+1; 		  // set address of channel 2 TIM1
	dma_ch->CMAR 	= (unsigned int)&DMA_buf;							// set addres of memory 
	dma_ch->CNDTR = (LEDS_NUM + 2) * COLRS * 8; 				// number of data to transfer
	
	
	dma_ch->CCR 	=   DMA_CCR_PL_0 | DMA_CCR_PL_1 			// [13:12] PL - Priority Level = 11 - Very High
									| DMA_CCR_MINC 											// [11:10] Memory Incremetn Mode = 1 - enabled
									| DMA_CCR_CIRC											// [5] Circular Mode = 1 - enabled
									| DMA_CCR_DIR												// [4] Data Transfer Direction = 1 - read from memory
									| DMA_CCR_TCIE											// [1] Transfer Complete Interrupt = 1 - enabled
									|	DMA_CCR_EN;												// [0] Channel enable = 1 - enabled
	
}



void setuptimer(void)
{
	GPIO_InitTypeDef 				gpio;
	TIM_TypeDef *tim = TIM1;
	TIM_OCInitTypeDef    		timoc;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	tim->CR1 = 0;
	tim->CR1 = TIM_CR1_ARPE;					//  TIM1 Auto-reload preload register is buffered
	tim->CR2 = 0;
	tim->PSC = 0;
	tim->ARR = WS2812B_TIM_ARR;
	TIM1->CCMR1 = 0;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2
								 | TIM_CCMR1_OC1PE;	
	
	
}

void setupPWM(void)
{
	/* Use TIM1 chanel 2, pin GPIOA 9 */
	
	GPIO_InitTypeDef 				gpio;
	TIM_TimeBaseInitTypeDef timer;
	TIM_OCInitTypeDef    		timoc;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 
	
	
  gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &gpio);
	
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);

  
  /* Time Base configuration */
  timer.TIM_Prescaler = 0;
  timer.TIM_CounterMode = TIM_CounterMode_Up;
  timer.TIM_Period = 0x0059;
  timer.TIM_ClockDivision = 0;
  timer.TIM_RepetitionCounter = 0;
	
  TIM_TimeBaseInit(TIM1, &timer);
	
	TIM1->CR1 |= (1<<7);										// Enable Auto load-preload (TIM1 ARR register is buffered)
	
	/* Setting CCMR1 register
				OC2M = 0110 - PWMmode 1 in upcounting channel active as long, as TIM1_CNT<TIM1_CCR1
				OC2PE = 1 - Out compare preload
				CC2S = 00 - Second channel is configurated as output
	*/
			
	TIM1->CCMR1 = 0;
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;				
	TIM1->CCMR1 |= TIM_CCMR1_OC2PE;

	/* Capture/Compare 2 DMA request enable */
	
	TIM1->DIER  = 0;
	TIM1->DIER |= TIM_DIER_CC2DE;
	
	
	/* Capture/Compare 2 output enable  */
	TIM1->CCER = 0;
	TIM1->CCER |= TIM_CCER_CC2E;
	//TIM1->CCER |= TIM_CCER_CC2P;

  /* Channel 2 configuration in PWM mode */
  timoc.TIM_OCMode = TIM_OCMode_PWM1;
  timoc.TIM_OutputState = TIM_OutputState_Enable;
  timoc.TIM_Pulse = High;
  timoc.TIM_OCPolarity = TIM_OCPolarity_High;
  timoc.TIM_OCIdleState = TIM_OCIdleState_Set;
  timoc.TIM_OCNIdleState = TIM_OCIdleState_Reset;


  TIM_OC2Init(TIM1, &timoc);

  TIM_Cmd(TIM1, ENABLE);


  TIM_CtrlPWMOutputs(TIM1, ENABLE);


}



void Clear_DMA(void)
{
  DMA_TypeDef *dma = DMA1;
  dma->IFCR = 0x0FFFFFFF;  // clear flags all in DMA ch
}




/////******************************************************* DELAY *************************************************////////////////////////
void delay_s (uint32_t time)
{
	uint32_t i;
	time = time*xtal;
	for (i=0; i<time; i++)
	{}
	
}


void delay_ms (uint32_t time)
{
	uint32_t i;
	time = time*(xtal/1000);
	for (i=0;i<time;i++)
	{}

}


void delay_us (uint32_t time)
{
	uint32_t i;
	
	time = time*(xtal/100000);
	for (i=0;i<time;i++)
	{}
}


void delay_ns (uint32_t time)
{
	uint32_t i;
	
	time = time*(xtal/100000000);
	for (i=0;i<time;i++)
	{}
}
