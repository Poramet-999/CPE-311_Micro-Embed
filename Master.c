/*
	Template project for CPE-214 STM32L1 Discovery Laboratory
	Author : Goragod
	Affiliation : Computer Engineering Dept., Factulty of Eng., 
	              Thai-Nichi Institute of Technology Thailand
	Version : 1
	Link : 
*/
/*Base register definition*/
#include "stm32l1xx.h"

#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
#include "stm32l1xx_ll_adc.h"
#include "stm32l1xx_ll_dac.h"
#include "stm32l1xx_ll_tim.h"

#include "stdio.h"

void SystemClock_Config(void);
void TIMx_IC_Config(void);

char disp_str[7];
uint16_t uwIC1 = 0;
uint16_t uwIC2 = 0;
uint16_t uwDiff = 0;
uint16_t uhICIndex = 0;
float period = 0;
int distance = 0;
int openthedoor = 0;
int score = 0;

uint32_t TIM4CLK;
uint32_t PSC;
uint32_t IC1PSC;

int main()
{
	uint8_t usr_button;
	uint8_t PC12;
	uint8_t PA11;
	uint8_t PA12;
	//uint16_t adc_data = 0;
	uint16_t LED = 0x000;
	
	TIMx_IC_Config();
	
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	//LL_ADC_InitTypeDef  ADC_InitStruct;
	//LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	
	SystemClock_Config();
	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	
	LCD_GLASS_Init();
	
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	//LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
	//LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
	GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
//	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_10B;
//	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
//	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_1RANK;
//	LL_ADC_Init(ADC1, &ADC_InitStruct);
//	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
//	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_48CYCLES);//ADC1->SMPR3 |= (4<<15);
//	ADC1->SQR5 |= (5<<0);
//	ADC1->CR2 |= (1<<0);
	/*ADC1->CR1 |= (1<<11);
	ADC1->CR1 &= ~((7<<13)|(1<<24)|(1<<25));
	ADC1->CR2 &= ~(1<<11);
	ADC1->SMPR3 |= (2<<12);
	ADC1->SQR5 |= (5<<0);
	ADC1->CR2 |= (1<<0);
	
	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
	DAC->DHR12R1 = LED;*/
	
	//0 : Wall , 1 : passage , 2 : door , 3 : spawn point , 4 : goal, 5 : point
	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
	DAC->DHR12R1 = 0x0000;
	uint8_t maze[12][12] = {{0,0,0,0,0,0,0,0,0,0,0,0}, //0
													{0,1,1,1,1,1,0,5,1,1,1,0}, //1
													{0,1,0,1,0,2,0,0,1,0,1,0}, //2
													{0,1,0,5,0,1,0,1,1,0,1,0}, //3
													{0,1,0,1,0,1,0,0,0,0,1,0}, //4
													{0,1,0,0,0,1,1,0,1,1,1,0}, //5
													{0,1,1,1,0,1,4,0,2,0,5,0}, //6
													{0,5,0,1,0,0,0,0,1,0,0,0}, //7
													{0,0,0,1,1,1,1,1,1,2,1,0}, //8
													{0,5,0,0,0,0,0,0,0,0,1,0}, //9
													{0,1,1,1,1,1,1,1,1,1,1,0}, //A
													{0,0,0,0,0,0,0,0,0,0,0,0}};//B
												// 0 1 2 3 4 5 6 7 8 9 A B
	int p_po[2] = {10,5};//p_po[0]=y,p_po[1]=x start = 10,5 goal = 5,5
	//uint8_t goal[2] = {6,6};
	uint8_t view = 0; //0=^, 1=<, 2=V, 3=> 
	
	while(1)
	{
		if(view == 0||view == 2)
		{
			int k=0;
			int i;
			if(p_po[1]>=2&&p_po[1]<=8)
				{i=p_po[1]-2;}
			else if(p_po[1]==1)
				{i=p_po[1]-1;}
			else if(p_po[1]==9)
				{i=p_po[1]-3;}
			else if(p_po[1]==10)
				{i=p_po[1]-4;}
			while(i <= (p_po[1]-1)+6)
				{//0=wall, ' '=passage, ^=player, >=player/wall, <=Unmovewall, := door, ;=player/door, *=goal,@=player/goal,+=point,-=piont/wall
				if(maze[p_po[0]][i]==0)
					{disp_str[k]='<';}
				else if(maze[p_po[0]][i]==2)
					{disp_str[k]=':';}
				else if(maze[p_po[0]][i]==4)
					{disp_str[k]='*';}
				else if(maze[p_po[0]][i]==5)
					{disp_str[k]='+';}
				else if(maze[p_po[0]-1+view][i]==0)
					{disp_str[k]='0';}
				else if(maze[p_po[0]-1+view][i]==1)
					{disp_str[k]=' ';}
				else if(maze[p_po[0]-1+view][i]==2)
					{disp_str[k]=':';}
				else if(maze[p_po[0]-1+view][i]==4)
					{disp_str[k]='*';}
				else if(maze[p_po[0]-1+view][i]==5)
					{disp_str[k]='+';}
				if(i==p_po[1]&&maze[p_po[0]-1+view][i]==1)
					{disp_str[k]='^';}
				else if(i==p_po[1]&&maze[p_po[0]-1+view][i]==0)
					{disp_str[k]='>';}
				else if(i==p_po[1]&&maze[p_po[0]-1+view][i]==2)
					{disp_str[k]=';';}
				else if(i==p_po[1]&&maze[p_po[0]-1+view][i]==4)
					{disp_str[k]='@';}
				else if(i==p_po[1]&&maze[p_po[0]-1+view][i]==5)
					{disp_str[k]='-';}
				++k;
				++i;
			}
		}
		else if(view == 1||view == 3)
		{
			int k=0;
			int i;
			if(p_po[0]>=2&&p_po[0]<=8)
				{i=p_po[0]-2;}
			else if(p_po[0]==1)
				{i=p_po[0]-1;}
			else if(p_po[0]==9)
				{i=p_po[0]-3;}
			else if(p_po[0]==10)
				{i=p_po[0]-4;}
			while(i <= (p_po[0]-1)+6)
				{//0=wall, ' '=passage, ^=player, >=player/wall, <=Unmovewall, := door, ;=player/door, *=goal,@=player/goal,+=point,-=piont/player
				if(maze[i][p_po[1]]==0)
					{disp_str[k]='<';}
				else if(maze[i][p_po[1]]==2)
					{disp_str[k]=':';}
				else if(maze[i][p_po[1]]==4)
					{disp_str[k]='*';}
				else if(maze[i][p_po[1]]==5)
					{disp_str[k]='+';}
				else if(maze[i][p_po[1]-2+view]==0)
					{disp_str[k]='0';}
				else if(maze[i][p_po[1]-2+view]==1)
					{disp_str[k]=' ';}
				else if(maze[i][p_po[1]-2+view]==2)
					{disp_str[k]=':';}
				else if(maze[i][p_po[1]-2+view]==4)
					{disp_str[k]='*';}
				else if(maze[i][p_po[1]-2+view]==5)
					{disp_str[k]='+';}
				if(i==p_po[0]&&maze[i][p_po[1]-2+view]==1)
					{disp_str[k]='^';}
				else if(i==p_po[0]&&maze[i][p_po[1]-2+view]==0)
					{disp_str[k]='>';}
				else if(i==p_po[0]&&maze[i][p_po[1]-2+view]==2)
					{disp_str[k]=';';}
				else if(i==p_po[0]&&maze[i][p_po[1]-2+view]==4)
					{disp_str[k]='@';}
				else if(i==p_po[0]&&maze[i][p_po[1]-2+view]==5)
					{disp_str[k]='-';}
				++k;
				++i;
				}
			}
		//sprintf(disp_str,"%s","010000");
		LCD_GLASS_DisplayString((uint8_t*)disp_str);
		for(int d=0;d<1000000;d++)
		{}
		PA11 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_11);
		if(PA11)
		{
			if((view==0||view==2)&&((maze[p_po[0]][p_po[1]+1])==1||(maze[p_po[0]][p_po[1]+1])==5))
			{
				if((maze[p_po[0]][p_po[1]+1])==5)
				{maze[p_po[0]][p_po[1]+1]=1;score+=0x0199;
				DAC->DHR12R1 = score;}//max score 1000 | 500 point | 500 win
				p_po[1] += 1;
			}
			else if((view==1||view==3)&&((maze[p_po[0]+1][p_po[1]])==1||(maze[p_po[0]+1][p_po[1]])==5))
			{
				if((maze[p_po[0]+1][p_po[1]])==5)
				{maze[p_po[0]+1][p_po[1]]=1;score+=0x0199;
				DAC->DHR12R1 = score;}
				p_po[0] += 1;
			}
		}
		PA12 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_12);
		if(PA12)
		{
			if((view==0||view==2)&&((maze[p_po[0]][p_po[1]-1])==1||(maze[p_po[0]][p_po[1]-1])==5))
			{
				if((maze[p_po[0]][p_po[1]-1])==5)
				{maze[p_po[0]][p_po[1]-1]=1;score+=0x0199;
				DAC->DHR12R1 = score;}
				p_po[1] -= 1;
			}
			else if((view==1||view==3)&&((maze[p_po[0]-1][p_po[1]])==1||(maze[p_po[0]-1][p_po[1]])==5))
			{
				if((maze[p_po[0]-1][p_po[1]])==5)
				{maze[p_po[0]-1][p_po[1]]=1;score+=0x0199;
				DAC->DHR12R1 = score;}
				p_po[0] -= 1;
			}
		}
		usr_button = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
		if(usr_button)
		{
			++view;
			view = view%4;
		}
		PC12 = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_12);
		if(PC12 && openthedoor == 0) // ult
		{
//			LL_ADC_REG_StartConversionSWStart(ADC1);
//			while(LL_ADC_REG_GetFlagEndOfConversion(ADC1));
//			adc_data = LL_ADC_REG_ReadConversionData10(ADC1);
			/*ADC1->CR2 |= (1<<30);
			//while((ADC1->SR & (1<<1)) == 0);
			adc_data = ADC1->DR;*/
			LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_2);
			LL_mDelay(1);
			LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_2);
			openthedoor = 1;
		}
		if(uhICIndex == 2)
			{
					//Period calculation
					PSC = LL_TIM_GetPrescaler(TIM4) + 1;
					TIM4CLK = SystemCoreClock / PSC;
					IC1PSC = __LL_TIM_GET_ICPSC_RATIO(LL_TIM_IC_GetPrescaler(TIM4, LL_TIM_CHANNEL_CH1));
					
					period = (uwDiff*(PSC) * 1.0) / (TIM4CLK *IC1PSC * 1.0); //calculate uptime period
				
					distance = (period * 340) /2 * 100;
					
					uhICIndex = 0;

			}
			if(distance>5&&p_po[0]==8&&p_po[1]==10&&openthedoor == 1)
			{
				if(view==0&&(maze[p_po[0]-1][p_po[1]])==2)
				{maze[p_po[0]-1][p_po[1]] = 1;}
				else if(view==2&&(maze[p_po[0]+1][p_po[1]])==2)
				{maze[p_po[0]+1][p_po[1]] = 1;}
				else if(view==1&&(maze[p_po[0]][p_po[1]-1])==2)
				{maze[p_po[0]][p_po[1]-1] = 1;}
				else if(view==2&&(maze[p_po[0]][p_po[1]+1])==2)
				{maze[p_po[0]][p_po[1]+1] = 1;}
			}
			if(distance<5&&p_po[0]==1&&p_po[1]==5&&openthedoor == 1)
			{
				if(view==0&&(maze[p_po[0]-1][p_po[1]])==2)
				{maze[p_po[0]-1][p_po[1]] = 1;}
				else if(view==2&&(maze[p_po[0]+1][p_po[1]])==2)
				{maze[p_po[0]+1][p_po[1]] = 1;}
				else if(view==1&&(maze[p_po[0]][p_po[1]-1])==2)
				{maze[p_po[0]][p_po[1]-1] = 1;}
				else if(view==2&&(maze[p_po[0]][p_po[1]+1])==2)
				{maze[p_po[0]][p_po[1]+1] = 1;}
			}
			if(distance<5&&p_po[0]==7&&p_po[1]==8&&openthedoor == 1)
			{
				if(view==0&&(maze[p_po[0]-1][p_po[1]])==2)
				{maze[p_po[0]-1][p_po[1]] = 1;}
				else if(view==2&&(maze[p_po[0]+1][p_po[1]])==2)
				{maze[p_po[0]+1][p_po[1]] = 1;}
				else if(view==1&&(maze[p_po[0]][p_po[1]-1])==2)
				{maze[p_po[0]][p_po[1]-1] = 1;}
				else if(view==2&&(maze[p_po[0]][p_po[1]+1])==2)
				{maze[p_po[0]][p_po[1]+1] = 1;}
			}
		if((p_po[0]==5 && p_po[1]==6) || (p_po[0]==6 && p_po[1]==5))
		{
			//LED = 0x0BBB;
			sprintf(disp_str,"%s","U Win ");
			LCD_GLASS_DisplayString((uint8_t*)disp_str);
			score+=500;
			DAC->DHR12R1 = score;
			while(1)
			{
				//DAC->DHR12R1 = LED;
//				if(LED<0x0FFF){
//				while(1)
//				{
//					//DAC->DHR12R1 = LED;
//					for(int i=0;i<50000;++i){DAC->DHR12R1 = LED;}
//					LED = LED + 0x0005;
//					if(LED>0x0FFF){LED = 0x0FFF;DAC->DHR12R1 = LED;break;}
//				}}
//				else if(LED>0x0BBB){
//				while(1)
//				{
//					//DAC->DHR12R1 = LED;
//					for(int i=0;i<50000;++i){DAC->DHR12R1 = LED;}
//					LED = LED - 0x0005;
//					if(LED<0x0BBB){LED = 0x0BBB;DAC->DHR12R1 = LED;break;}
//				}}
			}
		}
	}
}

void TIMx_IC_Config(void)
{
		LL_TIM_IC_InitTypeDef timic;
	
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
		
		//TIM_IC Configure CH1
		timic.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
		timic.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;
		timic.ICPolarity = LL_TIM_IC_POLARITY_RISING;
		timic.ICPrescaler = LL_TIM_ICPSC_DIV1;
		LL_TIM_IC_Init(TIM4, LL_TIM_CHANNEL_CH1, &timic);
	
		timic.ICPolarity = LL_TIM_IC_POLARITY_FALLING;
		LL_TIM_IC_Init(TIM4, LL_TIM_CHANNEL_CH2, &timic);
		
		NVIC_SetPriority(TIM4_IRQn, 0);
		
		NVIC_EnableIRQ(TIM4_IRQn);
		LL_TIM_EnableIT_CC1(TIM4);
		LL_TIM_EnableIT_CC2(TIM4);
		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
		
		LL_TIM_EnableCounter(TIM4);
}

void TIM4_IRQHandler(void)
{	  
		if(LL_TIM_IsActiveFlag_CC1(TIM4) == SET &&uhICIndex == 0)
		{
			LL_TIM_ClearFlag_CC1(TIM4);
				//Detect 1st rising edge
		
				uwIC1 = LL_TIM_IC_GetCaptureCH1(TIM4);
				uhICIndex = 1;
		}
			//Detect 2nd rising edge
		if(LL_TIM_IsActiveFlag_CC2(TIM4) == SET && uhICIndex == 1)
			{
				LL_TIM_ClearFlag_CC2(TIM4);
					uwIC2 = LL_TIM_IC_GetCaptureCH2(TIM4);
					
					if(uwIC2 > uwIC1)
						uwDiff = uwIC2 - uwIC1;
					else if(uwIC2 < uwIC1)
						uwDiff = ((LL_TIM_GetAutoReload(TIM4) - uwIC1) + uwIC2) + 1;
					uhICIndex = 2;
					openthedoor = 0;
			}

}

/*
@Func : SystemClock_Configure
@Param : none
@Ret : none
@Brief : Connfigure MCU for maximum performance by
					1. Reroute system clock to HSI-PLL (32MHz) instead of MSI (16MHz)
					2. Configure PLL parameter for 32MHz clock generation
					Procedure of doing is described in code, uncomment and call this function
					for god performance!
*/

/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}
