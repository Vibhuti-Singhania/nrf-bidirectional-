#include<stm32f4xx_gpio.h>
#include<stm32f4xx_rcc.h>
#include<stm32f4xx_usart.h>
#include <stm32f4xx_tim.h>
#include<misc.h>
#include<stm32f4xx_spi.h>
#include<tm_stm32f4_gpio.h>
#include<tm_stm32f4_nrf24l01.h>
#include<tm_stm32f4_spi.h>


	uint8_t TxAddress[] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	uint8_t MyAddress[] = {0x7E,0x7E,0x7E,0x7E,0x7E};
	uint8_t dataR[32], dataT[32];
	int j,data1,data2,data3,data4,flag;
	TM_NRF24L01_Transmit_Status_t transmissionStatus;

void InitializeGPIO()
{
	GPIO_InitTypeDef PortE;

	 RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOE , ENABLE);
	  PortE.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1;
	  PortE.GPIO_Mode = GPIO_Mode_OUT;
	  PortE.GPIO_Speed = GPIO_Speed_25MHz;
	  PortE.GPIO_OType = GPIO_OType_PP;
	  PortE.GPIO_PuPd = GPIO_PuPd_NOPULL;

	  GPIO_Init(GPIOE,&PortE);
}

void enableGPIO()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef PORTD;

	PORTD.GPIO_Pin= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15 ;
	PORTD.GPIO_Mode=GPIO_Mode_AF;
	PORTD.GPIO_Speed=GPIO_Speed_50MHz;
	PORTD.GPIO_PuPd=GPIO_PuPd_NOPULL;
	PORTD.GPIO_OType=GPIO_OType_PP;
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);

	GPIO_Init(GPIOD, &PORTD); //FOR PWM

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef PORTD2;

		PORTD2.GPIO_Pin= GPIO_Pin_0 | GPIO_Pin_1;
		PORTD2.GPIO_Mode=GPIO_Mode_OUT;
		PORTD2.GPIO_Speed=GPIO_Speed_50MHz;
		PORTD2.GPIO_PuPd=GPIO_PuPd_NOPULL;
		PORTD2.GPIO_OType=GPIO_OType_PP;

		GPIO_Init(GPIOB, &PORTD2);
		//FOR MOTOR1
		//FOR CW(pin 0 high) or CCW(pin 1 high)

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitTypeDef PORTE2;

		PORTE2.GPIO_Pin= GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_13 |GPIO_Pin_14;
		PORTE2.GPIO_Mode=GPIO_Mode_OUT;
		PORTE2.GPIO_Speed=GPIO_Speed_50MHz;
		PORTE2.GPIO_PuPd=GPIO_PuPd_NOPULL;
		PORTE2.GPIO_OType=GPIO_OType_PP;

		GPIO_Init(GPIOE, &PORTE2);
		//FOR MOTOR 2
		//FOR CW(pin 7 high) or CCW(pin 8 high)

		//FOR MOTOR 3
		//FOR CW(pin 9 high) or CCW(pin 10 high)

		//FOR MOTOR 4
		//FOR CW(pin 11 high) or CCW(pin 12 high)

		//FOR MOTOR 5
		//FOR CW(pin 13 high) or CCW(pin 14 high)

		GPIO_SetBits ( GPIOE , GPIO_Pin_13 );
		GPIO_ResetBits ( GPIOE , GPIO_Pin_14 );
}
void enabletimer4()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		TIM_TimeBaseInitTypeDef Timer4;
		Timer4.TIM_Prescaler=131;
		Timer4.TIM_CounterMode=TIM_CounterMode_Up;
		Timer4.TIM_Period=127;
		Timer4.TIM_ClockDivision=TIM_CKD_DIV1;
		Timer4.TIM_RepetitionCounter = 0;

		TIM_OC1PreloadConfig (TIM4,TIM_OCPreload_Enable);
		TIM_TimeBaseInit(TIM4, &Timer4);
}
/*void enabletimer2()
{
	  TIM_TimeBaseInitTypeDef timer;
				  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
					timer.TIM_Period = 30931;
					timer.TIM_Prescaler =83;
					timer.TIM_ClockDivision = 0;
					timer.TIM_CounterMode = TIM_CounterMode_Up;
					timer.TIM_RepetitionCounter = 0x0000;

					TIM_TimeBaseInit(TIM2,&timer);
					TIM_Cmd(TIM2,ENABLE);

				   NVIC_InitTypeDef interrupt;
					  interrupt.NVIC_IRQChannel=TIM2_IRQn;
					  interrupt.NVIC_IRQChannelPreemptionPriority=0;
					  interrupt.NVIC_IRQChannelSubPriority=1;
					  interrupt.NVIC_IRQChannelCmd=ENABLE;

					   NVIC_Init(&interrupt);

	    TIM_ITConfig (TIM2,TIM_IT_Update,ENABLE);
}
*/
void pwminit()
{
	//PAGE 529 of register description
	TIM_OCInitTypeDef pwm;
	pwm.TIM_OCMode=TIM_OCMode_PWM2;
	pwm.TIM_OCPolarity=TIM_OCPolarity_Low;
	pwm.TIM_OutputState=TIM_OutputState_Enable;
	pwm.TIM_Pulse=0;

	TIM_OC1Init ( TIM4 , &pwm );
	TIM_OC2Init ( TIM4 , &pwm );
	TIM_OC3Init ( TIM4 , &pwm );
	TIM_OC4Init ( TIM4 , &pwm );

	TIM_CCxCmd(TIM4, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxCmd(TIM4, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM4, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxCmd(TIM4, TIM_Channel_4, TIM_CCx_Enable);

	TIM_Cmd(TIM4, ENABLE);

	     TIM_SetCompare1(TIM4 , 0);
		 TIM_SetCompare2(TIM4 , 0);
		 TIM_SetCompare3(TIM4 , 0);
		 TIM_SetCompare4(TIM4 , 0);
}

void Receive()
{
	TM_NRF24L01_Init(20,32)	;
	TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_250k,TM_NRF24L01_OutputPower_0dBm);
	TM_NRF24L01_SetMyAddress(MyAddress);
	TM_NRF24L01_SetTxAddress(TxAddress);
}

void dowhile()
{
	do { transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
       }
	while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
}

void datainit()
{
    for(j=0;j<32;j++)
	{
		dataT[j]=04;
	}
}
int main(void)
{
	InitializeGPIO();
	enableGPIO();
//	 enabletimer2();
	enabletimer4();
	pwminit();
	Receive();
	datainit();
    while(1)
    {
        	if (TM_NRF24L01_DataReady())
    	{
    	            TM_NRF24L01_GetData(dataR);
    	            if(dataR[0] == 02)
                   {
    	            GPIO_SetBits(GPIOE , GPIO_Pin_1);
    	            		GPIO_SetBits ( GPIOB , GPIO_Pin_0 );
    	            		GPIO_ResetBits ( GPIOB , GPIO_Pin_1 );
    	            		TIM_SetCompare1(TIM4 , data1=(int)dataR[1]);

    	            		GPIO_SetBits ( GPIOE , GPIO_Pin_7 );
    	            		GPIO_ResetBits ( GPIOE , GPIO_Pin_8 );
    	            		TIM_SetCompare2(TIM4 , data2=(int)dataR[2]);

    	            		GPIO_SetBits ( GPIOE , GPIO_Pin_9 );
    	            		GPIO_ResetBits ( GPIOE , GPIO_Pin_10);
    	            		TIM_SetCompare3(TIM4 , data3=(int)dataR[3]);

    	            		GPIO_SetBits ( GPIOE , GPIO_Pin_11 );
    	            		GPIO_ResetBits ( GPIOE , GPIO_Pin_12 );
    	            		TIM_SetCompare4(TIM4 , data4=(int)dataR[4]);
                   }
//    	            if(flag==1){
//    	            	flag=0;
    	            TM_NRF24L01_Transmit(dataT);          //Automatically sets to transmit mode
    	            dowhile();
    	            GPIO_SetBits(GPIOE , GPIO_Pin_0);
    	            TM_NRF24L01_PowerUpRx();               //Sets back to receive mode
//    	            }
    	}

   }
}
/*
void TIM2_IRQHandler()
{

	  if (TIM_GetITStatus(TIM2, TIM_IT_Update)!= RESET)
	    	  {
	    	    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	    	    flag=1;
	    	  }

}

/*First calculate timer tick frequency :
Down timer 1mhz with prescaler

timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)
 1000000 = 84000000 / (prescaller_set + 1)
 prescaller = 83
Second calculate timer period:
Down timer 0.033khz with timer period (30ms)               / 0.0167kHz(60ms)

tim_frequency = timer_tick_frequency / (TIM_Period + 1)
 33.33 = 1000000 /(TIM_Period +1)
 TIM_Period = (1000000 / 33.33) - 1 
 TIM_Period = 30931                                        /  59987(60ms)                                     */ 
