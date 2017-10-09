#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_usart.h>
#include <tm_stm32f4_gpio.h>
#include <tm_stm32f4_spi.h>
#include <tm_stm32f4_nrf24l01.h>
#include <misc.h>

int flag = 0;
int k;
int flag_tim2=0;
int flag_tim4=0;

uint8_t MyAddress[] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t TxAddress[] = {0x7E,0x7E,0x7E,0x7E,0x7E};
uint8_t dataOut[32],dataIn[32];

TM_NRF24L01_Transmit_Status_t transmissionStatus;

void USART1_GPIO_Init();
void USART1_Initialize();
void NVIC_Initialize();
void Enable_USART1();
void nRF_Init();
void waitforsend();
void LEDinit();
void datainit();
void enabletimer2();
void enabletimer4();

void USART1_IRQHandler(){
    static int i = 0;                                      /* usart1 for printing data received by
                                                              the central bot from other bots*/
    if(USART_GetITStatus  (USART1,USART_IT_TXE)==SET){
        USART_ClearITPendingBit (USART1,USART_IT_TXE);
        USART_SendData (USART1,dataIn[i]);
        i++;
        if(i==32){
            i = 0;
            flag = 1;
        }
    }

}
void TIM2_IRQHandler()
{

	  if (TIM_GetITStatus(TIM2, TIM_IT_Update)!= RESET)          /*timer2 for receiving data in every 30ms*/
	    	  {
	    	    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	    	    flag_tim2=1;

	    	  }

}
void TIM4_IRQHandler()
{

	  if (TIM_GetITStatus(TIM4, TIM_IT_Update)!= RESET)           /*timer4 for receiving data in every 60ms*/
	    	  {
	    	    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	    	    flag_tim4=1;

	    	  }

}


int main(void)
{
    int j;

    USART1_GPIO_Init();
    USART1_Initialize();
    NVIC_Initialize();
    Enable_USART1();
    LEDinit();
   // enabletimer2();
    //enabletimer4();
    nRF_Init();
    datainit();

    while(1)
    {
    	    TM_NRF24L01_Transmit(dataOut);
            waitforsend();
            GPIO_SetBits(GPIOE,GPIO_Pin_0);
            TM_NRF24L01_PowerUpRx(); //Sets to receive mode.

                		for(j=0;j<100000;j++);                 //delay

                     	 if (TM_NRF24L01_DataReady())
                     	 {
                     		 GPIO_SetBits(GPIOE , GPIO_Pin_1);
                     		 TM_NRF24L01_GetData(dataIn);
                     		 if((dataIn[0]==03) && (flag_tim4=1))
                     		 {
                     			 	 flag_tim4=0;
                     			 	 if(flag==1)                      //interrupt for sendind data through usart after receiving from nrf
                     			 	 {
                     			 		 flag=0;
                     			 	     GPIO_SetBits(GPIOE , GPIO_Pin_1);
                     			 	 }
                     		 }
                     		 else if((dataIn[0]==04) && (flag_tim2=1))
                     		 {
                     				 flag_tim2=0;
									 if(flag==1)                      //interrupt for sendind data through usart after receiving from nrf
									 {
										 flag=0;
									     GPIO_SetBits(GPIOE , GPIO_Pin_1);
									 }
                     		 }

                     	   }
      }
}
void USART1_GPIO_Init(){

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //TX
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //RX

    GPIO_InitTypeDef gpio;
        gpio.GPIO_Mode = GPIO_Mode_AF;
        gpio.GPIO_OType = GPIO_OType_PP;
        gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        gpio.GPIO_PuPd = GPIO_PuPd_UP;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_Init(GPIOB, &gpio);
}
void USART1_Initialize(){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        USART_InitTypeDef usart1;
        usart1.USART_BaudRate = 9600;
        usart1.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        usart1.USART_Parity = USART_Parity_No;
        usart1.USART_StopBits = USART_StopBits_1;
        usart1.USART_WordLength = USART_WordLength_8b;
        usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

        USART_Init(USART1, &usart1);
}
void NVIC_Initialize(){

     NVIC_InitTypeDef interrupt;
     interrupt.NVIC_IRQChannel = USART1_IRQn; //In file stm32f4xx.h, line 140
     interrupt.NVIC_IRQChannelPreemptionPriority = 0;
     interrupt.NVIC_IRQChannelSubPriority = 0;
     interrupt.NVIC_IRQChannelCmd = ENABLE;

     NVIC_Init (&interrupt);

//     USART_ITConfig (USART1, USART_IT_RXNE, ENABLE);
     USART_ITConfig (USART1, USART_IT_TXE, ENABLE);
}
void Enable_USART1(){
    USART_Cmd (USART1,ENABLE);
}
void nRF_Init(){
     TM_NRF24L01_Init(20, 32);
     TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_250k, TM_NRF24L01_OutputPower_0dBm);
     TM_NRF24L01_SetMyAddress(MyAddress);
     TM_NRF24L01_SetTxAddress(TxAddress);
}
void waitforsend(){
    do {
        transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
    } while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
}
void LEDinit(){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
        GPIO_InitTypeDef led;

        led.GPIO_Mode = GPIO_Mode_OUT;
        led.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
        led.GPIO_PuPd = GPIO_PuPd_DOWN;
        led.GPIO_OType = GPIO_OType_PP;
        led.GPIO_Speed = GPIO_Speed_50MHz;

        GPIO_Init ( GPIOE , &led );
}
void datainit(){

		dataOut[0]=02;     // this is bot id
		for(k=1;k<5;k++)
		{
			dataOut[k]=25;    //25 is pwm dutycycle
		}
		for(k=5;k<32;k++)
		{
			dataOut[k]=1;
		}
		for(k=0;k<32;k++)
		{
			dataIn[k]=0;
		}
}
void enabletimer2()
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
void enabletimer4()
{
	  TIM_TimeBaseInitTypeDef timer;
				  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
					timer.TIM_Period = 59987;
					timer.TIM_Prescaler =83;
					timer.TIM_ClockDivision = 0;
					timer.TIM_CounterMode = TIM_CounterMode_Up;
					timer.TIM_RepetitionCounter = 0x0000;

					TIM_TimeBaseInit(TIM4,&timer);
					TIM_Cmd(TIM4,ENABLE);

				   NVIC_InitTypeDef interrupt;
					  interrupt.NVIC_IRQChannel=TIM4_IRQn;
					  interrupt.NVIC_IRQChannelPreemptionPriority=0;
					  interrupt.NVIC_IRQChannelSubPriority=1;
					  interrupt.NVIC_IRQChannelCmd=ENABLE;

					   NVIC_Init(&interrupt);

	    TIM_ITConfig (TIM4,TIM_IT_Update,ENABLE);
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

