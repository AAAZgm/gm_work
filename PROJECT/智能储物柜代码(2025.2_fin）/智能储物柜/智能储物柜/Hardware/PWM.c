#include "stm32f10x.h"                  // Device header
void pwm_init(void){
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP ;//注意这里是复用推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;//这里也要改
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  TIM_InternalClockConfig(TIM2);

	

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrycture;
	TIM_TimeBaseInitStrycture.TIM_ClockDivision=TIM_CKD_DIV1;//不分
	TIM_TimeBaseInitStrycture.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStrycture.TIM_Period=20000-1;//这里相当于20ms
	TIM_TimeBaseInitStrycture.TIM_Prescaler=72;//一微秒
	TIM_TimeBaseInitStrycture.TIM_RepetitionCounter=0;//高级定时器才有
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStrycture);

	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//因为有的用不到，不赋值又会出问题，所以用这个初始化
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	//TIM_OCInitStructure.TIM_OCNIdleState=;这里不用
	//TIM_OCInitStructure.TIM_OCNPolarity=;这里不用
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//就是不偏转
	TIM_OCInitStructure.TIM_OutputState=ENABLE;
	TIM_OCInitStructure.TIM_Pulse=0;//意思为脉冲，也就是ccr，占空比
	
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);//可多通道都用
	
	TIM_OC3Init(TIM2,&TIM_OCInitStructure);
	TIM_OC4Init(TIM2,&TIM_OCInitStructure);
  TIM_Cmd(TIM2,ENABLE);
}


void PWM_SetCompare4(float Compare4){
TIM_SetCompare4(TIM2,Compare4);//只改通道2的pluse

}

void PWM_SetCompare2(float Compare2){
TIM_SetCompare2(TIM2,Compare2);//只改通道2的pluse

}

void PWM_SetCompare3(float Compare3){
TIM_SetCompare3(TIM2,Compare3);//只改通道3的pluse

}

