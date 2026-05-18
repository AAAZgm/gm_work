#include "stm32f10x.h"                  // Device header
#include "PWM.H"

void motor_init(void){

pwm_init();
/*
	GPIO_InitTypeDef Structure;
	Structure.GPIO_Mode=GPIO_Mode_Out_PP;//PP是推挽
	Structure.GPIO_Pin=GPIO_Pin_11|GPIO_Pin_12;
	Structure.GPIO_Speed=GPIO_Speed_50MHz;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//使用各个外设前必须开启时钟，否则对外设的操作无效
	GPIO_Init(GPIOA,&Structure);//函数内部会自动根据结构体的参数配置相应寄存器
															//实现GPIOA的初始化
	//GPIO_SetBits
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);//给0，GPIOA的0
		GPIO_SetBits(GPIOA,GPIO_Pin_12);//给0，GPIOA的0
	*/

	
}

void motor_Setv(float v){
PWM_SetCompare3((v*20000)/100);
}
