#include "key.h"
uint8_t key_getnum(void)
{
	uint8_t KeyNum = 0;		//定义变量，默认键码值为0
	
	if (HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)==key_open)			//读PB1输入寄存器的状态，如果为0，则代表按键1按下
	{
		HAL_Delay(20);											//延时消抖
		while (HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin)==key_open);	//等待按键松手
		HAL_Delay(20);//延时消抖
		KeyNum = 1;												//置键码为1
	}
	else if (HAL_GPIO_ReadPin(key2_GPIO_Port,key2_Pin)==key_open)			//读PB1输入寄存器的状态，如果为0，则代表按键1按下
	{
		HAL_Delay(20);											//延时消抖
		while (HAL_GPIO_ReadPin(key2_GPIO_Port,key2_Pin)==key_open);	//等待按键松手
		HAL_Delay(20);//延时消抖
		KeyNum = 2;												//置键码为2
	}
//	 else if (HAL_GPIO_ReadPin(control_key3_GPIO_Port,control_key3_Pin)==key3_open)			//读PB1输入寄存器的状态，如果为0，则代表按键1按下
//	{
//		KeyNum = 3;											
//	}
//	
	return KeyNum;			//返回键码值，如果没有按键按下，所有if都不成立，则键码为默认值0
}
