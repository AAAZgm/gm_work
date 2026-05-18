#include "stm32f10x.h"                  // Device header
#include "MyFLASH.h"

#define STORE_START_ADDRESS		0x0800FC00		//存储的起始地址
#define STORE_COUNT				64			//存储数据的个数

uint16_t Store_Data[STORE_COUNT];				//定义SRAM数组，因为ram里面方便

/**
  * 函    数：参数存储模块初始化
  * 参    数：无
  * 返 回 值：无
  */
	void Store_Init(void)//上电时再加载回去
{
	/*判断是不是第一次使用*/
	if (MyFLASH_ReadHalfWord(STORE_START_ADDRESS) != 0x1235)	//读取第一个半字的标志位，if成立，则执行第一次使用的初始化
	{
		MyFLASH_ErasePage(STORE_START_ADDRESS);					//擦除指定页
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS, 0x0426);	//在第一个半字写入自己规定的标志位，用于判断是不是第一次使用
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS+2, 0x0003);
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS+4, 0x0000);
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS+6, 0x0007);
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS+8, 0x0002);
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS+10, 0x0001);
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS+12, 0x0001);
		for (uint16_t i = 7; i < STORE_COUNT; i ++)				//循环STORE_COUNT次，除了第一个标志位
		{
			MyFLASH_ProgramHalfWord(STORE_START_ADDRESS + i * 2, 0x0000);		//除了标志位的有效数据全部清0
		}
	}
	
	/*上电时，将闪存数据加载回SRAM数组，实现SRAM数组的掉电不丢失*/
	for (uint16_t i = 0; i < STORE_COUNT; i ++)					//循环STORE_COUNT次，包括第一个标志位
	{
		Store_Data[i] = MyFLASH_ReadHalfWord(STORE_START_ADDRESS + i * 2);		//将闪存的数据加载回SRAM数组
	}
}

/**
  * 函    数：参数存储模块保存数据到闪存
  * 参    数：无
  * 返 回 值：无
  */
void Store_Save(void)
{
	MyFLASH_ErasePage(STORE_START_ADDRESS);				//擦除指定页
	for (uint16_t i = 0; i < STORE_COUNT; i ++)			//循环STORE_COUNT次，包括第一个标志位
	{
		MyFLASH_ProgramHalfWord(STORE_START_ADDRESS + i * 2, Store_Data[i]);	//将SRAM数组的数据备份保存到闪存
	}
}

/**
  * 函    数：参数存储模块将所有有效数据清0
  * 参    数：无
  * 返 回 值：无
  */
void Store_Clear(void)
{
	for (uint16_t i = 7; i < STORE_COUNT; i ++)			//循环STORE_COUNT次，除了第一个标志位
	{
		Store_Data[i] = 0x0000;							//SRAM数组有效数据清0
	}
	Store_Save();										//保存数据到闪存
}
