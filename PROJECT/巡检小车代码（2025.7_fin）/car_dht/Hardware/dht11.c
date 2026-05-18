#include "dht11.h"
#include "Delay.h"
      
/*****************辰哥单片机设计******************
											STM32
 * 文件			:	DHT11温度湿度传感器c文件                   
 * 版本			: V1.0
 * 日期			: 2024.8.4
 * MCU			:	STM32F103C8T6
 * 接口			:	见dht11.h文件							
 * BILIBILI	:	辰哥单片机设计
 * CSDN			:	辰哥单片机设计
 * 作者			:	辰哥

**********************BEGIN***********************/			
			
			/*MCU 发送起始信号（低电平 > 18ms，高电平 20-40μs）
传感器响应（低电平 80μs，高电平 80μs）
传感器发送 40 位数据（5 字节：湿度整数、湿度小数、温度整数、温度小数、校验和）
校验数据有效性
*/
			
			
//复位DHT11
void DHT11_Rst(void)	   
{                 
	DHT11_Mode(OUT); 	//SET OUTPUT
	DHT11_Low; 	      //拉低DQ
	Delay_ms(20);    	//主机拉低18~30ms
	DHT11_High; 			//DQ=1 
	Delay_us(13);     	//主机拉高10~35us
}

//等待DHT11的回应
//返回1:未检测到DHT11的存在
//返回0:存在
u8 DHT11_Check(void) 	   
{   
	u8 retry=0;
	DHT11_Mode(IN);//SET INPUT	 
    while (GPIO_ReadInputDataBit(DHT11_GPIO_PORT,DHT11_GPIO_PIN)&&retry<100)//DHT11会拉低40~80us
	{
		retry++;
		Delay_us(1);
	};	 
	if(retry>=100)return 1;//未响应
	else retry=0;
    while (!GPIO_ReadInputDataBit(DHT11_GPIO_PORT,DHT11_GPIO_PIN)&&retry<100)//DHT11拉低后会再次拉高40~80us
	{
		retry++;
		Delay_us(1);
	};
	if(retry>=100)return 1;	    
	return 0;
}

//从DHT11读取一个位
/*数据位判断原理：
逻辑 0：高电平约 26-28μs
逻辑 1：高电平约 70μs
通过延时 40μs 后读取电平状态区分 0 和 1*/


//返回值：1/0
u8 DHT11_Read_Bit(void) 			 
{
 	u8 retry=0;
	while(GPIO_ReadInputDataBit(DHT11_GPIO_PORT,DHT11_GPIO_PIN)&&retry<100)//等待变为低电平
	{
		retry++;
		Delay_us(1);
	}
	retry=0;
	while(!GPIO_ReadInputDataBit(DHT11_GPIO_PORT,DHT11_GPIO_PIN)&&retry<100)//等待变高电平
	{
		retry++;
		Delay_us(1);
	}
	Delay_us(40);//等待40us
	if(GPIO_ReadInputDataBit(DHT11_GPIO_PORT,DHT11_GPIO_PIN))return 1;
	else return 0;		   
}

//从DHT11读取一个字节
//返回值：读到的数据
u8 DHT11_Read_Byte(void)    
{        
	u8 i,dat;
	dat=0;
	for (i=0;i<8;i++) 
	{
		dat<<=1; 
		dat|=DHT11_Read_Bit();
	}						    
	return dat;
}

//从DHT11读取一次数据
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败
u8 DHT11_Read_Data(u8 *temp,u8 *humi)    
{        
 	u8 buf[5];
	u8 i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++)//读取40位数据
		{
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])//校验位
		{
			*humi=buf[0];// 湿度整数部分
			*temp=buf[2];// 温度整数部分
		}
	}
	else return 1;
	return 0;	    
}

//初始化DHT11的IO口 DQ 同时检测DHT11的存在
//返回1:不存在
//返回0:存在    	 
u8 DHT11_Init(void)
{	 
 	GPIO_InitTypeDef  GPIO_InitStructure;	
 	RCC_APB2PeriphClockCmd(DHT11_GPIO_CLK, ENABLE);	 //使能PA端口时钟
 	GPIO_InitStructure.GPIO_Pin = DHT11_GPIO_PIN;				 //PG11端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStructure);				 //初始化IO口
 	GPIO_SetBits(DHT11_GPIO_PORT,DHT11_GPIO_PIN);						 //PG11 输出高
			    
	DHT11_Rst();  //复位DHT11
	return DHT11_Check();//等待DHT11的回应
} 

void DHT11_Mode(u8 mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	if(mode)
	{
		GPIO_InitStructure.GPIO_Pin = DHT11_GPIO_PIN;//输出模式使用推挽输出，确保信号驱动能力
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	}
	else
	{
		GPIO_InitStructure.GPIO_Pin =  DHT11_GPIO_PIN;//输入模式使用浮空输入，读取传感器发送的数据
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	}
	GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStructure);
}

