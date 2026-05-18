#include "stm32f10x.h"                  // Device header
#include "touch.h"
#include "OLED.h"
#include "PWM.h"
#include "motor.h"
#include "servo.h"
#include "Delay.h"
#include "dht11.h"
#include "Store.h"
#include "MyFLASH.h"
#include "Serial.h"
#include "MPU6050.h"
#include "buzz.h"
#include "Delay.h"
#include <stdlib.h>
#define STORE_START_ADDRESS		0x0800FC00	
#define SERVO_DELAY_COEFF 460 // 舵机移动延迟系数
 uint8_t chance;
uint8_t KeyNum;
uint8_t boxflag;
extern uint16_t Store_Data[];
uint8_t realcode[4];
/***************************************************************************************/
uint8_t flag=1;/*把一级菜单的flag放到全局变量,静态存储,不会丢失数据,所以从二级菜单回来以后
仍能回到那一行,而不是每次都回到第一行*/
/***************************************************************************************/

/*-----------------------------------------一级菜单----------------------------------------------*/
/**
  * @brief 一级菜单函数,用于显示一级菜单八行选项
  * @param  无
  * @retval 返回当前行是第几行
  */
int menu1(void)
{ 
	/*初始显示*/
	OLED_Clear();
	OLED_ShowString(0,0, "My box            ",OLED_8X16);
	OLED_ShowString(0,16,"蓝牙模式            ",OLED_8X16);
	OLED_ShowString(0,32,"声控模式           ",OLED_8X16);
	OLED_ShowString(0,48,"通风调控            ",OLED_8X16);
	OLED_Update();
	while(1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum == 1)//上一项
		{
		flag = (flag == 1) ? 7 : flag - 1; // 防止flag=0
		}
		if(KeyNum == 2)//下一项
		{
			 flag = (flag == 7) ? 1 : flag + 1; // 防止flag=8
		}
		if(KeyNum == 3)//确认
		{ 
			OLED_Clear();
	
			return flag;
		}
		
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)==0)  servo_Setangle2(2500);
	
		switch(flag)
		{
			case 1:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
								OLED_Clear();
				OLED_ShowString(0,0, "My box                ",OLED_8X16);
				OLED_ShowString(0,16,"蓝牙模式               ",OLED_8X16);
				OLED_ShowString(0,32,"声控模式              ",OLED_8X16);
				OLED_ShowString(0,48,"通风调控               ",OLED_8X16);
				OLED_ReverseArea(0,16,128,16);
				OLED_Update();
				break;
			}
			case 2:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
					OLED_Clear();
				OLED_ShowString(0,0, "My box                ",OLED_8X16);
				OLED_ShowString(0,16,"蓝牙模式               ",OLED_8X16);
				OLED_ShowString(0,32,"声控模式              ",OLED_8X16);
				OLED_ShowString(0,48,"通风调控               ",OLED_8X16);
				OLED_ReverseArea(0,32,128,16);
				OLED_Update();
				break;
				
			}
			case 3:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				OLED_ShowString(0,0, "My box                ",OLED_8X16);
				OLED_ShowString(0,16,"蓝牙模式               ",OLED_8X16);
				OLED_ShowString(0,32,"声控模式              ",OLED_8X16);
				OLED_ShowString(0,48,"通风调控               ",OLED_8X16);
				OLED_ReverseArea(0,48,128,16);
				OLED_Update();
				break;
			}
			case 4:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				OLED_ShowString(0,0, "修改密码             ",OLED_8X16);
				OLED_ShowString(0,16,"打开箱子             ",OLED_8X16);
				OLED_ShowString(0,32,"读取温湿度            ",OLED_8X16);
				OLED_ShowString(0,48,"关箱             ",OLED_8X16);
				OLED_ReverseArea(0,0,128,16);
				OLED_Update();
				break;
			}
			case 5:
			{
					//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				OLED_ShowString(0,0, "修改密码             ",OLED_8X16);
				OLED_ShowString(0,16,"打开箱子             ",OLED_8X16);
				OLED_ShowString(0,32,"读取温湿度            ",OLED_8X16);
				OLED_ShowString(0,48,"关箱             ",OLED_8X16);
				OLED_ReverseArea(0,16,128,16);
				
				OLED_Update();
				break;
			}
			
			case 6:
			{OLED_Clear();
				//	//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_ShowString(0,0, "修改密码             ",OLED_8X16);
				OLED_ShowString(0,16,"打开箱子             ",OLED_8X16);
				OLED_ShowString(0,32,"读取温湿度            ",OLED_8X16);
				OLED_ShowString(0,48,"关箱              ",OLED_8X16);
				OLED_ReverseArea(0,32,128,16);
				
				OLED_Update();
				break;
		
			}
			case 7:
			{	//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				OLED_ShowString(0,0, "修改密码             ",OLED_8X16);
				OLED_ShowString(0,16,"打开箱子             ",OLED_8X16);
				OLED_ShowString(0,32,"读取温湿度            ",OLED_8X16);
				OLED_ShowString(0,48,"关箱               ",OLED_8X16);
				OLED_ReverseArea(0,48,128,16);
				
				OLED_Update();
				break;
			}
			
		
		}
	}
}


int menu2_read(void)
{
	uint8_t humi=0,temp=0;
	DHT11_Read_Data(&temp,&humi);

	/*初始显示*/
	OLED_Clear();
	 OLED_ShowString(0,0,"<-               ",OLED_8X16);
	OLED_Printf(0,16,OLED_8X16,"当前温度:%d   ",temp);
	OLED_Printf(0,32,OLED_8X16,"当前湿度:%d   ",humi);
	OLED_Update();
	Delay_s(1);
	while(1)
	{  DHT11_Read_Data(&temp,&humi);
		Delay_ms(200);
		KeyNum = Key_GetNum();
		
		if(KeyNum == 3)//确认
		{
			
				OLED_Clear();
		return flag;
	
		}
		
	//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
								OLED_Clear();
				 OLED_ShowString(0,0,"<-               ",OLED_8X16);
	OLED_Printf(0,16,OLED_8X16,"当前温度:%d   ",temp);
	OLED_Printf(0,32,OLED_8X16,"当前湿度:%d   ",humi);
				OLED_Update();
				
			
			
		
		}
	}


int menu2_listen(void)
{
	uint8_t Rx_date=0;
	/*初始显示*/
	OLED_Clear();
	 OLED_ShowString(0,0,"聆听中...              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
	OLED_Update();
	Delay_s(10);
	motor_Setv(60);
	Delay_s(5);
	motor_Setv(0);
	while(1)
	{	KeyNum = Key_GetNum();
		if(KeyNum == 3){
			return flag;}
		if(Serial_GetRxFlag()==1)
		{
		Rx_date=Serial_GetRxData();
		if ((Rx_date<boxflag)&&Rx_date<=4&&Rx_date>=1){servo_Setangle1(1);
			Delay_ms(SERVO_DELAY_COEFF*(boxflag-Rx_date));
		servo_Setangle1(0);
					boxflag=Rx_date;}
			
		else if ((Rx_date>boxflag)&&Rx_date<=4&&Rx_date>=1){servo_Setangle1(-1);
		Delay_ms(SERVO_DELAY_COEFF*(Rx_date-boxflag));
		servo_Setangle1(0);
		boxflag=Rx_date;}
		
		else if(Rx_date==5){motor_Setv(60);}
		else if(Rx_date==6){motor_Setv(0);}}
		switch(Rx_date)
		{
			case 0x01:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"聆听中...              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"打开1号箱           ",OLED_8X16);
				OLED_Update();
				break;
			}
			
			case 0x02:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"聆听中...              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"打开2号箱           ",OLED_8X16);
				OLED_Update();
				break;
			}
			
			
			case 0x03:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"聆听中...              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"打开3号箱           ",OLED_8X16);
				OLED_Update();
				break;
			}
			
			
			case 0x04:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"聆听中...              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"重装次数并打开密码箱          ",OLED_8X16);
				chance=3;
				Store_Data[1]=3;
				Store_Save();
				OLED_Update();
				break;
			}
			case 0x05:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"聆听中...              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"打开风扇           ",OLED_8X16);
				OLED_Update();
				break;
			}
				case 0x06:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"聆听中...              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"关闭风扇           ",OLED_8X16);
				OLED_Update();
				break;
			}}
		
			if(Rx_date==0x07){OLED_Clear();
			OLED_Update();
			return flag;
		}		
		
	}
}


int menu2_bluetooth(void)
{
	uint8_t Rx_date=0;
	/*初始显示*/
	OLED_Clear();
	 OLED_ShowString(0,0,"<---               ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
	OLED_Update();
	Delay_s(1);
	while(1)
	{	KeyNum = Key_GetNum();
		if(KeyNum==3){return flag;}
		if(Serial3_GetRxFlag())
		{
		Rx_date=Serial3_GetRxData();
		if ((Rx_date<boxflag)&&Rx_date<=4&&Rx_date>=1){servo_Setangle1(1);
			Delay_ms(SERVO_DELAY_COEFF*(boxflag-Rx_date));
		servo_Setangle1(0);
					boxflag=Rx_date;}
			
		else if ((Rx_date>boxflag)&&Rx_date<=4&&Rx_date>=1){servo_Setangle1(-1);
		Delay_ms(SERVO_DELAY_COEFF*(Rx_date-boxflag));
		servo_Setangle1(0);
		boxflag=Rx_date;}
		
		else if(Rx_date==5){motor_Setv(60);}
		else if(Rx_date==6){motor_Setv(0);}}
		switch(Rx_date)
		{
			case 0x01:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"<---                ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"打开1号箱           ",OLED_8X16);
				OLED_Update();
				break;
			}
			
			case 0x02:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"<---              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"打开2号箱           ",OLED_8X16);
				OLED_Update();
				break;
			}
			
			
			case 0x03:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"<---               ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"打开3号箱           ",OLED_8X16);
				OLED_Update();
				break;
			}
			
			
			case 0x04:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"<---              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"重装次数并打开密码箱          ",OLED_8X16);
					chance=3;
				Store_Data[1]=3;
				Store_Save();
				OLED_Update();
				break;
			}
			case 0x05:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"<---              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"打开风扇           ",OLED_8X16);
				OLED_Update();
				break;
			}
				case 0x06:
			{
				//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
				OLED_Clear();
				 OLED_ShowString(0,0,"<---              ",OLED_8X16);
 OLED_ShowString(0,16,"当前操作：            ",OLED_8X16);
				 OLED_ShowString(0,32,"关闭风扇           ",OLED_8X16);
				OLED_Update();
				break;
			}	}
		if(Rx_date==0x07){OLED_Clear();
			OLED_Update();
			return flag;
		}
		
	}
}

int menu2_sleep(){
	MPU6050_Init();
	OLED_Clear();
				OLED_Update();
	uint8_t i=0;
servo_Setangle2(500);
	int16_t AX1, AY1, AZ1, GX1, GY1, GZ1,AX2, AY2, AZ2, GX2, GY2, GZ2;
	Delay_s(1);
	MPU6050_GetData(&AX1, &AY1, &AZ1, &GX1, &GY1, &GZ1);	
		Delay_s(1);
	MPU6050_GetData(&AX1, &AY1, &AZ1, &GX1, &GY1, &GZ1);	
	MPU6050_GetData(&AX2, &AY2, &AZ2, &GX2, &GY2, &GZ2);
while(1){
	i++;
	i=i%10;
	if(i==0)
MPU6050_GetData(&AX1, &AY1, &AZ1, &GX1, &GY1, &GZ1);		//获取MPU6050的数据
	
	if((abs(AZ1-AZ2)>80&&abs(AX1-AX2)>80)||(abs(AY1-AY2)>80&&abs(AX1-AX2)>80)||(abs(AZ1-AZ2)>80&&abs(AX1-AX2)>80))
		buzz1_ON();
	
	  if (Key_GetNum() == 3) // 按下确认键返回
        {
            OLED_Clear();
            return flag;
        }
AZ2=AZ1;
	AX2=AX1;
	AY2=AY1;
}
}


/*-----------------------------------二级风扇菜单------------------------------------------------*/
/**
  * @brief 风扇控制二级菜单函数
  * @param  无
  * @retval 
  */



int menu2_motor(void)
{uint8_t speed1=60,motorflag=0;
	OLED_Clear();
	uint8_t moflag = 1;//默认在第一行
	OLED_ShowString(0,0,"<-               ",OLED_8X16);
	OLED_Printf(0,16,OLED_8X16,"速度:%.2d     ",speed1);
	
	OLED_Update();
	while(1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum == 1)//上一项
		{if(speed1>=20&&moflag==2)
			{speed1-=20;}
			else if(moflag==1||speed1==0)
			moflag--;
			if(moflag == 0){moflag = 2;}
		}
		if(KeyNum == 2)//下一项
		{if(speed1<=80&&moflag==2)
			{speed1+=20;}
				else if(moflag==1||speed1==100)
			moflag++;
			if(moflag == 3){moflag = 1;}
		}
		if(KeyNum == 3)//确认
		{motorflag=moflag;
		if(motorflag==2) {moflag=1;
			motor_Setv(speed1);
			Delay_s(6);
			return flag;
		}
			
		}
		if(motorflag== 1){  OLED_Clear();motor_Setv(0);return flag;}//返回0,退到第一级菜单
		
	
		switch(moflag)
		{
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
			case 1:
			{	OLED_Clear();
			OLED_ShowString(0,0,"<-               ",OLED_8X16);
	OLED_Printf(0,16,OLED_8X16,"速度:%.2d     ",speed1);
					OLED_ReverseArea(0,0,128,16);
				OLED_Update();
				break;
			}
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
			case 2:
			{	OLED_Clear();
	OLED_ShowString(0,0,"<-               ",OLED_8X16);
OLED_Printf(0,16,OLED_8X16,"速度:%.2d     ",speed1);
	OLED_ReverseArea(0,16,128,16);
				OLED_Update();
				break;
			}

			
		}
	}
}

/*--------------------------------------------------------------------------------------

-----------------------------------二级舵机菜单------------------------------------------------*/
/**
  * @brief 风扇控制二级菜单函数
  * @param  无
  * @retval 
  */

int menu2_servo(void)
{

uint8_t code[4]={0};
	uint8_t wei=0;
	
	uint8_t servoflag=8;
	uint8_t seflag=1;
	
	 OLED_ShowString(0,0,"<-               ",OLED_8X16);
	OLED_ShowString(0,16,"1号箱        ",OLED_8X16);
	OLED_ShowString(0,32,"2号箱        ",OLED_8X16);
	OLED_ShowString(0,48,"3号箱        ",OLED_8X16);
	OLED_Update();
	while(1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum == 1)//上一项
		{
			
			
			if(seflag!=5)
			seflag--;
		else code[wei]=(--(code[wei]))%10;
			
			if(seflag == 0){seflag = 5;}
			
			
			
		}
		if(KeyNum == 2)//下一项
		{
			if(seflag!=5)
			seflag++;
		else code[wei]=(++(code[wei]))%10;
			
			if(seflag == 6){seflag = 1;}
		}
		if(KeyNum == 3)//确认,并开始操控舵机
		{
			servoflag=seflag-1;
			if(servoflag==4&&chance!=0)
        {if(wei!=3)
				{wei++;	}
				else if(code[0]==realcode[0]&&code[1]==realcode[1]&&code[2]==realcode[2]&&code[3]==realcode[3]&&wei==3)
				 {boxflag=4;chance=3;
				Store_Data[1]=3;
					 	Store_Data[6]=4;
				Store_Save();				}
        else {seflag=1;
					 wei=0;
					 	chance--;
				 code[0]=0;code[1]=0;code[2]=0;code[3]=0;}
				}
				
				if((servoflag!=boxflag)&&servoflag!=1){
					if ((servoflag)<boxflag){servo_Setangle1(1);
			Delay_ms(SERVO_DELAY_COEFF*(boxflag-servoflag));
		servo_Setangle1(0);
					boxflag=servoflag;}
			
		else if (servoflag>boxflag){servo_Setangle1(-1);
		Delay_ms(SERVO_DELAY_COEFF*(servoflag-boxflag));
		servo_Setangle1(0);
		boxflag=servoflag;}
				}}
			
				
			
		if(servoflag==0) {OLED_Clear();return flag;}//返回了要清除
		Store_Data[6]=boxflag;
	Store_Data[1]=chance;
		Store_Save();
		switch(seflag)
		{
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
			case 1:
			{OLED_Clear();
				OLED_ShowString(0,0,"<-             ",OLED_8X16);
				OLED_ShowString(0,16,"1号箱       ",OLED_8X16);
				OLED_ShowString(0,32,"2号箱       ",OLED_8X16);
				OLED_ShowString(0,48,"3号箱       ",OLED_8X16);
				
				OLED_ReverseArea(0,0,128,16);
				OLED_Update();
				break;
			}
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
			case 2:
			{OLED_Clear();
				 OLED_ShowString(0,0,"<-             ",OLED_8X16);
				OLED_ShowString(0,16,"1号箱        ",OLED_8X16);
				OLED_ShowString(0,32,"2号箱        ",OLED_8X16);
				OLED_ShowString(0,48,"3号箱        ",OLED_8X16);
				
				OLED_ReverseArea(0,16,128,16);
				OLED_Update();
				break;
			}
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
			case 3:
			{OLED_Clear();
				 OLED_ShowString(0,0,"<-              ",OLED_8X16);
				OLED_ShowString(0,16,"1号箱         ",OLED_8X16);
				OLED_ShowString(0,32,"2号箱         ",OLED_8X16);
				OLED_ShowString(0,48,"3号箱         ",OLED_8X16);
			
				OLED_ReverseArea(0,32,128,16);
				OLED_Update();
				break;
			}
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
			case 4:
			{	OLED_Clear();
				 OLED_ShowString(0,0,"<-              ",OLED_8X16);
				OLED_ShowString(0,16,"1号箱         ",OLED_8X16);
				OLED_ShowString(0,32,"2号箱         ",OLED_8X16);
				OLED_ShowString(0,48,"3号箱         ",OLED_8X16);
			
				OLED_ReverseArea(0,48,128,16);
				OLED_Update();
				break;
			}
					case 5:
			{	OLED_Clear();
				 OLED_ShowString(0,0,"<-              ",OLED_8X16);
		OLED_ShowString(0,16,"请输入密码         ",OLED_8X16);
				OLED_Printf(0,32,OLED_8X16,"%d %d %d %d   ",code[0],code[1],code[2],code[3]);
				OLED_ReverseArea(wei*16,32,16,16);
				if(boxflag==4)	OLED_ShowString(0,48,"密码箱已开       ",OLED_8X16);
				else OLED_Printf(0,48,OLED_8X16,"chance :%d   ",chance);
				OLED_Update();
				break;
			}
			
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
	}
}}


int menu2_code(void)
{uint8_t code[4]={0};
	uint8_t wei=0;
	uint8_t coflag=1,codeflag=0;
	 OLED_ShowString(0,0,"<-               ",OLED_8X16);
if(boxflag!=4) OLED_ShowString(0,16,"无权限        ",OLED_8X16);
				else
				{OLED_Printf(0,32,OLED_8X16,"%d %d %d %d   ",code[0],code[1],code[2],code[3]);
		OLED_ShowString(0,16,"修改密码             ",OLED_8X16);}
	OLED_Update();
	while(1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum == 1&&boxflag==4)//上一项
		{
			if(coflag!=2)
			coflag--;
		else code[wei]=(--(code[wei]))%10;
			
			if(coflag == 3){coflag = 1;}
			
		}
		if(KeyNum == 2&&boxflag==4)//下一项
		{
			if(coflag!=2&&boxflag==4)
			coflag++;
		else code[wei]=(++(code[wei]))%10;
			
			if(coflag == 0){coflag = 1;}
		}
		if(KeyNum == 3)//确认,并开始操控舵机
		{
			codeflag=coflag;
			if(codeflag==2&&boxflag==4)
        {
					
					if(wei!=3)
				{wei++;}
				else 
         {coflag=1;
					 wei=0;
realcode[0]=code[0];realcode[1]=code[1];
					 realcode[2]=code[2];
					 realcode[3]=code[3];
					 
					 	Store_Data[2]=realcode[0];
		Store_Data[3]=realcode[1];
		Store_Data[4]=realcode[2];
		Store_Data[5]=realcode[3];
			chance=3;
				Store_Data[1]=3;
		Store_Save();
				}
			}}
				
			
		if(codeflag==1) {OLED_Clear();return flag;}//返回了要清除
		
	
		switch(coflag)
		{
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
			case 1:
			{OLED_Clear();
			OLED_ShowString(0,0,"<-               ",OLED_8X16);
				if(boxflag!=4) OLED_ShowString(0,16,"无权限        ",OLED_8X16);
				else
				{OLED_Printf(0,32,OLED_8X16,"%d %d %d %d   ",code[0],code[1],code[2],code[3]);
		OLED_ShowString(0,16,"修改密码             ",OLED_8X16);}
				OLED_ReverseArea(0,0,128,16);
				OLED_Update();
				break;
			}
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
			case 2:
			{	OLED_Clear();
				
			
			OLED_ShowString(0,0,"<-               ",OLED_8X16);
	OLED_ShowString(0,16,"修改密码        ",OLED_8X16);
OLED_Printf(0,32,OLED_8X16,"%d %d %d %d   ",code[0],code[1],code[2],code[3]);
					OLED_ReverseArea(wei*16,32,128,16);
				OLED_Update();
				break;
			}
			
			
			
			//再次显示,解决乱码和闪烁问题(确保反相前是亮的状态)
	}
}}


