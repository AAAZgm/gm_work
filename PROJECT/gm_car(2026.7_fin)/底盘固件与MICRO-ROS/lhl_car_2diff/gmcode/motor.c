#include "motor.h"
 
 //m1 pwm用的timer1 ch3ch4（pe14,13)，读取用tim5定(pa0,pa1),左，1，编码顺，驱动逆
 //m3 pwm用的timer9 ch1ch2（pe5,6)，读取用tim4定(pb6,pb7),右，2，编码顺，驱动顺
 //要对m1的读取改极性，驱动和m3反
 //不懂就看原理图
PID L_AddPID;
PID R_AddPID;
 


void L_PID_Init(void)
{		L_AddPID.PrevError=0;
    L_AddPID.target_val = 0;
    L_AddPID.output_val = 0.0;
    L_AddPID.Error = 0.0;
    L_AddPID.LastError = 0.0;
    L_AddPID.integral = 0.0;
    L_AddPID.Kp = 1;
    L_AddPID.Ki = 0.5;
    L_AddPID.Kd = 0; 
 
}
void R_PID_Init(void)
{		R_AddPID.PrevError=0;
    R_AddPID.target_val = 0;
    R_AddPID.output_val = 0.0;
    R_AddPID.Error = 0.0;
    R_AddPID.LastError = 0.0;
    R_AddPID.integral = 0.0;//增量式不用
    R_AddPID.Kp = 1;
    R_AddPID.Ki = 0.5;
    R_AddPID.Kd = 0; 
 
}
 
 void motor_init(void){
	 
HAL_TIM_Base_Start(&htim4);
HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

HAL_TIM_Base_Start(&htim5);
HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
	
	 
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim9);
	//__HAL_TIM_ENABLE(&htim1);
	//__HAL_TIM_MOE_ENABLE(&htim1);
	 
	 
		HAL_TIM_PWM_Init(&htim1);
	 __HAL_TIM_MOE_ENABLE(&htim1);//1，8需要
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);  
   
	  HAL_TIM_PWM_Init(&htim9);
	 // __HAL_TIM_MOE_ENABLE(&htim9);
		HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);  
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);  
	 
	 
 __HAL_TIM_SET_COUNTER(&htim5,0);
 __HAL_TIM_SET_COUNTER(&htim4,0);
	R_PID_Init();
	L_PID_Init();
	 
//Set_PID_TargetSpeed(0,1);//实际上你不能给百分比，给的圈数，因为kpki无单位的
//Set_PID_TargetSpeed(0,2);	
 }
 

 void Set_PID_TargetSpeed( float target,uint8_t which) 
{
    // 限制目标值范围（根据编码器实际量程，如0~100脉冲/50ms）
    if (target < Min_PWM_Value) target = Min_PWM_Value;
    if (target > Max_PWM_Value) target = Max_PWM_Value;

    // 根据电机编号设置目标值
    if (which == 1) {
        L_AddPID.target_val = target;
    } else if (which == 2) {
        R_AddPID.target_val = target;
    }
}
 
 //紧急操作：用set_v直接干预（如停止、急加速），此时会暂时覆盖 PID 输出。它是-1000-1000，和pwm独立,这个确实百分比
//pwm两个通道一个控制正一个控制反
 void set_v(int16_t speed,uint8_t which)//which=1=左
{
speed = (speed >Max_PWM_Value ) ? Max_PWM_Value : (speed < Min_PWM_Value) ? Min_PWM_Value : speed;
	if(speed>0)
{ 
	switch (which) {
    case 1:
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,speed);
			 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		 
        break;  // 跳出switch结构（不执行后续case）
    case 2:
         __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,0);
			   __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,speed);
		  
        break;  // 最后一个break可省略，但建议加上
}
}
else if(speed<0)
{
	switch (which) {
    case 1:
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
			 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,-speed);
		
        break;  // 跳出switch结构（不执行后续case）
    case 2:
         __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
			   __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,-speed);
		
        break;  // 最后一个break可省略，但建议加上
}
}
else
{
	switch (which) {
    case 1:
       __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
			 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
        break;  // 跳出switch结构（不执行后续case）
    case 2:
         __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,0);
			   __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,0);
        break;  // 最后一个break可省略，但建议加上
}
}
}
 
 
 

 
/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
  *	@note 	无
  * @retval 通过PID计算后的输出
  */
float addPID_realize(PID *pid, float actual_val)
{
	/*计算目标值与实际值的误差*/
	pid->Error = pid->target_val - actual_val;
	/*PID算法实现，照搬公式*/
	pid->output_val += pid->Kp * (pid->Error - pid-> LastError) +
	                  pid->Ki * pid->Error +
	                  pid->Kd *(pid->Error -2*pid->LastError+pid->PrevError);
	/*误差传递*/
	pid-> PrevError = pid->LastError;
	pid-> LastError = pid->Error;
	/*返回当前实际值*/
	    if(pid->output_val>Max_PWM_Value)
{
        pid->output_val=Max_PWM_Value;
        
}
		else if(pid->output_val < Min_PWM_Value) 
{
    pid->output_val = Min_PWM_Value;
}
			
	  return pid->output_val;
}


//int16_t actual1,actual2;
//int16_t targ1,targ2;
//float out1,out2;



//int16_t l_total_cycle = 0; 
//int16_t r_total_cycle = 0; 


static int16_t L_Encoder_Speed,R_Encoder_Speed;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

 //可以加个滤波但没必要
  //  2号定时器中断
  if(htim == (&htim2))
  {		 static int16_t R_Output_Val,L_Output_Val;
		 R_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
		 L_Encoder_Speed = (int16_t)__HAL_TIM_GET_COUNTER(&htim5);
	  //  编码器数据清零
    __HAL_TIM_SET_COUNTER(&htim4, 0);//这里已经帮你处理正负了
		__HAL_TIM_SET_COUNTER(&htim5, 0);
		
//		actual1=L_Encoder_Speed;
//    targ1=L_AddPID.target_val;
//    out1=L_Output_Val;
//    actual2=R_Encoder_Speed;
//    targ2=R_AddPID.target_val;
//    out2=R_Output_Val;
//serial_printf("left%+5d,%+5d,%+05.2f\r\nright%+5d,%+5d,%+05.2f\r\n",actual1,targ1,out1,actual2,targ2,out2);
		R_Output_Val=(int16_t)addPID_realize(&R_AddPID,R_Encoder_Speed);
		L_Output_Val=(int16_t)addPID_realize(&L_AddPID,L_Encoder_Speed);
//serial_printf("%d",while_Command_GetCommand(now_command));
	  set_v(L_Output_Val,1);
	  set_v(R_Output_Val,2);

    //  设置测量频率为20Hz(50ms)
  //  uint8_t rate = 20;
		
    //  获取编码器信号数，慢速778

		update_odom(50.0f);

//serial_printf("R_Encoder_Speed%+5d,r_total_cycle%d,r_total_angle%05.2f,r_v%05.2f\r\n",
//R_Encoder_Speed,r_total_cycle,get_angular(2),get_percentspeed(2));
//serial_printf("L_Encoder_Speed%+5d,l_total_cycle%d,l_total_angle%05.2f,l_v%05.2f\r\n",
//L_Encoder_Speed,l_total_cycle,get_angular(1),get_percentspeed(1));
//    serial_printf("x%05.2f,y%05.2f,wheel_dis%f,angle%+05.2f\r\n",
//     odom_def.x_,odom_def.y_,diff_car.wheel_distance_,odom_def.angle_);



		send_msg(unhandle_msg);
  }

}
float get_angular(int16_t cycle)//读出累计角度
{
		return (float)((cycle*360.0f)/PULSE_A_circle);
}

float get_percentspeed(uint8_t which)//mm/ms==m/s
{
if(which==1)
		return (float)L_Encoder_Speed*(PI*RADIUS/PULSE_A_circle/1000.0f)/(1.0f/RATE);//t=1/f=1/20 mm/s
else
		return (float)R_Encoder_Speed*(PI*RADIUS/PULSE_A_circle/1000.0f)/(1.0f/RATE);
}
float get_targetpulse(float targetv)//mm/ms==m/s
{
		return (float)targetv*(1.0f/RATE)/(PI*RADIUS/PULSE_A_circle/1000.0f);//t=1/f=1/20 mm/s

}
//一个脉冲是3.1415926*RADIUS/PULSE_A mm