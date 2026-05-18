#ifndef __MOTOR_H__
#define __MOTOR_H__


#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"
#include "..\Core\Inc\tim.h"
#include "msg.h"//用于send_msg


//#define HALL_PAIRS 11       // 霍尔对极数
//#define GEAR_RATIO 90       // 减速比
//#define ENCODER_MULTIPLE 2  // 编码器倍频（双边沿采样）
//#define FULL_SPEED_RPM 320  // 满转转速（rpm)

//#define Max_Pid_Value 360 //最大转速时的脉冲数量
//#define Min_Pid_Value -360
#define Max_PWM_Value 1000 //这里得看那个hal库怎么配置hz，两个通道一个控制正一个控制反
#define Min_PWM_Value -1000

#define RATE 20.0f
#define PULSE_A_circle 3960.0f
#define RADIUS 65.0f //mm
#define PI 3.1415926f
typedef struct
{
	float target_val;   //目标值,注意是值，不是百分比
	float Error;          /*第 k 次偏差 */
	float LastError;     /* Error[-1],第 k-1 次偏差 */
	float PrevError;    /* Error[-2],第 k-2 次偏差 */
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
	float output_val;   //输出值
}PID;
extern int16_t l_total_cycle,r_total_cycle;
extern PID R_AddPID;
extern PID L_AddPID;


extern int16_t actual1,actual2;
extern int16_t targ1,targ2;
extern float out1,out2;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void L_PID_Init(void);
void R_PID_Init(void);
void set_v(int16_t speed,uint8_t which);
float addPID_realize(PID *pid, float actual_val);
void motor_init(void);
float get_angular(int16_t cycle);
float get_percentspeed(uint8_t which);
float get_targetpulse(float targetv);
void Set_PID_TargetSpeed( float target,uint8_t which);
static int16_t tf_speed(uint8_t *command, uint8_t offset);
#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
