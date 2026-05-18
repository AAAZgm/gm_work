#ifndef __KINEMATIC_H__
#define __KINEMATIC_H__

#define WHEEL_DIS 0.25f //m
                  
#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"

typedef struct
{
	float wheel_distance_;
	float left_v; 
	float right_v; //m/s,不过这两个其实没有使用
	//这两乘了1000
//	float angular_v; //弧度/s
//	float cmd_vel; //m/s
}WHEEL;

//这些是小车实际的
typedef struct
{
	
	float liner_speed_;
	float angle_speed_;//m/s,用于·计算
	float x_;
	float y_;//m
	float angle_;//弧度
}ODOM;

extern WHEEL diff_car;
extern ODOM odom_def;

void foward_kinematic(float left,float right,float *all_v,float *all_w);
void inverse_kinematic(float all_v,float all_w,float *left,float *right);
void clear_odom(ODOM *odom);
void update_odom(uint16_t dt_ms);
static void TF_angel_PI(float angle); //被 static 修饰的函数（也叫 “静态函数”），只能在其定义所在的 .c 文件（编译单元）内被访问和调用
#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
