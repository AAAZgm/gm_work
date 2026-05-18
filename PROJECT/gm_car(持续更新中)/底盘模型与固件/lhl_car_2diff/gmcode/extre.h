#ifndef __EXTRE_H__
#define __EXTRE_H__


#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"
typedef enum 
{
nothing=0,clear_odom_task=0x01
} Task_type;
extern Task_type current_task;
void task_clear(void);
void doing_task(uint8_t task_type);
#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
