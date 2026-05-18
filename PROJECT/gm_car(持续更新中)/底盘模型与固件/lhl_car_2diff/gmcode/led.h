#ifndef __LED_H__
#define __LED_H__

#define led_open GPIO_PIN_RESET
#define led_close GPIO_PIN_SET


#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"

void led1_on(void);
void led1_off(void);
void led1_turn(void);

#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
