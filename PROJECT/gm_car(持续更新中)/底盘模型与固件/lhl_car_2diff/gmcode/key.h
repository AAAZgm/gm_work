#ifndef __KEY_H__
#define __KEY_H__

#define key_open GPIO_PIN_RESET
#define key_close GPIO_PIN_SET
#define key3_open GPIO_PIN_RESET//原理图错了
#define key3_close  GPIO_PIN_SET
                  
#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"

uint8_t key_getnum(void);

#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
