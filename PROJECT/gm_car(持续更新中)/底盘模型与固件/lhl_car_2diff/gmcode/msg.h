#ifndef __MSG_H__
#define __MSG_H__

#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"
#include "motor.h"

typedef enum 
{
RDK=0x01U,HC_08=0x02U,ERROR_NONE=0x03U,
} CONTROL;

void send_msg(uint16_t *mes);
CONTROL use_msg(uint8_t *command);
#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
