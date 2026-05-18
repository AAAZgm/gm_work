#ifndef __MYSERIAL_H__
#define __MYSERIAL_H__

#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"
#include "..\Core\Inc\usart.h"
#include <stdarg.h>  // 提供可变参数相关定义
#include <stdio.h>   // 提供vsprintf函数
#include <string.h>
#define sp_msg_send 128

// 命令最小长度定义
#define COMMAND_MIN_LENGTH 16
// 缓冲区大小定义
#define BUFFER_SIZE 128
//#define msg_receive 128
extern uint8_t readbuffer[BUFFER_SIZE];
extern uint8_t now_command[COMMAND_MIN_LENGTH];
uint8_t Command_GetCommand(uint8_t *command);
uint8_t while_Command_GetCommand(uint8_t *command);
uint8_t Command_Write(uint8_t *data, uint8_t length);
void serial_printf(char *format, ...);
void serial_printf2(char *format, ...);
#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
