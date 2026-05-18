#ifndef __ADC_DMA_H__
#define __ADC_DMA_H__


#ifdef __cplusplus  // 第一次判断：如果是C++编译器
extern "C" {        // 用C语言规则编译内部代码
#endif

/* ... 中间是C语言的头文件包含、函数声明等 ... */
#include "..\Core\Inc\main.h"
#include "..\Core\Inc\adc.h"
float dma_start_collect(uint16_t*msg);
float start_collect(void);
void dma_adc_init(void);

extern uint16_t unhandle_msg[1];
#ifdef __cplusplus  // 第二次判断：对应开头的#ifdef
}                   // 结束extern "C"块
#endif


#endif
