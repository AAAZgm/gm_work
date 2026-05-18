#include "adc_dma.h"
//f4不支持校准
uint16_t unhandle_msg[1]={0};//原电压信息
float start_collect(void)
{
	static float old_volt=0;
	float true_volt=0;
HAL_ADC_Start(&hadc1);
HAL_ADC_PollForConversion(&hadc1,500);//waiting for this
if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))
{
float volt=HAL_ADC_GetValue(&hadc1);
	if(old_volt<1)
	{old_volt=volt;}
	true_volt=(volt>1&&volt<4095)?old_volt=0.5f*old_volt+0.5f*volt:old_volt;//make sure float,but double
	return (11.0f*3.3f*true_volt)/4095.0f;
}
return 0;
}

float dma_start_collect(uint16_t*msg)
{   static float old_volt = 0.0f;
    float filtered_volt;

    // 读取DMA更新的原始ADC值（输入）
   

    // 滤波逻辑：仅有效范围更新，异常值沿用旧值
    if (msg[0] >= 1 && msg[0] <= 4095) {  // 有效ADC值（1~4095）
        filtered_volt = 0.5f * old_volt + 0.5f * (float)msg[0];
        old_volt = filtered_volt;  // 仅有效时更新历史值
    } else {  // 原始值为0或超出范围（视为异常）
        filtered_volt = old_volt;  // 沿用旧值，不重置为0
    }

    // 转换为实际电压（输出到msg）
    return(11.0f * 3.3f * filtered_volt) / 4095.0f;
}
void dma_adc_init(void){
HAL_ADC_Start_DMA(&hadc1,(uint32_t*)unhandle_msg,1);//adc会被带着启动（用于adc）
}
