#include "led.h"
void led1_on(void)//任务指示灯
{
HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,led_open);
}
void led1_off(void)
{
HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,led_close);
}

void led1_turn(void)
{
HAL_GPIO_TogglePin(led1_GPIO_Port,led1_Pin);
}
