#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void servo_init(void){

pwm_init();

}

void servo_Setangle1(float r){
PWM_SetCompare2(1500+r*500);
}


void servo_Setangle2(float r){
PWM_SetCompare4(r);
}

