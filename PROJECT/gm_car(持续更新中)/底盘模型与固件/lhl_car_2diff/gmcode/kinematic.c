#include "kinematic.h"
#include <math.h>
WHEEL diff_car={.wheel_distance_=WHEEL_DIS};


ODOM odom_def;
void foward_kinematic(float left,float right,float *all_v,float *all_w){

*all_v=(left+right)/2.0f;
*all_w=(right-left)/diff_car.wheel_distance_;

}
void inverse_kinematic(float all_v,float all_w,float *left,float *right){

*left=all_v-(all_w*diff_car.wheel_distance_)/2.0f;
*right=all_v+(all_w*diff_car.wheel_distance_)/2.0f;

}

void update_odom(uint16_t dt_ms){//dt是ms

float dt_s=(float)dt_ms/1000.0f;
//求线速度角速度
foward_kinematic(get_percentspeed(1),get_percentspeed(2),&odom_def.liner_speed_,&odom_def.angle_speed_);

odom_def.angle_+=odom_def.angle_speed_*dt_s;
TF_angel_PI(odom_def.angle_);
float delta_distance=odom_def.liner_speed_*dt_s;
odom_def.x_+=delta_distance*cos(odom_def.angle_);	
odom_def.y_+=delta_distance*sin(odom_def.angle_);
}

void clear_odom(ODOM *odom){
odom->x_=0.0;//(*odom).x_ 或直接 odom->x_
odom->y_=0.0;
odom->angle_=0.0;
odom->angle_speed_=0.0;
odom->liner_speed_=0.0;
}
static void TF_angel_PI(float angle){//必须传odom_def.angle_
if(angle>PI){
odom_def.angle_-=2*PI;
}
else if (angle<-PI){
odom_def.angle_+=2*PI;
}
}