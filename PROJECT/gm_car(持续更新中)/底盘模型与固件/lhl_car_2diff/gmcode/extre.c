#include "extre.h"
Task_type current_task = nothing;
void task_clear(void){
current_task=nothing;
}
void doing_task(uint8_t task_type){
	
if(task_type==clear_odom_task)
{clear_odom(&odom_def);
}
task_clear();
}