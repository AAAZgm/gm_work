#include "msg.h"
#include <math.h>  // 需要引入 fabs 函数
#define SEND_COMMAND_MIN_LENGTH 16
#define FRAME_HEAD 0          // 帧头位置
#define BAT_VOLT_OFFSET 1    // 电池电压起始位置


//这三个是国际单位，但是乘1000
#define V_OFFSET 3    //速度
#define W_OFFSET 5    //角速度

//这三个是国际单位，但是乘1000
#define X_POSITION 7    //里程计位置
#define Y_POSITION 9    //里程计位置
#define ANGLE_POSITION  11  //当前角度

//#define TASKER 13    //任务（扫码扫到多少）
#define CHECK_OFFSET 14   // 校验位位置
#define FRAME_TAIL 15         // 帧尾位
//头都是0xAA,尾巴有区别

//如果 send_buff 是函数内的局部变量（没有 static），DMA 传输是异步的，
//函数执行完后栈空间会被释放，DMA 可能还在传输，导致数据被覆盖，出现随机值。
//一定要加 static 或定义为全局变量

void send_msg(uint16_t *mes){//mes是电压
static uint8_t send_buff[SEND_COMMAND_MIN_LENGTH];


 
//填充动态字段
float bat_volt =dma_start_collect(mes); // 电池电压
uint8_t data_int=bat_volt;
uint8_t data_f=(uint8_t)((uint32_t)(bat_volt * 100 + 0.5f) % 100);
//若原数值的小数部分 ≥ 0.5，加 0.5 后会进位（如 398.5 + 0.5 = 399.0，截断后为 399）。
//c若原数值的小数部分 < 0.5，加 0.5 后不会进位（如 398.4 + 0.5 = 398.9，截断后为 398）。
send_buff[BAT_VOLT_OFFSET] = data_int;       // 替代 memcpy
send_buff[BAT_VOLT_OFFSET + 1] = data_f;


int16_t data=0;

// 线速度
data = (int16_t)(odom_def.liner_speed_ * 1000 + 0.5f);
memcpy(&(send_buff[V_OFFSET]), &data, 2);

// 角速度
data = (int16_t)(odom_def.angle_speed_ * 1000 + 0.5f);
memcpy(&(send_buff[W_OFFSET]), &data, 2);

// x位置
data= (int16_t)(odom_def.x_ * 1000 + 0.5f);
memcpy(&(send_buff[X_POSITION]), &data, 2);

// y位置
data= (int16_t)(odom_def.y_ * 1000 + 0.5f);
memcpy(&(send_buff[Y_POSITION]), &data, 2);

// 当前角度（弧度）
data= (int16_t)(odom_def.angle_ * 1000 + 0.5f);
memcpy(&(send_buff[ANGLE_POSITION]), &data, 2);



//填充固定字段
send_buff[FRAME_HEAD] = 0xAA;    // 帧头
send_buff[FRAME_TAIL] = 0x7E;   // 帧尾

// if(The_task.is_ok==something){
// send_buff[TASKER]=The_task.which;
// The_task.which=0x00;//清除当前事件
//HAL_UARTEx_ReceiveToIdle_IT(&huart4,&The_task.which,1);
// The_task.is_ok=doing;
// }
send_buff[CHECK_OFFSET]=0;
//计算和校验
for(uint8_t i=0;i<SEND_COMMAND_MIN_LENGTH-2;i++)
{send_buff[CHECK_OFFSET]+=send_buff[i];}

//serial_printf("\r\n");  // 修正：/ → \

// 5. 发送数据（二进制字节流）
// 优化监视逻辑：按「帧尾、校验位、预留、帧头、电池电压」的顺序打印，更易读
//serial_printf(
//    "0=0x%02X,14=0x%02X, 13=0x%02X, 12=0x%02X, 0=0x%02X, 1=0x%02X, 2=0x%02X\r\n",
//    send_buff[15],    // 帧尾
//    send_buff[14],    // 校验位
//    send_buff[13],    // 预留13
//    send_buff[12],    // 预留12
//    send_buff[0],     // 帧头
//    send_buff[1],     // 电池电压整数
//    send_buff[2]      // 电池电压小数
//);
// 发送二进制数据帧


//serial_printf("\r\n");  // 修正：/ → \

HAL_UART_Transmit_DMA(&huart1,(uint8_t*)send_buff,16);
}

//void send_msg(uint16_t *mes){
//    // 1. 定义固定的16字节数组（完全手写，无任何动态逻辑）
//    uint8_t send_buff[16] = {
//        0xAA, 0x04, 0x16,  // 帧头+电池电压4.22V
//        0x00, 0x00,        // 左编码器速度0
//        0x00, 0x00,        // 右编码器速度0
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 预留7字节
//        0x00,              // 校验位（空）
//        0x7E               // 帧尾
//    };

//    // 2. 直接发送，无任何其他逻辑
//    HAL_UART_Transmit_DMA(&huart1, send_buff, 16);
//}
#define RECOMMAND_MIN_LENGTH 16
#define REFRAME_HEAD 0          // 帧头位置
//#define REL_V_OFFSET 2    			//速度
//#define RER_V_OFFSET 4    			//速度
#define REV_OFFSET 1    				//V速度
#define REW_OFFSET 3    				//W速度
#define TASKER 13
#define RECHECK_OFFSET 14    						//校验和
#define REFRAME_TAIL 15         // 帧尾位
//收的也乘1000
// 单独定义解析速度的函数（提取重复逻辑）
static int16_t tf_speed(uint8_t *command, uint8_t offset) {
    // 1. 拼接高、低字节为16位无符号数（小端模式）
    uint16_t unsigned_speed = ( (uint16_t)*(command + offset + 1) << 8 )  // 高字节左移8位
                            | (uint16_t)*(command + offset);             // 低字节直接拼接
    // 2. 强制转换为有符号int16_t，自动识别补码（支持负数）
    return (int16_t)unsigned_speed;
}
static float last_angular_speed = 0.0f;
CONTROL use_msg(uint8_t *command) {
    uint8_t check_bit = 0x00;

    // 计算校验和
    for (uint8_t i = 0; i < RECOMMAND_MIN_LENGTH - 2; i++) {
        check_bit += *(command + i);
    }

    if (check_bit == *(command + RECHECK_OFFSET)) {

        // ===== 处理 RDK 命令 (帧尾 0x01) =====
        if (*(command + RECOMMAND_MIN_LENGTH - 1) == 0x01U) {

            if (*(command + TASKER) != 0x00U) {
                doing_task(*(command + TASKER));
            }

            float liner_speed = tf_speed(command, REV_OFFSET) / 1000.0f;
            float angular_speed = tf_speed(command, REW_OFFSET) / 1000.0f;

            // ===== 核心修改：转弯结束后平衡PID输出 =====
            // 检测条件：上次有角速度(转弯中)，这次没有(直行)
            // fabs 判断阈值 0.01，避免浮点精度问题
            if (fabs(last_angular_speed) > 0.01f && fabs(angular_speed) < 0.01f) {
                // 取左右PID输出的平均值，强制拉平
                float avg_output = (L_AddPID.output_val + R_AddPID.output_val) / 2.0f;
                L_AddPID.output_val = avg_output;
                R_AddPID.output_val = avg_output;
            }
            // 更新历史角速度
            last_angular_speed = angular_speed;

            // 运动学逆解，计算左右轮目标速度
            inverse_kinematic(liner_speed, angular_speed, &diff_car.left_v, &diff_car.right_v);

            Set_PID_TargetSpeed(get_targetpulse(diff_car.left_v), 1);
            Set_PID_TargetSpeed(get_targetpulse(diff_car.right_v), 2);

            *(command + RECOMMAND_MIN_LENGTH - 1) = 0;
            return RDK;
        }

        // ===== 处理 HC-08 蓝牙命令 (帧尾 0x02) =====
        else if (*(command + RECOMMAND_MIN_LENGTH - 1) == 0x02U) {

            float liner_speed = tf_speed(command, REV_OFFSET) / 1000.0f;
            float angular_speed = tf_speed(command, REW_OFFSET) / 1000.0f;

            // ===== 同样的转弯平衡逻辑 =====
            if (fabs(last_angular_speed) > 0.01f && fabs(angular_speed) < 0.01f) {
                float avg_output = (L_AddPID.output_val + R_AddPID.output_val) / 2.0f;
                L_AddPID.output_val = avg_output;
                R_AddPID.output_val = avg_output;
            }
            last_angular_speed = angular_speed;

            inverse_kinematic(liner_speed, angular_speed, &diff_car.left_v, &diff_car.right_v);

            Set_PID_TargetSpeed(get_targetpulse(diff_car.left_v), 1);
            Set_PID_TargetSpeed(get_targetpulse(diff_car.right_v), 2);

            *(command + RECOMMAND_MIN_LENGTH - 1) = 0;
            return HC_08;
        }

        else {
            *(command + RECOMMAND_MIN_LENGTH - 1) = 0;
            return ERROR_NONE;
        }
    }
    else {
        *(command + RECOMMAND_MIN_LENGTH - 1) = 0;
        return ERROR_NONE;
    }
}


////接收两种
//CONTROL use_msg(uint8_t *command){

//if(*(command+COMMAND_MIN_LENGTH-1)==0x01U)
//{serial_printf("RDK\r\n");
//Set_PID_TargetSpeed((*(command+L_V_OFFSET))*256+(*(command+L_V_OFFSET+1)),1);
//Set_PID_TargetSpeed((*(command+R_V_OFFSET))*256+(*(command+R_V_OFFSET+1)),2);
//	return RDK;}
//else if(*(command+COMMAND_MIN_LENGTH-1)==0x02U)
//{serial_printf("HC\r\n");
//Set_PID_TargetSpeed((*(command+L_V_OFFSET))*256+(*(command+L_V_OFFSET+1)),1);
//Set_PID_TargetSpeed((*(command+R_V_OFFSET))*256+(*(command+R_V_OFFSET+1)),2);
//	return HC_08;}
//else return ERR;//错误来源
//}
