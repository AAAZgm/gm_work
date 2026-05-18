#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "AD.h"
#include "dht11.h"
#include "Serial.h"

// 全局变量
static uint16_t ADValue;
static float Voltage;
static uint8_t Temperature = 0;
static uint8_t Humidity = 0;

// 初始化所有模块
void System_Init(void)
{
    // 1. 先初始化串口，用于调试输出
    Serial_Init();
    
    // 2. 延时确保系统稳定
    Delay_ms(100);
    
    // 3. 初始化OLED
    OLED_Init();
    
    // 4. 延时确保OLED稳定
    Delay_ms(200);  // 增加延时
    
    // 5. 清屏并显示
    OLED_Clear();  // 先清屏
    OLED_ShowString(0, 0, "Temp:", OLED_8X16);
    OLED_ShowString(0, 16, "Humi:", OLED_8X16);
    OLED_ShowString(0, 32, "Volt:", OLED_8X16);
    OLED_Update();
    
    // 6. 输出调试信息
    printf("System initialized successfully!\r\n");
    printf("OLED initialized and showing static text.\r\n");
    
    // 7. 初始化其他模块
    AD_Init();
    DHT11_Init();
}

// 读取传感器数据
void Read_Sensors(void)
{
    ADValue = AD_GetValue();
    Voltage = (float)ADValue / 4095 * 3.3;
    
    if (DHT11_Read_Data(&Temperature, &Humidity) != 0)
    {
        printf("DHT11 read failed!\r\n");
        // 设置默认值或错误提示
        Temperature = 0;
        Humidity = 0;
    }

}

// OLED显示更新
void Update_Sensor(void)
{
    // 清除数值显示区域
    OLED_ClearArea(40, 0, 88, 48);  // 清除数值显示区域
    
    // 显示温度 (X=40, Y=0)
    OLED_ShowNum(40, 0, Temperature, 2, OLED_8X16);
    OLED_ShowString(56, 0, "C", OLED_8X16);
    
    // 显示湿度 (X=40, Y=16)
    OLED_ShowNum(40, 16, Humidity, 2, OLED_8X16);
    OLED_ShowString(56, 16, "%", OLED_8X16);
    
    // 显示电压 (X=40, Y=32)，使用浮点数函数
    OLED_ShowFloatNum(40, 32, Voltage, 1, 2, OLED_8X16);
    OLED_ShowString(72, 32, "V", OLED_8X16);
    
    // 更新显示到屏幕
    OLED_Update();
}

// 串口数据发送
void Serial_Send(void)
{
    printf("Temp:%02dC Humi:%02d%% Volt:%.2fV\r\n", 
           Temperature, Humidity, Voltage);
}

int main(void)
{
    System_Init();
    
    // 显示初始状态
    printf("System running...\r\n");
    
    while (1)
    {
        Read_Sensors();
        Update_Sensor();
        Serial_Send();
        Delay_ms(1000);
    }
}