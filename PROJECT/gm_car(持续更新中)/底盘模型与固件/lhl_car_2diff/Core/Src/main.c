/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//uint8_t readbuffer[128];//环形

//空闲接收时先串口中断，如果持续dma中断则是dma进中断而串口不进
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&huart1)//给上位机的
{
	Command_Write(readbuffer,Size);
	//serial_printf("RDK%d",Command_Write(readbuffer,Size));
	HAL_UARTEx_ReceiveToIdle_IT(&huart1,readbuffer,sizeof(readbuffer));
	
	//HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//功能：HAL_UARTEx_ReceiveToIdle_DMA()的中断回调函数（需用户重写）。
}
else if(huart==&huart2)//给蓝牙的
{//serial_printf("BLE%d",Command_Write(readbuffer,Size));
	Command_Write(readbuffer,Size);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2,readbuffer,sizeof(readbuffer));
	
	//HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//功能：HAL_UARTEx_ReceiveToIdle_DMA()的中断回调函数（需用户重写）。
}
//else if(huart==&huart4)
//{
//	if(The_task.is_ok==nothing)
//	{The_task.is_ok=something;
//	led1_on();}
//}
}

//发给上位机，若用了dma则只有dma进中断
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {//必须开dma的中断，因为用dma发送的，要让dma叫,uart handle里面会判断如果是dma那就清除
    if (huart == &huart1) {
        // 空实现即可，但必须重写（覆盖弱函数）
        // 作用：HAL 库会在此回调中重置 gState 为 READY
//			__HAL_UART_CLEAR_FLAG(&huart1,UART_FLAG_TC);
//			huart->gState = HAL_UART_STATE_READY;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

		
HAL_UARTEx_ReceiveToIdle_IT(&huart1,readbuffer,sizeof(readbuffer));//上位机接收
HAL_UARTEx_ReceiveToIdle_IT(&huart2,readbuffer,sizeof(readbuffer));//蓝牙
HAL_TIM_Base_Start_IT(&htim2);//开一次就行,
dma_adc_init();
motor_init();
//set_v(100,1);
//set_v(1000,2);
//task_init();																	

  while (1)
  {//serial_printf("1111111%d",key_getnum());
//if(key_getnum()==1){HAL_UARTEx_ReceiveToIdle_IT(&huart2,readbuffer,sizeof(readbuffer)); led1_turn();}//蓝牙
//else if(key_getnum()==2){HAL_UARTEx_ReceiveToIdle_IT(&huart4,&The_task.which,1); led1_turn();HAL_Delay(1000);led1_turn();}//扫码任务
//else if(key_getnum()==3){check_self();}
		//send_msg();
    //commandLength = while_Command_GetCommand(command);

while_Command_GetCommand(now_command);
        use_msg(now_command);
    
		//serial_printf("whilefirst%d,last%f",unhandle_msg[0],dma_start_collect(unhandle_msg));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
