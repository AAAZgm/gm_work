/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../gmcode/led.h"
#include "../../gmcode/key.h"
#include "../../gmcode/myserial.h"
#include "../../gmcode/adc_dma.h"
#include "../../gmcode/motor.h"
//#include "../../gmcode/buzzer.h"
#include "../../gmcode/kinematic.h"
#include "../../gmcode/msg.h"
#include "../../gmcode/extre.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define battary_v_Pin GPIO_PIN_0
#define battary_v_GPIO_Port GPIOB
#define led1_Pin GPIO_PIN_10
#define led1_GPIO_Port GPIOE
#define control_key3_Pin GPIO_PIN_3
#define control_key3_GPIO_Port GPIOD
#define key2_Pin GPIO_PIN_0
#define key2_GPIO_Port GPIOE
#define key1_Pin GPIO_PIN_1
#define key1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
