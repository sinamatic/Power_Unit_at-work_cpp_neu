/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STM_INTERN_SENSE_Pin GPIO_PIN_0
#define STM_INTERN_SENSE_GPIO_Port GPIOA
#define STM_12V_SENSE_Pin GPIO_PIN_3
#define STM_12V_SENSE_GPIO_Port GPIOA
#define STM_DRIVE_1_SENSE_Pin GPIO_PIN_4
#define STM_DRIVE_1_SENSE_GPIO_Port GPIOA
#define STM_5V_SENSE_Pin GPIO_PIN_6
#define STM_5V_SENSE_GPIO_Port GPIOA
#define STM_DRIVE_1_SW_Pin GPIO_PIN_7
#define STM_DRIVE_1_SW_GPIO_Port GPIOA
#define STM_DRIVE_2_SENSE_Pin GPIO_PIN_1
#define STM_DRIVE_2_SENSE_GPIO_Port GPIOB
#define STM_DRIVE_2_SW_Pin GPIO_PIN_10
#define STM_DRIVE_2_SW_GPIO_Port GPIOB
#define STM_ARM_SW_Pin GPIO_PIN_11
#define STM_ARM_SW_GPIO_Port GPIOB
#define STM_ARM_SENSE_Pin GPIO_PIN_12
#define STM_ARM_SENSE_GPIO_Port GPIOB
#define STM_48V_SENSE_Pin GPIO_PIN_14
#define STM_48V_SENSE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
