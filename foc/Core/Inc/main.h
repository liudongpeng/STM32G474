/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FOC_TIM1_ARR 4200
#define Led1_Pin GPIO_PIN_13
#define Led1_GPIO_Port GPIOC
#define Led2_Pin GPIO_PIN_14
#define Led2_GPIO_Port GPIOC
#define Led3_Pin GPIO_PIN_15
#define Led3_GPIO_Port GPIOC
#define Va_Pin GPIO_PIN_0
#define Va_GPIO_Port GPIOC
#define Vb_Pin GPIO_PIN_1
#define Vb_GPIO_Port GPIOC
#define UserTest_Pin GPIO_PIN_3
#define UserTest_GPIO_Port GPIOC
#define Ib_Pin GPIO_PIN_1
#define Ib_GPIO_Port GPIOA
#define Ia_Pin GPIO_PIN_2
#define Ia_GPIO_Port GPIOA
#define Led_RGB_Pin GPIO_PIN_5
#define Led_RGB_GPIO_Port GPIOA
#define Vbus_Pin GPIO_PIN_5
#define Vbus_GPIO_Port GPIOC
#define Key4_Pin GPIO_PIN_6
#define Key4_GPIO_Port GPIOC
#define Key3_Pin GPIO_PIN_7
#define Key3_GPIO_Port GPIOC
#define Key2_Pin GPIO_PIN_8
#define Key2_GPIO_Port GPIOC
#define Key1_Pin GPIO_PIN_9
#define Key1_GPIO_Port GPIOC
#define Lcd_RES_Pin GPIO_PIN_15
#define Lcd_RES_GPIO_Port GPIOA
#define Lcd_SCK_Pin GPIO_PIN_10
#define Lcd_SCK_GPIO_Port GPIOC
#define Lcd_DC_Pin GPIO_PIN_11
#define Lcd_DC_GPIO_Port GPIOC
#define Lcd_SDA_Pin GPIO_PIN_12
#define Lcd_SDA_GPIO_Port GPIOC
#define Encoder_CS_Pin GPIO_PIN_2
#define Encoder_CS_GPIO_Port GPIOD
#define Encoder_SCK_Pin GPIO_PIN_3
#define Encoder_SCK_GPIO_Port GPIOB
#define Encoder_MISO_Pin GPIO_PIN_4
#define Encoder_MISO_GPIO_Port GPIOB
#define Encoder_MOSI_Pin GPIO_PIN_5
#define Encoder_MOSI_GPIO_Port GPIOB
#define Lcd_CS_Pin GPIO_PIN_6
#define Lcd_CS_GPIO_Port GPIOB
#define Lcd_BLK_Pin GPIO_PIN_7
#define Lcd_BLK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
