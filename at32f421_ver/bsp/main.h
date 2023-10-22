/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "at32f421.h"

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
#define LED0_Pin 						GPIO_PINS_0
#define LED0_GPIO_Port 			GPIOA
#define IN1_Pin 						GPIO_PINS_1
#define IN1_GPIO_Port 			GPIOA
#define IN2_Pin 						GPIO_PINS_2
#define IN2_GPIO_Port 			GPIOA
#define IN3_Pin 						GPIO_PINS_3
#define IN3_GPIO_Port 			GPIOA
#define BUTTON_Pin 			 		GPIO_PINS_4
#define BUTTON_GPIO_Port 		GPIOA
#define Relay3_Pin 					GPIO_PINS_6
#define Relay3_GPIO_Port 		GPIOA
#define Relay2_Pin 					GPIO_PINS_7
#define Relay2_GPIO_Port 		GPIOA
#define Relay1_Pin 					GPIO_PINS_1
#define Relay1_GPIO_Port 		GPIOB
#define RS485_CTR_Pin 			GPIO_PINS_5
#define RS485_CTR_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
void			HAL_IncTick(void);
uint32_t 	HAL_GetTick(void);
/* delay function */
void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_sec(uint16_t sec);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
