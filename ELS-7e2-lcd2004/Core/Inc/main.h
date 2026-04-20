/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define ADC_FEED_Pin GPIO_PIN_6
#define ADC_FEED_GPIO_Port GPIOF
#define Z_STEP_Pin GPIO_PIN_0
#define Z_STEP_GPIO_Port GPIOC
#define Z_DIR_Pin GPIO_PIN_1
#define Z_DIR_GPIO_Port GPIOC
#define Z_EN_Pin GPIO_PIN_2
#define Z_EN_GPIO_Port GPIOC
#define X_STEP_Pin GPIO_PIN_3
#define X_STEP_GPIO_Port GPIOC
#define ENC_SP_A_Pin GPIO_PIN_6
#define ENC_SP_A_GPIO_Port GPIOA
#define ENC_SP_B_Pin GPIO_PIN_7
#define ENC_SP_B_GPIO_Port GPIOA
#define X_DIR_Pin GPIO_PIN_4
#define X_DIR_GPIO_Port GPIOC
#define X_EN_Pin GPIO_PIN_5
#define X_EN_GPIO_Port GPIOC
#define JOY_L_Pin GPIO_PIN_12
#define JOY_L_GPIO_Port GPIOF
#define JOY_R_Pin GPIO_PIN_13
#define JOY_R_GPIO_Port GPIOF
#define JOY_U_Pin GPIO_PIN_14
#define JOY_U_GPIO_Port GPIOF
#define JOY_D_Pin GPIO_PIN_15
#define JOY_D_GPIO_Port GPIOF
#define MODE_D0_Pin GPIO_PIN_0
#define MODE_D0_GPIO_Port GPIOG
#define MODE_D1_Pin GPIO_PIN_1
#define MODE_D1_GPIO_Port GPIOG
#define LCD_RS_Pin GPIO_PIN_7
#define LCD_RS_GPIO_Port GPIOE
#define LCD_E_Pin GPIO_PIN_8
#define LCD_E_GPIO_Port GPIOE
#define LCD_D4_Pin GPIO_PIN_9
#define LCD_D4_GPIO_Port GPIOE
#define LCD_D5_Pin GPIO_PIN_10
#define LCD_D5_GPIO_Port GPIOE
#define LCD_D6_Pin GPIO_PIN_11
#define LCD_D6_GPIO_Port GPIOE
#define LCD_D7_Pin GPIO_PIN_12
#define LCD_D7_GPIO_Port GPIOE
#define TACHO_OUT_Pin GPIO_PIN_13
#define TACHO_OUT_GPIO_Port GPIOE
#define OUT_DIVIDER_Pin GPIO_PIN_14
#define OUT_DIVIDER_GPIO_Port GPIOE
#define BTN_MENU_U_Pin GPIO_PIN_10
#define BTN_MENU_U_GPIO_Port GPIOB
#define BTN_MENU_D_Pin GPIO_PIN_11
#define BTN_MENU_D_GPIO_Port GPIOB
#define BTN_MENU_SEL_Pin GPIO_PIN_12
#define BTN_MENU_SEL_GPIO_Port GPIOB
#define SUBMODE_0_Pin GPIO_PIN_8
#define SUBMODE_0_GPIO_Port GPIOD
#define SUBMODE_1_Pin GPIO_PIN_9
#define SUBMODE_1_GPIO_Port GPIOD
#define SUBMODE_2_Pin GPIO_PIN_10
#define SUBMODE_2_GPIO_Port GPIOD
#define AXIS_Z_Pin GPIO_PIN_11
#define AXIS_Z_GPIO_Port GPIOD
#define ENC_HC_A_Pin GPIO_PIN_12
#define ENC_HC_A_GPIO_Port GPIOD
#define ENC_HC_B_Pin GPIO_PIN_13
#define ENC_HC_B_GPIO_Port GPIOD
#define AXIS_X_Pin GPIO_PIN_15
#define AXIS_X_GPIO_Port GPIOD
#define MODE_D2_Pin GPIO_PIN_2
#define MODE_D2_GPIO_Port GPIOG
#define MODE_D3_Pin GPIO_PIN_3
#define MODE_D3_GPIO_Port GPIOG
#define MODE_D4_Pin GPIO_PIN_4
#define MODE_D4_GPIO_Port GPIOG
#define MODE_D5_Pin GPIO_PIN_5
#define MODE_D5_GPIO_Port GPIOG
#define MODE_D6_Pin GPIO_PIN_6
#define MODE_D6_GPIO_Port GPIOG
#define MODE_D7_Pin GPIO_PIN_7
#define MODE_D7_GPIO_Port GPIOG
#define LIM_REAR_Pin GPIO_PIN_8
#define LIM_REAR_GPIO_Port GPIOA
#define LIM_FRONT_Pin GPIO_PIN_9
#define LIM_FRONT_GPIO_Port GPIOA
#define LIM_RIGHT_Pin GPIO_PIN_10
#define LIM_RIGHT_GPIO_Port GPIOA
#define LIM_LEFT_Pin GPIO_PIN_11
#define LIM_LEFT_GPIO_Port GPIOA
#define BEEPER_Pin GPIO_PIN_0
#define BEEPER_GPIO_Port GPIOD
#define SCALE_X1_Pin GPIO_PIN_5
#define SCALE_X1_GPIO_Port GPIOD
#define SCALE_X10_Pin GPIO_PIN_6
#define SCALE_X10_GPIO_Port GPIOD
#define BTN_RAPID_Pin GPIO_PIN_7
#define BTN_RAPID_GPIO_Port GPIOD
#define LED_LIM_REAR_Pin GPIO_PIN_12
#define LED_LIM_REAR_GPIO_Port GPIOG
#define LED_LIM_FRONT_Pin GPIO_PIN_13
#define LED_LIM_FRONT_GPIO_Port GPIOG
#define LED_LIM_RIGHT_Pin GPIO_PIN_14
#define LED_LIM_RIGHT_GPIO_Port GPIOG
#define LED_LIM_LEFT_Pin GPIO_PIN_15
#define LED_LIM_LEFT_GPIO_Port GPIOG
#define BTN_MENU_L_Pin GPIO_PIN_8
#define BTN_MENU_L_GPIO_Port GPIOB
#define BTN_MENU_R_Pin GPIO_PIN_9
#define BTN_MENU_R_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
