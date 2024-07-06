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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POT_WAVEFORM_Pin GPIO_PIN_0
#define POT_WAVEFORM_GPIO_Port GPIOA
#define POT_TONE_Pin GPIO_PIN_1
#define POT_TONE_GPIO_Port GPIOA
#define MIC_Pin GPIO_PIN_2
#define MIC_GPIO_Port GPIOA
#define POT_PITCH_Pin GPIO_PIN_6
#define POT_PITCH_GPIO_Port GPIOA
#define POT_RANGE_Pin GPIO_PIN_7
#define POT_RANGE_GPIO_Port GPIOA
#define LED_EYES_Pin GPIO_PIN_0
#define LED_EYES_GPIO_Port GPIOB
#define PITCH_IN_Pin GPIO_PIN_8
#define PITCH_IN_GPIO_Port GPIOA
#define MIDI_TX_Pin GPIO_PIN_9
#define MIDI_TX_GPIO_Port GPIOA
#define BUT_CALIB_Pin GPIO_PIN_10
#define BUT_CALIB_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define MOTOR_Pin GPIO_PIN_4
#define MOTOR_GPIO_Port GPIOB
#define SMOKE_Pin GPIO_PIN_5
#define SMOKE_GPIO_Port GPIOB
#define VOLUME_IN_Pin GPIO_PIN_6
#define VOLUME_IN_GPIO_Port GPIOB
#define LED_NOODLE_Pin GPIO_PIN_7
#define LED_NOODLE_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void pitchMeasurementInterruptHandler();
void volumeMeasurementInterruptHandler();
void calibrate();
void readPotentiometers();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
