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
#include "stm32f0xx_hal.h"

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
#define DINP_2_Pin GPIO_PIN_13
#define DINP_2_GPIO_Port GPIOC
#define DINP_3_Pin GPIO_PIN_14
#define DINP_3_GPIO_Port GPIOC
#define DINP_4_Pin GPIO_PIN_15
#define DINP_4_GPIO_Port GPIOC
#define M_K_TEMP_ANALOG_Pin GPIO_PIN_0
#define M_K_TEMP_ANALOG_GPIO_Port GPIOA
#define M_ANALOG_IN_Pin GPIO_PIN_1
#define M_ANALOG_IN_GPIO_Port GPIOA
#define RS485_TX_U2_Pin GPIO_PIN_2
#define RS485_TX_U2_GPIO_Port GPIOA
#define RS485_RX_U2_Pin GPIO_PIN_3
#define RS485_RX_U2_GPIO_Port GPIOA
#define M_CH_C_Pin GPIO_PIN_4
#define M_CH_C_GPIO_Port GPIOA
#define RS485_DIR_U2_Pin GPIO_PIN_5
#define RS485_DIR_U2_GPIO_Port GPIOA
#define M_4TO20MA_PWM_2_Pin GPIO_PIN_6
#define M_4TO20MA_PWM_2_GPIO_Port GPIOA
#define M_4TO20MA_PWM_1_Pin GPIO_PIN_7
#define M_4TO20MA_PWM_1_GPIO_Port GPIOA
#define M_CH_A_Pin GPIO_PIN_0
#define M_CH_A_GPIO_Port GPIOB
#define FACTORY_RST_Pin GPIO_PIN_1
#define FACTORY_RST_GPIO_Port GPIOB
#define TRIAC_1_Pin GPIO_PIN_10
#define TRIAC_1_GPIO_Port GPIOB
#define TRIAC_2_Pin GPIO_PIN_11
#define TRIAC_2_GPIO_Port GPIOB
#define RLY_1_Pin GPIO_PIN_12
#define RLY_1_GPIO_Port GPIOB
#define RLY_2_Pin GPIO_PIN_13
#define RLY_2_GPIO_Port GPIOB
#define DOUT_2_Pin GPIO_PIN_14
#define DOUT_2_GPIO_Port GPIOB
#define ZERO_CROSS_Pin GPIO_PIN_15
#define ZERO_CROSS_GPIO_Port GPIOB
#define ZERO_CROSS_EXTI_IRQn EXTI4_15_IRQn
#define RLY_3_Pin GPIO_PIN_8
#define RLY_3_GPIO_Port GPIOA
#define DBUG_TX_U1_Pin GPIO_PIN_9
#define DBUG_TX_U1_GPIO_Port GPIOA
#define DBUG_RX_U1_Pin GPIO_PIN_10
#define DBUG_RX_U1_GPIO_Port GPIOA
#define RLY_4_Pin GPIO_PIN_11
#define RLY_4_GPIO_Port GPIOA
#define DOUT_4_Pin GPIO_PIN_12
#define DOUT_4_GPIO_Port GPIOA
#define D1_SCR_GATE_1_Pin GPIO_PIN_15
#define D1_SCR_GATE_1_GPIO_Port GPIOA
#define D1_SCR_GATE_2_Pin GPIO_PIN_3
#define D1_SCR_GATE_2_GPIO_Port GPIOB
#define D2_SCR_GATE_1_Pin GPIO_PIN_4
#define D2_SCR_GATE_1_GPIO_Port GPIOB
#define D2_SCR_GATE_2_Pin GPIO_PIN_5
#define D2_SCR_GATE_2_GPIO_Port GPIOB
#define M_CH_B_Pin GPIO_PIN_6
#define M_CH_B_GPIO_Port GPIOB
#define DOUT_1_Pin GPIO_PIN_7
#define DOUT_1_GPIO_Port GPIOB
#define DINP_1_Pin GPIO_PIN_8
#define DINP_1_GPIO_Port GPIOB
#define M_NEG_5V_PWM_Pin GPIO_PIN_9
#define M_NEG_5V_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
