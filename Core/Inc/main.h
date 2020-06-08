/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f7xx_hal.h"

#include "GUI.h"
#include "HW_Init.h"
#include "GUI_App.h"
#include "STemwin_wrapper.h"

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
#define ZCT1_FAILN_Pin GPIO_PIN_2
#define ZCT1_FAILN_GPIO_Port GPIOE
#define ZCT2_FAILN_Pin GPIO_PIN_3
#define ZCT2_FAILN_GPIO_Port GPIOE
#define CP_S_02_Pin GPIO_PIN_8
#define CP_S_02_GPIO_Port GPIOI
#define ADC_SPI1_CHSEL_Pin GPIO_PIN_13
#define ADC_SPI1_CHSEL_GPIO_Port GPIOC
#define AC_DETECT1_Pin GPIO_PIN_6
#define AC_DETECT1_GPIO_Port GPIOF
#define PWM1_TM11_CH1_Pin GPIO_PIN_7
#define PWM1_TM11_CH1_GPIO_Port GPIOF
#define D01_Pin GPIO_PIN_8
#define D01_GPIO_Port GPIOF
#define DO2_Pin GPIO_PIN_9
#define DO2_GPIO_Port GPIOF
#define AC_DETECT1A0_Pin GPIO_PIN_0
#define AC_DETECT1A0_GPIO_Port GPIOA
#define AC_DETECT2_Pin GPIO_PIN_1
#define AC_DETECT2_GPIO_Port GPIOA
#define AC_DETECT2C5_Pin GPIO_PIN_5
#define AC_DETECT2C5_GPIO_Port GPIOC
#define RLY2_DRV_Pin GPIO_PIN_6
#define RLY2_DRV_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOH
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOH
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOH
#define UART3_TXEN_Pin GPIO_PIN_12
#define UART3_TXEN_GPIO_Port GPIOD
#define ETH_INTN_Pin GPIO_PIN_13
#define ETH_INTN_GPIO_Port GPIOD
#define DO3_Pin GPIO_PIN_2
#define DO3_GPIO_Port GPIOG
#define DO4_Pin GPIO_PIN_3
#define DO4_GPIO_Port GPIOG
#define CP_S_01_Pin GPIO_PIN_6
#define CP_S_01_GPIO_Port GPIOC
#define SD_CD_Pin GPIO_PIN_7
#define SD_CD_GPIO_Port GPIOC
#define DI1_Pin GPIO_PIN_9
#define DI1_GPIO_Port GPIOA
#define DI2_Pin GPIO_PIN_11
#define DI2_GPIO_Port GPIOA
#define DI3_Pin GPIO_PIN_12
#define DI3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_13
#define LED4_GPIO_Port GPIOH
#define ETH_RESETN_Pin GPIO_PIN_1
#define ETH_RESETN_GPIO_Port GPIOI
#define HEARTBEAT_Pin GPIO_PIN_4
#define HEARTBEAT_GPIO_Port GPIOD
#define LCD_RESETN_Pin GPIO_PIN_7
#define LCD_RESETN_GPIO_Port GPIOD
#define DO5_Pin GPIO_PIN_11
#define DO5_GPIO_Port GPIOG
#define DO6_Pin GPIO_PIN_13
#define DO6_GPIO_Port GPIOG
#define RLY1_DRV_Pin GPIO_PIN_14
#define RLY1_DRV_GPIO_Port GPIOG
#define DI4_Pin GPIO_PIN_4
#define DI4_GPIO_Port GPIOB
#define PWM0_TIM3_CH2_Pin GPIO_PIN_5
#define PWM0_TIM3_CH2_GPIO_Port GPIOB
#define BL_CTRL_Pin GPIO_PIN_6
#define BL_CTRL_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
