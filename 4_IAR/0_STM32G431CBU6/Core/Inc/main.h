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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "SEGGER_RTT.h"	// библиотека для отладочного RTT вывода через интерфейс отладчика
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
										/* настройка голых портов ввода/вывода */
#define LED_ON()   				        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_OFF()  				        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)

										/* макросы для RTT */
#ifdef SEGGER_RTT_H
#define LOG_START()					SEGGER_RTT_WriteString(0, RTT_CTRL_CLEAR RTT_CTRL_RESET RTT_CTRL_TEXT_BRIGHT_GREEN); 		/* очистка терминала и установка стандартных параметров оформления */
#define LOG_FILE()					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "\r\n\r\nFILE: " __FILE__ RTT_CTRL_TEXT_BRIGHT_GREEN); 	/* путь к текущему файлу */
#define LOGT(text)					SEGGER_RTT_WriteString(0, text); 															/* простой вывод текста */
#define LOGF(format, args...)			        SEGGER_RTT_printf(0, "\r\n%d " format, __LINE__, args); 											/* форматирваный вывод текста с номером строки в нулевой терминал */
#define LOG(text)					SEGGER_RTT_printf(0, "\r\n%d %s", __LINE__, text); 											/* вывод текста с номером строки в нулевой терминал */
#define LOG0(text)					SEGGER_RTT_printf(0, "\r\n%d %s", __LINE__, text);
#define LOG1(text)					SEGGER_RTT_printf(1, "\r\n%d %s", __LINE__, text);	 
#define LOG2(text)					SEGGER_RTT_printf(2, "\r\n%d %s", __LINE__, text);
#define LOG3(text)					SEGGER_RTT_printf(3, "\r\n%d %s", __LINE__, text);
#define LOG4(text)					SEGGER_RTT_printf(4, "\r\n%d %s", __LINE__, text);
#define LOG5(text)					SEGGER_RTT_printf(5, "\r\n%d %s", __LINE__, text);
#define LOG_NMI()					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED	"\r\n\r\n\t +------------------+\r\n\t |       NMI        |\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
#define LOG_PVD()					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED	"\r\n\r\n\t +------------------+\r\n\t |       PVD        |\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
#define LOG_HARDFAULT()					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED	"\r\n\r\n\t +------------------+\r\n\t |    HARD FAULT    |\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
#define LOG_MEMMANAGE()					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED	"\r\n\r\n\t +------------------+\r\n\t |    MEM MANAGE    |\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
#define LOG_BUSFAULT()					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED	"\r\n\r\n\t +------------------+\r\n\t |    BUS  FAULT    |\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
#define LOG_USAGEFAULT()				SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED	"\r\n\r\n\t +------------------+\r\n\t |    USAGE FAULT   |\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
#define LOG_HALERROR()					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED	"\r\n\r\n\t +------------------+\r\n\t |    HAL  ERROR    |\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
#define LOG_POWEROFF()					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_RED	"\r\n\r\n\t +------------------+\r\n\t |    POWER  OFF    |\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
//#define LOG_LCD(L1, L2)					SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN	"\r\n\r\n\t +------------------+"); SEGGER_RTT_printf(0,  "\r\n\t | %16.16s |", L1); SEGGER_RTT_printf(0, "\r\n\t | %16.16s |", L2); SEGGER_RTT_WriteString(0, "\r\n\t +------------------+\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN);
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
