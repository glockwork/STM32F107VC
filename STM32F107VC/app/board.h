/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_H
#define __BOARD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/******************************* Define for CC1120 ****************************/
#define CC1120_SPI                  SPI1
#define GPIO_Port_CC1120_SPI        GPIOA
#define GPIO_Port_CC1120_NSS        GPIOA
#define GPIO_Port_CC1120_RESET      GPIOC
#define GPIO_Pin_CC1120_SCK         GPIO_Pin_5
#define GPIO_Pin_CC1120_MOSI        GPIO_Pin_6
#define GPIO_Pin_CC1120_MISO        GPIO_Pin_7
#define GPIO_Pin_CC1120_NSS         GPIO_Pin_4
#define GPIO_Pin_CC1120_RESET       GPIO_Pin_9

#endif /* __BOARD_H */
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
