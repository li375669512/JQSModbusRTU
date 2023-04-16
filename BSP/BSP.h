// 2022年10月21日 23点46分

#ifndef __BSP_H__
#define __BSP_H__

#include "main.h"

//#define LED0_Pin GPIO_PIN_0
//#define LED0_GPIO_Port GPIOA
//#define IN1_Pin GPIO_PIN_1
//#define IN1_GPIO_Port GPIOA
//#define IN2_Pin GPIO_PIN_2
//#define IN2_GPIO_Port GPIOA
//#define IN3_Pin GPIO_PIN_3
//#define IN3_GPIO_Port GPIOA
//#define BUTTON_Pin GPIO_PIN_4
//#define BUTTON_GPIO_Port GPIOA
//#define Relay3_Pin GPIO_PIN_6
//#define Relay3_GPIO_Port GPIOA
//#define Relay2_Pin GPIO_PIN_7
//#define Relay2_GPIO_Port GPIOA
//#define Relay1_Pin GPIO_PIN_1
//#define Relay1_GPIO_Port GPIOB

#define CARD_OUT_SIG			  IN1_GPIO_Port, IN1_Pin
#define GAME_OVER_SIG				IN2_GPIO_Port, IN2_Pin
#define CLEAR_ALERT_SIG     IN3_GPIO_Port, IN3_Pin


#define ALERT_LED_CTR				Relay3_GPIO_Port, Relay3_Pin
#define CARD_OUT_CTR				Relay2_GPIO_Port, Relay2_Pin

/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated 
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF1_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF1_USART1

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

extern UART_HandleTypeDef UartHandle;

#endif
