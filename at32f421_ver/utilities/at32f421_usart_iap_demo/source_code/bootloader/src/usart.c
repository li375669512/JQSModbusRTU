/**
  **************************************************************************
  * @file     usart.c
  * @brief    usart program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "usart.h"
#include "tmr.h"

/** @addtogroup UTILITIES_examples
  * @{
  */

/** @addtogroup USART_IAP_bootloader
  * @{
  */

usart_group_type usart_group_struct;

/**
  * @brief  init usart.
  * @param  none
  * @retval none
  */
void uart_init(uint32_t baudrate)
{
  gpio_init_type gpio_init_struct;
  /* enable the usart and it's io clock */
  crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  /* set default parameter */
  gpio_default_para_init(&gpio_init_struct);

  /* configure the usart1_tx/rx pa9/pa10 */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_9 | GPIO_PINS_10;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* config usart1 iomux */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);

  /*configure usart nvic interrupt */
  nvic_irq_enable(USART1_IRQn, 0, 0);

  /*configure usart param*/
  usart_init(USART1, baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART1, TRUE);
  usart_receiver_enable(USART1, TRUE);
  usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
  usart_enable(USART1, TRUE);
}

/**
  * @brief  usart1 interrupt handler.
  * @param  none
  * @retval none
  */
void USART1_IRQHandler(void)
{
  uint16_t reval;
  time_ira_cnt = 0;  /* clear upgrade time out flag */
  if(usart_flag_get(USART1, USART_RDBF_FLAG) != RESET)
  {
    reval = usart_data_receive(USART1);
    if(usart_group_struct.count > (USART_REC_LEN - 1))
    {
      usart_group_struct.count = 0;
      usart_group_struct.head = 0;
      usart_group_struct.tail = 0;
    }
    else
    {
      usart_group_struct.count++;
      usart_group_struct.buf[usart_group_struct.head++] = reval;
      if(usart_group_struct.head > (USART_REC_LEN - 1))
      {
        usart_group_struct.head = 0;
      }
    }
  }
}

/**
  * @}
  */

/**
  * @}
  */
