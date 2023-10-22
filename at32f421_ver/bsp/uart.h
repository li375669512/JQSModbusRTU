/*!
    \brief:
    \Author: libowen 
    \Date: 2023年4月7日
    \File: uart.h
 */
 
#ifndef __UART_H__
#define __UART_H__

#include "at32f421.h"
#include <stdio.h>
#include "bsp.h"

#define RS485_TX_ENABLE()                gpio_bits_write(RS485_CTR_GPIO_Port, RS485_CTR_Pin, TRUE)//bsp_gpio_write(RS485_TR_CTR, SET)
#define RS485_TX_DISABLE()               gpio_bits_write(RS485_CTR_GPIO_Port, RS485_CTR_Pin, FALSE)//bsp_gpio_write(RS485_TR_CTR, RESET)
#define RS485_TX_ENABLE_IO()             (gpio_input_data_bit_read(RS485_CTR_GPIO_Port, RS485_CTR_Pin))// bsp_gpio_read(RS485_TR_CTR) && 

#define  RINGBUFF_LEN          (30)     //定义最大接收字节数 500
#define  RINGBUFF_OK           1     
#define  RINGBUFF_ERR          0   



/* 定义包头及指令信息 */
#define HEADER1         0x7E    // 包头
#define HEADER2         0xFF
#define DEV_ADDR        0x06    // 设备地址
#define TAIL            0xEF    // 包尾

#define MAX_DATA_LEN    20

/* 枚举读取数据报文的状态 */
typedef enum
{
    eIDLE,
    eHEADER1,
    eHEADER2,
    eDEV_ADDR,
    eCMD,
    eResponeOrNo,
    eDATA,
    eTAIL,
}eRXDataGramState_t;

int uart_init(void);
uint8_t uart_rd_ring_buff(uint8_t *rData);
uint8_t uart_wr_ring_buff(uint8_t data);
uint8_t uart_data_available(void);
int8_t uart_rx_process_start_cnt(uint16_t ms);
int8_t uart_rx_process_is_time_out(void);
void uart_rx_process_tmr_cnt(void);
void uart_dis_tx_en_rx(void);
int uart_send_data(const uint8_t *ch, uint16_t len);
int com_rx_parse(void);

#endif
