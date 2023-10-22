/*!
    \brief:
    \Author: libowen 
    \Date: 2023年4月7日
    \File: uart.c
 */

#include "at32f421.h"
#include <stdio.h>
#include "uart.h"
//#include "FreeRTOS.h"
//#include "task.h"
#include "ball_game.h"

static uint16_t timecnt;
static uint8_t rx_byte;

uint16_t tx_size = 0;
uint16_t tx_count = 0;
const uint8_t *tx_p = 0;
uint8_t tx_ing = 2;

void uart_ring_buff_init(void);
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int uart_init(void)
{
    uart_ring_buff_init();
//    if (HAL_UART_Receive_IT(&UartHandle, &rx_byte, 1) != HAL_OK) {
//        __NOP();
//    }
		usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    return 0;
}

typedef struct
{
    uint16_t Head;           
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_LEN];
} ring_buff_t;
ring_buff_t ring_buf;//创建一个ring_buf的缓冲区

/*!
    \brief  uart_ring_buff_init
    \param  void
    \return void
    \note   初始化环形缓冲区
*/
void uart_ring_buff_init(void)
{
    //初始化相关信息
    ring_buf.Head = 0;
    ring_buf.Tail = 0;
    ring_buf.Lenght = 0;
}

/*!
    \brief  uart_wr_ring_buff
    \param  uint8_t data
    \return FLASE:环形缓冲区已满，写入失败;TRUE:写入成功
    \note   往环形缓冲区写入uint8_t类型的数据
*/
uint8_t uart_wr_ring_buff(uint8_t data)
{
    if(ring_buf.Lenght >= RINGBUFF_LEN)                 // 判断缓冲区是否已满
    {
        return RINGBUFF_ERR;
    }
    
    ring_buf.Ring_data[ring_buf.Tail] = data;
    ring_buf.Tail = (ring_buf.Tail+1) % RINGBUFF_LEN;   // 防止越界非法访问
    
    //taskDISABLE_INTERRUPTS();
    ring_buf.Lenght++;                                  // 注意临界问题
    //taskENABLE_INTERRUPTS();
    
    return RINGBUFF_OK;
}

/*!
    \brief  Read_RingBuff
    \param  uint8_t *rData，用于保存读取的数据
    \return FLASE:环形缓冲区没有数据，读取失败;TRUE:读取成功
    \note   从环形缓冲区读取一个u8类型的数据
*/
uint8_t uart_rd_ring_buff(uint8_t *rData)
{
    if(ring_buf.Lenght == 0)                            // 判断非空
    {
        return RINGBUFF_ERR;
    }
    
    *rData = ring_buf.Ring_data[ring_buf.Head];         // 先进先出FIFO，从缓冲区头出
    ring_buf.Head = (ring_buf.Head+1) % RINGBUFF_LEN;   // 防止越界非法访问
    
    // 注意临界问题：执行到这里的时候，发生了中断
    __disable_irq();//taskDISABLE_INTERRUPTS();
    ring_buf.Lenght--;                                  // 注意临界问题
    __enable_irq();//taskENABLE_INTERRUPTS();
    
    return RINGBUFF_OK;
}


/*!
    \brief      读取接收到的有效数据数量
    \param[in]  none
    \param[out] none
    \retval     接收到的有效数据数量
    \notice     
*/
uint8_t uart_data_available(void)
{
    uint8_t ret = 0;
    
    __disable_irq();//taskDISABLE_INTERRUPTS();
    ret = ring_buf.Lenght;                              // 注意临界问题
    __enable_irq();//taskENABLE_INTERRUPTS();
    
    return ret;
}


/*!
    \brief      串口发送完后，过一会才失能发送，否则可能发送不完整，波特率越低，所需时间越长。
                此函数在定时器中断里面周期性调用。
    \param[in]  none
    \param[out] none
    \retval     none
    \notice     TODO: 根据波特率调整时间
*/
void uart_dis_tx_en_rx(void)
{
    static uint16_t cnt = 0;
    if (tx_ing == 1) {
        if (cnt++ >= 2) {
            cnt = 0;
            tx_ing = 2;
            RS485_TX_DISABLE();
        }
    }
}

/*!
    \brief      通过串口发送数据
    \param[in]  ch，数据指针
    \param[in]  len，数据长度
    \param[out] none
    \retval     none
    \notice     
*/
int uart_send_data(const uint8_t *ch, uint16_t len)
{
    if (tx_ing != 2) {
        return -1;
    }

    RS485_TX_ENABLE();
    
    __disable_irq();//taskDISABLE_INTERRUPTS();
    tx_ing = 0;
		tx_size = len;
		tx_p = ch;
		tx_count = 0;
    __enable_irq();//taskENABLE_INTERRUPTS();
    
    // HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)ch, len);
		usart_interrupt_enable(USART1, USART_TDBE_INT, TRUE);
		
		return 0;
}

/*!
    \brief      启动接收超时计时
    \param[in]  ms，超时设为ms
    \param[out] none
    \retval     0-成功
    \notice     
*/
int8_t uart_rx_process_start_cnt(uint16_t ms)
{
    __disable_irq();//taskDISABLE_INTERRUPTS();
    timecnt = ms;
    __enable_irq();//taskENABLE_INTERRUPTS();
    
    return 0;
}

/*!
    \brief      判断接收是否超时了
    \param[in]  none
    \param[out] none
    \retval     0-未超时，1-超时了
    \notice     
*/
int8_t uart_rx_process_is_time_out(void)
{
    uint16_t tmp;
    
    __disable_irq();//taskDISABLE_INTERRUPTS();
    tmp = timecnt;
    __enable_irq();//taskENABLE_INTERRUPTS();
    
    if (tmp == 0x8000) {
        __disable_irq();//taskDISABLE_INTERRUPTS();
        timecnt = 0;
        __enable_irq();//taskENABLE_INTERRUPTS();
        return 1;
    }
    return 0;
}

/*!
    \brief      超时计时。在定时器中断里周期调用。
    \param[in]  none
    \param[out] none
    \retval     none
    \notice     
*/
void uart_rx_process_tmr_cnt(void)
{
    if (timecnt & ~0x8000) {
        timecnt--;
        if (timecnt == 0) {
            timecnt = 0x8000;
        }
    }
}

typedef struct com_callback_t {
  uint8_t CMD;
  uint8_t (*Callback)(uint8_t *buf);
} sCOMCallback_t;

const sCOMCallback_t CallbackTable[] = {
  {0x3E, com_rx_cb_play_finished},
  {0x41, com_rx_cb_cmd_respone},
};

/* 定义数据包接收状态的变量,并初始化为空闲状态 */
eRXDataGramState_t rx_state = eIDLE;
uint8_t rx_idx = 0;
uint8_t DataBuf[MAX_DATA_LEN];

// 美养头是crc-16-modbus校验
// 协议数据处理函数
// 该段代码来自于marlin固件
int com_rx_parse(void)
{
    uint8_t receivedbyte;

    while (uart_rd_ring_buff(&receivedbyte)) {   // 缓存中有数据
        if (uart_rx_process_is_time_out()) {           // 超时没有走完流程，重置状态机
            rx_state = eIDLE;
        }
        if ((rx_state == eIDLE) || (rx_state == eHEADER1)) {
            rx_idx = 0;
        }
        DataBuf[rx_idx++] = receivedbyte;

        uart_rx_process_start_cnt(5);
        switch(rx_state) {
            case eIDLE:
            case eHEADER1:
                if (HEADER1 == receivedbyte) {   // TODO: 启动超时检测，如果收到数据头后，一段时间内没有走完所有状态机，那么要重新开始  
                    rx_state = eHEADER2;
                }
                break;
            case eHEADER2:
                rx_state = (HEADER2 == receivedbyte) ? eDEV_ADDR : eIDLE;
                break;
            case eDEV_ADDR:
                rx_state = (DEV_ADDR == receivedbyte) ? eCMD : eIDLE;
                break;
            case eCMD: // 3E 播放完成 41 命令响应 
            {
								int i;
                rx_state = eResponeOrNo;
                for (i=0; i<sizeof(CallbackTable)/sizeof(CallbackTable[0]); i++) {
                    if ((DataBuf[3] == CallbackTable[i].CMD) && (CallbackTable[i].Callback != NULL)) {
                        uint8_t len = CallbackTable[i].Callback(&DataBuf[5]);
                    }
                }
                break;
            }
                break;
            case eResponeOrNo:
                rx_state = eDATA;
            break;
            case eDATA:
            case eTAIL:
                if (receivedbyte == TAIL) {
                    rx_state = eIDLE;
                }
                if (rx_idx >= 20) {
                    rx_state = eIDLE;
                    break;
                }
                break;
            default:
                rx_state = eIDLE;
                break;
        }
    }
    return 0;
}


/**
  * @brief  this function handles usart1 handler.
  * @param  none
  * @retval none
  */
void USART1_IRQHandler(void)
{
  if(USART1->ctrl1_bit.rdbfien != RESET)
  {
    if(usart_flag_get(USART1, USART_RDBF_FLAG) != RESET)
    {
      /* read one byte from the receive data register */
      //usart1_rx_buffer[usart1_rx_counter++] = usart_data_receive(USART1);
			uint8_t byte = usart_data_receive(USART1);
			uart_wr_ring_buff(byte);

      //if(usart1_rx_counter == usart2_tx_buffer_size)
      {
        /* disable the usart1 receive interrupt */
      //  usart_interrupt_enable(USART1, USART_RDBF_INT, FALSE);
      }
    }
  }
  
  if(USART1->ctrl1_bit.tdbeien != RESET)
  {
    if(usart_flag_get(USART1, USART_TDBE_FLAG) != RESET)
    {
      /* write one byte to the transmit data register */
      usart_data_transmit(USART1, tx_p[tx_count++]);

      if(tx_count >= tx_size)
      {
        /* disable the usart1 transmit interrupt */
        usart_interrupt_enable(USART1, USART_TDBE_INT, FALSE);
				tx_ing = 1;
      }
    }
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
//  /* Set transmission flag: transfer complete */
//  //UartReady = SET;

//  tx_ing = 1;
//  
//}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
//  /* Set transmission flag: transfer complete */
//  //UartReady = SET;
//  /* receive data */
//  uart_wr_ring_buff(rx_byte);
//  HAL_UART_Receive_IT(UartHandle, &rx_byte, 1);
//}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
//{
//    //Error_Handler();
//}
