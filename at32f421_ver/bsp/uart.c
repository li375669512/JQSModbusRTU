/*!
    \brief:
    \Author: libowen 
    \Date: 2023��4��7��
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
ring_buff_t ring_buf;//����һ��ring_buf�Ļ�����

/*!
    \brief  uart_ring_buff_init
    \param  void
    \return void
    \note   ��ʼ�����λ�����
*/
void uart_ring_buff_init(void)
{
    //��ʼ�������Ϣ
    ring_buf.Head = 0;
    ring_buf.Tail = 0;
    ring_buf.Lenght = 0;
}

/*!
    \brief  uart_wr_ring_buff
    \param  uint8_t data
    \return FLASE:���λ�����������д��ʧ��;TRUE:д��ɹ�
    \note   �����λ�����д��uint8_t���͵�����
*/
uint8_t uart_wr_ring_buff(uint8_t data)
{
    if(ring_buf.Lenght >= RINGBUFF_LEN)                 // �жϻ������Ƿ�����
    {
        return RINGBUFF_ERR;
    }
    
    ring_buf.Ring_data[ring_buf.Tail] = data;
    ring_buf.Tail = (ring_buf.Tail+1) % RINGBUFF_LEN;   // ��ֹԽ��Ƿ�����
    
    //taskDISABLE_INTERRUPTS();
    ring_buf.Lenght++;                                  // ע���ٽ�����
    //taskENABLE_INTERRUPTS();
    
    return RINGBUFF_OK;
}

/*!
    \brief  Read_RingBuff
    \param  uint8_t *rData�����ڱ����ȡ������
    \return FLASE:���λ�����û�����ݣ���ȡʧ��;TRUE:��ȡ�ɹ�
    \note   �ӻ��λ�������ȡһ��u8���͵�����
*/
uint8_t uart_rd_ring_buff(uint8_t *rData)
{
    if(ring_buf.Lenght == 0)                            // �жϷǿ�
    {
        return RINGBUFF_ERR;
    }
    
    *rData = ring_buf.Ring_data[ring_buf.Head];         // �Ƚ��ȳ�FIFO���ӻ�����ͷ��
    ring_buf.Head = (ring_buf.Head+1) % RINGBUFF_LEN;   // ��ֹԽ��Ƿ�����
    
    // ע���ٽ����⣺ִ�е������ʱ�򣬷������ж�
    __disable_irq();//taskDISABLE_INTERRUPTS();
    ring_buf.Lenght--;                                  // ע���ٽ�����
    __enable_irq();//taskENABLE_INTERRUPTS();
    
    return RINGBUFF_OK;
}


/*!
    \brief      ��ȡ���յ�����Ч��������
    \param[in]  none
    \param[out] none
    \retval     ���յ�����Ч��������
    \notice     
*/
uint8_t uart_data_available(void)
{
    uint8_t ret = 0;
    
    __disable_irq();//taskDISABLE_INTERRUPTS();
    ret = ring_buf.Lenght;                              // ע���ٽ�����
    __enable_irq();//taskENABLE_INTERRUPTS();
    
    return ret;
}


/*!
    \brief      ���ڷ�����󣬹�һ���ʧ�ܷ��ͣ�������ܷ��Ͳ�������������Խ�ͣ�����ʱ��Խ����
                �˺����ڶ�ʱ���ж����������Ե��á�
    \param[in]  none
    \param[out] none
    \retval     none
    \notice     TODO: ���ݲ����ʵ���ʱ��
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
    \brief      ͨ�����ڷ�������
    \param[in]  ch������ָ��
    \param[in]  len�����ݳ���
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
    \brief      �������ճ�ʱ��ʱ
    \param[in]  ms����ʱ��Ϊms
    \param[out] none
    \retval     0-�ɹ�
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
    \brief      �жϽ����Ƿ�ʱ��
    \param[in]  none
    \param[out] none
    \retval     0-δ��ʱ��1-��ʱ��
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
    \brief      ��ʱ��ʱ���ڶ�ʱ���ж������ڵ��á�
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

/* �������ݰ�����״̬�ı���,����ʼ��Ϊ����״̬ */
eRXDataGramState_t rx_state = eIDLE;
uint8_t rx_idx = 0;
uint8_t DataBuf[MAX_DATA_LEN];

// ����ͷ��crc-16-modbusУ��
// Э�����ݴ�����
// �öδ���������marlin�̼�
int com_rx_parse(void)
{
    uint8_t receivedbyte;

    while (uart_rd_ring_buff(&receivedbyte)) {   // ������������
        if (uart_rx_process_is_time_out()) {           // ��ʱû���������̣�����״̬��
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
                if (HEADER1 == receivedbyte) {   // TODO: ������ʱ��⣬����յ�����ͷ��һ��ʱ����û����������״̬������ôҪ���¿�ʼ  
                    rx_state = eHEADER2;
                }
                break;
            case eHEADER2:
                rx_state = (HEADER2 == receivedbyte) ? eDEV_ADDR : eIDLE;
                break;
            case eDEV_ADDR:
                rx_state = (DEV_ADDR == receivedbyte) ? eCMD : eIDLE;
                break;
            case eCMD: // 3E ������� 41 ������Ӧ 
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
