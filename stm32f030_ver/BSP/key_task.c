/*
 * @Description:
 * @Author: libowen 
 * @Date: 2022-08-24 14:43:17
 * @File: key_task.c
 */
 
#include "bsp.h"
#include "key.h"
#include "string.h"
#include "uart.h"
#include "cmsis_os.h"

/* �����ͷ��ָ����Ϣ */
#define HEADER1         0x48    // ��ͷ
#define HEADER2         0x5A
#define DEV_ADDR        0x02    // �豸��ַ
#define TAIL1           0xEE    // ��β
#define TAIL2           0xFF    // 

#define MAX_DATA_LEN    100

/* ö�ٶ�ȡ���ݱ��ĵ�״̬ */
typedef enum
{
    eIDLE,
    eHEADER1,
    eHEADER2,
    eDEV_ADDR,
    eDATA_LEN,
    eRESERVE,
    eDATA,
    eDATA_CRC16_H,
    eDATA_CRC16_L,
    eTAIL1,
    eTAIL2,
}eRXDataGramState_t;

static osThreadId processRxTaskHandle;
static void process_rx_task(const void *p); 

extern QueueHandle_t keyQueue;

int com_rx_parse(void);

void key_detec_task(const void *args)
{
    osThreadDef(processRx_task, process_rx_task, osPriorityBelowNormal, 0, 256);
    processRxTaskHandle = osThreadCreate(osThread(processRx_task), 0);
    while (1)
    {
    }
}


static void process_rx_task(const void *p)
{
    int ret;
    while (1) {
        ret = com_rx_parse();
        if (ret != 0) {
            
        }
        osDelay(1);
    }
}

/*!
    \brief    ����ModBus�������CRC16
    \param[in]
        _pBuf:���������ݻ�����,����õ��Ľ������_pBuf��������ֽ�
        _usLen:���������ݳ���(�ֽ���)
    \return   16λУ��ֵ
  */
static uint16_t modbus_crc16(uint8_t *_pBuf, uint16_t _usLen)
{
    uint16_t CRCValue = 0xFFFF;           //��ʼ��CRC������λΪ1
    uint8_t i,j;

    for (i=0; i<_usLen; ++i) {
        CRCValue ^= *(_pBuf+i);                     //��ǰ�������CRC���ֽ�
        for(j=0;j<8;++j) {                          //һ���ֽ��ظ�����8��
            if((CRCValue & 0x01) == 0x01) {         //�ж�����ǰ���λ�Ƿ�Ϊ1
                 CRCValue = (CRCValue >> 1)^0xA001; //���Ϊ1�����Ʋ������ʽ
            } else {
                CRCValue >>= 1;                     //����ֱ������һλ
            }           
        }
    } 
    return CRCValue;
} 

uint8_t com_set_level(uint8_t *buf)
{
    uint8_t len = 1;
    buf[len++] = 0;

    return len;
}

uint8_t com_get_state(uint8_t *buf)
{
    uint8_t len = 1;
    buf[len++] = 0;
    buf[len++] = 0;
    return len;
}

typedef struct com_callback_t {
  uint8_t CMD;
  uint8_t (*Callback)(uint8_t *buf);
} sCOMCallback_t;

sCOMCallback_t CallbackTable[] = {
  {0x03, com_set_level},
  {0xC0, com_get_state},
};

/* �������ݰ�����״̬�ı���,����ʼ��Ϊ����״̬ */
eRXDataGramState_t rx_state = eIDLE;
uint8_t rx_idx = 0;
uint8_t DataLen = 0;
uint16_t CRC16 = 0;
uint16_t CRC16Rev = 0;
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
                rx_state = (DEV_ADDR == receivedbyte) ? eDATA_LEN : eIDLE;
                break;
            case eDATA_LEN:
                DataLen = receivedbyte;  //���ݰ��ĳ���
                if(MAX_DATA_LEN < DataLen) {
                    rx_state = eIDLE;
                } else {
                    rx_state = eRESERVE;
                }
                break;
            case eRESERVE:
                rx_state = eDATA;
            break;
            case eDATA:
                if (rx_idx >= (DataLen+5)) {
                    CRC16 = modbus_crc16(&DataBuf[2], DataLen+3);
                    rx_state = eDATA_CRC16_H;
                    break;
                }
                break;
            case eDATA_CRC16_H:
                CRC16Rev = receivedbyte << 8;
                rx_state = eDATA_CRC16_L;
                break;
            case eDATA_CRC16_L:
                CRC16Rev |= receivedbyte;
                if ((CRC16 == CRC16Rev) || (CRC16Rev == 0xaaaa)) {
                    rx_state = eTAIL1;
                } else {
                    rx_state = eIDLE;
                }
                break;
            case eTAIL1:
                rx_state = (TAIL1 == receivedbyte) ? eTAIL2 : eIDLE;
                break;
            case eTAIL2:
            {
                uint8_t cb_ok = 0;
                rx_state = eIDLE;
                if (TAIL2 != receivedbyte) { 
                    break;
                }
                for (int i=0; i<sizeof(CallbackTable)/sizeof(CallbackTable[0]); i++) {
                    if ((DataBuf[5] == CallbackTable[i].CMD) && (CallbackTable[i].Callback != NULL)) {
                        uint8_t len = CallbackTable[i].Callback(&DataBuf[5]);
                        DataBuf[0] = HEADER1;   DataBuf[1] = HEADER2;
                        DataBuf[2] = DEV_ADDR;  DataBuf[3] = len;
                        DataBuf[4] = 0x00;
                        uint16_t crc = 0;
                        crc = modbus_crc16(&DataBuf[5], len);
                        DataBuf[5+len] = crc>>8;  DataBuf[6+len] = crc;
                        DataBuf[7+len] = TAIL1;   DataBuf[8+len] = TAIL2;
                        uart_send_data(DataBuf, len+5+4);
                        cb_ok = 1;
                    }
                }
                if (!cb_ok) {
                    //buzzer_control(0.5, 100*(2.0/5), 2);
                }
                break;
            }
            default:
                rx_state = eIDLE;
                break;
        }
    }
    return 0;
}

