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

/* 定义包头及指令信息 */
#define HEADER1         0x48    // 包头
#define HEADER2         0x5A
#define DEV_ADDR        0x02    // 设备地址
#define TAIL1           0xEE    // 包尾
#define TAIL2           0xFF    // 

#define MAX_DATA_LEN    100

/* 枚举读取数据报文的状态 */
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
    \brief    根据ModBus规则计算CRC16
    \param[in]
        _pBuf:待计算数据缓冲区,计算得到的结果存入_pBuf的最后两字节
        _usLen:待计算数据长度(字节数)
    \return   16位校验值
  */
static uint16_t modbus_crc16(uint8_t *_pBuf, uint16_t _usLen)
{
    uint16_t CRCValue = 0xFFFF;           //初始化CRC变量各位为1
    uint8_t i,j;

    for (i=0; i<_usLen; ++i) {
        CRCValue ^= *(_pBuf+i);                     //当前数据异或CRC低字节
        for(j=0;j<8;++j) {                          //一个字节重复右移8次
            if((CRCValue & 0x01) == 0x01) {         //判断右移前最低位是否为1
                 CRCValue = (CRCValue >> 1)^0xA001; //如果为1则右移并异或表达式
            } else {
                CRCValue >>= 1;                     //否则直接右移一位
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

/* 定义数据包接收状态的变量,并初始化为空闲状态 */
eRXDataGramState_t rx_state = eIDLE;
uint8_t rx_idx = 0;
uint8_t DataLen = 0;
uint16_t CRC16 = 0;
uint16_t CRC16Rev = 0;
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
                rx_state = (DEV_ADDR == receivedbyte) ? eDATA_LEN : eIDLE;
                break;
            case eDATA_LEN:
                DataLen = receivedbyte;  //数据包的长度
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

