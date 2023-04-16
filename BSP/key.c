/*!
    \brief:
    \Author: libowen 
    \Date: 2023年4月3日
    \File: key.c
 */
 
#include "key.h"

void key_init(void)
{
}


// 游戏结束信号
uint8_t key_ball_sig(void)
{
    uint8_t ret = 0;
    static uint8_t evt = 0;
    static uint16_t hcnt, lcnt;
  
    if (HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin) == RESET)
    {
        hcnt = 0;
        if (lcnt < 10000) lcnt++;
        if (lcnt == 5)
        {
            evt = 1;
        }
    }
    else
    {
        lcnt = 0;
        if (hcnt++ >= 5)
        {
            ret = evt;
            evt = 0;
        }
    }
    return ret;
}
