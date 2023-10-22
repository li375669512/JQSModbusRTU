// 2022年10月21日 23点46分

#include "CardMachine.h"
#include "BSP.h"

#define CARD_OUT_IS_RUN()			(HAL_GPIO_ReadPin(CARD_OUT_CTR) == SET)

#define CARD_OUT_SIG_GET()		(HAL_GPIO_ReadPin(CARD_OUT_SIG) == RESET)
#define CARD_OUT_RUN()				(HAL_GPIO_WritePin(CARD_OUT_CTR, GPIO_PIN_SET))
#define CARD_OUT_STOP()				(HAL_GPIO_WritePin(CARD_OUT_CTR, GPIO_PIN_RESET))

#define CLEAR_ALERT_SIG_GET()	(HAL_GPIO_ReadPin(CLEAR_ALERT_SIG) == RESET)

#define GAME_OVER_SIG_GET()		(HAL_GPIO_ReadPin(GAME_OVER_SIG) == SET)

#define ALERT_LED_ON()				(HAL_GPIO_WritePin(ALERT_LED_CTR, GPIO_PIN_SET))
#define ALERT_LED_OFF()				(HAL_GPIO_WritePin(ALERT_LED_CTR, GPIO_PIN_RESET))


enum eCardErrCode
{
		eCardErrNone,
		eCardErrTimeOut,
};

static uint8_t current_card_val = 0;
static uint16_t card_time = 0;
static enum eCardErrCode card_err = eCardErrNone;

void card_machine_init(void)
{
		CARD_OUT_STOP();
		ALERT_LED_OFF();
}

// 清除告警信号
static _Bool card_machine_get_clear_alert_sig(void)
{
    static uint16_t state = 0; // Current debounce status
    uint8_t ret = 0;
    
    //if (CARD_OUT_IS_RUN())
	  {
        state = (state<<1) | !CLEAR_ALERT_SIG_GET() | 0xff00;
      
        if ( state == 0xfff0 )
        {
            __NOP();
            ret = 1;
        }
    }
    
    return ret;
}

// 游戏结束信号
static _Bool card_machine_get_game_over_sig(void)
{
    static uint16_t state = 0; // Current debounce status
    
    //if (CARD_OUT_IS_RUN())
	  {
        state = (state<<1) | !GAME_OVER_SIG_GET() | 0xff00;
      
        if ( state == 0xfff0 )
            return 1;
    }
    
    return 0;
}

// 卡片送出信号
static _Bool card_machine_get_sig(void)
{
    static uint16_t state = 0; // Current debounce status
    
    if (CARD_OUT_IS_RUN())
	  {
        state = (state<<1) | !CARD_OUT_SIG_GET() | 0xff00;
      
        if ( state == 0xfff0 )
            return 1;
    }
    
    return 0;
}

// 周期回调
void card_machine_period_callback(void)
{
		// 收到了游戏结束信号
		if (card_machine_get_game_over_sig() & 0x01)
		{
				current_card_val = 1;	// 送出一张卡片
		}
    
    if (card_machine_get_clear_alert_sig() & 0x01)
    {
        __NOP();
    }

		// 收到了清除告警信号，清除告警之后，会继续执行送出控制流程
		if (card_err == eCardErrTimeOut)
		{
				if (card_machine_get_clear_alert_sig() & 0x01)
				{
						card_err = eCardErrNone;
						ALERT_LED_OFF();
				}
	  }
		
		// 没有卡片，或者有错误，那么直接退出
    if ((card_err != eCardErrNone) || (current_card_val == 0))
		{
				return;
		}
		
		// 如果有卡片要送出	
    if (current_card_val > 0)
		{
				card_time++;
        CARD_OUT_RUN();
        if (card_machine_get_sig() & 0x01)	
			  {
            current_card_val--;
            card_time = 0;
            if (current_card_val == 0)
						{
                CARD_OUT_STOP();
            }
        }
        if (card_time >= 2000)		// 送出超时
				{
             CARD_OUT_STOP();
						 ALERT_LED_ON();
             card_time = 0;
             card_err = eCardErrTimeOut;
        }
    }
		else													// 无卡片要送出
		{
				CARD_OUT_STOP();
				card_time = 0;
    }
}
