// 2022��10��21�� 23��46��

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

// ����澯�ź�
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

// ��Ϸ�����ź�
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

// ��Ƭ�ͳ��ź�
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

// ���ڻص�
void card_machine_period_callback(void)
{
		// �յ�����Ϸ�����ź�
		if (card_machine_get_game_over_sig() & 0x01)
		{
				current_card_val = 1;	// �ͳ�һ�ſ�Ƭ
		}
    
    if (card_machine_get_clear_alert_sig() & 0x01)
    {
        __NOP();
    }

		// �յ�������澯�źţ�����澯֮�󣬻����ִ���ͳ���������
		if (card_err == eCardErrTimeOut)
		{
				if (card_machine_get_clear_alert_sig() & 0x01)
				{
						card_err = eCardErrNone;
						ALERT_LED_OFF();
				}
	  }
		
		// û�п�Ƭ�������д�����ôֱ���˳�
    if ((card_err != eCardErrNone) || (current_card_val == 0))
		{
				return;
		}
		
		// ����п�ƬҪ�ͳ�	
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
        if (card_time >= 2000)		// �ͳ���ʱ
				{
             CARD_OUT_STOP();
						 ALERT_LED_ON();
             card_time = 0;
             card_err = eCardErrTimeOut;
        }
    }
		else													// �޿�ƬҪ�ͳ�
		{
				CARD_OUT_STOP();
				card_time = 0;
    }
}
