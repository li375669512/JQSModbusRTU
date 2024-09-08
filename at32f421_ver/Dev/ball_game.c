// 2022��10��21�� 23��46��

#include <stdlib.h>
#include <string.h>
#include "ball_game.h"
#include "BSP.h"
#include "key.h"
#include "uart.h"

#define LED_ON()				(gpio_bits_write(Relay3_GPIO_Port, Relay3_Pin, TRUE))
#define LED_OFF()				(gpio_bits_write(Relay3_GPIO_Port, Relay3_Pin, FALSE))

#define EFFECT_SND_NUM	11
#define MUSIC_NUM				2


enum eErrCode
{
		eErrNone,
		eErrTimeOut,
};

static enum eErrCode game_err = eErrNone; 

uint8_t effect_played = 1, music_played, score_get, led_step, minute = 2;
uint32_t last_play_tick = -60*1000*2-1000, last_score_tick;
static uint8_t tx_buf[10];

void ball_game_init(void)
{
}

// 7E --- ��ʼ���� 
// FF --- �汾��Ϣ 
// 06 --- ���ݳ���(������У��) 
// 03 --- �����Ʒ��� 
// 00 --- �Ƿ���ҪӦ��[0x01:��ҪӦ��0x00:����Ҫ����Ӧ��] 
// 00 --- ��Ŀ�ĸ��ֽ�[DH] 
// 01 --- ��Ŀ�ĵ��ֽ�[DL],���������ǵ�һ�׸貥�� 
// FF --- У��ĸ��ֽ� 
// E6 --- У��ĵ��ֽ� 
// EF --- ��������
// 01�ļ���001�ļ�
// 7E FF 06 0F 00 01 01 EF	
int sound_play_sndx(uint8_t f, uint8_t x)
{
    int ret;
  
    const uint8_t cmd[] = {
      0x7E, 0xFF, 0x06, 0x0F, 0x01, 0x01, 0x01, 0xEF
    };
    
    memcpy(tx_buf, cmd, sizeof(cmd));
    
    tx_buf[5] = f;
    tx_buf[6] = x;
    
    ret = uart_send_data(tx_buf, sizeof(cmd));
    
    minute = 5;                       // Ĭ��Ϊ����Ӻ��ٲ�������
    
    return ret;
}



// ָ������
// 7E FF 06 00 00 00 1E EF	
int sound_set_volume(uint8_t x)
{
    int ret;
  
    const uint8_t cmd[] = {
      0x7E, 0xFF, 0x06, 0x06, 0x01, 0x00, 0x01, 0xEF
    };
    
    memcpy(tx_buf, cmd, sizeof(cmd));

    tx_buf[6] = x;
    
    ret = uart_send_data(tx_buf, sizeof(cmd));
    
    return ret;
} 

// ֹͣ����
// 7E FF 06 16 00 00 1E EF	
int sound_stop(void)
{
    int ret;
  
    const uint8_t cmd[] = {
      0x7E, 0xFF, 0x06, 0x0E, 0x00, 0x00, 0x00, 0xEF
    };
    
    memcpy(tx_buf, cmd, sizeof(cmd));

    ret = uart_send_data(tx_buf, sizeof(cmd));
    
    return ret;
}


uint8_t com_rx_cb_play_finished(uint8_t *buf)
{
    if (effect_played) {
        minute = 1;                                    	// �յ������ֲ�����ɷ���������2���Ӻ��ٲ��ű�������
    }
    if (music_played) {
        minute = 2; 
    }
    last_play_tick = HAL_GetTick();          						// ����ʱ������
    return 0;
}

uint8_t com_rx_cb_cmd_respone(uint8_t *buf)
{
    return 0;
}

// ���ڻص�
void ball_game_period_callback(void)
{
		// �յ�����Ϸ�����ź�
		if (key_ball_sig() == 1) {
        last_play_tick = HAL_GetTick();     										// �������ֲ���ʱ��
        effect_played = 1;
      
        if (music_played) {																			// ���ű��������У���ֹͣ���ţ�Ȼ����������
            music_played = 0;
            sound_stop();
            delay_ms(50);
            sound_set_volume(30);
            delay_ms(50);
        }
        
        score_get = 1;
				sound_play_sndx(2, abs(rand())%11 + 1);	  							// ������Ч
		}
    
    // ��Ч�������ֲ�����󣬹�һ�����ٲ������֡�
    // �����⵽������������ɣ���ô������һ���Ӻ�ʼ��������
    // ��������Ӻ��ٲ��ţ�֮����������ӣ�����Ϊ���ǵ��󲿷�����ʱ�䲻��������ӣ�
    // �������ⱳ�����ֲ��ŵĹ����б���ϡ�
    if ((HAL_GetTick() - last_play_tick) > 60*1000*minute) {
        last_play_tick = HAL_GetTick();
        music_played = 1;
        
        if (effect_played) {									  								// ���Ž�����Ч�У���ֹͣ���ţ�Ȼ���������С���������ڲ��ŵ���Ч����������ͻȻ��С
            effect_played = 0;
            sound_stop();
            delay_ms(50);
            sound_set_volume(20);
            delay_ms(50);
        }
        
				sound_play_sndx(1, abs(rand())%MUSIC_NUM + 1);					// ���ű�������
    }
    
    static uint8_t led_idle_step;
    if (score_get) {																						// �÷ֺ�LED��˸
        
        if ((HAL_GetTick() - last_score_tick) > 200) {
            last_score_tick = HAL_GetTick();
            led_step %= 4;
            led_step++;
            if (led_step%2 == 0) {
                LED_OFF();
            } else {
                LED_ON();
            }
            if (led_step >= 4) {
                score_get = 0;
                led_idle_step = 0;
            }
        }
    } else {                                                    // ���е�ʱ��������˸
        static uint32_t last_led_tick = 0;
        static uint8_t sw;
        
        if (led_idle_step == 0) {
            if ((HAL_GetTick() - last_led_tick) >= 3000) {
                // last_led_tick = HAL_GetTick();
                led_idle_step = 1;
                sw = 0;
            }
        } else if (led_idle_step == 1) {
            if ((HAL_GetTick() - last_led_tick) >= 3000) {
                last_led_tick = HAL_GetTick();
                sw ^= 1;
                if (sw) {
                    LED_ON();
                } else {
                    LED_OFF();
                }
            }
        }
    }
}
