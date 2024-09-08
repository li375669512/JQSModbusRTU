// 2022年10月21日 23点46分

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

// 7E --- 起始命令 
// FF --- 版本信息 
// 06 --- 数据长度(不包含校验) 
// 03 --- 代表产品编号 
// 00 --- 是否需要应答[0x01:需要应答，0x00:不需要返回应答] 
// 00 --- 曲目的高字节[DH] 
// 01 --- 曲目的低字节[DL],这里代表的是第一首歌播放 
// FF --- 校验的高字节 
// E6 --- 校验的低字节 
// EF --- 结束命令
// 01文件夹001文件
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
    
    minute = 5;                       // 默认为五分钟后再播放音乐
    
    return ret;
}



// 指定音量
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

// 停止播放
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
        minute = 1;                                    	// 收到了音乐播放完成反馈，你们2分钟后再播放背景音乐
    }
    if (music_played) {
        minute = 2; 
    }
    last_play_tick = HAL_GetTick();          						// 播放时间重置
    return 0;
}

uint8_t com_rx_cb_cmd_respone(uint8_t *buf)
{
    return 0;
}

// 周期回调
void ball_game_period_callback(void)
{
		// 收到了游戏结束信号
		if (key_ball_sig() == 1) {
        last_play_tick = HAL_GetTick();     										// 重置音乐播放时间
        effect_played = 1;
      
        if (music_played) {																			// 播放背景音乐中，先停止播放，然后把声音设大
            music_played = 0;
            sound_stop();
            delay_ms(50);
            sound_set_volume(30);
            delay_ms(50);
        }
        
        score_get = 1;
				sound_play_sndx(2, abs(rand())%11 + 1);	  							// 播放音效
		}
    
    // 音效或者音乐播放完后，过一阵子再播放音乐。
    // 如果检测到是声音播放完成，那么接下来一分钟后开始播放音乐
    // 否则，五分钟后再播放，之所以是五分钟，是因为考虑到大部分音乐时间不超过五分钟，
    // 这样避免背景音乐播放的过程中被打断。
    if ((HAL_GetTick() - last_play_tick) > 60*1000*minute) {
        last_play_tick = HAL_GetTick();
        music_played = 1;
        
        if (effect_played) {									  								// 播放进球音效中，先停止播放，然后把声音设小，避免正在播放的音效声音听起来突然变小
            effect_played = 0;
            sound_stop();
            delay_ms(50);
            sound_set_volume(20);
            delay_ms(50);
        }
        
				sound_play_sndx(1, abs(rand())%MUSIC_NUM + 1);					// 播放背景音乐
    }
    
    static uint8_t led_idle_step;
    if (score_get) {																						// 得分后，LED闪烁
        
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
    } else {                                                    // 空闲的时候，慢慢闪烁
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
