// 2022��10��21�� 23��46��

#ifndef __BALL_GAME_H__
#define __BALL_GAME_H__

#include "BSP.h"

void ball_game_period_callback(void);

uint8_t com_rx_cb_play_finished(uint8_t *buf);

uint8_t com_rx_cb_cmd_respone(uint8_t *buf);

#endif
