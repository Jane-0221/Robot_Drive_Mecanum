/**
 * @file music.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief 音乐播放模块
 * @version 0.1
 * @date 2025-04-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "music.h"

// ===== 《两只老虎》旋律数组 =====
Note melody[] = {
    // 第一段：主题重复
    {MID_DO, 500},
    {MID_RE, 500},
    {MID_MI, 500},
    {MID_DO, 500}, // 1 2 3 1
    {MID_DO, 500},
    {MID_RE, 500},
    {MID_MI, 500},
    {MID_DO, 500}, // 1 2 3 1
    {MID_MI, 500},
    {MID_FA, 500},
    {MID_SOL, 1000}, // 3 4 5 -
    {MID_MI, 500},
    {MID_FA, 500},
    {MID_SOL, 1000}, // 3 4 5 -

    // 第二段：旋律展开
    {MID_SOL, 250},
    {MID_LA, 250},
    {MID_SOL, 250},
    {MID_FA, 250},
    {MID_MI, 500},
    {MID_DO, 500}, // 5 6 5 4 3 1
    {MID_SOL, 250},
    {MID_LA, 250},
    {MID_SOL, 250},
    {MID_FA, 250},
    {MID_MI, 500},
    {MID_DO, 500}, // 5 6 5 4 3 1

    // 结尾部分：回到主音
    {MID_DO, 500},
    {HIGH_SOL, 500},
    {MID_DO, 1000}, // 1 5 1 -
    {MID_DO, 500},
    {HIGH_SOL, 500},
    {MID_DO, 1000}, // 1 5 1 -
    {MUSIC_FINISH,500},
};

// ===== 《56个民族》旋律数组 =====
Note melody_56_nations[] = {
    // 前奏部分
    {MID_SOL, 500},
    {MID_LA, 500},
    {HIGH_DO, 500},
    {MID_LA, 500}, // 5 6 1 6
    {MID_SOL, 500},
    {MID_MI, 500},
    {MID_SOL, 500},
    {MID_LA, 500}, // 5 3 5 6
    
    // 主旋律第一部分
    {HIGH_DO, 500},
    {MID_LA, 500},
    {MID_SOL, 500},
    {MID_MI, 500}, // 1 6 5 3
    {MID_SOL, 500},
    {MID_MI, 500},
    {MID_RE, 500},
    {MID_DO, 500}, // 5 3 2 1
    
    // 主旋律第二部分
    {MID_DO, 500},
    {MID_RE, 500},
    {MID_MI, 500},
    {MID_SOL, 500}, // 1 2 3 5
    {MID_MI, 500},
    {MID_RE, 500},
    {MID_DO, 500},
    {MID_RE, 500}, // 3 2 1 2
    
    // 高潮部分
    {MID_SOL, 500},
    {HIGH_DO, 500},
    {MID_LA, 500},
    {HIGH_DO, 500}, // 5 1 6 1
    {HIGH_RE, 500},
    {HIGH_DO, 500},
    {MID_LA, 500},
    {MID_SOL, 500}, // 2 1 6 5
    
    // 结尾部分
    {MID_MI, 500},
    {MID_SOL, 500},
    {MID_LA, 500},
    {HIGH_DO, 1000}, // 3 5 6 1
    {MID_SOL, 500},
    {MID_LA, 500},
    {HIGH_DO, 1000}, // 5 6 1
    {MUSIC_FINISH,500},
};

void Music_init()
{
    Buzzer_init();
}

void Music_play(Note *note)
{
    for (int i = 0; note[i].fre!=MUSIC_FINISH; i++)
    {
        MUSIC_SET_FREQUENCY(note[i].fre);
        MUSIC_ON();
        MUSIC_DELAY(note[i].time);
    }
    MUSIC_OFF();
}

/**
 * @brief 播放《两只老虎》歌曲
 */
void Music_play_two_tigers()
{
    Music_play(melody);
}

/**
 * @brief 播放《56个民族》歌曲
 */
void Music_play_56_nations()
{
    Music_play(melody_56_nations);
}
