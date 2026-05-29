#include "LED.h"
#include "ws2812.h"

void hexToRGB(uint32_t hex, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = (hex >> 16) & 0xFF; // 提取红色分量
    *g = (hex >> 8) & 0xFF;  // 提取绿色分量
    *b = hex & 0xFF;         // 提取蓝色分量
}

/**
 * @brief LED灯显示颜色
 *
 * @param color 颜色的rgb编码
 */
void LEDshowcolor(uint32_t color)
{
    uint8_t r, g, b;
    hexToRGB(color, &r, &g, &b);
    WS2812_Ctrl(r, g, b);
}

//void LEDchangecolor(uint32_t)