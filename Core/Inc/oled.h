#ifndef __OLED_H__
#define __OLED_H__

#include "main.h"
#include <string.h>
#include <stdio.h>

/* SSD1306 I2C 地址 (7-bit) */
#define OLED_ADDR       0x3C

/* OLED 尺寸 */
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      8

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t intLen, uint8_t decLen, uint8_t size);
void OLED_Printf(uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...);

#endif /* __OLED_H__ */
