/*
 * SSD1306 OLED显示驱动头文件
 * OLED.h
 */

#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// OLED显示屏参数
#define OLED_WIDTH 128  // OLED宽度
#define OLED_HEIGHT 64  // OLED高度
#define OLED_PAGES 8    // OLED页数 (64/8)

// OLED I2C命令
#define OLED_CMD_DISPLAY_OFF 0xAE
#define OLED_CMD_DISPLAY_ON 0xAF
#define OLED_CMD_SET_CONTRAST 0x81
#define OLED_CMD_ENTIRE_DISPLAY_ON 0xA5
#define OLED_CMD_NORMAL_DISPLAY 0xA6
#define OLED_CMD_INVERSE_DISPLAY 0xA7
#define OLED_CMD_SET_MULTIPLEX 0xA8
#define OLED_CMD_SET_DISPLAY_OFFSET 0xD3
#define OLED_CMD_SET_START_LINE 0x40
#define OLED_CMD_CHARGE_PUMP 0x8D
#define OLED_CMD_MEMORY_MODE 0x20
#define OLED_CMD_SEGMENT_REMAP 0xA1
#define OLED_CMD_COM_SCAN_DEC 0xC8
#define OLED_CMD_SET_COM_PINS 0xDA
#define OLED_CMD_SET_PRECHARGE 0xD9
#define OLED_CMD_SET_VCOM_DETECT 0xDB
#define OLED_CMD_SET_CLOCK_DIV 0xD5
#define OLED_CMD_SET_COLUMN_ADDR 0x21
#define OLED_CMD_SET_PAGE_ADDR 0x22

// 字体大小
#define FONT_SIZE_8 8
#define FONT_SIZE_16 16

// OLED状态枚举
typedef enum { OLED_OK = 0, OLED_ERROR } OLED_Status_t;

// 函数声明
OLED_Status_t OLED_Init(void);
OLED_Status_t OLED_Clear(void);
OLED_Status_t OLED_Refresh(void);
OLED_Status_t OLED_SetPixel(uint8_t x, uint8_t y, uint8_t color);
OLED_Status_t OLED_ShowString(uint8_t x, uint8_t y, const char *str,
                              uint8_t fontSize);
OLED_Status_t OLED_ShowNumber(uint8_t x, uint8_t y, float number,
                              uint8_t precision, uint8_t fontSize);
OLED_Status_t OLED_ShowTemperature(float temperature, float target);
OLED_Status_t OLED_ShowHumidity(float humidity, float target);
OLED_Status_t OLED_ShowSystemStatus(void);

// 内部函数
OLED_Status_t OLED_WriteCommand(uint8_t cmd);
OLED_Status_t OLED_WriteData(uint8_t *data, uint16_t len);
void OLED_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t fontSize);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */