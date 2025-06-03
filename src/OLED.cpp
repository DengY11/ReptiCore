/*
 * SSD1306 OLED显示驱动实现
 * OLED.cpp
 */

#include "OLED.h"
#include <stdio.h>

// 外部变量
extern I2C_HandleTypeDef hi2c1;
extern SensorData_t sensorData;
extern ControlConfig_t controlConfig;

// OLED显存缓冲区
static uint8_t OLED_Buffer[OLED_WIDTH * OLED_PAGES];

// 8x16字体数据表 (简化版，只包含数字和常用字符)
static const uint8_t Font8x16[][16] = {
    // 字符 '0' (0x30)
    {0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00},
    // 字符 '1' (0x31)
    {0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00},
    // 字符 '2' (0x32)
    {0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00},
    // 字符 '3' (0x33)
    {0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00},
    // 字符 '4' (0x34)
    {0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00},
    // 字符 '5' (0x35)
    {0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00},
    // 字符 '6' (0x36)
    {0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00},
    // 字符 '7' (0x37)
    {0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00},
    // 字符 '8' (0x38)
    {0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00},
    // 字符 '9' (0x39)
    {0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00},
    // 字符 '.' (0x2E)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00},
    // 字符 '°' (度符号)
    {0x00,0x00,0x00,0x18,0x24,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // 字符 'C'
    {0x00,0xE0,0x10,0x08,0x08,0x10,0x20,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x08,0x00},
    // 字符 '%'
    {0x00,0xF0,0x08,0x08,0xF0,0x00,0xC0,0x30,0x00,0x18,0x26,0x21,0x1E,0x02,0x01,0x00},
    // 字符 ' ' (空格)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    // 字符 ':'
    {0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00}
};

// 字符到字体索引的映射
static uint8_t GetFontIndex(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    switch (ch) {
        case '.': return 10;
        case 'C': return 12;
        case '%': return 13;
        case ' ': return 14;
        case ':': return 15;
        default: 
            // 处理度符号（UTF-8编码：0xC2 0xB0）
            if ((uint8_t)ch == 0xB0) return 11; // 度符号
            return 14; // 默认返回空格
    }
}

// OLED初始化
OLED_Status_t OLED_Init(void)
{
    HAL_Delay(100); // 等待OLED稳定

    // 发送初始化命令序列
    OLED_WriteCommand(OLED_CMD_DISPLAY_OFF);
    OLED_WriteCommand(OLED_CMD_SET_CLOCK_DIV);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(OLED_CMD_SET_MULTIPLEX);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(OLED_CMD_SET_DISPLAY_OFFSET);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(OLED_CMD_SET_START_LINE | 0x00);
    OLED_WriteCommand(OLED_CMD_CHARGE_PUMP);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(OLED_CMD_MEMORY_MODE);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(OLED_CMD_SEGMENT_REMAP | 0x01);
    OLED_WriteCommand(OLED_CMD_COM_SCAN_DEC);
    OLED_WriteCommand(OLED_CMD_SET_COM_PINS);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(OLED_CMD_SET_CONTRAST);
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(OLED_CMD_SET_PRECHARGE);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(OLED_CMD_SET_VCOM_DETECT);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(OLED_CMD_ENTIRE_DISPLAY_ON);
    OLED_WriteCommand(OLED_CMD_NORMAL_DISPLAY);
    OLED_WriteCommand(OLED_CMD_DISPLAY_ON);

    // 清空显存
    OLED_Clear();
    OLED_Refresh();

    return OLED_OK;
}

// 清空显存
OLED_Status_t OLED_Clear(void)
{
    memset(OLED_Buffer, 0, sizeof(OLED_Buffer));
    return OLED_OK;
}

// 刷新显示
OLED_Status_t OLED_Refresh(void)
{
    OLED_WriteCommand(OLED_CMD_SET_COLUMN_ADDR);
    OLED_WriteCommand(0);
    OLED_WriteCommand(127);
    OLED_WriteCommand(OLED_CMD_SET_PAGE_ADDR);
    OLED_WriteCommand(0);
    OLED_WriteCommand(7);

    return OLED_WriteData(OLED_Buffer, sizeof(OLED_Buffer));
}

// 设置像素点
OLED_Status_t OLED_SetPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) {
        return OLED_ERROR;
    }

    if (color) {
        OLED_Buffer[x + (y / 8) * OLED_WIDTH] |= 1 << (y % 8);
    } else {
        OLED_Buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }

    return OLED_OK;
}

// 显示字符串
OLED_Status_t OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t fontSize)
{
    while (*str) {
        OLED_DrawChar(x, y, *str, fontSize);
        x += 8; // 字符宽度为8像素
        if (x >= OLED_WIDTH - 8) break;
        str++;
    }
    return OLED_OK;
}

// 显示数字
OLED_Status_t OLED_ShowNumber(uint8_t x, uint8_t y, float number, uint8_t precision, uint8_t fontSize)
{
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%.*f", precision, number);
    return OLED_ShowString(x, y, buffer, fontSize);
}

// 显示温度信息
OLED_Status_t OLED_ShowTemperature(float temperature, float target)
{
    char buffer[32];
    
    // 显示当前温度
    OLED_ShowString(0, 0, "温度:", FONT_SIZE_16);
    snprintf(buffer, sizeof(buffer), "%.1f°C", temperature);
    OLED_ShowString(40, 0, buffer, FONT_SIZE_16);
    
    // 显示目标温度
    OLED_ShowString(0, 16, "目标:", FONT_SIZE_16);
    snprintf(buffer, sizeof(buffer), "%.1f°C", target);
    OLED_ShowString(40, 16, buffer, FONT_SIZE_16);
    
    return OLED_OK;
}

// 显示湿度信息
OLED_Status_t OLED_ShowHumidity(float humidity, float target)
{
    char buffer[32];
    
    // 显示当前湿度
    OLED_ShowString(0, 32, "湿度:", FONT_SIZE_16);
    snprintf(buffer, sizeof(buffer), "%.1f%%", humidity);
    OLED_ShowString(40, 32, buffer, FONT_SIZE_16);
    
    // 显示目标湿度
    OLED_ShowString(0, 48, "目标:", FONT_SIZE_16);
    snprintf(buffer, sizeof(buffer), "%.1f%%", target);
    OLED_ShowString(40, 48, buffer, FONT_SIZE_16);
    
    return OLED_OK;
}

// 显示系统状态
OLED_Status_t OLED_ShowSystemStatus(void)
{
    // 在右侧显示继电器状态
    uint8_t x = 90;
    
    // 加热器状态
    if (HAL_GPIO_ReadPin(RELAY_HEATER_Port, RELAY_HEATER_Pin)) {
        OLED_ShowString(x, 0, "H", FONT_SIZE_16);
    }
    
    // 风扇状态
    if (HAL_GPIO_ReadPin(RELAY_FAN_Port, RELAY_FAN_Pin)) {
        OLED_ShowString(x + 10, 0, "F", FONT_SIZE_16);
    }
    
    // 雾化器状态
    if (HAL_GPIO_ReadPin(RELAY_HUMIDIFIER_Port, RELAY_HUMIDIFIER_Pin)) {
        OLED_ShowString(x + 20, 0, "M", FONT_SIZE_16);
    }
    
    return OLED_OK;
}

// 写入命令
OLED_Status_t OLED_WriteCommand(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd}; // 0x00表示命令
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS, data, 2, 100);
    return (status == HAL_OK) ? OLED_OK : OLED_ERROR;
}

// 写入数据
OLED_Status_t OLED_WriteData(uint8_t *data, uint16_t len)
{
    uint8_t buffer[len + 1];
    buffer[0] = 0x40; // 0x40表示数据
    memcpy(&buffer[1], data, len);
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS, buffer, len + 1, 1000);
    return (status == HAL_OK) ? OLED_OK : OLED_ERROR;
}

// 绘制字符
void OLED_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t fontSize)
{
    uint8_t fontIndex = GetFontIndex(ch);
    
    if (fontSize == FONT_SIZE_16) {
        const uint8_t *fontData = Font8x16[fontIndex];
        
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                if (fontData[i] & (0x80 >> j)) {
                    OLED_SetPixel(x + j, y + i, 1);
                }
            }
        }
        
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                if (fontData[i + 8] & (0x80 >> j)) {
                    OLED_SetPixel(x + j, y + i + 8, 1);
                }
            }
        }
    }
} 