/*
 * SSD1306 OLED显示驱动实现文件
 * OLED.cpp
 * C实现 + C++包裹层
 */

#include "OLED.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 外部变量声明
extern "C" {
extern I2C_HandleTypeDef hi2c1;
}

// C++命名空间包裹C实现
namespace ReptileController::Display {

// === 纯C实现部分 ===

// OLED显示缓冲区
static uint8_t s_displayBuffer[OLED_WIDTH * OLED_PAGES];

// 8x8字体数据（简化版，只包含数字和常用字符）
static const uint8_t s_font8x8[][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // 空格 (32)
    {0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00}, // ! (33)
    {0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // " (34)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // # (35) - 占位
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // $ (36) - 占位
    {0x00, 0x66, 0x66, 0x0C, 0x18, 0x33, 0x33, 0x00}, // % (37)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // & (38) - 占位
    {0x06, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00}, // ' (39)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ( (40) - 占位
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ) (41) - 占位
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // * (42) - 占位
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // + (43) - 占位
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x06}, // , (44)
    {0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00}, // - (45)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00}, // . (46)
    {0x00, 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x00}, // / (47)
    {0x00, 0x3E, 0x63, 0x67, 0x6B, 0x73, 0x3E, 0x00}, // 0 (48)
    {0x00, 0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x3F, 0x00}, // 1 (49)
    {0x00, 0x1E, 0x33, 0x30, 0x1C, 0x06, 0x3F, 0x00}, // 2 (50)
    {0x00, 0x1E, 0x33, 0x30, 0x1C, 0x33, 0x1E, 0x00}, // 3 (51)
    {0x00, 0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x00}, // 4 (52)
    {0x00, 0x3F, 0x03, 0x1F, 0x30, 0x33, 0x1E, 0x00}, // 5 (53)
    {0x00, 0x1C, 0x06, 0x03, 0x1F, 0x33, 0x1E, 0x00}, // 6 (54)
    {0x00, 0x3F, 0x30, 0x18, 0x0C, 0x06, 0x06, 0x00}, // 7 (55)
    {0x00, 0x1E, 0x33, 0x33, 0x1E, 0x33, 0x1E, 0x00}, // 8 (56)
    {0x00, 0x1E, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00}, // 9 (57)
    {0x00, 0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00}, // : (58)
};

// 获取字符在字体数组中的索引
static uint8_t getCharIndex(char ch) {
    if (ch >= 32 && ch <= 58) {
        return ch - 32;
    }
    // 对于中文字符的简单处理，返回特定字符
    switch (ch) {
        case 'C': case 'c': return 35; // 用于温度显示
        default: return 0; // 默认空格
    }
}

// 纯C函数：写命令
static OLED_Status_t writeCommand_C(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd}; // 0x00表示命令
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        &hi2c1, 0x78, data, 2, 100);
    return (status == HAL_OK) ? OLED_OK : OLED_ERROR;
}

// 纯C函数：写数据
static OLED_Status_t writeData_C(const uint8_t* buffer, uint16_t len) {
    if (!buffer || len == 0) return OLED_ERROR;
    
    // 动态分配临时缓冲区
    uint8_t* tempBuffer = (uint8_t*)malloc(len + 1);
    if (!tempBuffer) return OLED_ERROR;
    
    tempBuffer[0] = 0x40; // 0x40表示数据
    memcpy(&tempBuffer[1], buffer, len);
    
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        &hi2c1, 0x78, tempBuffer, len + 1, 1000);
    
    free(tempBuffer);
    return (status == HAL_OK) ? OLED_OK : OLED_ERROR;
}

// 纯C函数：绘制字符
static void drawChar_C(uint8_t x, uint8_t y, char ch, uint8_t fontSize) {
    const uint8_t charIndex = getCharIndex(ch);
    const uint8_t* charData = s_font8x8[charIndex];
    
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t line = charData[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (0x80 >> j)) {
                if (fontSize == 16) {
                    // 16号字体 - 2x2像素
                    for (uint8_t dx = 0; dx < 2; dx++) {
                        for (uint8_t dy = 0; dy < 2; dy++) {
                            uint16_t px = x + j * 2 + dx;
                            uint16_t py = y + i * 2 + dy;
                            if (px < OLED_WIDTH && py < OLED_HEIGHT) {
                                uint16_t page = py / 8;
                                uint8_t bit = py % 8;
                                s_displayBuffer[px + page * OLED_WIDTH] |= (1 << bit);
                            }
                        }
                    }
                } else {
                    // 8号字体
                    uint16_t px = x + j;
                    uint16_t py = y + i;
                    if (px < OLED_WIDTH && py < OLED_HEIGHT) {
                        uint16_t page = py / 8;
                        uint8_t bit = py % 8;
                        s_displayBuffer[px + page * OLED_WIDTH] |= (1 << bit);
                    }
                }
            }
        }
    }
}

// 纯C函数：格式化浮点数
static void formatFloat_C(char* buffer, size_t bufferSize, float number, uint8_t precision) {
    if (!buffer || bufferSize == 0) return;
    
    int intPart = (int)number;
    float fracPart = number - intPart;
    
    if (precision == 0) {
        snprintf(buffer, bufferSize, "%d", intPart);
    } else {
        int multiplier = 1;
        for (uint8_t i = 0; i < precision; i++) {
            multiplier *= 10;
        }
        int fracInt = (int)(fracPart * multiplier);
        snprintf(buffer, bufferSize, "%d.%d", intPart, fracInt);
    }
}

// === C++包裹层 ===

// C++接口类
class OLEDController {
public:
    static OLED_Status_t initialize() {
        HAL_Delay(100);
        
        // 初始化命令序列
        const uint8_t initCommands[] = {
            OLED_CMD_DISPLAY_OFF,
            OLED_CMD_SET_CLOCK_DIV, 0x80,
            OLED_CMD_SET_MULTIPLEX, 0x3F,
            OLED_CMD_SET_DISPLAY_OFFSET, 0x00,
            OLED_CMD_SET_START_LINE | 0x00,
            OLED_CMD_CHARGE_PUMP, 0x14,
            OLED_CMD_MEMORY_MODE, 0x00,
            OLED_CMD_SEGMENT_REMAP | 0x01,
            OLED_CMD_COM_SCAN_DEC,
            OLED_CMD_SET_COM_PINS, 0x12,
            OLED_CMD_SET_CONTRAST, 0xCF,
            OLED_CMD_SET_PRECHARGE, 0xF1,
            OLED_CMD_SET_VCOM_DETECT, 0x40,
            OLED_CMD_ENTIRE_DISPLAY_ON,
            OLED_CMD_NORMAL_DISPLAY,
            OLED_CMD_DISPLAY_ON
        };
        
        for (size_t i = 0; i < sizeof(initCommands); i++) {
            if (writeCommand_C(initCommands[i]) != OLED_OK) {
                return OLED_ERROR;
            }
        }
        
        return clear();
    }
    
    static OLED_Status_t clear() {
        memset(s_displayBuffer, 0, sizeof(s_displayBuffer));
        return OLED_OK;
    }
    
    static OLED_Status_t refresh() {
        // 设置列地址
        if (writeCommand_C(OLED_CMD_SET_COLUMN_ADDR) != OLED_OK) return OLED_ERROR;
        if (writeCommand_C(0) != OLED_OK) return OLED_ERROR;
        if (writeCommand_C(OLED_WIDTH - 1) != OLED_OK) return OLED_ERROR;
        
        // 设置页地址
        if (writeCommand_C(OLED_CMD_SET_PAGE_ADDR) != OLED_OK) return OLED_ERROR;
        if (writeCommand_C(0) != OLED_OK) return OLED_ERROR;
        if (writeCommand_C(OLED_PAGES - 1) != OLED_OK) return OLED_ERROR;
        
        // 发送显示缓冲区
        return writeData_C(s_displayBuffer, sizeof(s_displayBuffer));
    }
    
    static OLED_Status_t setPixel(uint8_t x, uint8_t y, uint8_t color) {
        if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return OLED_ERROR;
        
        uint16_t page = y / 8;
        uint8_t bit = y % 8;
        uint16_t index = x + page * OLED_WIDTH;
        
        if (color) {
            s_displayBuffer[index] |= (1 << bit);
        } else {
            s_displayBuffer[index] &= ~(1 << bit);
        }
        
        return OLED_OK;
    }
    
    static OLED_Status_t showString(uint8_t x, uint8_t y, const char* str, uint8_t fontSize) {
        if (!str) return OLED_ERROR;
        
        uint8_t currentX = x;
        const uint8_t charWidth = (fontSize == 16) ? 16 : 8;
        
        while (*str && currentX + charWidth <= OLED_WIDTH) {
            drawChar_C(currentX, y, *str, fontSize);
            currentX += charWidth;
            str++;
        }
        
        return OLED_OK;
    }
    
    static OLED_Status_t showNumber(uint8_t x, uint8_t y, float number, uint8_t precision, uint8_t fontSize) {
        char buffer[32];
        formatFloat_C(buffer, sizeof(buffer), number, precision);
        return showString(x, y, buffer, fontSize);
    }
    
    static OLED_Status_t showTemperature(float temperature, float target) {
        showString(0, 0, "Temp:", 16);
        showNumber(48, 0, temperature, 1, 16);
        showString(96, 0, "C", 16);
        
        showString(0, 16, "Targ:", 16);
        showNumber(48, 16, target, 1, 16);
        showString(96, 16, "C", 16);
        
        return OLED_OK;
    }
    
    static OLED_Status_t showHumidity(float humidity, float target) {
        showString(0, 32, "Humi:", 16);
        showNumber(48, 32, humidity, 1, 16);
        showString(96, 32, "%", 16);
        
        showString(0, 48, "Targ:", 16);
        showNumber(48, 48, target, 1, 16);
        showString(96, 48, "%", 16);
        
        return OLED_OK;
    }
    
    static OLED_Status_t showSystemStatus() {
        static uint32_t counter = 0;
        counter++;
        
        const char* indicator = "*";
        if (counter % 4 == 1) indicator = ".";
        else if (counter % 4 == 2) indicator = "*";
        else if (counter % 4 == 3) indicator = ".";
        
        return showString(120, 0, indicator, 8);
    }
};

// 全局实例
static OLEDController g_oledController;

}  // namespace ReptileController::Display

// === C接口实现 ===
extern "C" {

OLED_Status_t OLED_Init(void) {
    return ReptileController::Display::OLEDController::initialize();
}

OLED_Status_t OLED_Clear(void) {
    return ReptileController::Display::OLEDController::clear();
}

OLED_Status_t OLED_Refresh(void) {
    return ReptileController::Display::OLEDController::refresh();
}

OLED_Status_t OLED_SetPixel(uint8_t x, uint8_t y, uint8_t color) {
    return ReptileController::Display::OLEDController::setPixel(x, y, color);
}

OLED_Status_t OLED_ShowString(uint8_t x, uint8_t y, const char* str, uint8_t fontSize) {
    return ReptileController::Display::OLEDController::showString(x, y, str, fontSize);
}

OLED_Status_t OLED_ShowNumber(uint8_t x, uint8_t y, float number, uint8_t precision, uint8_t fontSize) {
    return ReptileController::Display::OLEDController::showNumber(x, y, number, precision, fontSize);
}

OLED_Status_t OLED_ShowTemperature(float temperature, float target) {
    return ReptileController::Display::OLEDController::showTemperature(temperature, target);
}

OLED_Status_t OLED_ShowHumidity(float humidity, float target) {
    return ReptileController::Display::OLEDController::showHumidity(humidity, target);
}

OLED_Status_t OLED_ShowSystemStatus(void) {
    return ReptileController::Display::OLEDController::showSystemStatus();
}

OLED_Status_t OLED_WriteCommand(uint8_t cmd) {
    return ReptileController::Display::writeCommand_C(cmd);
}

OLED_Status_t OLED_WriteData(uint8_t* data, uint16_t len) {
    return ReptileController::Display::writeData_C(data, len);
}

void OLED_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t fontSize) {
    ReptileController::Display::drawChar_C(x, y, ch, fontSize);
}

}  // extern "C"