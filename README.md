# STM32 温湿度智能控制系统（基于 STM32F407 + FreeRTOS）

## 📋 项目简介

本项目是基于 **STM32F407VET6** 开发板、使用 **FreeRTOS 实时操作系统** 和 **HAL 驱动库** 实现的一个温湿度智能控制系统。项目采用 **C++** 开发，在 PlatformIO + VS Code 环境下编译，具有良好的可扩展性与模块化设计，适用于智能农业、孵化器控制、小型环境监测等场景。

---

## 🧩 功能特性

### 核心功能
- **DHT22 传感器**：高精度温湿度采集（±0.1°C/±0.1%RH）
- **SSD1306 OLED显示**：128x64像素实时显示温湿度数据和系统状态
- **智能继电器控制**：
  - 加热垫控制（PC13）- 温度调节
  - 风扇控制（PC14）- 散热/通风
  - 雾化器控制（PC15）- 湿度调节
- **PID智能控制算法**：精确的温湿度闭环控制
- **FreeRTOS多任务调度**：传感器采集、显示更新、控制逻辑并行运行

### 安全特性
- **过热保护**：加热器工作时间限制和强制冷却
- **防干烧保护**：雾化器定时停止机制
- **错误检测**：传感器故障检测和处理
- **紧急停止**：异常情况下的安全关断

### 控制模式
- **自动模式**：基于PID算法的智能控制
- **手动模式**：用户手动控制各设备
- **关闭模式**：安全关闭所有设备

---

## 📁 项目结构

```
TempControl/
├── src/                        # 源文件目录
│   ├── main.cpp               # 主程序入口 + FreeRTOS任务
│   ├── DHT22.cpp              # DHT22温湿度传感器驱动
│   ├── OLED.cpp               # SSD1306 OLED显示驱动
│   ├── RelayControl.cpp       # 继电器控制模块
│   └── TempControl.cpp        # 温湿度控制算法
├── include/                    # 头文件目录
│   ├── main.h                 # 主配置头文件
│   ├── DHT22.h                # DHT22驱动头文件
│   ├── OLED.h                 # OLED驱动头文件
│   ├── RelayControl.h         # 继电器控制头文件
│   └── TempControl.h          # 温控算法头文件
├── lib/                       # 自定义库目录
├── test/                      # 测试文件目录
├── platformio.ini             # PlatformIO配置文件
└── README.md                  # 项目说明文档
```

---

## ⚙️ 硬件连接

### STM32F407VET6 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| DHT22 数据线 | PA1 | 温湿度传感器单总线 |
| OLED SCL | PB6 | I2C时钟线 |
| OLED SDA | PB7 | I2C数据线 |
| 加热器继电器 | PC13 | 控制加热垫 |
| 风扇继电器 | PC14 | 控制散热风扇 |
| 雾化器继电器 | PC15 | 控制湿度雾化器 |
| 调试串口 | PA9/PA10 | USART1 (115200波特率) |

### 外围设备连接
- **DHT22**: VCC→3.3V, GND→GND, DATA→PA1
- **SSD1306 OLED**: VCC→3.3V, GND→GND, SCL→PB6, SDA→PB7
- **继电器模块**: 控制端分别连接PC13/PC14/PC15

---

## 🛠️ 开发环境配置

### 必需软件
- **PlatformIO IDE** (推荐VS Code插件)
- **STM32CubeMX** (可选，用于配置生成)
- **ST-Link驱动** (用于程序下载和调试)

### 依赖库
- `STM32duino FreeRTOS` - FreeRTOS实时操作系统
- `STM32Cube HAL` - STM32硬件抽象层

### 编译配置
```ini
[env:black_f407ve]
platform = ststm32
board = black_f407ve
framework = stm32cube
build_flags = 
    -DSTM32F407xx
    -DUSE_HAL_DRIVER
    -DHSE_VALUE=25000000
    -DUSE_FREERTOS=1
monitor_speed = 115200
upload_protocol = stlink
```

---

## 🚀 快速开始

### 1. 克隆项目
```bash
git clone <repository-url>
cd TempControl
```

### 2. 使用PlatformIO编译
```bash
# 安装依赖
pio lib install

# 编译项目
pio run

# 上传到开发板
pio run --target upload

# 监控串口输出
pio device monitor
```

### 3. 系统配置
在 `main.cpp` 中修改默认参数：
```cpp
ControlConfig_t controlConfig = {
    .targetTemp = 25.0f,      // 目标温度 25°C
    .targetHumidity = 60.0f,  // 目标湿度 60%
    .tempTolerance = 1.0f,    // 温度容差 ±1°C
    .humidityTolerance = 5.0f // 湿度容差 ±5%
};
```

---

## 📊 系统架构

### FreeRTOS任务分配
1. **SensorTask** (优先级: Normal)
   - 每2秒读取DHT22传感器数据
   - 数据校验和错误处理
   - 串口调试输出

2. **DisplayTask** (优先级: Low)
   - 每500ms更新OLED显示
   - 显示温湿度数据和系统状态
   - 设备状态指示器

3. **ControlTask** (优先级: High)
   - 每1秒执行控制算法
   - PID控制器更新
   - 继电器状态管理

### PID控制参数
- **温度PID**: Kp=2.0, Ki=0.1, Kd=0.5
- **湿度PID**: Kp=1.5, Ki=0.05, Kd=0.3

---

## 🔧 API接口说明

### 温湿度传感器 (DHT22)
```cpp
DHT22_Status_t DHT22_Init(void);
DHT22_Status_t DHT22_ReadData(float *temperature, float *humidity);
```

### OLED显示 (SSD1306)
```cpp
OLED_Status_t OLED_Init(void);
OLED_Status_t OLED_ShowTemperature(float temperature, float target);
OLED_Status_t OLED_ShowHumidity(float humidity, float target);
```

### 继电器控制
```cpp
void RelayControl_Set(RelayType_t relay, RelayState_t state);
RelayState_t RelayControl_Get(RelayType_t relay);
void RelayControl_AllOff(void);
```

### 温控算法
```cpp
void TempControl_Update(SensorData_t *sensorData, ControlConfig_t *config);
void TempControl_SetMode(ControlMode_t mode);
void TempControl_SetTargetTemperature(float temperature);
```

---

## 🐛 故障排除

### 常见问题

1. **DHT22读取失败**
   - 检查接线是否正确
   - 确认上拉电阻（内部已配置）
   - 验证供电电压3.3V

2. **OLED显示异常**
   - 检查I2C接线（SCL/SDA）
   - 确认I2C地址0x78
   - 验证显示器供电

3. **继电器不工作**
   - 检查PC13/PC14/PC15输出
   - 验证继电器模块供电
   - 确认继电器触发电平

### 调试方法
- 使用串口监控器查看调试信息
- 检查OLED显示的系统状态
- 使用示波器测量DHT22时序

---

## 📈 性能特点

- **响应速度**: 控制周期1秒，显示刷新500ms
- **精度**: 温度±0.5°C，湿度±2%RH
- **稳定性**: PID控制算法，减少振荡
- **安全性**: 多重保护机制，防止设备损坏

---

## 🔮 扩展功能

### 计划中的功能
- [ ] **WiFi/蓝牙通信**: ESP32模块集成
- [ ] **数据记录**: SD卡存储历史数据
- [ ] **Web界面**: 远程监控和控制
- [ ] **报警系统**: 超出阈值时的通知
- [ ] **时间调度**: 根据时间段调整目标值
- [ ] **多传感器支持**: 增加更多监测点

### 硬件扩展建议
- 增加温度传感器校准电路
- 添加LCD显示屏支持更丰富界面
- 集成蜂鸣器报警功能
- 添加按钮进行手动控制

---

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

---

## 🤝 贡献指南

欢迎提交Issue和Pull Request来改进项目：

1. Fork 本仓库
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

---

## 📞 技术支持

如有问题或建议，请通过以下方式联系：

- 提交 [GitHub Issues](./issues)
- 技术讨论和交流
- 项目改进建议

---

**项目状态**: ✅ 已完成基础功能开发，可直接编译使用 