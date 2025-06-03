/*
 * STM32F407 温湿度智能控制系统
 * 主程序入口文件 - main.cpp
 * 现代C++17 + FreeRTOS + STM32 HAL 重构版本
 */

#include "main.h"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>

#include "DHT22.h"
#include "FreeRTOS.h"
#include "OLED.h"
#include "RelayControl.h"
#include "TempControl.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

TaskHandle_t sensorTaskHandle = nullptr;
TaskHandle_t displayTaskHandle = nullptr;
TaskHandle_t controlTaskHandle = nullptr;

extern "C" {
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
}

namespace RC = ReptileController;
RC::SensorData g_sensorData;
RC::ControlConfig g_controlConfig{30.0f, 40.0f, 2.0f, 8.0f};  // 猪鼻蛇配置
SemaphoreHandle_t sensorDataMutex = nullptr;//g_sensorsData被三个任务同时访问，使用互斥量保护

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

// FreeRTOS 任务函数声明
// 需要c接口
extern "C" {
void SensorTask(void *argument);
void DisplayTask(void *argument);
void ControlTask(void *argument);
}

// 检查函数是否可以用SensorData&调用并返回void
template <typename F, typename = void>
struct is_sensor_data_processor : std::false_type {};
template <typename F>
struct is_sensor_data_processor<F, std::void_t<decltype(std::declval<F>()(
                                       std::declval<RC::SensorData &>()))>>
    : std::is_same<void, decltype(std::declval<F>()(
                             std::declval<RC::SensorData &>()))> {};
template <typename F>
constexpr bool is_sensor_data_processor_v = is_sensor_data_processor<F>::value;

template <typename F>
auto protectedSensorDataAccess(F &&func)
    -> std::enable_if_t<is_sensor_data_processor_v<std::decay_t<F>>, void> {
  if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    std::forward<F>(func)(g_sensorData);
    xSemaphoreGive(sensorDataMutex);
  }
}

void printMessage(const std::string &message) {
  HAL_UART_Transmit(&huart1, reinterpret_cast<const uint8_t *>(message.c_str()),
                    static_cast<uint16_t>(message.length()), 1000);
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  RC::DHT22::g_sensor.initialize();
  OLED_Init();
  RC::Relay::g_controller.initialize();
  TempControl_Init();

  printMessage("STM32F407 爬宠温湿度控制系统启动...\r\n");
  printMessage("配置: 猪鼻蛇模式 - 目标温度30°C, 目标湿度40%\r\n");

  sensorDataMutex = xSemaphoreCreateMutex();
  if (sensorDataMutex == nullptr) {
    printMessage("错误: 无法创建互斥量\r\n");
    Error_Handler();
  }

  const UBaseType_t sensorPriority = 3;
  const UBaseType_t displayPriority = 2;
  const UBaseType_t controlPriority = 4;
  const uint16_t stackSize = 256;

  if (xTaskCreate(SensorTask, "SensorTask", stackSize, nullptr, sensorPriority,
                  &sensorTaskHandle) != pdPASS) {
    printMessage("错误: 无法创建传感器任务\r\n");
    Error_Handler();
  }
  if (xTaskCreate(DisplayTask, "DisplayTask", stackSize, nullptr,
                  displayPriority, &displayTaskHandle) != pdPASS) {
    printMessage("错误: 无法创建显示任务\r\n");
    Error_Handler();
  }
  if (xTaskCreate(ControlTask, "ControlTask", stackSize, nullptr,
                  controlPriority, &controlTaskHandle) != pdPASS) {
    printMessage("错误: 无法创建控制任务\r\n");
    Error_Handler();
  }
  printMessage("所有任务创建成功，启动调度器...\r\n");
  vTaskStartScheduler();

  printMessage("错误: 调度器意外退出\r\n");
  while (true) {
    HAL_Delay(1000);
  }
}

void SensorTask(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  constexpr TickType_t xFrequency = pdMS_TO_TICKS(2000);  // 2秒周期

  printMessage("[传感器任务] 启动\r\n");

  for (;;) {
    const auto sensorResult = RC::DHT22::g_sensor.readData();
    if (sensorResult.valid) {
      const float temperature = sensorResult.temperature;
      const float humidity = sensorResult.humidity;
      protectedSensorDataAccess([&](RC::SensorData &data) {
        data.updateData(temperature, humidity, HAL_GetTick());
      });
      const std::string debugMsg =
          "[传感器] 温度: " + std::to_string(temperature) +
          "°C, 湿度: " + std::to_string(humidity) + "%\r\n";
      printMessage(debugMsg);
    } else {
      protectedSensorDataAccess(
          [](RC::SensorData &data) { data.invalidate(); });
      printMessage("[传感器] DHT22读取失败!\r\n");
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void DisplayTask(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  constexpr TickType_t xFrequency = pdMS_TO_TICKS(500);

  printMessage("[显示任务] 启动\r\n");

  for (;;) {
    OLED_Clear();
    RC::SensorData localSensorData{};
    protectedSensorDataAccess(
        [&](RC::SensorData &data) { localSensorData = data; });

    if (localSensorData.isValid()) {
      OLED_ShowTemperature(localSensorData.getTemperature(),
                           g_controlConfig.getTargetTemp());
      OLED_ShowHumidity(localSensorData.getHumidity(),
                        g_controlConfig.getTargetHumidity());
    } else {
      OLED_ShowString(0, 0, "传感器错误", 16);
    }

    OLED_ShowSystemStatus();
    OLED_Refresh();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void ControlTask(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  constexpr TickType_t xFrequency = pdMS_TO_TICKS(1000);

  printMessage("[控制任务] 启动\r\n");

  for (;;) {
    RC::SensorData localSensorData{};
    protectedSensorDataAccess(
        [&](RC::SensorData &data) { localSensorData = data; });

    if (localSensorData.isValid()) {
      if (RC::Control::g_controller) {
        RC::Control::g_controller->update(localSensorData, g_controlConfig);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                    GPIO_PIN_RESET);

  GPIO_InitTypeDef GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void) {
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168 - 1;  // 1MHz计数频率
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  HAL_TIM_Base_Start(&htim2);
}

void Error_Handler(void) {
  __disable_irq();
  printMessage("错误: 系统进入错误处理状态\r\n");
  constexpr uint32_t errorBlinkDelay = 200;
  while (true) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(errorBlinkDelay);
  }
}

// FreeRTOS 静态内存分配回调函数
#if (configSUPPORT_STATIC_ALLOCATION == 1)

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
  static StaticTask_t xIdleTaskTCB;
  static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
  static StaticTask_t xTimerTaskTCB;
  static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

#endif /* configSUPPORT_STATIC_ALLOCATION */