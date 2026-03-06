#include "led_manager.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "led_manager";

static bool s_isInit = false;

static QueueHandle_t s_ledCommandQueue = NULL;
static TaskHandle_t s_taskHandle = NULL;

static uint8_t s_ledIoNums[3] = {1, 2, 3};
static uint8_t s_ledEnabled[3] = {0, 0, 0};
static uint8_t s_ledStates[3] = {0, 0, 0};

typedef enum {
  LED_COMMAND_TURN_ON,
  LED_COMMAND_TURN_OFF,
  LED_COMMAND_BLINK,
} led_command_type_t;

typedef struct {
  led_command_type_t type;
  uint8_t ledNumBitField;
} led_command_t;

static void task(void *arg) {
  led_command_t cmd;
  uint8_t ledEnabled[3] = {0, 0, 0};
  for (;;) {
    if (xQueueReceive(s_ledCommandQueue, &cmd, portMAX_DELAY)) {
      memcpy(ledEnabled, s_ledEnabled, 3);
      ESP_LOGI(TAG, "Received a command (type=0x%hhu, ledNumBitField: 0x%hhu)", cmd.type, cmd.ledNumBitField);
      switch (cmd.type) {
        case LED_COMMAND_TURN_ON:
          for (int i = 0; i < 3; ++i) {
            if (ledEnabled[i] && (cmd.ledNumBitField & (1 << i))) {
              gpio_set_level(s_ledIoNums[i], 1);
              s_ledStates[i] = 1;
            }
          }
          break;
        case LED_COMMAND_TURN_OFF:
          for (int i = 0; i < 3; ++i) {
            if (ledEnabled[i] && (cmd.ledNumBitField & (1 << i))) {
              gpio_set_level(s_ledIoNums[i], 0);
              s_ledStates[i] = 0;
            }
          }
          break;
        case LED_COMMAND_BLINK: {
          for (int j = 0; j < 10; ++j) {
            for (int i = 0; i < 3; ++i) {
              if (cmd.ledNumBitField & (1 << i)) {
                gpio_set_level(s_ledIoNums[i], 1);
              }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            for (int i = 0; i < 3; ++i) {
              if (cmd.ledNumBitField & (1 << i)) {
                gpio_set_level(s_ledIoNums[i], 0);
              }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
          }
          for (int i = 0; i < 3; ++i) {
            if (ledEnabled[i]) {
              gpio_set_level(s_ledIoNums[i], s_ledStates[i]);
            }
          }
          break;
        }
      }
    }
  }
}

bool led_manager_init() {
  if (s_isInit) return false;
  s_ledCommandQueue = xQueueCreate(10, sizeof(led_command_t));
  gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ULL << s_ledIoNums[0] | 1ULL << s_ledIoNums[1] | 1ULL << s_ledIoNums[2]),
    .pull_down_en = 0,
    .pull_up_en = 0,
  };
  gpio_config(&io_conf);
  s_isInit = true;
  return true;
}

bool led_manager_deinit() {
  if (!s_isInit) return false;
  if (s_ledCommandQueue) {
    vQueueDelete(s_ledCommandQueue);
    s_ledCommandQueue = NULL;
  }
  s_isInit = false;
  return true;
}

bool led_manager_start() {
  if (!s_isInit) {
    ESP_LOGE(TAG, "Init first");
    return false;
  }
  if (s_taskHandle != NULL) {
    ESP_LOGW(TAG, "Already started");
    return false;
  }
  if (xTaskCreate(task, "led_manager", 4096, NULL, 5, &s_taskHandle) != pdPASS) {
    vTaskDelete(s_taskHandle);
    return false;
  }
  return true;
}

bool led_manager_stop() {
  if (s_taskHandle == NULL) return false;
  vTaskDelete(s_taskHandle);
  s_taskHandle = NULL;
  return true;
}

bool led_manager_getLedEnabled(uint8_t ledNum) {
  if (ledNum > 2) {
    ESP_LOGE(TAG, "LED num must be between 0 and 2");
    return false;
  }
  return s_ledEnabled[ledNum];
}

bool led_manager_setLedEnabled(uint8_t ledNum, bool enabled) {
  if (ledNum > 2) {
    ESP_LOGE(TAG, "LED num must be between 0 and 2");
    return false;
  }
  s_ledEnabled[ledNum] = enabled;
  return true;
}

bool led_manager_turnOnLed(uint8_t ledNum) {
  if (ledNum > 2) {
    ESP_LOGE(TAG, "LED num must be between 0 and 2");
    return false;
  }
  led_command_t cmd = {
    .type = LED_COMMAND_TURN_ON,
    .ledNumBitField = 1 << ledNum,
  };
  xQueueSend(s_ledCommandQueue, &cmd, portMAX_DELAY);
  return true;
}

bool led_manager_turnOffLed(uint8_t ledNum) {
  if (ledNum > 2) {
    ESP_LOGE(TAG, "LED num must be between 0 and 2");
    return false;
  }
  led_command_t cmd = {
    .type = LED_COMMAND_TURN_OFF,
    .ledNumBitField = 1 << ledNum,
  };
  xQueueSend(s_ledCommandQueue, &cmd, portMAX_DELAY);
  return true;
}

bool led_manager_blinkLeds() {
  led_command_t cmd = {
    .type = LED_COMMAND_BLINK,
    .ledNumBitField = 0xFF,
  };
  xQueueSend(s_ledCommandQueue, &cmd, portMAX_DELAY);
  return true;
}