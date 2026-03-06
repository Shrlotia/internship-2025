#include "lock_manager.h"

#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "led_manager.h"

static const char *TAG = "lock_manager";

static bool s_isInit = false;

static QueueHandle_t s_lockEventQueue = NULL;
static TaskHandle_t s_taskHandle = NULL;

static uint16_t s_lockDurationMs = 5000;

static uint8_t s_lockEnabled[2] = {0, 0};
static uint8_t s_lockIoNums[2] = {42, 41};
static uint8_t s_lockStates[2] = {0, 0};

static void task(void *arg) {
  uint8_t lockBitField = 0;
  uint8_t lockEnabled[2] = {0, 0};
  for (;;) {
    if (xQueueReceive(s_lockEventQueue, &lockBitField, portMAX_DELAY)) {
      memcpy(lockEnabled, s_lockEnabled, 2);
      ESP_LOGI(TAG, "Received an event (lockBitField=0x%hhu)", lockBitField);
      for (int i = 0; i < 2; ++i) {
        if (lockEnabled[i] && (lockBitField & (1 << i))) {
          s_lockStates[i] = 1;
          gpio_set_level(s_lockIoNums[i], 1);
        }
      }
      led_manager_turnOnLed(2);
      vTaskDelay(pdMS_TO_TICKS(s_lockDurationMs));
      led_manager_turnOffLed(2);
      for (int i = 0; i < 2; ++i) {
        if (lockEnabled[i] && (lockBitField & (1 << i))) {
          gpio_set_level(s_lockIoNums[i], 0);
          s_lockStates[i] = 0;
        }
      }
    }
  }
}

bool lock_manager_init() {
  if (s_isInit) return false;
  s_lockEventQueue = xQueueCreate(10, sizeof(uint8_t));
  gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ULL << s_lockIoNums[0] | 1ULL << s_lockIoNums[1]),
    .pull_down_en = 0,
    .pull_up_en = 0,
  };
  gpio_config(&io_conf);
  s_isInit = true;
  return true;
}

bool lock_manager_deinit() {
  if (!s_isInit) return false;
  if (s_lockEventQueue) {
    vQueueDelete(s_lockEventQueue);
    s_lockEventQueue = NULL;
  }
  s_isInit = false;
  return true;
}

bool lock_manager_start() {
  if (!s_isInit) {
    ESP_LOGE(TAG, "Init first");
    return false;
  }
  if (s_taskHandle != NULL) {
    ESP_LOGW(TAG, "Already started");
    return false;
  }
  if (xTaskCreate(task, "lock_manager", 4096, NULL, 5, &s_taskHandle) != pdPASS) {
    vTaskDelete(s_taskHandle);
    return false;
  }
  return true;
}

bool lock_manager_stop() {
  if (s_taskHandle == NULL) return false;
  vTaskDelete(s_taskHandle);
  s_taskHandle = NULL;
  return true;
}

static bool isRunning() {
  return s_taskHandle != NULL;
}

uint16_t lock_manager_getLockDurationMs() {
  return s_lockDurationMs;
}

bool lock_manager_setLockDurationMs(uint16_t lockDurationMs) {
  s_lockDurationMs = lockDurationMs;
  return true;
}

bool lock_manager_getLockEnabled(uint8_t lockNum) {
  if (lockNum > 1) {
    ESP_LOGE(TAG, "Lock num must be between 0 and 1");
    return false;
  }
  return s_lockEnabled[lockNum];
}

bool lock_manager_setLockEnabled(uint8_t lockNum, bool enabled) {
  if (lockNum > 1) {
    ESP_LOGE(TAG, "Lock num must be between 0 and 1");
    return false;
  }
  s_lockEnabled[lockNum] = enabled;
  return true;
}

bool lock_manager_getAllLockStates(uint8_t *states) {
  if (states == NULL) {
    ESP_LOGE(TAG, "States is NULL");
    return false;
  }
  memcpy(states, s_lockStates, 2);
  return true;
}

bool lock_manager_unlock(uint8_t lockNum) {
  if (lockNum > 1) {
    ESP_LOGE(TAG, "Lock num must be between 0 and 1");
    return false;
  }
  if (!isRunning()) {
    ESP_LOGE(TAG, "Not running");
    return false;
  }
  uint8_t lockBitField = 1 << lockNum;
  if (xQueueSend(s_lockEventQueue, &lockBitField, pdMS_TO_TICKS(20)) != pdPASS) {
    ESP_LOGW(TAG, "Queue is full.");
    return false;
  }
  return true;
}

bool lock_manager_unlockAll() {
  if (!isRunning()) {
    ESP_LOGE(TAG, "Not running");
    return false;
  }
  uint8_t lockBitField = 0xFF;
  if (xQueueSend(s_lockEventQueue, &lockBitField, pdMS_TO_TICKS(20)) != pdPASS) {
    ESP_LOGW(TAG, "Queue is full.");
    return false;
  }
  return true;
}