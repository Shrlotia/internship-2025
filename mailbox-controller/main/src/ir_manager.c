#include "ir_manager.h"
#include "led_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "ir_manager";

static bool s_isInit = false;

static QueueHandle_t s_irEventQueue = NULL;
static TaskHandle_t s_taskHandle = NULL;

static bool s_irTrioLatched = false;
static bool s_irMonoLatched = false;

static bool s_irTrioEnabled[4] = {false, false, false, false};
static bool s_irMonoEnabled[2] = {false, false};

static uint8_t s_irTrioIoNums[4][3] = {
  {6, 7, 8},
  {9, 10, 11},
  {12, 13, 14},
  {15, 16, 17}
};

static uint8_t s_irMonoIoNums[2] = {4, 5};

static bool s_irTrioStates[4][3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

static bool s_irMonoStates[2] = {0, 0};

typedef struct {
  uint8_t boardNum;
  uint8_t index;
  uint8_t type; // 0 - trio, 1 - mono
} ir_event_t;

static void IRAM_ATTR trioIsrHandler(void* arg) {
  uint32_t code = (uint32_t)arg;
  ir_event_t evt = {
    .boardNum = code >> 16,
    .index = code,
    .type = 0,
  };
  xQueueSendFromISR(s_irEventQueue, &evt, NULL);
}

static void IRAM_ATTR monoIsrHandler(void *arg) {
  uint32_t code = (uint32_t)arg;
  ir_event_t evt = {
    .boardNum = code >> 16,
    .index = code,
    .type = 1,
  };
  xQueueSendFromISR(s_irEventQueue, &evt, NULL);
}

static void task(void* arg) {
  ir_event_t evt;
  int level = 0;
  for (;;) {
    if (xQueueReceive(s_irEventQueue, &evt, portMAX_DELAY) == pdPASS) {
      ESP_LOGI(TAG, "Received an event (boardNum=%u, index=%u, type=%u)", evt.boardNum, evt.index, evt.type);
      if (evt.type == 0) {
        if (!s_irTrioEnabled[evt.boardNum]) continue;
        uint8_t ioNum = s_irTrioIoNums[evt.boardNum][evt.index];
        level = gpio_get_level(ioNum);
        s_irTrioStates[evt.boardNum][evt.index] = level;
        if (level == 1 && !s_irTrioLatched) {
          s_irTrioLatched = true;
          led_manager_turnOnLed(0);
        }
      } else if (evt.type == 1) {
        if (!s_irMonoEnabled[evt.boardNum]) continue;
        level = gpio_get_level(s_irMonoIoNums[evt.boardNum]);
        s_irMonoStates[evt.boardNum] = level;
        if (level == 1 && !s_irMonoLatched) {
          s_irMonoLatched = true;
          led_manager_turnOnLed(1);
        }
      }
    }
  }
}

bool ir_manager_init() {
  if (s_isInit) return true;
  s_irEventQueue = xQueueCreate(10, sizeof(ir_event_t));
  gpio_install_isr_service(0);
    
  gpio_config_t ioConf = {
    .intr_type = GPIO_INTR_ANYEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = 0,
    .pull_down_en = 1,
    .pull_up_en = 0,
  };
  uint8_t *ioNums = NULL;
  for (int i = 0; i < 4; i++) {
    ioNums = s_irTrioIoNums[i];
    ioConf.pin_bit_mask |= (1ULL << ioNums[0] | 1ULL << ioNums[1] | 1ULL << ioNums[2]);
  }
  ioConf.pin_bit_mask |= (1ULL << s_irMonoIoNums[0] | 1ULL << s_irMonoIoNums[1]);
  gpio_config(&ioConf);
  
  s_isInit = true;

  return true;
}

bool ir_manager_deinit() {
  if (!s_isInit) return false;
  gpio_uninstall_isr_service();

  if (s_irEventQueue) {
    vQueueDelete(s_irEventQueue);
    s_irEventQueue = NULL;
  }
   
  s_isInit = false;

  return true;
}

bool ir_manager_start() {
  if (!s_isInit) {
    ESP_LOGE(TAG, "Init first");
    return false;
  }
  if (s_taskHandle != NULL) {
    ESP_LOGW(TAG, "Already started");
    return false;
  }
  if (xTaskCreate(task, "ir_manager", 4096, NULL, 5, &s_taskHandle) != pdPASS) {
    vTaskDelete(s_taskHandle);
    return false;
  }
  return true;
}

bool ir_manager_stop() {
  if (s_taskHandle == NULL) return false;
  vTaskDelete(s_taskHandle);
  s_taskHandle = NULL;
  return true;
}

bool ir_manager_getTrioEnabled(uint8_t boardNum) {
  if (boardNum > 3) {
    ESP_LOGE(TAG, "Board num must be between 0 and 3");
    return false;
  }
  return s_irTrioEnabled[boardNum];
}

bool ir_manager_getMonoEnabled(uint8_t boardNum) {
  if (boardNum > 1) {
    ESP_LOGE(TAG, "Board num must be between 0 and 1");
    return false;
  }
  return s_irMonoEnabled[boardNum];
}

bool ir_manager_setTrioEnabled(uint8_t boardNum, bool enabled) {
  if (boardNum > 3) {
    ESP_LOGE(TAG, "Board num must be between 0 and 3");
    return false;
  }
  uint8_t *ioNums = s_irTrioIoNums[boardNum];
  bool wasEnabled = s_irTrioEnabled[boardNum];
  if (enabled) {
    if (wasEnabled) return false;
    for (int i = 0; i < 3; ++i) {
      uint32_t code = (boardNum << 16) + i;
      gpio_isr_handler_add(ioNums[i], trioIsrHandler, (void*)code);
    }
    s_irTrioEnabled[boardNum] = true;
  } else {
    if (!wasEnabled) return false;
    for (int i = 0; i < 3; ++i) {
      gpio_isr_handler_remove(ioNums[i]);
    }
    s_irTrioEnabled[boardNum] = false;
  }
  return true;
}

bool ir_manager_setMonoEnabled(uint8_t boardNum, bool enabled) {
  if (boardNum > 1) {
    ESP_LOGE(TAG, "Board num must be between 0 and 1");
    return false;
  }
  bool wasEnabled = s_irMonoEnabled[boardNum];
  if (enabled) {
    if (wasEnabled) return false;
    gpio_isr_handler_add(s_irMonoIoNums[boardNum], monoIsrHandler, (void*)(uint32_t)s_irMonoIoNums[boardNum]);
    s_irMonoEnabled[boardNum] = true;
  } else {
    if (!wasEnabled) return false;
    gpio_isr_handler_remove(s_irMonoIoNums[boardNum]);
    s_irMonoEnabled[boardNum] = false;
  }
  return true;
}

bool ir_manager_getAllTrioStates(uint8_t *states) {
  if (states == NULL) return false;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      states[i * 3 + j] = s_irTrioStates[i][j];
    }
  }
  return true;
}

bool ir_manager_getAllMonoStates(uint8_t *states) {
  if (states == NULL) return false;
  for (int i = 0; i < 2; ++i) {
    states[i] = s_irMonoStates[i];
  }
  return true;
}

bool ir_manager_getTrioLatched() {
  return s_irTrioLatched;
}

bool ir_manager_getMonoLatched() {
  return s_irMonoLatched;
}

void ir_manager_resetTrioLatched() {
  led_manager_turnOffLed(0);
  s_irTrioLatched = false;
}

void ir_manager_resetMonoLatched() {
  led_manager_turnOffLed(1);
  s_irMonoLatched = false;
}