#include "server/server.h"

#include "mailbox/mailbox.h"
#include "mailbox/server_event_handler.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "modbus/modbus_function.h"
#include "server/json_handler.h" 
#include "cJSON.h"

static const char *TAG = "SERVER_MASTER";

static TaskHandle_t serverTaskHandle = NULL;

static void server_task(void *pvParameters) {
  for(;;) {
    server_event_handler_t receivedEvent;
    if(xQueueReceive(serverQueue, (void *)&receivedEvent, portMAX_DELAY) == pdPASS) {
      switch (receivedEvent.event) {
        case EV_MODBUS_DIFF_RES:
          modbus_function_t function = receivedEvent.settings->function;
          uint8_t slaveAddress = receivedEvent.settings->slaveAddress;
          uint8_t coilId = receivedEvent.params.setting->modbus_data.coilValue->coilAddress - 1;
          bool value = receivedEvent.params.setting->modbus_data.coilValue->value;
        case EV_MODBUS_STATE_RES:

          break;
        default:
          ESP_LOGW(TAG, "Unknown event type: %d", receivedEvent.event);
          break;
      }
    }
  }
}

void sendServerEvent(server_event_handler_t event) {
  if (serverQueue == NULL) {
    ESP_LOGE(TAG, "serverQueue is NULL");
    return;
  }
  if (xQueueSend(serverQueue, (void *)&event, pdMS_TO_TICKS(1000)) != pdPASS) {
    ESP_LOGE(TAG, "Failed to send event to serverQueue");
  }
}

void server_init() {
  ESP_LOGI(TAG, "Initializing modbus master...");
  if (serverTaskHandle != NULL) vTaskDelete(serverTaskHandle);
  BaseType_t modbusMaster = xTaskCreate(server_task, "server_task", 4096, NULL, 10, &serverTaskHandle);
  if (modbusMaster != pdPASS) {
    vTaskDelete(serverTaskHandle);
    ESP_LOGE(TAG, "Modbus stack task creation error (0x%x)", modbusMaster);
    ESP_LOGE(TAG, "Failed to create modbus master task");
    return;
  } else {
    vTaskSuspend(serverTaskHandle);
  }
}

void server_start() {
  ESP_LOGI(TAG, "Starting modbus master...");
  vTaskResume(serverTaskHandle);
}

void server_stop() {
  ESP_LOGI(TAG, "Stopping mailbox...");
  vTaskSuspend(serverTaskHandle);
}