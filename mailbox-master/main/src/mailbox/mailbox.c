#include "mailbox/mailbox.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "mailbox/server_event_handler.h"
#include "mailbox/modbus_event_handler.h"

static const char *TAG = "MAILBOX";

QueueHandle_t modbusQueue = NULL;
QueueHandle_t serverQueue = NULL;

void mailbox_init() {
  ESP_LOGI(TAG, "Initializing mailbox...");
  modbusQueue = xQueueCreate(10, sizeof(modbus_event_handler_t));
  if (modbusQueue == NULL) {
    ESP_LOGE(TAG, "Failed to create modbusQueue");
    return;
  } 
  serverQueue = xQueueCreate(10, sizeof(server_event_handler_t));
  if (serverQueue == NULL) {
    ESP_LOGE(TAG, "Failed to create mailboxQueue");
    return;
  }
}