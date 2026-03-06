#include "modbus/modbus_master.h"

#include "mailbox/mailbox.h"
#include "mailbox/modbus_event_handler.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "modbus/modbus_event.h"
#include "modbus/modbus_task.h"
#include "modbus/modbus_data_api.h"
#include "event/modbus_return.h"

static const char *TAG = "MODBUS_MASTER";

static TaskHandle_t modbusMasterTaskHandle = NULL;
static TaskHandle_t modbusEventTaskHandle = NULL;
static QueueHandle_t modbusEventQueue = NULL;

static void modbus_master_task(void *pvParameters) {
  for(;;) {
    modbus_event_handler_t receivedEvent;
    if(xQueueReceive(modbusQueue, (void *)&receivedEvent, portMAX_DELAY) == pdPASS) {
      switch (receivedEvent.event) {
        case EV_INIT_REQUEST:
          modbus_data_init();
        case EV_DATA_REQUEST:
          uint8_t slaveAddress = receivedEvent.params.setting->slaveAddress;
          if (slaveAddressChecking(slaveAddress) == TASK_FAIL) return;
          modbus_function_t function = receivedEvent.params.setting->function;
          uint8_t rangeId = getRangeIdByFunctionType(receivedEvent.params.setting->function);
          if (rangeId == 255) return;
          modbus_t modbusType = getRangeIdType(rangeId);
          if (modbusType == MODBUS_COIL) {
            uint8_t coilId = receivedEvent.params.setting->modbus_data.coilValue->coilAddress - 1;
            bool value;
            getCurrentSettingCoilValue(rangeId, coilId, &value, NULL);
          }
          if (modbusType == MODBUS_REGISTER) {
            uint16_t value;
            getCurrentSettingRegisterValue(rangeId, &value);
          }
        case EV_WRITE_REQUEST:
          modbus_event modbusEvent;
          modbusEvent.event = EV_MODBUS_WRITE;
          modbusEvent.params.setting = malloc(sizeof(modbus_data_param_t));
          if (modbusEvent.params.setting) {
            *(modbusEvent.params.setting) = *(receivedEvent.params.setting);
          }
          sendModbusEvent(modbusEvent);
        case EV_CONFIG_SLAVE_INIT:
          
      } 
    }
  }
}

void sendModbusEvent(modbus_event event) {
  if (modbusEventQueue == NULL) {
    ESP_LOGE(TAG, "modbusEventQueue is NULL");
    return;
  }
  if (xQueueSend(modbusEventQueue, (void *)&event, pdMS_TO_TICKS(1000)) != pdPASS) {
    ESP_LOGE(TAG, "Failed to send event to modbusEventQueue");
  }
}

static void modbus_event_task(void *pvParameters) {
  for(;;) {
    modbus_event receivedEvent;
    if(xQueueReceive(modbusEventQueue, (void *)&receivedEvent, portMAX_DELAY) == pdPASS) {
      switch (receivedEvent.event) {
        case EV_MODBUS_READ_ALL:
          requestSetting(ALL_USE);
        case EV_MODBUS_READ_DEFAULT:
          requestSetting(DEFAULT_USE);
        case EV_MODBUS_READ_CUSTOM:
          requestSetting(CUSTOM_USE);
        case EV_MODBUS_WRITE:
          uint8_t slaveAddress = receivedEvent.params.setting->slaveAddress;
          modbus_function_t function = receivedEvent.params.setting->function;
          uint8_t rangeId = getRangeIdByFunctionType(receivedEvent.params.setting->function);
          modbus_t modbusType = getRangeIdType(rangeId);
          if (modbusType == MODBUS_COIL) {
            uint8_t coilId = receivedEvent.params.setting->modbus_data.coilValue->coilAddress - 1;
            bool value = receivedEvent.params.setting->modbus_data.coilValue->value;
            requestCoilUpdate(slaveAddress, rangeId, coilId, value);
          }
          if (modbusType == MODBUS_REGISTER) {
            uint16_t value = receivedEvent.params.setting->modbus_data.registerValue->value;
            requestRegisterUpdate(slaveAddress,rangeId, value);
          }
          free(receivedEvent.params.setting);
      } 
    }
  }
}

void sendServerEvent(server_event_handler_t event) {
  if (serverQueue == NULL) {
    ESP_LOGE(TAG, "modbusEventQueue is NULL");
    return;
  }
  if (xQueueSend(serverQueue, (void *)&event, pdMS_TO_TICKS(1000)) != pdPASS) {
    ESP_LOGE(TAG, "Failed to send event to modbusEventQueue");
  }
}

void creatSingleCoilServerEvent(modbus_function_t setting, uint8_t slaveAddress, uint8_t coilAddress, bool value) {
  server_event_handler_t sendEvent;
  sendEvent.event = EV_MODBUS_DIFF_RES;
  modbus_data_param_t *param = (modbus_data_param_t *)malloc(sizeof(modbus_data_param_t));
  modbus_data_coil_data *coilData = (modbus_data_coil_data *)malloc(sizeof(modbus_data_coil_data));
  if (!param || !coilData) {
    ESP_LOGE(TAG, "Memory allocation failed in creatSingleCoilServerEvent");
    free(param);
    free(coilData);
    return;
  }
  coilData->coilAddress = coilAddress;
  coilData->value = value;
  param->slaveAddress = slaveAddress;
  param->function = setting;
  param->modbus_data.coilValue = coilData;
  sendEvent.params.settings = param;
  sendServerEvent(sendEvent);
}

void creatSingleRegisterServerEvent(modbus_function_t setting, uint8_t slaveAddress, uint16_t value) {
  server_event_handler_t sendEvent;
  sendEvent.event = EV_MODBUS_DIFF_RES;
  modbus_data_param_t *param = (modbus_data_param_t *)malloc(sizeof(modbus_data_param_t));
  modbus_data_register_data *registerData = (modbus_data_register_data *)malloc(sizeof(modbus_data_register_data));
  if (!param || !registerData) {
    ESP_LOGE(TAG, "Memory allocation failed in creatSingleRegisterServerEvent");
    free(param);
    free(registerData);
    return;
  }
  registerData->value = value;
  param->slaveAddress = slaveAddress;
  param->function = setting;
  param->modbus_data.registerValue = registerData;
  sendEvent.params.settings = param;
  sendServerEvent(sendEvent);
}

void createAllSettingServerEvent(modbus_function_t setting, uint8_t slaveAddress, modbus_data_all_data *allSettings) {
  server_event_handler_t sendEvent;
  sendEvent.event = EV_MODBUS_DIFF_RES;
  modbus_data_param_t *param = (modbus_data_param_t *)malloc(sizeof(modbus_data_param_t));
  if (!param) {
    ESP_LOGE(TAG, "Memory allocation failed in createAllSettingServerEvent");
    return;
  }
  param->slaveAddress = slaveAddress;
  param->function = setting;
  param->modbus_data.settings = allSettings;
  sendEvent.params.settings = param;
  sendServerEvent(sendEvent);
}

void modbus_master_init() {
  ESP_LOGI(TAG, "Initializing modbus master...");
  if (modbusMasterTaskHandle != NULL) vTaskDelete(modbusMasterTaskHandle);
  if (modbusEventTaskHandle != NULL) vTaskDelete(modbusEventTaskHandle);
  if (modbusEventQueue != NULL) vQueueDelete(modbusEventQueue);
  BaseType_t modbusMaster = xTaskCreate(modbus_master_task, "modbus_master_task", 4096, NULL, 10, &modbusMasterTaskHandle);
  if (modbusMaster != pdPASS) {
    vTaskDelete(modbusMasterTaskHandle);
    ESP_LOGE(TAG, "Modbus stack task creation error (0x%x)", modbusMaster);
    ESP_LOGE(TAG, "Failed to create modbus master task");
    return;
  } else {
    vTaskSuspend(modbusMasterTaskHandle);
  }
  BaseType_t modbusEvent = xTaskCreate(modbus_event_task, "modbus_event_task", 4096, NULL, 10, &modbusEventTaskHandle);
  if (modbusEvent != pdPASS) {
    vTaskDelete(modbusMasterTaskHandle);
    ESP_LOGE(TAG, "Modbus stack task creation error (0x%x)", modbusEvent);
    ESP_LOGE(TAG, "Failed to create modbus master task");
    return;
  } else {
    vTaskSuspend(modbusMasterTaskHandle);
  }
  modbusEventQueue = xQueueCreate(10, sizeof(modbus_event));
  if (modbusEventQueue == NULL) {
    ESP_LOGE(TAG, "Failed to create modbusEventQueue");
    return;
  } 
}

void modbus_master_start() {
  ESP_LOGI(TAG, "Starting modbus master...");
  vTaskResume(modbusMasterTaskHandle);
}

void modbus_master_stop() {
  ESP_LOGI(TAG, "Stopping mailbox...");
  vTaskSuspend(modbusMasterTaskHandle);
}
