#include "config.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "config";

static nvs_handle_t s_nvsHandle;

static bool init() {
  esp_err_t err = nvs_flash_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init nvs flash: %u", err);
    return false;
  }
  err = nvs_open("config", NVS_READWRITE, &s_nvsHandle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open nvs namespace: %u", err);
    return false;
  }
  return true;
}

bool loadIrConfig(mailbox_ir_config_t *config) {
  if (s_nvsHandle == NULL) {
    if (!init()) return false;
  }
  size_t size = sizeof(mailbox_ir_config_t);
  esp_err_t ret = nvs_get_blob(s_nvsHandle, "ir_config", config, &size);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    config->trio1Enabled = 1;
    config->trio2Enabled = 1;
    config->trio3Enabled = 1;
    config->trio4Enabled = 1;
    config->mono1Enabled = 1;
    config->mono2Enabled = 0;
    return saveIrConfig(config);
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get blob: 0x%x", ret);
    return false;
  }
  return true;
}

bool loadLockConfig(mailbox_lock_config_t *config) {
  if (s_nvsHandle == NULL) {
    if (!init()) return false;
  }
  size_t size = sizeof(mailbox_lock_config_t);
  esp_err_t ret = nvs_get_blob(s_nvsHandle, "lock_config", config, &size);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    config->lock1Enabled = 1;
    config->lock2Enabled = 0;
    config->lockDurationMs = 5000;
    return saveLockConfig(config);
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get blob: %u", ret);
    return false;
  }
  return true;
}

bool loadLedConfig(mailbox_led_config_t *config) {
  if (s_nvsHandle == NULL) {
    if (!init()) return false;
  }
  size_t size = sizeof(mailbox_led_config_t);
  esp_err_t ret = nvs_get_blob(s_nvsHandle, "led_config", config, &size);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    config->led1Enabled = 1;
    config->led2Enabled = 1;
    config->led3Enabled = 1;
    return saveLedConfig(config);
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get blob: %u", ret);
    return false;
  }
  return true;
}

bool saveIrConfig(mailbox_ir_config_t *config) {
  if (s_nvsHandle == NULL) {
    if (!init()) return false;
  }
  esp_err_t ret = nvs_set_blob(s_nvsHandle, "ir_config", config, sizeof(mailbox_ir_config_t));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set blob: %u", ret);
    return false;
  }
  return true;
}

bool saveLockConfig(mailbox_lock_config_t *config) {
  if (s_nvsHandle == NULL) {
    if (!init()) return false;
  }
  esp_err_t ret = nvs_set_blob(s_nvsHandle, "lock_config", config, sizeof(mailbox_lock_config_t));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set blob: %u", ret);
    return false;
  }
  return true;
}

bool saveLedConfig(mailbox_led_config_t *config) {
  if (s_nvsHandle == NULL) {
    if (!init()) return false;
  }
  esp_err_t ret = nvs_set_blob(s_nvsHandle, "led_config", config, sizeof(mailbox_led_config_t));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set blob: %u", ret);
    return false;
  }
  return true;
}
