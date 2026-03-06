#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "ir_manager.h"
#include "lock_manager.h"
#include "led_manager.h"
#include "modbus_master.h"
#include "config.h"

static const char *TAG = "main";

static bool initIrManager() {
  mailbox_ir_config_t config;
  if (!loadIrConfig(&config)) return false;
  if (!ir_manager_init()) return false;
  ESP_LOGI(TAG, "Trio: %u %u %u %u", config.trio1Enabled, config.trio2Enabled, config.trio3Enabled, config.trio4Enabled);
  ESP_LOGI(TAG, "Mono: %u %u", config.mono1Enabled, config.mono2Enabled);
  ir_manager_setTrioEnabled(0, config.trio1Enabled);
  ir_manager_setTrioEnabled(1, config.trio2Enabled);
  ir_manager_setTrioEnabled(2, config.trio3Enabled);
  ir_manager_setTrioEnabled(3, config.trio4Enabled);
  ir_manager_setMonoEnabled(0, config.mono1Enabled);
  ir_manager_setMonoEnabled(1, config.mono2Enabled);
  return true;
}

static bool initLockManager() {
  mailbox_lock_config_t config;
  if (!loadLockConfig(&config)) return false;
  if (!lock_manager_init()) return false;
  ESP_LOGI(TAG, "Lock: %u %u", config.lock1Enabled, config.lock2Enabled);
  ESP_LOGI(TAG, "Lock duration: %u", config.lockDurationMs);
  lock_manager_setLockEnabled(0, config.lock1Enabled);
  lock_manager_setLockEnabled(1, config.lock2Enabled);
  lock_manager_setLockDurationMs(config.lockDurationMs);
  return true;
}

static bool initLedManager() {
  mailbox_led_config_t config;
  if (!loadLedConfig(&config)) return false;
  if (!led_manager_init()) return false;
  ESP_LOGI(TAG, "LED: %u %u %u", config.led1Enabled, config.led2Enabled, config.led3Enabled);
  led_manager_setLedEnabled(0, config.led1Enabled);
  led_manager_setLedEnabled(1, config.led2Enabled);
  led_manager_setLedEnabled(2, config.led3Enabled);
  return true;
}

void app_main(void) {
  if (!initIrManager()) {
    ESP_LOGE(TAG, "Failed to init IR manager");
    return;
  }
  if (!initLockManager()) {
    ESP_LOGE(TAG, "Failed to init lock manager");
    return;
  }
  if (!initLedManager()) {
    ESP_LOGE(TAG, "Failed to init LED manager");
    return;
  }
  modbus_master_init();

  ir_manager_start();
  lock_manager_start();
  led_manager_start();
  modbus_master_start();
}

