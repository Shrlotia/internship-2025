#include "sa_ota.h"

#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "sa_ota";

static bool s_isInOtaMode = false;
static bool s_imageHeaderWasChecked = false;
static const esp_partition_t *s_updatePartition = NULL;
static esp_ota_handle_t s_updateHandle = 0;
static size_t s_bytesWritten = 0;
static esp_timer_handle_t s_restartTimerHandle = 0;
static bool s_forceUpdate = false;

bool sa_ota_isInOtaMode() {
  return s_isInOtaMode;
}

bool sa_ota_begin(bool forceUpdate) {
  if (s_isInOtaMode) {
    ESP_LOGW(TAG, "Is already in ota mode.");
    return false;
  }
  s_updatePartition = esp_ota_get_next_update_partition(NULL);
  if (s_updatePartition == NULL) {
    ESP_LOGE(TAG, "No partition for next update.");
    return false;
  }
  s_isInOtaMode = true;
  s_forceUpdate = forceUpdate;
  s_imageHeaderWasChecked = false;
  s_updateHandle = 0;
  s_bytesWritten = 0;
  ESP_LOGI(TAG, "OTA mode has begin. New app will be written to partition subtype %d at offset 0x%"PRIx32, s_updatePartition->subtype, s_updatePartition->address);
  return true;
}

void sa_ota_timer_expired(void *pvParameters) {
  esp_restart();
}

bool sa_ota_write(const void *data, size_t size) {
  if (!s_isInOtaMode) {
    ESP_LOGW(TAG, "App is not in OTA mode.");
    return false;
  }
  esp_err_t err;
  if (!s_imageHeaderWasChecked) {
    if (size < sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t) - 80) {
      ESP_LOGE(TAG, "Received header was too short.");
      sa_ota_abort();
      return false;
    }
    esp_app_desc_t newAppInfo;
    memcpy(&newAppInfo, &data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t) - 80);
    ESP_LOGI(TAG, "New firmware version: %s", newAppInfo.version);

    const esp_partition_t *runningPartition = esp_ota_get_running_partition();
    esp_app_desc_t currentAppInfo;
    esp_ota_get_partition_description(runningPartition, &currentAppInfo);
    ESP_LOGI(TAG, "Current firmware version: %s", currentAppInfo.version);

    if (!s_forceUpdate) {
      const esp_partition_t *lastInvalidPartition = esp_ota_get_last_invalid_partition();
      if (lastInvalidPartition != NULL) {
        esp_app_desc_t lastInvalidAppInfo;
        esp_ota_get_partition_description(lastInvalidPartition, &lastInvalidAppInfo);
        ESP_LOGI(TAG, "Last invalid firmware version: %s", lastInvalidAppInfo.version);
        if (memcmp(lastInvalidAppInfo.version, newAppInfo.version, sizeof(newAppInfo.version)) == 0) {
          ESP_LOGW(TAG, "New version is the same as invalid version.");
          ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", lastInvalidAppInfo.version);
          ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
          sa_ota_abort();
          return false;
        }
      }
      if (memcmp(newAppInfo.version, currentAppInfo.version, sizeof(newAppInfo.version)) == 0) {
        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
        sa_ota_abort();
        return false;
      }
    }
    s_imageHeaderWasChecked = true;
    err = esp_ota_begin(s_updatePartition, OTA_WITH_SEQUENTIAL_WRITES, &s_updateHandle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
      sa_ota_abort();
      return false;
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded.");
  }
  err = esp_ota_write(s_updateHandle, data, size);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
    sa_ota_abort();
    return false;
  }
  s_bytesWritten += size;
  ESP_LOGI(TAG, "%zu bytes for app image written", s_bytesWritten);
  return true;
}

bool sa_ota_end() {
  if (!s_isInOtaMode) {
    ESP_LOGW(TAG, "App is not in OTA mode.");
    return false;
  }
  if (s_updateHandle == 0) {
    ESP_LOGW(TAG, "OTA has not really began.");
    return false;
  }
  ESP_LOGI(TAG, "Ending OTA mode...");
  esp_err_t err = esp_ota_end(s_updateHandle);
  if (err != ESP_OK) {
    if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
      ESP_LOGE(TAG, "Image validation failed, image is corrupted");
    } else {
      ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
    }
    s_updateHandle = 0;
    s_isInOtaMode = false;
    return false;
  }
  err = esp_ota_set_boot_partition(s_updatePartition);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
    s_updateHandle = 0;
    s_isInOtaMode = false;
    return false;
  }
  ESP_LOGI(TAG, "Successfully ended OTA. Prepare to restart system in 3 seconds!");
  esp_timer_create_args_t timerConf = {
    .name = "restart timer",
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .callback = sa_ota_timer_expired,
  };
  esp_timer_create(&timerConf, &s_restartTimerHandle);
  esp_timer_start_once(s_restartTimerHandle, 3000000);
  return true;
}

bool sa_ota_abort() {
  ESP_LOGI(TAG, "Aborting OTA...");
  if (s_updateHandle != 0) {
    esp_ota_abort(s_updateHandle);
    s_updateHandle = 0;
  }
  s_isInOtaMode = false;
  return true;
}
