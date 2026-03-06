#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t trio1Enabled : 1;
  uint8_t trio2Enabled : 1;
  uint8_t trio3Enabled : 1;
  uint8_t trio4Enabled : 1;
  uint8_t mono1Enabled : 1;
  uint8_t mono2Enabled : 1;
} mailbox_ir_config_t;

typedef struct {
  uint8_t lock1Enabled : 1;
  uint8_t lock2Enabled : 1;
  uint16_t lockDurationMs;
} mailbox_lock_config_t;

typedef struct {
  uint8_t led1Enabled : 1;
  uint8_t led2Enabled : 1;
  uint8_t led3Enabled : 1;
} mailbox_led_config_t;

bool loadIrConfig(mailbox_ir_config_t *config);
bool loadLockConfig(mailbox_lock_config_t *config);
bool loadLedConfig(mailbox_led_config_t *config);

bool saveIrConfig(mailbox_ir_config_t *config);
bool saveLockConfig(mailbox_lock_config_t *config);
bool saveLedConfig(mailbox_led_config_t *config);
