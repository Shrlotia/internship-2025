#include "modbus_master.h"

#include <stdint.h>
#include <inttypes.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_app_desc.h"

#include "sa_utils.h"
#include "sa_modbus.h"
#include "ir_manager.h"
#include "lock_manager.h"
#include "config.h"
#include "led_manager.h"
#include "sa_ota.h"

static const char *TAG = "modbus_master";

static bool s_isInit = false;

// switcher gpio num
static uint8_t s_dsIoNums[8] = {20, 19, 21, 33, 34, 35, 36, 37};
uint16_t currentVersion[3] = {0, 0, 0};

// Coil Table
static const uint16_t COIL_IR_TRIO_ENABLED_START      = 0x1000;
static const uint16_t COIL_IR_TRIO_ENABLED_END        = 0x1004;
static const uint16_t COIL_IR_MONO_ENABLED_START      = 0x1010;
static const uint16_t COIL_IR_MONO_ENABLED_END        = 0x1012;
static const uint16_t COIL_LOCK_ENABLED_START         = 0x1020;
static const uint16_t COIL_LOCK_ENABLED_END           = 0x1022;
static const uint16_t COIL_LED_ENABLED_START          = 0x1030;
static const uint16_t COIL_LED_ENABLED_END            = 0x1033;
static const uint16_t COIL_IR_TRIO_STATE_START        = 0x2000;
static const uint16_t COIL_IR_TRIO_STATE_END          = 0x200C;
static const uint16_t COIL_IR_MONO_STATE_START        = 0x2010;
static const uint16_t COIL_IR_MONO_STATE_END          = 0x2012;
static const uint16_t COIL_IR_LATCHED_STATE_START     = 0x2020;
static const uint16_t COIL_IR_LATCHED_STATE_END       = 0x2022;
static const uint16_t COIL_BLINK_LEDS                 = 0x2030;
static const uint16_t COIL_LOCK_STATE_START           = 0x2040;
static const uint16_t COIL_LOCK_STATE_END             = 0x2042;
static const uint16_t COIL_OTA_UPDATE_START_COMMAND   = 0xA010;
static const uint16_t COIL_OTA_UPDATE_FINISH_COMMAND  = 0xA011;
static const uint16_t COIL_OTA_UPDATE_ABORT_COMMAND   = 0xA012;

// Register table
static const uint16_t REGISTER_LOCK_DURATION          = 0x1020;
static const uint16_t REGISTER_CURRENT_VERSION_START  = 0xA000;  
static const uint16_t REGISTER_CURRENT_VERSION_END    = 0xA003;
static const uint16_t REGISTER_OTA_UPDATE_DATA_START  = 0xA020; 

static void _saveIrConfig() {
  mailbox_ir_config_t config;
  config.trio1Enabled = ir_manager_getTrioEnabled(0);
  config.trio2Enabled = ir_manager_getTrioEnabled(1);
  config.trio3Enabled = ir_manager_getTrioEnabled(2);
  config.trio4Enabled = ir_manager_getTrioEnabled(3);
  config.mono1Enabled = ir_manager_getMonoEnabled(0);
  config.mono2Enabled = ir_manager_getMonoEnabled(1);
  saveIrConfig(&config);
};

static void _saveLockConfig() {
  mailbox_lock_config_t config;
  config.lock1Enabled = lock_manager_getLockEnabled(0);
  config.lock2Enabled = lock_manager_getLockEnabled(1);
  config.lockDurationMs = lock_manager_getLockDurationMs();
  saveLockConfig(&config);
}

static sa_modbus_ret_t readCoilsCb(
  uint16_t startingAddress,
  uint16_t quantityOfCoils,
  uint8_t *coilStatus,
  uint8_t *byteCount
) {
  ESP_LOGI(TAG, "Reading coils (startingAddress=0x%" PRIx16 ", quantityOfCoils=%" PRIu16 ")...", startingAddress, quantityOfCoils);
  if (startingAddress >= COIL_IR_TRIO_ENABLED_START && startingAddress < COIL_IR_TRIO_ENABLED_END) {
    if (startingAddress + quantityOfCoils > COIL_IR_TRIO_ENABLED_END) return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
    coilStatus[0] = 0;
    for (int i = startingAddress - COIL_IR_TRIO_ENABLED_START; i < quantityOfCoils; ++i) {
      coilStatus[0] |= (ir_manager_getTrioEnabled(i) << i);
    }
    *byteCount = 1;
    return SA_MODBUS_OK;
  } else if (startingAddress >= COIL_IR_MONO_ENABLED_START && startingAddress < COIL_IR_MONO_ENABLED_END) {
    if (startingAddress + quantityOfCoils > COIL_IR_MONO_ENABLED_END) return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
    coilStatus[0] = 0;
    for (int i = startingAddress - COIL_IR_MONO_ENABLED_START; i < quantityOfCoils; ++i) {
      coilStatus[0] |= (ir_manager_getMonoEnabled(i) << i);
    }
    *byteCount = 1;
    return SA_MODBUS_OK;
  } else if (startingAddress >= COIL_LOCK_ENABLED_START && startingAddress < COIL_LOCK_ENABLED_END) {
    coilStatus[0] = 0;
    for (int i = startingAddress - COIL_LOCK_ENABLED_START; i < quantityOfCoils; ++i) {
      coilStatus[0] |= (ir_manager_getMonoEnabled(i) << i);
    }
    *byteCount = 1;
    return SA_MODBUS_OK;
  } else if (startingAddress >= COIL_LED_ENABLED_START && startingAddress < COIL_LED_ENABLED_END) {
    if (startingAddress + quantityOfCoils > COIL_LED_ENABLED_END) return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
    coilStatus[0] = 0;
    for (int i = startingAddress - COIL_LED_ENABLED_START; i < quantityOfCoils; ++i) {
      coilStatus[0] |= (led_manager_getLedEnabled(i) << i);
    }
    *byteCount = 1;
    return SA_MODBUS_OK;
  } else if (startingAddress >= COIL_IR_TRIO_STATE_START && startingAddress < COIL_IR_TRIO_STATE_END) {
    if (startingAddress + quantityOfCoils > COIL_IR_TRIO_STATE_END) return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
    coilStatus[0] = 0;
    coilStatus[1] = 0;
    uint8_t states[12];
    ir_manager_getAllTrioStates(states);
    int j = 0;
    for (int i = startingAddress - COIL_IR_TRIO_STATE_START; i < quantityOfCoils; ++i, ++j) {
      coilStatus[j / 8] |= (states[i] << j);
    }
    *byteCount = j / 8 + 1; 
    return SA_MODBUS_OK;
  } else if (startingAddress >= COIL_IR_MONO_STATE_START && startingAddress < COIL_IR_MONO_STATE_END) {
    if (startingAddress + quantityOfCoils > COIL_IR_MONO_STATE_END) return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
    coilStatus[0] = 0;
    uint8_t states[2];
    ir_manager_getAllMonoStates(states);
    for (int i = startingAddress - COIL_IR_MONO_STATE_START, j = 0; i < quantityOfCoils; ++i, ++j) {
      coilStatus[0] |= (states[i] << j);
    }
    *byteCount = 1;
    return SA_MODBUS_OK;
  } else if (startingAddress >= COIL_IR_LATCHED_STATE_START && startingAddress < COIL_IR_LATCHED_STATE_END) {
    if (startingAddress + quantityOfCoils > COIL_IR_LATCHED_STATE_END) return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
    coilStatus[0] = 0;
    if (startingAddress == COIL_IR_LATCHED_STATE_START) {
      coilStatus[0] = ir_manager_getTrioLatched();
      if (quantityOfCoils > 1) {
        coilStatus[0] |= (ir_manager_getMonoLatched() << 1);
      }
    } else {
      coilStatus[0] = ir_manager_getMonoLatched();
    }
    *byteCount = 1;
    return SA_MODBUS_OK;
  } else if (startingAddress >= COIL_LOCK_STATE_START && startingAddress < COIL_LOCK_STATE_END) {
    if (startingAddress + quantityOfCoils > COIL_LOCK_STATE_END) return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
    coilStatus[0] = 0;
    uint8_t states[2];
    lock_manager_getAllLockStates(states);
    for (int i = startingAddress - COIL_LOCK_STATE_START, j = 0; i < quantityOfCoils; ++i, ++j) {
      coilStatus[0] |= (states[i] << j);
    }
    *byteCount = 1;
    return SA_MODBUS_OK;
  } else {
    return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
  }
}

static sa_modbus_ret_t readHoldingRegistersCb(
  uint16_t startingAddress,
  uint16_t quantityOfRegisters,
  uint8_t *registerValue,
  uint8_t *byteCount
) {
  ESP_LOGI(TAG, "Reading holding registers (startingAddress=0x%" PRIx16 ", quantityOfRegisters=%" PRIu16 ")...", startingAddress, quantityOfRegisters);
  if (startingAddress == REGISTER_LOCK_DURATION) {
    uint16_t lockDurationMs = lock_manager_getLockDurationMs();
    writeUInt16BE(registerValue, lockDurationMs, 0);
    *byteCount = 2;
    return SA_MODBUS_OK;
  } else if (startingAddress == REGISTER_CURRENT_VERSION_START) {
    if (startingAddress + quantityOfRegisters > REGISTER_CURRENT_VERSION_END) return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
    for (int i = startingAddress - REGISTER_CURRENT_VERSION_START; i < quantityOfRegisters; ++i) {
      writeUInt16BE(registerValue, currentVersion[i], i * 2);
    }
    *byteCount = quantityOfRegisters * 2;
    return SA_MODBUS_OK;
  } else {
    return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
  }
}

static sa_modbus_ret_t writeSingleCoilCb(uint16_t outputAddress, uint16_t outputValue) {
  ESP_LOGI(TAG, "Writing single coil (outputAddress=0x%" PRIx16 ", outputValue=%" PRIx16 ")...", outputAddress, outputValue);
  if (outputAddress >= COIL_IR_TRIO_ENABLED_START && outputAddress < COIL_IR_TRIO_ENABLED_END) {
    ir_manager_setTrioEnabled(outputAddress - COIL_IR_TRIO_ENABLED_START, outputValue == 0xFF00);
    _saveIrConfig();
    return SA_MODBUS_OK;
  } else if (outputAddress >= COIL_IR_MONO_ENABLED_START && outputAddress < COIL_IR_MONO_ENABLED_END) {
    ir_manager_setMonoEnabled(outputAddress - COIL_IR_MONO_ENABLED_START, outputValue == 0xFF00);
    _saveIrConfig();
    return SA_MODBUS_OK;
  } else if (outputAddress >= COIL_LOCK_ENABLED_START && outputAddress < COIL_LOCK_ENABLED_END) {
    lock_manager_setLockEnabled(outputAddress - COIL_LOCK_ENABLED_START, outputValue == 0xFF00);
    _saveLockConfig();
    return SA_MODBUS_OK;
  } else if (outputAddress >= COIL_LED_ENABLED_START && outputAddress < COIL_LED_ENABLED_END) {
    return SA_MODBUS_OK;
  } else if (outputAddress >= COIL_IR_LATCHED_STATE_START && outputAddress < COIL_IR_LATCHED_STATE_END) {
    if (outputAddress == COIL_IR_LATCHED_STATE_START) {
      ir_manager_resetTrioLatched();
    } else {
      ir_manager_resetMonoLatched();
    }
    return SA_MODBUS_OK;
  } else if (outputAddress == COIL_BLINK_LEDS) {
    led_manager_blinkLeds();
    return SA_MODBUS_OK;
  } else if (outputAddress >= COIL_LOCK_STATE_START && outputAddress < COIL_LOCK_STATE_END) {
    lock_manager_unlock(outputAddress - COIL_LOCK_STATE_START);
    return SA_MODBUS_OK;
  } else if (outputAddress == COIL_OTA_UPDATE_START_COMMAND) {
    if (!sa_ota_begin(false)) return SA_MODBUS_SERVER_DEVICE_FAILURE;
    return SA_MODBUS_OK;
  } else if (outputAddress == COIL_OTA_UPDATE_FINISH_COMMAND) {
    if (!sa_ota_end()) return SA_MODBUS_SERVER_DEVICE_FAILURE;
    return SA_MODBUS_OK;
  } else if (outputAddress == COIL_OTA_UPDATE_ABORT_COMMAND) {
    sa_ota_abort();
    return SA_MODBUS_OK;
  } else {
    return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
  }
}

static sa_modbus_ret_t writeSingleRegisterCb(
  uint16_t registerAddress,
  uint16_t registerValue
) {
  ESP_LOGI(TAG, "Writing single register (registerAddress=0x%" PRIx16 ", registerValue=%" PRIu16 ")...", registerAddress, registerValue);
  if (registerAddress == REGISTER_LOCK_DURATION) {
    lock_manager_setLockDurationMs(registerValue);
    _saveLockConfig();
    return SA_MODBUS_OK;
  } else {
    return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
  }
}

static sa_modbus_ret_t writeMultipleRegistersCb(
  uint16_t startingAddress,
  uint16_t quantityOfRegisters,
  const uint8_t *registersValue,
  uint8_t byteCount
) {
  ESP_LOGI(TAG, "Writing multiple registers (startingAddress=0x%" PRIx16 ", quantityOfRegisters=%" PRIu16 ")...", startingAddress, quantityOfRegisters);
  if (startingAddress == REGISTER_OTA_UPDATE_DATA_START) {
    if (!sa_ota_write(registersValue, byteCount)) return SA_MODBUS_SERVER_DEVICE_FAILURE;
    return SA_MODBUS_OK;
  } else {
    return SA_MODBUS_ILLEGAL_DATA_ADDRESS;
  }
}

bool modbus_master_init() {
  if (s_isInit) return true;
  gpio_config_t ioConf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL << s_dsIoNums[0] | 1ULL << s_dsIoNums[1] | 1ULL << s_dsIoNums[2] | 1ULL << s_dsIoNums[3] | 1ULL << s_dsIoNums[4] | 1ULL << s_dsIoNums[5] | 1ULL << s_dsIoNums[6] | 1ULL << s_dsIoNums[7]),
    .pull_down_en = 1,
    .pull_up_en = 0,
  };
  //configue server address
  gpio_config(&ioConf);
  uint8_t serverAddress = 0;
  for (int i = 0; i < 8; ++i) {
    serverAddress |= gpio_get_level(s_dsIoNums[i]) << i;
  }
  if (serverAddress == 0) {
    serverAddress = 127;
  }
  ESP_LOGI(TAG, "Server address: %u", serverAddress);
  sa_modbus_init(1, 38, 40, 39, serverAddress, readCoilsCb, readHoldingRegistersCb, writeSingleCoilCb, writeSingleRegisterCb, writeMultipleRegistersCb);
  const esp_app_desc_t *apDesc = esp_app_get_description();
  sscanf(apDesc->version, "%hu.%hu.%hu", &currentVersion[0], &currentVersion[1], &currentVersion[2]);
  ESP_LOGI(TAG, "Current version: %hu.%hu.%hu", currentVersion[0], currentVersion[1], currentVersion[2]);
  s_isInit = true;
  return true;
}

bool modbus_master_start() {
  if (!s_isInit) {
    ESP_LOGE(TAG, "Init first");
    return false;
  }
  sa_modbus_start();
  return true;
}

bool modbus_master_stop() {
  sa_modbus_stop();
  return true;
}

bool modbus_master_deinit() {
  return true;
}
