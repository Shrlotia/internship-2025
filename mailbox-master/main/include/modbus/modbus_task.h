#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  ALL_USE,
  DEFAULT_USE,
  CUSTOM_USE,
} modbus_request_t;

ret_t getCoilSetting(uint8_t slaveAddress, uint8_t rangeId);
ret_t getRegisterSetting(uint8_t slaveAddress, uint8_t rangeId);
ret_t loopSetting(uint8_t slaveAddress, modbus_loop_t settings);
void requestSetting(modbus_request_t allLoopType);

void requestCoilUpdate(uint8_t slaveAddress, uint8_t rangeId, uint8_t coilId, bool value);
void requestRegisterUpdate(uint8_t slaveAddress, uint8_t rangeId, uint16_t value);

ret_t slaveAddressChecking(uint8_t slaveAddress);
modbus_t getRangeIdType(uint8_t rangeId);

void modbus_data_init();