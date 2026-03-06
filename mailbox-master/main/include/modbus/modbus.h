#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef void (*FHCallback)();
void registerCallback(FHCallback _callback);

typedef enum {
  MODBUS_OK,
  MODBUS_FAILED,
} modbus_ret_t;

modbus_ret_t sendReadCoilRegister(uint8_t slaveAddress, uint16_t startingAddress, uint16_t quantityOfOutputs, uint8_t *buf);
modbus_ret_t sendReadHoldingRegister(uint8_t slaveAddress, uint16_t startingAddress, uint16_t quantityOfRegisters, uint8_t *buf);
modbus_ret_t sendWriteSingleCoil(uint8_t slaveAddress, uint16_t coilAddress, uint16_t coilValue);
modbus_ret_t sendWriteSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t registerValue);

void modbus_init();

// void sendRequest();
void modbus_start();
void modbus_stop();