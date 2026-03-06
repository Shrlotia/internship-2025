#pragma once

#include <stdio.h>
#include <stdint.h>

typedef enum {
  SA_MODBUS_OK,
  SA_MODBUS_ILLEGAL_FUNCTION,
  SA_MODBUS_ILLEGAL_DATA_ADDRESS,
  SA_MODBUS_ILLEGAL_DATA_VALUE,
  SA_MODBUS_SERVER_DEVICE_FAILURE,
} sa_modbus_ret_t;

typedef sa_modbus_ret_t (*sa_modbus_readCoils_cb)(uint16_t startingAddress, uint16_t quantityOfCoils, uint8_t *coilStatus, uint8_t *byteCount);
typedef sa_modbus_ret_t (*sa_modbus_readHoldingRegisters_cb)(uint16_t startingAddress, uint16_t quantityOfRegisters, uint8_t *registerValue, uint8_t *byteCount);
typedef sa_modbus_ret_t (*sa_modbus_writeSingleCoil_cb)(uint16_t outputAddress, uint16_t outputValue);
typedef sa_modbus_ret_t (*sa_modbus_writeSingleRegister_cb)(uint16_t registerAddress, uint16_t registerValue);
typedef sa_modbus_ret_t (*sa_modbus_writeMultipleRegisters_cb)(uint16_t startingAddress, uint16_t quantityOfRegisters, const uint8_t *registersValue, uint8_t byteCount);

void sa_modbus_init(
  int portNum,
  int txIoNum,
  int rxIoNum,
  int rtsIoNum,
  uint8_t serverAddr,
  sa_modbus_readCoils_cb readCoilsCb,
  sa_modbus_readHoldingRegisters_cb readHoldingRegistersCb,
  sa_modbus_writeSingleCoil_cb writeSingleCoilCb,
  sa_modbus_writeSingleRegister_cb writeSingleRegisterCb,
  sa_modbus_writeMultipleRegisters_cb writeMultipleRegistersCb
);

void sa_modbus_start();
void sa_modbus_stop();
