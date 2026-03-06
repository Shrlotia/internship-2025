#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

ret_t getCurrentSettingCoilValue(uint8_t rangeId, uint8_t *index, bool *value, size_t *value_size);
ret_t getCurrentSettingRegisterValue(uint8_t rangeId, uint16_t *value);
ret_t setCurrentSettingCoilValue(uint8_t rangeId, uint8_t *index, bool *values, size_t values_size);
ret_t setCurrentSettingRegisterValue(uint8_t rangeId, uint16_t value);

ret_t getRecievedSettingCoilValue(uint8_t rangeId, uint8_t *index, bool *value, size_t *value_size);
ret_t getRecievedSettingRegisterValue(uint8_t rangeId, uint16_t *value);

ret_t setDesiredSettingCoilValue(uint8_t rangeId, uint8_t *index, bool *values, size_t values_size);
ret_t setDesiredSettingRegisterValue(uint8_t rangeId, uint16_t value);

ret_t getSlaveRangeStart(uint8_t rangeId, uint16_t *value);
ret_t getSlaveRangeEnd(uint8_t rangeId, uint16_t *value);
ret_t getSlaveRangeCount(uint8_t rangeId, uint16_t *value);

// Functions to handle Modbus requests and responses
ret_t setReadCoilRegistersRes(uint16_t serverAddr, uint8_t rangeId, uint8_t *rxBuf);
ret_t setReadHoldingRegistersRes(uint8_t serverAddr, uint8_t rangeId, uint8_t *rxBuf);
uint8_t getRangeIdForModbus(uint16_t startingAddress, uint16_t quantityOfOutputs, uint8_t rangeSize);

// Function prototypes
uint16_t getCoilValue(bool coilState);
uint8_t getNumOfSlaves();
uint8_t getSlaveAddressBySlaveId (uint8_t slaveId);
uint8_t getSlaveID(uint8_t slaveAddress);
uint8_t getSlaveAddressBySetting(modbus_data_t *setting)
uint8_t getRangeIdByFunctionType(modbus_function_t setting);
modbus_function_t getFunctionTypeByRangeId(uint8_t rangeId)
void setSlaveAddress(modbus_data_t *setting, uint16_t slaveAddress);
bool areSettingsEqual(modbus_data_t *currentSettings, modbus_data_t *newSettings);
bool areArrayEqual(uint8_t rangeId, uint8_t slaveAddress);
bool areCoilValueEqual(uint8_t slaveAddress, uint8_t rangeId, uint8_t coilNumber, bool newValue);
bool areRegisterValueEqual(uint8_t slaveAddress, uint8_t rangeId, uint16_t newValue);
void pasteStructureToStructure(modbus_data_t *setting, modbus_data_t *target)

// Initialize Modbus data structures
void numOfSlavesReq();
void init_slaves(uint8_t numOfSlaves);
void init_slave_malloc(uint8_t num_of_slaves)
void modbus_data_init();