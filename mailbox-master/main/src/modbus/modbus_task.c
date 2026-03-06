#include "modbus_task.h"

#include "esp_log.h"

#include "event/modbus_return.h"
#include "modbus/modbus.h"
#include "modbus/modbus_data_api.h"
#include "mailbox/mailbox.h"
#include "mailbox/modbus_event_handler.h"
#include "modbus/modbus_master.h"
#include "utils.h"

static const char *TAG = "TASK";

// Define the ranges for coils and registers
static const uint8_t allCoilUseStartAddress = 0;
static const uint8_t allCoilUseEndAddress = 11;
static const uint8_t allRegisterUseStartAddress = 12;
static const uint8_t allRegisterUseEndAddress = 14;

static const uint8_t defaultCoilUse[] = {5, 6, 7, 9};
static uint8_t defaultCoilUseCount = sizeof(defaultCoilUse) / sizeof(defaultCoilUse[0]);

static const uint8_t defaultRegisterUse[] = {13};
static uint8_t defaultRegisterUseCount = sizeof(defaultRegisterUse) / sizeof(defaultRegisterUse[0]);

static uint8_t customCoilUse[];
customCoilUse[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static uint8_t customCoilUseCount = sizeof(customCoilUse) / sizeof(customCoilUse[0]);

static uint8_t customRegisterUse[];
customRegisterUse[] = {12, 13, 14};
static uint8_t customRegisterUseCount = sizeof(customRegisterUse) / sizeof(customRegisterUse[0]);

ret_t differentValueCb(uint8_t slaveAddress, uint8_t rangeId, uint16_t count) {
  modbus_t modbusType = getRangeIdType(rangeId);
  if (modbusType == MODBUS_COIL) {
    bool currentCoilValue;
    bool recievedCoilValue;
    for (uint8_t i = 0; i < count; i++) {
      if (getCurrentSettingCoilValue(rangeId, i, &currentCoilValue, NULL) == DATA_FAIL) return DATA_FAIL;
      if (getRecievedSettingCoilValue(rangeId, i, &recievedCoilValue, NULL) == DATA_FAIL) return DATA_FAIL;
      if (currentCoilValue != recievedCoilValue) {
        modbus_function_t setting = getFunctionTypeByRangeId(rangeId);
        creatSingleCoilServerEvent(setting, slaveAddress, i++, recievedCoilValue);
        setCurrentSettingCoilValue(rangeId, i, recievedCoilValue, NULL);
      }
    }
    return TASK_OK;
  }
  if (modbusType == MODBUS_REGISTER) {
    uint16_t currentRegisterValue;
    uint16_t recievedRegisterValue;
    for (uint8_t i = 0; i < count; i++) {
      if (getCurrentSettingRegisterValue(rangeId, currentRegisterValue) == DATA_FAIL) return DATA_FAIL;
      if (getRecievedSettingRegisterValue(rangeId, recievedRegisterValue) == DATA_FAIL) return DATA_FAIL;
      if (currentRegisterValue != recievedRegisterValue) {
        modbus_function_t setting = getFunctionTypeByRangeId(rangeId);
        creatSingleRegisterServerEvent(setting, slaveAddress, recievedRegisterValue);
        setCurrentSettingRegisterValue(rangeId, recievedRegisterValue);
      }
    }
    return TASK_OK;
  }
}

// Funtion 01 get Setting
ret_t getCoilSetting(uint8_t slaveAddress, uint8_t rangeId) {
  uint8_t buf[256];
  uint8_t slaveId = getSlaveID(slaveAddress);
  uint16_t start;
  uint16_t count;
  if (getSlaveRangeStart(rangeId, &start) == DATA_FAIL) return DATA_FAIL;
  if (getSlaveRangeCount(rangeId, &count) == DATA_FAIL) return DATA_FAIL;
  if (sendReadCoilRegister(slaveAddress, start, count, &buf) != MODBUS_OK) return MODBUS_FAIL;
  ESP_LOGI(TAG, "Setting salve %d", slaveAddress);
  if (setReadCoilRegistersRes(slaveAddress, rangeId, buf) != DATA_OK) return DATA_FAIL;
  ESP_LOGI(TAG, "Coil range read successfully for slave %d", slaveAddress);
  if (!areArrayEqual(rangeId, slaveAddress)) {
    if (differentValueCb(slaveAddress, rangeId, count) == DATA_FAIL) return DATA_FAIL;
  }
  return TASK_OK;
}

// Function 03 get Setting
ret_t getRegisterSetting(uint8_t slaveAddress, uint8_t rangeId) {
  uint8_t buf[256];
  uint16_t start;
  uint16_t count;
  getSlaveRangeStart(rangeId, &start);
  getSlaveRangeCount(rangeId, &count);
  if (sendReadHoldingRegister(slaveAddress, start, count, &buf) != MODBUS_OK) return TASK_FAIL;
  ESP_LOGI(TAG, "Setting to salve %d", slaveAddress);
  if (setReadHoldingRegistersRes(slaveAddress, rangeId, buf) != DATA_OK) return TASK_FAIL;
  ESP_LOGI(TAG, "Register range read successfully for slave %d", slaveAddress);
  if (!areArrayEqual)
}

ret_t loopSetting(uint8_t slaveAddress, modbus_loop_t settings) {
  switch (settings) {
    case ALL_COIL_USE:
      //Function 01 read allCoil
      for (int i = allCoilUseStartAddress; i < allCoilUseEndAddress; i++){
        if (getCoilSetting(slaveAddress, i) == TASK_FAIL) return TASK_FAIL;
      }
      return TASK_OK;
    case ALL_REGISTER_USE:
      //Function 03 read allRegister
      for (int i = allRegisterUseStartAddress; i < allRegisterUseEndAddress; i++){
        if (getRegisterSetting(slaveAddress, i) == TASK_FAIL) return TASK_FAIL;
      }
      return TASK_OK;
    case DEFAULT_COIL_USE:
      //Function 01 read defaultCoilUse
      for (int i = 0; i < defaultCoilUseCount; i++) {
        uint8_t rangeId = defaultCoilUse[i];
        if (getCoilSetting(slaveAddress, rangeId) == TASK_FAIL) return TASK_FAIL;
      }
      return TASK_OK;
    case DEFAULT_REGISTER_USE:
      //Function 03 read defaultRegisterUse
      for (int i = 0; i < defaultRegisterUseCount; i++) {
        uint8_t rangeId = defaultRegisterUse[i];
        if (getRegisterSetting(slaveAddress, rangeId) == TASK_FAIL) return TASK_FAIL;
      }
      return TASK_OK;
    case CUSTOM_COIL_USE:
      //Function 01 read customCoilUse
      for (int i = 0; i < customCoilUseCount; i++) {
        uint8_t rangeId = customCoilUse[i];
        if (getCoilSetting(slaveAddress, rangeId) == TASK_FAIL) return TASK_FAIL;
      }
      return TASK_OK;
    case CUSTOM_REGISTER_USE:
      //Function 03 read customRegisterUse
      for (int i = 0; i < customRegisterUseCount; i++) {
        uint8_t rangeId = customRegisterUse[i];
        if (getRegisterSetting(slaveAddress, rangeId) == TASK_FAIL) return TASK_FAIL;
      }
      return TASK_OK;
    default:
      return TASK_FAIL;
  }
}

void requestSetting(modbus_request_t allLoopType) {
  modbus_loop_t coilLoopSetting;
  modbus_loop_t registerLoopSetting;
  switch(allLoopType) {
    case ALL_USE:
      coilLoopSetting = ALL_COIL_USE;
      registerLoopSetting = ALL_REGISTER_USE;
      break;
    case DEFAULT_USE:
      coilLoopSetting = DEFAULT_COIL_USE;
      registerLoopSetting = DEFAULT_REGISTER_USE;
      break;
    case CUSTOM_USE:
      coilLoopSetting = CUSTOM_COIL_USE;
      registerLoopSetting = CUSTOM_REGISTER_USE;
      break;
    default:
      coilLoopSetting = ALL_COIL_USE;
      registerLoopSetting = ALL_REGISTER_USE;
      break;
  }
  uint8_t num_of_slave = getNumOfSlaves();
  for (uint8_t i = 0; i < num_of_slave; i++) {
    uint8_t slaveAddress = getSlaveAddressBySlaveId(i);
    if (loopSetting(slaveAddress, coilLoopSetting) == TASK_FAIL) return;
    if (loopSetting(slaveAddress, registerLoopSetting) == TASK_FAIL) return;
  }
}

void requestCoilUpdate(uint8_t slaveAddress, uint8_t rangeId, uint8_t coilId, bool value) {
  uint16_t start;
  getSlaveRangeStart(rangeId, &start);
  uint16_t coilAddress = start + coilId;
  uint16_t coilValue = getCoilValue(value);
  sendWriteSingleCoil(slaveAddress, coilId, coilValue);
}

void requestRegisterUpdate(uint8_t slaveAddress, uint8_t rangeId, uint16_t value) {
  uint16_t start;
  getSlaveRangeStart(rangeId, &start);
  sendWriteSingleRegister(slaveAddress, start, value);
}


ret_t slaveAddressChecking(uint8_t slaveAddress) {
  if(getSlaveID(slaveAddress) == 255) return TASK_FAIL;
  return TASK_OK;
}

task_ret_t coilNumberChecking(uint8_t rangeId, uint8_t coilNumber) {
  uint8_t num_of_range_map = getNumOfRangeMap();
  const Range *slaveRanges = getSlaveRanges();
  if(rangeId >= num_of_range_map || coilNumber > slaveRanges[rangeId].count) return TASK_FAIL;
  return TASK_OK;
}

uint8_t getBooleanCount(bool array) {
  return sizeof(array) / sizeof(array[0]);
}

modbus_t getRangeIdType(uint8_t rangeId) {
  if (rangeId >= allCoilUseStartAddress && rangeId <= allCoilUseEndAddress) return MODBUS_COIL;
  if (rangeId >= allRegisterUseStartAddress && rangeId <= allRegisterUseEndAddress) return MODBUS_REGISTER;
}

void pasteValueToStructure(modbus_data_t *setting) {
  if (setting->Coil_IR_Trio != NULL) setting->Coil_IR_Trio = desiredSettings.Coil_IR_Trio;
  if (setting->Coil_IR_Mono != NULL) setting->Coil_IR_Mono = desiredSettings.Coil_IR_Mono;
  if (setting->Coil_Lock != NULL) setting->Coil_Lock = desiredSettings.Coil_Lock;
  if (setting->Coil_IR_Trio_State != NULL) setting->Coil_IR_Trio_State = desiredSettings.Coil_IR_Trio_State;
  if (setting->Coil_IR_Mono_State != NULL) setting->Coil_IR_Mono_State = desiredSettings.Coil_IR_Mono_State;
  if (setting->Coil_IR_Latched_State != NULL) setting->Coil_IR_Latched_State = desiredSettings.Coil_IR_Latched_State;
  if (setting->Coil_Blink_LEDS != NULL) setting->Coil_Blink_LEDS = desiredSettings.Coil_Blink_LEDS;
  if (setting->Coil_Lock_State != NULL) setting->Coil_Lock_State = desiredSettings.Coil_Lock_State;
  if (setting->Coil_OTA_Update_Start_Command != NULL) setting->Coil_OTA_Update_Start_Command = desiredSettings.Coil_OTA_Update_Start_Command;
  if (setting->Coil_OTA_Update_Finish_Command != NULL) setting->Coil_OTA_Update_Finish_Command = desiredSettings.Coil_OTA_Update_Finish_Command;
  if (setting->Coil_OTA_Update_Abort_Command != NULL) setting->Coil_OTA_Update_Abort_Command = desiredSettings.Coil_OTA_Update_Abort_Command;
  if (setting->Register_Lock_Duration != NULL) setting->Register_Lock_Duration = desiredSettings.Register_Lock_Duration;
  if (setting->Register_Current_Version != NULL) setting->Register_Current_Version = desiredSettings.Register_Current_Version;
  if (setting->Register_OTA_Update_Data != NULL) setting->Register_OTA_Update_Data = xdesiredSettings.Register_OTA_Update_Data; 
}