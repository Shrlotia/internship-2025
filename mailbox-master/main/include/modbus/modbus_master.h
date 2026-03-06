#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "mailbox/server_event_handler.h"

void sendServerEvent(server_event_handler_t event);
void creatSingleCoilServerEvent(modbus_function_t setting, uint8_t slaveAddress, uint8_t coilAddress, bool value);
void creatSingleRegisterServerEvent(modbus_function_t setting, uint8_t slaveAddress, uint16_t value);
void createAllSettingServerEvent(modbus_function_t setting, uint8_t slaveAddress, modbus_data_all_data *allSettings);

void modbus_master_init();
void modbus_master_start();
void modbus_master_stop();