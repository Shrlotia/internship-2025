#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "modbus/modbus_function.h"

typedef struct {
	bool *Coil_IR_Trio;
	bool *Coil_IR_Mono;
	bool *Coil_Lock;
	bool *Coil_LED;
	bool *Coil_IR_Trio_State;
	bool *Coil_IR_Mono_State;
	bool *Coil_IR_Latched_State;
	bool *Coil_Blink_LEDS;
	bool *Coil_Lock_State;
	bool *Coil_OTA_Update_Start_Command;
	bool *Coil_OTA_Update_Finish_Command;
	bool *Coil_OTA_Update_Abort_Command;
	uint16_t *Register_Lock_Duration;
	uint16_t *Register_Current_Version;
	uint16_t *Register_OTA_Update_Data;
} modbus_data_all_data;

typedef struct {
	uint8_t coilAddress;
	bool value;
} modbus_data_coil_data;

typedef struct {
	uint16_t value;
} modbus_data_register_data;

typedef struct {
	uint8_t slaveAddress;
	modbus_function_t function;
	union {
		modbus_data_all_data *settings;
		modbus_data_coil_data *coilValue;
		modbus_data_register_data *registerValue;
	} modbus_data;
} modbus_data_param_t;