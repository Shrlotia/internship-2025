#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "modbus/modbus_function.h"

#define MAX_SLAVE_ID 247

// Modbus Coil Table Definitions
#define COIL_IR_TRIO_ENABLED_START      0x1000
#define COIL_IR_TRIO_ENABLED_END        0x1004
#define COIL_IR_MONO_ENABLED_START      0x1010
#define COIL_IR_MONO_ENABLED_END        0x1012
#define COIL_LOCK_ENABLED_START         0x1020
#define COIL_LOCK_ENABLED_END           0x1022
#define COIL_LED_ENABLED_START          0x1030
#define COIL_LED_ENABLED_END            0x1033
#define COIL_IR_TRIO_STATE_START        0x2000
#define COIL_IR_TRIO_STATE_END          0x200C
#define COIL_IR_MONO_STATE_START        0x2010
#define COIL_IR_MONO_STATE_END          0x2012
#define COIL_IR_LATCHED_STATE_START     0x2020
#define COIL_IR_LATCHED_STATE_END       0x2022
#define COIL_BLINK_LEDS                 0x2030
#define COIL_LOCK_STATE_START           0x2040
#define COIL_LOCK_STATE_END             0x2042
#define COIL_OTA_UPDATE_START_COMMAND   0xA010
#define COIL_OTA_UPDATE_FINISH_COMMAND  0xA011
#define COIL_OTA_UPDATE_ABORT_COMMAND   0xA012

// Modbus Register Table Definitions
#define REGISTER_LOCK_DURATION          0x1020
#define REGISTER_CURRENT_VERSION_START  0xA000
#define REGISTER_CURRENT_VERSION_END    0xA003
#define REGISTER_OTA_UPDATE_DATA_START  0xA020

// Modbus Coil and Register Counts
#define COIL_IR_TRIO_ENABLED_COUNT      			(COIL_IR_TRIO_ENABLED_END - COIL_IR_TRIO_ENABLED_START)
#define COIL_IR_MONO_ENABLED_COUNT      			(COIL_IR_MONO_ENABLED_END - COIL_IR_MONO_ENABLED_START)
#define COIL_LOCK_ENABLED_COUNT         			(COIL_LOCK_ENABLED_END - COIL_LOCK_ENABLED_START)
#define COIL_LED_ENABLED_COUNT          			(COIL_LED_ENABLED_END - COIL_LED_ENABLED_START)
#define COIL_IR_TRIO_STATE_COUNT        			(COIL_IR_TRIO_STATE_END - COIL_IR_TRIO_STATE_START)
#define COIL_IR_MONO_STATE_COUNT        			(COIL_IR_MONO_STATE_END - COIL_IR_MONO_STATE_START)
#define COIL_IR_LATCHED_STATE_COUNT   				(COIL_IR_LATCHED_STATE_END - COIL_IR_LATCHED_STATE_START)
#define COIL_BLINK_LEDS_COUNT           			1
#define COIL_LOCK_STATE_COUNT           		  (COIL_LOCK_STATE_END - COIL_LOCK_STATE_START)
#define COIL_OTA_UPDATE_START_COMMAND_COUNT   1
#define COIL_OTA_UPDATE_FINISH_COMMAND_COUNT  1
#define COIL_OTA_UPDATE_ABORT_COMMAND_COUNT   1
#define REGISTER_LOCK_DURATION_COUNT     			1
#define REGISTER_CURRENT_VERSION_COUNT   			(REGISTER_CURRENT_VERSION_END - REGISTER_CURRENT_VERSION_START)
#define REGISTER_OTA_UPDATE_DATA_COUNT   			1

// Modbus data structure for holding slave information
typedef struct {
	uint8_t slaveAddress;
	bool Online;
	bool Changed;
	modbus_function_t setting;
	uint8_t index;
}	modbus_stage_t;

// Modbus data structure for holding coil and register values
typedef struct {
	uint8_t slaveAddress;
	bool Coil_IR_Trio[COIL_IR_TRIO_ENABLED_COUNT];
	bool Coil_IR_Mono[COIL_IR_MONO_ENABLED_COUNT];
	bool Coil_Lock[COIL_LOCK_ENABLED_COUNT];
	bool Coil_LED[COIL_LED_ENABLED_COUNT];
	bool Coil_IR_Trio_State[COIL_IR_TRIO_STATE_COUNT];
	bool Coil_IR_Mono_State[COIL_IR_MONO_STATE_COUNT];
	bool Coil_IR_Latched_State[COIL_IR_LATCHED_STATE_COUNT];
	bool Coil_Blink_LEDS[COIL_BLINK_LEDS_COUNT];
	bool Coil_Lock_State[COIL_LOCK_STATE_COUNT];
	bool Coil_OTA_Update_Start_Command[COIL_OTA_UPDATE_START_COMMAND_COUNT];
	bool Coil_OTA_Update_Finish_Command[COIL_OTA_UPDATE_FINISH_COMMAND_COUNT];
	bool Coil_OTA_Update_Abort_Command[COIL_OTA_UPDATE_ABORT_COMMAND_COUNT];
	uint16_t Register_Lock_Duration;
	uint16_t Register_Current_Version;
	uint16_t Register_OTA_Update_Data;
} modbus_data_t;