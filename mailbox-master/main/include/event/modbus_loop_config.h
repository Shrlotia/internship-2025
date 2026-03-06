#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  ALL_COIL_USE,
  ALL_REGISTER_USE,
  DEFAULT_COIL_USE,
  DEFAULT_REGISTER_USE,
  CUSTOM_COIL_USE,
  CUSTOM_REGISTER_USE,
} modbus_loop_t;

typedef enum {
	LOOP_TYPE,
	LOOP_COIL_SETTER,
	LOOP_REGISTER_SETTER,
} loop_t;

typedef struct {
	loop_t function;
	union {
		modbus_loop_t method;
		uint8_t *value;
	} loop_data;
} modbus_loop_config_param_t;