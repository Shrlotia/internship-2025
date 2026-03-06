#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "event/modbus_data_event.h"
#include "event/modbus_init_config.h"
#include "event/modbus_loop_config.h"

typedef enum {
  EV_INIT_REQUEST,
  EV_DATA_REQUEST,
  EV_WRITE_REQUEST,
	EV_CONFIG_SLAVE_INIT,
} modbus_event_handler_event_t;

typedef struct {
	event_t event;
  TaskHandle_t taskHandle;
	union {
		modbus_data_param_t *setting;
    modbus_loop_config_param_t *loopConfig;
    modbus_slave_init_param_t *initConfig;
	} params;
}	modbus_event_handler_t;