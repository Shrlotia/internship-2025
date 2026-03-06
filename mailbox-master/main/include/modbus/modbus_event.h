#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "event/modbus_data_event.h"
#include "event/modbus_loop_config.h"
#include "event/modbus_init_config.h"

typedef enum {
  EV_MODBUS_READ_ALL,
  EV_MODBUS_READ_DEFAULT,
  EV_MODBUS_READ_CUSTOM,
  EV_MODBUS_WRITE,
  EV_MODBUS_INIT,
  EV_MODBUS_CONFIG,
} modbus_event_t;

typedef struct {
	modbus_event_t event;
	union {
		modbus_data_param_t *setting;
    modbus_loop_config_param_t *loopConfig;
    modbus_slave_init_param_t *initConfig;
	} params;
}	modbus_event;