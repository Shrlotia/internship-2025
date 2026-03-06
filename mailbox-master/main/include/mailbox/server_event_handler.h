#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "event/modbus_data_event.h"

typedef enum {
	EV_MODBUS_DIFF_RES,
	EV_MODBUS_STATE_RES,
} server_event_handler_event_t;

typedef struct {
	server_event_handler_event_t event;
	modbus_data_param_t *settings;
}	server_event_handler_t;