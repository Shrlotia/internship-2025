#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
	MODBUS_OK,
	MODBUS_FAIL,
	DATA_OK,
	DATA_FAIL,
	TASK_OK,
	TASK_FAIL,
	CHECKING_OK,
	CHECKING_FAIL,
	MODBUS_EVENT_OK,
	MODBUS_EVENT_FAIL,
	SERVER_EVENT_OK,
	SERVER_EVENT_FAIL,
} ret_t;