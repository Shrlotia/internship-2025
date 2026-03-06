#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
	INIT_SLAVE_BY_SLAVEADDRESS,
	INIT_SLAVE_BY_NUM_OF_SLAVES,
} modbus_slave_init_t;

typedef struct {
	modbus_slave_init_t method;
	union {
		uint8_t *slaveId;
		uint8_t numOfSlaves;
	} slave_init_data;
}	modbus_slave_init_param_t;