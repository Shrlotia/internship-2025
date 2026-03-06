#pragma once

#include <stdbool.h>
#include <stdint.h>

bool ir_manager_init();
bool ir_manager_deinit();
bool ir_manager_start();
bool ir_manager_stop();

bool ir_manager_getTrioEnabled(uint8_t boardNum);
bool ir_manager_getMonoEnabled(uint8_t boardNum);
bool ir_manager_setTrioEnabled(uint8_t boardNum, bool enabled);
bool ir_manager_setMonoEnabled(uint8_t boardNum, bool enabled);

bool ir_manager_getAllTrioStates(uint8_t *states);
bool ir_manager_getAllMonoStates(uint8_t *states);

bool ir_manager_getTrioLatched();
bool ir_manager_getMonoLatched();
void ir_manager_resetTrioLatched();
void ir_manager_resetMonoLatched();
