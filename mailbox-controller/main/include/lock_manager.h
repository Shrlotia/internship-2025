#pragma once

#include <stdbool.h>
#include <stdint.h>

bool lock_manager_init();
bool lock_manager_deinit();
bool lock_manager_start();
bool lock_manager_stop();

uint16_t lock_manager_getLockDurationMs();
bool lock_manager_setLockDurationMs(uint16_t lockDurationMs);
bool lock_manager_getLockEnabled(uint8_t lockNum);
bool lock_manager_setLockEnabled(uint8_t lockNum, bool enabled);
bool lock_manager_getAllLockStates(uint8_t *states);
bool lock_manager_unlock(uint8_t lockNum);
bool lock_manager_unlockAll();