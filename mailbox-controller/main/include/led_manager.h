#pragma once

#include <stdbool.h>
#include <stdint.h>

bool led_manager_init();
bool led_manager_deinit();
bool led_manager_start();
bool led_manager_stop();

bool led_manager_getLedEnabled(uint8_t ledNum);
bool led_manager_setLedEnabled(uint8_t ledNum, bool enabled);
bool led_manager_turnOnLed(uint8_t ledNum);
bool led_manager_turnOffLed(uint8_t ledNum);
bool led_manager_blinkLeds();