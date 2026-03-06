#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

QueueHandle_t modbusQueue;
QueueHandle_t serverQueue;

void mailbox_init();