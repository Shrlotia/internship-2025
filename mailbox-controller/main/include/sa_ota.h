#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <wchar.h>

bool sa_ota_isInOtaMode();

bool sa_ota_begin(bool forceUpdate);
bool sa_ota_write(const void *data, size_t size);
bool sa_ota_end();
bool sa_ota_abort();
