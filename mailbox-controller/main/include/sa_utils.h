#pragma once

#include <stddef.h>
#include <stdint.h>

uint16_t readUInt16BE(const uint8_t *buf, size_t offset);
uint16_t readUInt16LE(const uint8_t *buf, size_t offset);

void writeUInt16BE(uint8_t *buf, uint16_t value, size_t offset);
void writeUInt16LE(uint8_t *buf, uint16_t value, size_t offset);
