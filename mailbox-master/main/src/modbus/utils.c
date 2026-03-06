#include "utils.h"

uint16_t readUInt16BE(const uint8_t *buf, size_t offset) {
  return (buf[offset] << 8) | buf[offset + 1];
}

uint16_t readUInt16LE(const uint8_t *buf, size_t offset) {
  return (buf[offset + 1] << 8) | buf[offset];
}

void writeUInt16BE(uint8_t *buf, uint16_t value, size_t offset) {
  buf[offset] = value >> 8;
  buf[offset + 1] = value & 0xFF;
}

void writeUInt16LE(uint8_t *buf, uint16_t value, size_t offset) {
  buf[offset] = value & 0xFF;
  buf[offset + 1] = value >> 8;
}
