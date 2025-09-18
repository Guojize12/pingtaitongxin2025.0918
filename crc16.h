#pragma once

#include <Arduino.h>

// CRC-16/MODBUS (init 0xFFFF, poly 0xA001, LSB-first)
uint16_t crc16_modbus(const uint8_t* data, size_t len);