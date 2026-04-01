#pragma once
#include <cstdint>

#define START_FRAMEH 0X55
#define START_FRAMEL 0X55

typedef struct {
    uint8_t start_frameH;
    uint8_t start_frameL;
    uint8_t data_size;
    uint8_t device_address;
    uint8_t data[58];
    uint16_t crc;
} packet_t;


