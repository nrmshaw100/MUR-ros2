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

#define OUTPUT_CMD_LIST(X) \
    X(STATUS, 0) \
    X(DATA, 1) \
    X(THRUST_CMD, 2) \
    X(MOD_R1, 3) \
    X(MOD_R2, 4) \
    X(MOD_R3, 5) \
    X(MOD_R4, 6) \
    X(MOD_R5, 7) \
    X(MOD_R6, 8) \
    X(UPDATE_T, 9) 

typedef enum {
    #define X(name, value) name = value,
    OUTPUT_CMD_LIST(X)
    #undef X
} output_cmd_t;