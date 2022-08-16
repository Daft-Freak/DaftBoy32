#pragma once
#include <cstdint>

struct BESSHeader
{
    char id[4];
    uint32_t len;
};

struct BESSCore
{
    uint16_t versionMajor;
    uint16_t versionMinor;

    char model[4];

    uint16_t pc;
    uint16_t af;
    uint16_t bc;
    uint16_t de;
    uint16_t hl;
    uint16_t sp;

    uint8_t ime;
    uint8_t ie;
    uint8_t execState;
    uint8_t reserved;
    uint8_t ioRegs[128];

    uint32_t ramSize;
    uint32_t ramOff;
    uint32_t vramSize;
    uint32_t vramOff;
    uint32_t cartRAMSize;
    uint32_t cartRAMOff;
    uint32_t oamSize;
    uint32_t oamOff;
    uint32_t hramSize;
    uint32_t hramOff;
    uint32_t bgPalSize;
    uint32_t bgPalOff;
    uint32_t objPalSize;
    uint32_t objPalOff;
};
static_assert(sizeof(BESSCore) == 208);