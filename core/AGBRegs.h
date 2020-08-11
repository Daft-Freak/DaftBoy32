#pragma once

enum AGBIOReg
{
    // LCD
    IO_DISPSTAT = 0x4,
    IO_VCOUNT = 0x6,

    // input
    IO_KEYINPUT = 0x130,

    // interrupts
    IO_IE = 0x200,
    IO_IF = 0x202,
    IO_IME = 0x208,
};

enum DISPSTATBits
{
    DISPSTAT_VBlank     = 1 << 0,
    DISPSTAT_HBlank     = 1 << 1,
    DISPSTAT_VCount     = 1 << 2,
    DISPSTAT_VBlankInt  = 1 << 3,
    DISPSTAT_HBlankInt  = 1 << 4,
    DISPSTAT_VCountInt  = 1 << 5

    //8-15 is VCount value to compare
};