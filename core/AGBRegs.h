#pragma once

enum AGBIOReg
{
    // LCD
    IO_DISPCNT = 0x0,
    IO_DISPSTAT = 0x4,
    IO_VCOUNT = 0x6,
    IO_BG0CNT = 0x8,
    IO_BG1CNT = 0xA,
    IO_BG2CNT = 0xC,
    IO_BG3CNT = 0xE,

    // input
    IO_KEYINPUT = 0x130,

    // interrupts
    IO_IE = 0x200,
    IO_IF = 0x202,
    IO_IME = 0x208,
};

enum DISPCNTBits
{
    DISPCNT_Mode       = 0x7,

    DISPCNT_Frame       = (1 << 4),
    DISPCNT_HBlankFree  = (1 << 5),
    DISPCNT_OBJChar1D   = (1 << 6),
    DISPCNT_ForceBlank  = (1 << 7),
    DISPCNT_BG0On       = (1 << 8),
    DISPCNT_BG1On       = (1 << 9),
    DISPCNT_BG2On       = (1 << 10),
    DISPCNT_BG3On       = (1 << 11),
    DISPCNT_OBJOn       = (1 << 12),
    DISPCNT_Window0On   = (1 << 13),
    DISPCNT_Window1On   = (1 << 14),
    DISPCNT_OBJWindowOn = (1 << 15),
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

enum BGCNTBits
{
    BGCNT_Priority = 0x3,
    //...
};