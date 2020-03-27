#pragma once


enum IOReg
{
    IO_JOYP = 0x00,

    IO_DIV = 0x04,
    IO_TIMA,
    IO_TMA,
    IO_TAC,

    IO_NR10 = 0x10, // ch1 sweep
    IO_NR11, // ch1 len/duty
    IO_NR12, // ch1 envelope/volume
    IO_NR13, // ch1 freq lo
    IO_NR14, // ch1 freq hi

    IO_NR21 = 0x16, // ch2 len/duty
    IO_NR22, // ch2 envelope/volume
    IO_NR23, // ch2 freq lo
    IO_NR24, // ch2 freq hi

    IO_NR30 = 0x1A, // ch3 on/off
    IO_NR31, // ch3 len
    IO_NR32, // ch3 out level
    IO_NR33, // ch3 freq lo
    IO_NR34, // ch3 freq hi

    IO_NR41 = 0x20, // ch4 len
    IO_NR42, // ch4 envelope/volume
    IO_NR43, // ch4
    IO_NR44, // ch4

    IO_NR50 = 0x24, // volume
    IO_NR51, // channel selection
    IO_NR52, // sound on/off

    IO_LCDC = 0x40,
    IO_STAT,
    IO_SCY,
    IO_SCX,
    IO_LY,
    IO_LYC,
    IO_DMA,
    IO_BGP = 0x47,
    IO_OBP0,
    IO_OBP1,
    IO_WY,
    IO_WX,

    IO_IF = 0x0F, // interrupt flag
    IO_IE = 0xFF // interrupt enable
};

enum JOYPBits
{
    JOYP_Right         = 1 << 0,
    JOYP_Left          = 1 << 1,
    JOYP_Up            = 1 << 2,
    JOYP_Down          = 1 << 3,

    JOYP_A             = 1 << 0,
    JOYP_B             = 1 << 1,
    JOYP_Select        = 1 << 2,
    JOYP_Start         = 1 << 3,

    JOYP_SelectDir     = 1 << 4,
    JOYP_SelectButtons = 1 << 5
};

enum TACBits
{
    TAC_Start = 1 << 2,
    TAC_Clock = 0x3,
};

enum NR10Bits
{
    NR10_Shift = 0x7,
    NR10_Negate = 1 << 3,
    NR10_Period = 0x70
};

// common bits for all channels
enum NRx4Bits
{
    NRx4_Counter = 1 << 6,
    NRx4_Trigger = 1 << 7
};

enum NR52Bits
{
    NR52_Ch1On  = 1 << 0,
    NR52_Ch2On  = 1 << 1,
    NR52_Ch3On  = 1 << 2,
    NR52_Ch4On  = 1 << 3,

    NR52_Enable = 1 << 7
};

enum LCDCBits
{
    LCDC_BGDisp            = 1 << 0,
    LCDC_OBJDisp           = 1 << 1,
    LCDC_Sprite8x16        = 1 << 2,
    LCDC_BGTileMap9C00     = 1 << 3,
    LCDC_TileData8000      = 1 << 4,
    LCDC_WindowEnable      = 1 << 5,
    LCDC_WindowTileMap9C00 = 1 << 6,
    LCDC_DisplayEnable     = 1 << 7
};

enum STATBits
{
    STAT_Mode           = 0x3,
    STAT_Coincidence    = 1 << 2,
    STAT_HBlankInt      = 1 << 3,
    STAT_VBlankInt      = 1 << 4,
    STAT_OAMInt         = 1 << 5,
    STAT_CoincidenceInt = 1 << 6
};