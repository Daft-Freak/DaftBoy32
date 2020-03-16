#pragma once


enum IOReg
{
    IO_JOYP = 0x00,

    IO_DIV = 0x04,

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
