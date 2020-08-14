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
    IO_BG0HOFS = 0x10,
    IO_BG0VOFS = 0x12,
    IO_BG1HOFS = 0x14,
    IO_BG1VOFS = 0x16,
    IO_BG2HOFS = 0x18,
    IO_BG2VOFS = 0x1A,
    IO_BG3HOFS = 0x1C,
    IO_BG3VOFS = 0x1E,

    // DMA (mostly 32-bit)
    IO_DMA0SAD = 0xB0,
    IO_DMA0DAD = 0xB4,
    IO_DMA0CNT_L = 0xB8,
    IO_DMA0CNT_H = 0xBA,
    IO_DMA1SAD = 0xBC,
    IO_DMA1DAD = 0xC0,
    IO_DMA1CNT_L = 0xC4,
    IO_DMA1CNT_H = 0xC6,
    IO_DMA2SAD = 0xC8,
    IO_DMA2DAD = 0xCC,
    IO_DMA2CNT_L = 0xD0,
    IO_DMA2CNT_H = 0xD2,
    IO_DMA3SAD = 0xD4,
    IO_DMA3DAD = 0xD8,
    IO_DMA3CNT_L = 0xDC,
    IO_DMA3CNT_H = 0xDE,

    // timers
    IO_TM0CNT_L = 0x100, // counter
    IO_TM0CNT_H = 0x102, // control
    IO_TM1CNT_L = 0x104,
    IO_TM1CNT_H = 0x106,
    IO_TM2CNT_L = 0x108,
    IO_TM2CNT_H = 0x10A,
    IO_TM3CNT_L = 0x10C,
    IO_TM3CNT_H = 0x10E,

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
    BGCNT_Priority   = 0x3,
    BGCNT_CharBase   = 0x3 << 2,
    BGCNT_Mosaic     = 1 << 6,
    BGCNT_SinglePal  = 1 << 7,
    BGCNT_ScreenBase = 0x1F << 8,
    BGCNT_Wrap       = 1 << 13,
    BGCNT_ScreenSize = 0x3 << 14
};

enum DMACNTHBits
{
    DMACNTH_DestMode  = 0x3 << 5,
    DMACNTH_SrcMode   = 0x3 << 7,
    DAMCNTH_Repeat    = 1 << 9,
    DMACNTH_32Bit     = 1 << 10,
    DMACNTH_GamePak   = 1 << 11, // 3 only
    DMACNTH_Start     = 0x3 << 12,
    DMACNTH_IRQEnable = 1 << 14,
    DMACNTH_Enable    = 1 << 15
};

enum TMCNTHBits
{
    TMCNTH_Prescaler = 0x3,
    TMCNTH_CountUp   = 1 << 2,
    TMCNTH_IRQEnable = 1 << 6,
    TMCNTH_Enable    = 1 << 7
};