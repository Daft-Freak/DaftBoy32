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

    IO_BG2PA = 0x20,
    IO_BG2PB = 0x22,
    IO_BG2PC = 0x24,
    IO_BG2PD = 0x26,
    IO_BG2X_L = 0x28,
    IO_BG2X_H = 0x2A,
    IO_BG2Y_L = 0x2C,
    IO_BG2Y_H = 0x2E,
    IO_BG3PA = 0x30,
    IO_BG3PB = 0x32,
    IO_BG3PC = 0x34,
    IO_BG3PD = 0x36,
    IO_BG3X_L = 0x38,
    IO_BG3X_H = 0x3A,
    IO_BG3Y_L = 0x3C,
    IO_BG3Y_H = 0x3E,

    IO_WIN0H = 0x40,
    IO_WIN1H = 0x42,
    IO_WIN0V = 0x44,
    IO_WIN1V = 0x46,
    IO_WININ = 0x48,
    IO_WINOUT = 0x4A,

    IO_MOSAIC = 0x4C,

    IO_BLDCNT = 0x50,
    IO_BLDALPHA = 0x52,
    IO_BLDY = 0x54,

    // Audio
    IO_SOUND1CNT_L = 0x60,
    IO_SOUND1CNT_H = 0x62,
    IO_SOUND1CNT_X = 0x64,
    IO_SOUND2CNT_L = 0x68,
    IO_SOUND2CNT_H = 0x6C,
    IO_SOUND3CNT_L = 0x70,
    IO_SOUND3CNT_H = 0x72,
    IO_SOUND3CNT_X = 0x74,
    IO_SOUND4CNT_L = 0x78,
    IO_SOUND4CNT_H = 0x7C,

    IO_SOUNDCNT_L = 0x80,
    IO_SOUNDCNT_H = 0x82,
    IO_SOUNDCNT_X = 0x84,
    IO_SOUNDBIAS = 0x88,

    // 90-9F wave ram

    // these are 32-bit
    IO_FIFO_A = 0xA0,
    IO_FIFO_B = 0xA4,

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

    // system/interrupts
    IO_IE = 0x200,
    IO_IF = 0x202,
    IO_WAITCNT = 0x204,
    IO_IME = 0x208,

    IO_HALTCNT = 0x301, // yeah, odd addr
};

enum DISPCNTBits
{
    DISPCNT_Mode        = 0x7,
    DISPCNT_CGBMode     = (1 << 3),
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

enum WININBits
{
    WININ_Win0BG0    = 1 << 0,
    WININ_Win0BG1    = 1 << 1,
    WININ_Win0BG2    = 1 << 2,
    WININ_Win0BG3    = 1 << 3,
    WININ_Win0Object = 1 << 4,
    WININ_Win0Effect = 1 << 5,
    
    WININ_Win1BG0    = 1 << 8,
    WININ_Win1BG1    = 1 << 9,
    WININ_Win1BG2    = 1 << 10,
    WININ_Win1BG3    = 1 << 11,
    WININ_Win1Object = 1 << 12,
    WININ_Win1Effect = 1 << 13,
};

enum WINOUTBits
{
    WINOUT_OutsideBG0    = 1 << 0,
    WINOUT_OutsideBG1    = 1 << 1,
    WINOUT_OutsideBG2    = 1 << 2,
    WINOUT_OutsideBG3    = 1 << 3,
    WINOUT_OutsideObject = 1 << 4,
    WINOUT_OutsideEffect = 1 << 5,
    
    WINOUT_ObjWinBG0     = 1 << 8,
    WINOUT_ObjWinBG1     = 1 << 9,
    WINOUT_ObjWinBG2     = 1 << 10,
    WINOUT_ObjWinBG3     = 1 << 11,
    WINOUT_ObjWinObject  = 1 << 12,
    WINOUT_ObjWinEffect  = 1 << 13,
};

enum BLDCNTBits
{
    BLDCNT_SrcBG0      = 1 << 0,
    BLDCNT_SrcBG1      = 1 << 1,
    BLDCNT_SrcBG2      = 1 << 2,
    BLDCNT_SrcBG3      = 1 << 3,
    BLDCNT_SrcObject   = 1 << 4,
    BLDCNT_SrcBackdrop = 1 << 5,

    BLDCNT_Effect      = 3 << 6,
    
    BLDCNT_DstBG0      = 1 << 8,
    BLDCNT_DstBG1      = 1 << 9,
    BLDCNT_DstBG2      = 1 << 10,
    BLDCNT_DstBG3      = 1 << 11,
    BLDCNT_DstObject   = 1 << 12,
    BLDCNT_DstBackdrop = 1 << 13,
};

enum SOUND1CNTBits
{
    SOUND1CNT_L_Shift = 0x7,
    SOUND1CNT_L_Negate = 1 << 3,
    SOUND1CNT_L_Period = 0x70
};

// common bits for all channels
enum SOUNDxCNTBits // sometimes H, sometimes X...
{
    SOUNDxCNT_Length = 1 << 14,
    SOUNDxCNT_Trigger = 1 << 15
};

enum SOUNDCNTBits
{
    SOUNDCNT_X_Ch1On  = 1 << 0,
    SOUNDCNT_X_Ch2On  = 1 << 1,
    SOUNDCNT_X_Ch3On  = 1 << 2,
    SOUNDCNT_X_Ch4On  = 1 << 3,

    SOUNDCNT_X_Enable = 1 << 7
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

enum WAITCNTBits
{
    WAITCNT_SRAM     = 3,
    WAITCNT_ROMWS0N  = 3 << 2,
    WAITCNT_ROMWS0S  = 1 << 4,
    WAITCNT_ROMWS1N  = 3 << 5,
    WAITCNT_ROMWS1S  = 1 << 7,
    WAITCNT_ROMWS2N  = 3 << 8,
    WAITCNT_ROMWS2S  = 1 << 10,
    WAITCNT_PHI      = 3 << 11,
    WAITCNT_Prefetch = 1 << 14,
    WAITCNT_GameType = 1 << 15
};