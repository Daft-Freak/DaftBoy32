#include <cassert>

#include "ThumbBuilder.h"

void ThumbBuilder::bl(int32_t off)
{
    if(off < 0)
        off -= 4;

    auto s = off < 0 ? (1 << 10) : 0;
    auto imm11 = (off >> 1) & 0x7FF;
    auto imm10 = (off >> 12) & 0x3FF;
    auto j2 = (off & (1 << 22)) ? (1 << 11) : 0;
    auto j1 = (off & (1 << 23)) ? (1 << 13) : 0;

    if(!s)
    {
        j1 ^= 1 << 13;
        j2 ^= 1 << 11;
    }

    write(0xF000 | s | imm10);
    write(0xD000 | j1 | j2 | imm11);
}

void ThumbBuilder::bx(Reg r)
{
    int reg = static_cast<int>(r);
    write(0x4700 | reg << 3);
}

// literal
void ThumbBuilder::ldr(LowReg t, uint16_t imm)
{
    assert((imm & 3) == 0);
    assert(imm <= 1020);

    int reg = static_cast<int>(t.val);
    write(0x4800 | reg << 8 | imm >> 2);
}

void ThumbBuilder::lsl(LowReg d, LowReg m, uint8_t imm)
{
    assert(imm < 32);

    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(imm << 6 | mReg << 3 | dReg);
}

// imm
void ThumbBuilder::mov(LowReg d, uint8_t imm)
{
    int reg = static_cast<int>(d.val);
    write(0x2000 | reg << 8 | imm);
}

// reg
void ThumbBuilder::mov(Reg d, Reg m)
{
    int dReg = static_cast<int>(d);
    int mReg = static_cast<int>(m);

    // there's another encoding that sets flags, but only does r0-7
    write(0x4600 | (dReg & 8) << 4 | mReg << 3 | (dReg & 7));
}

void ThumbBuilder::orr(LowReg d, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0x4300 | mReg << 3 | dReg);
}

void ThumbBuilder::pop(uint8_t regList, bool pc)
{
    write(0xBC00 | (pc ? (1 << 8) : 0) | regList);
}

void ThumbBuilder::push(uint8_t regList, bool lr)
{
    write(0xB400 | (lr ? (1 << 8) : 0) | regList);
}

// reg
void ThumbBuilder::strh(LowReg t, LowReg n, LowReg m)
{
    int tReg = static_cast<int>(t.val);
    int nReg = static_cast<int>(n.val);
    int mReg = static_cast<int>(m.val);
    write(0x5200 | mReg << 6 | nReg << 3 | tReg);
}

void ThumbBuilder::resetPtr(uint16_t *oldPtr)
{
    assert(oldPtr <= ptr);
    ptr = oldPtr;
    error = false;
}

void ThumbBuilder::write(uint16_t hw)
{
    if(ptr + 1 != endPtr)
        *ptr++ = hw;
    else
        error = true;
}