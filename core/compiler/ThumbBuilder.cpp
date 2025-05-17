#include <cassert>
#include <cstring>

#include "ThumbBuilder.h"

void ThumbBuilder::adc(LowReg dn, LowReg m)
{
    int dnReg = static_cast<int>(dn.val);
    int mReg = static_cast<int>(m.val);
    write(0x4140 | mReg << 3 | dnReg);
}

// imm
void ThumbBuilder::add(LowReg d, LowReg n, uint8_t imm)
{
    int dReg = static_cast<int>(d.val);
    int nReg = static_cast<int>(n.val);
    assert(imm <= 7);

    write(0x1C00 | imm << 6 | nReg << 3 | dReg);
}

// imm
void ThumbBuilder::add(LowReg dn, uint8_t imm)
{
    int dnReg = static_cast<int>(dn.val);
    write(0x3000 | dnReg << 8 | imm);
}

// imm
void ThumbBuilder::add(Reg d, Reg n, uint32_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);

    // try to use shorter encodings
    if(s && dReg < 8 && nReg < 8)
    {
        if(dReg == nReg && imm <= 0xFF)
            return add(d, imm);
        if(imm <= 7)
            return add(d, n, imm);
    }

    auto exImm = encodeModifiedImmediate(imm);
    write(0xF100 | (s ? (1 << 4) : 0) | exImm >> 16 | nReg);
    write((exImm & 0xFFFF) | dReg << 8);
}

// reg
void ThumbBuilder::add(Reg dn, Reg m)
{
    int mReg = static_cast<int>(m);
    int dnReg = static_cast<int>(dn);

    write(0x4400 | (dnReg & 8) << 4 | mReg << 3 | (dnReg & 7));
}

// reg
void ThumbBuilder::add(LowReg d, LowReg n, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int nReg = static_cast<int>(n.val);
    int mReg = static_cast<int>(m.val);

    write(0x1800 | mReg << 6 | nReg << 3 | dReg);
}

// shifted reg
void ThumbBuilder::add(Reg d, Reg n, Reg m, bool s, ShiftType shiftType, int shift)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    if(shiftType == ShiftType::LSL && shift == 0)
    {
        // try to use shorter encodings
        if(!s && d == n)
            return add(d, n);
        if(s && dReg < 8 && nReg < 8 && mReg < 8)
            return add(d, n, m);
    }

    auto shiftVal = encodeShiftedRegister(shiftType, shift);
    write(0xEB00 | (s ? (1 << 4) : 0) | nReg);
    write(shiftVal | dReg << 8 | mReg);
}

// imm
void ThumbBuilder::and_(Reg d, Reg n, uint32_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);

    auto exImm = encodeModifiedImmediate(imm);
    write(0xF000 | (s ? (1 << 4) : 0) | exImm >> 16 | nReg);
    write((exImm & 0xFFFF) | dReg << 8);
}

void ThumbBuilder::and_(LowReg dn, LowReg m)
{
    int mReg = static_cast<int>(m.val);
    int dnReg = static_cast<int>(dn.val);
    write(0x4000 | mReg << 3 | dnReg);
}

// shifted reg
void ThumbBuilder::and_(Reg d, Reg n, Reg m, bool s, ShiftType shiftType, int shift)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    if(shiftType == ShiftType::LSL && shift == 0)
    {
        // try to use shorter encodings
        if(s && dReg < 8 && nReg == dReg && mReg < 8)
            return and_(d, m);
    }

    auto shiftVal = encodeShiftedRegister(shiftType, shift);
    write(0xEA00 | (s ? (1 << 4) : 0) | nReg);
    write(shiftVal | dReg << 8 | mReg);
}

// imm
void ThumbBuilder::asr(LowReg d, LowReg m, uint8_t imm)
{
    assert(imm > 0 && imm <= 32);

    if(imm == 32)
        imm = 0;

    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0x1000 | imm << 6 | mReg << 3 | dReg);
}

// imm
void ThumbBuilder::asr(Reg d, Reg m, uint8_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int mReg = static_cast<int>(m);

    // try to use shorter encodings
    if(s && dReg < 8 && mReg < 8)
        return asr(d, m, imm);

    assert(imm > 0 && imm <= 32);

    if(imm == 32)
        imm = 0;

    write(0xEA4F | (s ? (1 << 4) : 0));
    write(0x0020 | (imm >> 2) << 12 | dReg << 8 | (imm & 3) << 6 | mReg);
}

// reg
void ThumbBuilder::asr(LowReg dn, LowReg m)
{
    int dnReg = static_cast<int>(dn.val);
    int mReg = static_cast<int>(m.val);

    write(0x4100 | mReg << 3 | dnReg);
}

// reg
void ThumbBuilder::asr(Reg d, Reg n, Reg m, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    // try to use shorter encodings
    if(s && dReg < 8 && nReg == dReg && mReg < 8)
        return asr(d, m);

    write(0xFA40 | (s ? (1 << 4) : 0) | nReg);
    write(0xF000 | dReg << 8 | mReg);
}

void ThumbBuilder::b(Condition cond, int imm)
{
    assert(cond != Condition::AL);
    assert(imm >= -252 && imm <= 256);

    imm -= 2;

    if(imm < 0)
        imm -= 2;

    write(0xD000 | static_cast<int>(cond) << 8 | ((imm >> 1) & 0xFF));
}

void ThumbBuilder::b(int imm)
{
    assert(imm >= -2044 && imm <= 2048);

    imm -= 2;

    if(imm < 0)
        imm -= 2;

    write(0xE000 | ((imm >> 1) & 0x7FF));
}

// imm
void ThumbBuilder::bic(Reg d, Reg n, uint32_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);

    auto exImm = encodeModifiedImmediate(imm);
    write(0xF020 | (s ? (1 << 4) : 0) | exImm >> 16 | nReg);
    write((exImm & 0xFFFF) | dReg << 8);
}

void ThumbBuilder::bic(LowReg dn, LowReg m)
{
    int mReg = static_cast<int>(m.val);
    int dnReg = static_cast<int>(dn.val);
    write(0x4380 | mReg << 3 | dnReg);
}

// shifted reg
void ThumbBuilder::bic(Reg d, Reg n, Reg m, bool s, ShiftType shiftType, int shift)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    if(shiftType == ShiftType::LSL && shift == 0)
    {
        // try to use shorter encodings
        if(s && dReg < 8 && nReg == dReg && mReg < 8)
            return bic(d, m);
    }

    auto shiftVal = encodeShiftedRegister(shiftType, shift);
    write(0xEA20 | (s ? (1 << 4) : 0) | nReg);
    write(shiftVal | dReg << 8 | mReg);
}

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

void ThumbBuilder::blx(Reg r)
{
    int reg = static_cast<int>(r);
    write(0x4780 | reg << 3);
}

void ThumbBuilder::bx(Reg r)
{
    int reg = static_cast<int>(r);
    write(0x4700 | reg << 3);
}

// imm
void ThumbBuilder::cmp(Reg n, uint32_t imm)
{
    int nReg = static_cast<int>(n);
    if(nReg < 8 && imm <= 0xFF)
        write(0x2800 | nReg << 8 | imm);
    else
    {
        auto exImm = encodeModifiedImmediate(imm);
        write(0xF1B0 | exImm >> 16 | nReg);
        write(0x0F00 | (exImm & 0xFFFF));
    }
}

// reg
void ThumbBuilder::cmp(Reg n, Reg m)
{
    int mReg = static_cast<int>(m);
    int nReg = static_cast<int>(n);

    if(mReg < 8 && nReg < 8)
        write(0x4280 | mReg << 3 | nReg);
    else
        write(0x4500 | (nReg & 8) << 4 | mReg << 3 | (nReg & 7));
}

// imm
void ThumbBuilder::eor(Reg d, Reg n, uint32_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);

    auto exImm = encodeModifiedImmediate(imm);
    write(0xF080 | (s ? (1 << 4) : 0) | exImm >> 16 | nReg);
    write((exImm & 0xFFFF) | dReg << 8);
}

void ThumbBuilder::eor(LowReg dn, LowReg m)
{
    int mReg = static_cast<int>(m.val);
    int dnReg = static_cast<int>(dn.val);
    write(0x4040 | mReg << 3 | dnReg);
}

// shifted reg
void ThumbBuilder::eor(Reg d, Reg n, Reg m, bool s, ShiftType shiftType, int shift)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    if(shiftType == ShiftType::LSL && shift == 0)
    {
        // try to use shorter encodings
        if(s && dReg < 8 && nReg == dReg && mReg < 8)
            return eor(d, m);
    }

    auto shiftVal = encodeShiftedRegister(shiftType, shift);
    write(0xEA80 | (s ? (1 << 4) : 0) | nReg);
    write(shiftVal | dReg << 8 | mReg);
}

void ThumbBuilder::ldm(uint16_t regList, Reg n, bool w)
{
    // TODO: 16-bit encoding?
    assert(!(regList & (1 << 13))); // no SP
    assert(!(regList & (1 << 15)) || !(regList & (1 << 14))); // no PC + LR

    int nReg = static_cast<int>(n);

    write(0xE890 | (w ? (1 << 5) : 0) | nReg);
    write(regList);
}

// imm
void ThumbBuilder::ldr(Reg t, Reg n, uint16_t imm)
{
    int tReg = static_cast<int>(t);
    int nReg = static_cast<int>(n);

    if(tReg < 8 && nReg < 8 && !(imm & 0xFF83))
        write(0x6800 | (imm >> 2) << 6 | nReg << 3 | tReg);
    else if(tReg < 8 && n == Reg::SP && imm <= 1020 && !(imm & 3))
        write(0x9800 | tReg << 8 | imm >> 2);
    else
    {
        assert(n != Reg::PC);
        assert(imm <= 4095);

        write(0xF8D0 | nReg);
        write(tReg << 12 | imm);
    }
}

// literal
void ThumbBuilder::ldr(Reg t, int16_t imm)
{
    int reg = static_cast<int>(t);

    if(reg < 8 && imm >= 0 && imm <= 1020 && !(imm & 3))
        write(0x4800 | reg << 8 | imm >> 2);
    else
    {
        bool u = imm >= 0;
        if(!u)
            imm = -imm;

        assert(imm <= 4095);

        write(0xF85F | (u ? (1 << 7) : 0));
        write(reg << 12 | imm);
    }
}

// reg
void ThumbBuilder::ldr(LowReg t, LowReg n, LowReg m)
{
    int tReg = static_cast<int>(t.val);
    int nReg = static_cast<int>(n.val);
    int mReg = static_cast<int>(m.val);
    write(0x5800 | mReg << 6 | nReg << 3 | tReg);
}

// imm
void ThumbBuilder::ldrb(LowReg t, LowReg n, uint8_t imm)
{
    assert(imm <= 31);

    int tReg = static_cast<int>(t.val);
    int nReg = static_cast<int>(n.val);
    write(0x7800 | imm << 6 | nReg << 3 | tReg);
}

// imm
void ThumbBuilder::ldrh(Reg t, Reg n, uint16_t imm)
{
    int tReg = static_cast<int>(t);
    int nReg = static_cast<int>(n);
    if(tReg < 8 && nReg < 8 && !(imm & 0xFFC1)) // <= 62, aligned
        write(0x8800 | (imm >> 1) << 6 | nReg << 3 | tReg);
    else
    {
        assert(n != Reg::PC);
        assert(imm <= 4095);

        write(0xF8B0 | nReg);
        write(tReg << 12 | imm);
    }
}

// reg
void ThumbBuilder::ldrh(LowReg t, LowReg n, LowReg m)
{
    int tReg = static_cast<int>(t.val);
    int nReg = static_cast<int>(n.val);
    int mReg = static_cast<int>(m.val);
    write(0x5A00 | mReg << 6 | nReg << 3 | tReg);
}

// imm
void ThumbBuilder::lsl(LowReg d, LowReg m, uint8_t imm)
{
    assert(imm < 32);

    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(imm << 6 | mReg << 3 | dReg);
}

// imm
void ThumbBuilder::lsl(Reg d, Reg m, uint8_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int mReg = static_cast<int>(m);

    // try to use shorter encodings
    if(s && dReg < 8 && mReg < 8)
        return lsl(d, m, imm);

    assert(imm < 32);

    write(0xEA4F | (s ? (1 << 4) : 0));
    write(0x0000 | (imm >> 2) << 12 | dReg << 8 | (imm & 3) << 6 | mReg);
}

// reg
void ThumbBuilder::lsl(LowReg dn, LowReg m)
{
    int dnReg = static_cast<int>(dn.val);
    int mReg = static_cast<int>(m.val);

    write(0x4080 | mReg << 3 | dnReg);
}

// reg
void ThumbBuilder::lsl(Reg d, Reg n, Reg m, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    // try to use shorter encodings
    if(s && dReg < 8 && nReg == dReg && mReg < 8)
        return lsl(d, m);

    write(0xFA00 | (s ? (1 << 4) : 0) | nReg);
    write(0xF000 | dReg << 8 | mReg);
}

// imm
void ThumbBuilder::lsr(LowReg d, LowReg m, uint8_t imm)
{
    assert(imm > 0 && imm <= 32);

    if(imm == 32)
        imm = 0;

    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0x0800 | imm << 6 | mReg << 3 | dReg);
}

// imm
void ThumbBuilder::lsr(Reg d, Reg m, uint8_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int mReg = static_cast<int>(m);

    // try to use shorter encodings
    if(s && dReg < 8 && mReg < 8)
        return lsr(d, m, imm);

    assert(imm > 0 && imm <= 32);

    if(imm == 32)
        imm = 0;

    write(0xEA4F | (s ? (1 << 4) : 0));
    write(0x0010 | (imm >> 2) << 12 | dReg << 8 | (imm & 3) << 6 | mReg);
}

// reg
void ThumbBuilder::lsr(LowReg dn, LowReg m)
{
    int dnReg = static_cast<int>(dn.val);
    int mReg = static_cast<int>(m.val);

    write(0x40C0 | mReg << 3 | dnReg);
}

// reg
void ThumbBuilder::lsr(Reg d, Reg n, Reg m, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    // try to use shorter encodings
    if(s && dReg < 8 && nReg == dReg && mReg < 8)
        return lsr(d, m);

    write(0xFA20 | (s ? (1 << 4) : 0) | nReg);
    write(0xF000 | dReg << 8 | mReg);
}

// imm
void ThumbBuilder::mov(Reg d, uint32_t imm)
{
    int reg = static_cast<int>(d);
    if(reg < 8 && imm <= 0xFF)
        write(0x2000 | reg << 8 | imm);
    else if(isValidModifiedImmediate(imm))
    {
        bool s = true;
        auto exImm = encodeModifiedImmediate(imm);
        write(0xF04F | (s ? (1 << 4) : 0) | exImm >> 16);
        write((exImm & 0xFFFF) | reg << 8);
    }
    else // MOVW
    {
        assert(imm <= 0xFFFF);
        write(0xF240 | (imm & 0x800) >> 1 | imm >> 12);
        write((imm & 0x700) << 4 | reg << 8 | (imm & 0xFF));
    }
}

// reg
void ThumbBuilder::mov(Reg d, Reg m, bool s)
{
    int dReg = static_cast<int>(d);
    int mReg = static_cast<int>(m);

    if(!s)
        write(0x4600 | (dReg & 8) << 4 | mReg << 3 | (dReg & 7));
    else if(dReg < 8 && mReg < 8)
        write(mReg << 3 | dReg); // all the other bits are zeros
    else
    {
        write(0xEA4F | (s ? (1 << 4) : 0)); // S will always be one here anyway...
        write(dReg << 8 | mReg); // huh, it happened again
    }
}

// imm to top half
void ThumbBuilder::movt(Reg d, uint16_t imm)
{
    int reg = static_cast<int>(d);
    write(0xF2C0 | (imm & 0x800) >> 1 | imm >> 12);
    write((imm & 0x700) << 4 | reg << 8 | (imm & 0xFF));
}

void ThumbBuilder::mrs(Reg d, uint8_t sysm)
{
    int dReg = static_cast<int>(d);

    write(0xF3EF);
    write(0x8000 | (dReg << 8) | sysm);
}

void ThumbBuilder::msr(Reg n, uint8_t mask, uint8_t sysm)
{
    int nReg = static_cast<int>(n);
    write(0xF380 | nReg);
    write(0x8000 | mask << 10 | sysm);
}

// imm
void ThumbBuilder::mvn(Reg d, uint32_t imm, bool s)
{
    int reg = static_cast<int>(d);
    auto exImm = encodeModifiedImmediate(imm);
    write(0xF06F | (s ? (1 << 4) : 0) | exImm >> 16);
    write((exImm & 0xFFFF) | reg << 8);
}

// reg
void ThumbBuilder::mvn(LowReg d, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0x43C0 | mReg << 3 | dReg);
}

// shifted reg
void ThumbBuilder::mvn(Reg d,Reg m, bool s, ShiftType shiftType, int shift)
{
    int dReg = static_cast<int>(d);
    int mReg = static_cast<int>(m);

    if(shiftType == ShiftType::LSL && shift == 0)
    {
        // try to use shorter encodings
        if(s && dReg < 8 && mReg < 8)
            return mvn(d, m);
    }

    auto shiftVal = encodeShiftedRegister(shiftType, shift);
    write(0xEA6F | (s ? (1 << 4) : 0));
    write(shiftVal | dReg << 8 | mReg);
}

void ThumbBuilder::nop()
{
    write(0xBF00);
}

// imm
void ThumbBuilder::orr(Reg d, Reg n, uint32_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);

    auto exImm = encodeModifiedImmediate(imm);
    write(0xF040 | (s ? (1 << 4) : 0) | exImm >> 16 | nReg);
    write((exImm & 0xFFFF) | dReg << 8);
}

void ThumbBuilder::orr(LowReg d, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0x4300 | mReg << 3 | dReg);
}

// shifted reg
void ThumbBuilder::orr(Reg d, Reg n, Reg m, bool s, ShiftType shiftType, int shift)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    if(shiftType == ShiftType::LSL && shift == 0)
    {
        // try to use shorter encodings
        if(s && dReg < 8 && nReg == dReg && mReg < 8)
            return orr(d, m);
    }

    auto shiftVal = encodeShiftedRegister(shiftType, shift);
    write(0xEA40 | (s ? (1 << 4) : 0) | nReg);
    write(shiftVal | dReg << 8 | mReg);
}


void ThumbBuilder::pop(uint8_t regList, bool pc)
{
    write(0xBC00 | (pc ? (1 << 8) : 0) | regList);
}

void ThumbBuilder::pop(uint16_t regList)
{
    assert(regList);

    // try to use shorter encodings
    if(!(regList & 0b0111111100000000))
        return pop(regList, regList & (1 << 15));

    if(!(regList & (regList - 1))) // one bit set / power of two
    {
        write(0xF85D);
        write(0x0B04 | __builtin_ctz(regList) << 12);
    }
    else
        ldm(regList, Reg::SP, true);
}

void ThumbBuilder::push(uint8_t regList, bool lr)
{
    write(0xB400 | (lr ? (1 << 8) : 0) | regList);
}

void ThumbBuilder::push(uint16_t regList)
{
    assert(regList);
    assert(!(regList & (1 << 13)) && !(regList & (1 << 15))); // no SP or PC

    // try to use shorter encodings
    if(!(regList & 0b1011111100000000))
        return push(regList, regList & (1 << 14));

    if(!(regList & (regList - 1))) // one bit set / power of two
    {
        write(0xF84D);
        write(0x0D04 | __builtin_ctz(regList) << 12);
    }
    else
    {
        // STMDB SP! ...
        write(0xE92D);
        write(regList);
    }
}

// imm
void ThumbBuilder::ror(Reg d, Reg m, uint8_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int mReg = static_cast<int>(m);

    assert(imm > 0 && imm < 32);

    // imm == 0 would be RRX

    write(0xEA4F | (s ? (1 << 4) : 0));
    write(0x0030 | (imm >> 2) << 12 | dReg << 8 | (imm & 3) << 6 | mReg);
}

// reg
void ThumbBuilder::ror(LowReg dn, LowReg m)
{
    int dnReg = static_cast<int>(dn.val);
    int mReg = static_cast<int>(m.val);

    write(0x41C0 | mReg << 3 | dnReg);
}

// reg
void ThumbBuilder::ror(Reg d, Reg n, Reg m, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    // try to use shorter encodings
    if(s && dReg < 8 && nReg == dReg && mReg < 8)
        return ror(d, m);

    write(0xFA60 | (s ? (1 << 4) : 0) | nReg);
    write(0xF000 | dReg << 8 | mReg);
}

// imm
void ThumbBuilder::str(Reg t, Reg n, uint16_t imm)
{
    int tReg = static_cast<int>(t);
    int nReg = static_cast<int>(n);

    if(tReg < 8 && nReg < 8 && !(imm & 0xFF83))
        write(0x6000 | (imm >> 2) << 6 | nReg << 3 | tReg);
    else if(tReg < 8 && n == Reg::SP && imm <= 1020 && !(imm & 3))
        write(0x9000 | tReg << 8 | imm >> 2);
    else
    {
        assert(imm <= 4095);
        write(0xF8C0 | nReg);
        write(tReg << 12 | imm);
    }
}

// reg
void ThumbBuilder::str(LowReg t, LowReg n, LowReg m)
{
    int tReg = static_cast<int>(t.val);
    int nReg = static_cast<int>(n.val);
    int mReg = static_cast<int>(m.val);
    write(0x5000 | mReg << 6 | nReg << 3 | tReg);
}

// imm
void ThumbBuilder::strb(LowReg t, LowReg n, uint8_t imm)
{
    assert(imm <= 31);

    int tReg = static_cast<int>(t.val);
    int nReg = static_cast<int>(n.val);
    write(0x7000 | imm << 6 | nReg << 3 | tReg);
}

// imm
void ThumbBuilder::strh(Reg t, Reg n, uint16_t imm)
{
    int tReg = static_cast<int>(t);
    int nReg = static_cast<int>(n);

    if(tReg < 8 && nReg < 8 && !(imm & 0xFFC1)) // <= 62, aligned
        write(0x8000 | (imm >> 1) << 6 | nReg << 3 | tReg);
    else
    {
        assert(n != Reg::PC);
        assert(imm <= 4095);

        write(0xF8A0 | nReg);
        write(tReg << 12 | imm);
    }
}

// reg
void ThumbBuilder::strh(LowReg t, LowReg n, LowReg m)
{
    int tReg = static_cast<int>(t.val);
    int nReg = static_cast<int>(n.val);
    int mReg = static_cast<int>(m.val);
    write(0x5200 | mReg << 6 | nReg << 3 | tReg);
}

void ThumbBuilder::sbc(LowReg dn, LowReg m)
{
    int dnReg = static_cast<int>(dn.val);
    int mReg = static_cast<int>(m.val);
    write(0x4180 | mReg << 3 | dnReg);
}

// imm
void ThumbBuilder::sub(LowReg dn, uint8_t imm)
{
    int dnReg = static_cast<int>(dn.val);
    write(0x3800 | dnReg << 8 | imm);
}

// imm
void ThumbBuilder::sub(Reg d, Reg n, uint32_t imm, bool s)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);

    // try to use shorter encodings
    if(s && dReg < 8 && nReg < 8)
    {
        if(dReg == nReg && imm <= 0xFF)
            return sub(d, imm);
        //if(imm <= 7)
        //    return sub(d, n, imm);
    }

    auto exImm = encodeModifiedImmediate(imm);
    write(0xF1A0 | (s ? (1 << 4) : 0) | exImm >> 16 | nReg);
    write((exImm & 0xFFFF) | dReg << 8);
}

// reg
void ThumbBuilder::sub(LowReg d, LowReg n, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int nReg = static_cast<int>(n.val);
    int mReg = static_cast<int>(m.val);

    write(0x1A00 | mReg << 6 | nReg << 3 | dReg);
}

// shifted reg
void ThumbBuilder::sub(Reg d, Reg n, Reg m, bool s, ShiftType shiftType, int shift)
{
    int dReg = static_cast<int>(d);
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    if(shiftType == ShiftType::LSL && shift == 0)
    {
        // try to use shorter encodings
        if(s && dReg < 8 && nReg < 8 && mReg < 8)
            return sub(d, n, m);
    }

    auto shiftVal = encodeShiftedRegister(shiftType, shift);
    write(0xEBA0 | (s ? (1 << 4) : 0) | nReg);
    write(shiftVal | dReg << 8 | mReg);
}

void ThumbBuilder::sxtb(LowReg d, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0xB240 | mReg << 3 | dReg);
}

void ThumbBuilder::sxth(LowReg d, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0xB200 | mReg << 3 | dReg);
}

void ThumbBuilder::tst(Reg n, Reg m, ShiftType shiftType, int shift)
{
    int nReg = static_cast<int>(n);
    int mReg = static_cast<int>(m);

    if(shiftType == ShiftType::LSL && shift == 0 && nReg < 8 && mReg < 8)
        write(0x4200 | mReg << 3 | nReg);
    else
    {
        auto shiftVal = encodeShiftedRegister(shiftType, shift);
        write(0xEA00 | (1 << 4) | nReg);
        write(0x0F00 | shiftVal | mReg);
    }
}

void ThumbBuilder::uxtb(LowReg d, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0xB2C0 | mReg << 3 | dReg);
}

void ThumbBuilder::uxth(LowReg d, LowReg m)
{
    int dReg = static_cast<int>(d.val);
    int mReg = static_cast<int>(m.val);
    write(0xB280 | mReg << 3 | dReg);
}

void ThumbBuilder::data(uint16_t d)
{
    write(d);
}

bool ThumbBuilder::isValidModifiedImmediate(uint32_t val)
{
    if(val <= 0xFF) // 00 00 00 xx
        return true;

    if((val & 0xFFFF) == (val >> 16)) // repeated halfwords
    {
        if(!(val & 0xFF00)) // 00 xx 00 xx
            return true;
        
        if(!(val & 0xFF)) // xx 00 xx 00
            return true;

        if((val & 0xFF) == ((val >> 8) & 0xFF)) // xx xx xx xx
            return true;
    }

    return val >> __builtin_ctz(val) <= 0xFF; // shifted
}

void ThumbBuilder::resetPtr(uint16_t *oldPtr)
{
    assert(oldPtr <= ptr);
    ptr = oldPtr;
    error = false;
}

void ThumbBuilder::patch(uint16_t *patchPtr, uint16_t *patchEndPtr)
{
    assert(patchPtr < ptr);
    assert(patchEndPtr < ptr);

    savedPtr = ptr;
    savedEndPtr = endPtr;

    ptr = patchPtr;
    endPtr = patchEndPtr;
}

void ThumbBuilder::endPatch()
{
    ptr = savedPtr;
    endPtr = savedEndPtr;
}

void ThumbBuilder::removeRange(uint16_t *startPtr, uint16_t *endPtr)
{
    if(error)
        return;

    // making sure this doesn't break any jumps is the user's problem

    assert(startPtr < ptr && endPtr <= ptr);
    assert(endPtr > startPtr);

    // move any code after the end of the removed range
    if(endPtr != ptr)
        memmove(startPtr, endPtr, (ptr - endPtr) * 2);

    this->ptr -= (endPtr - startPtr);
}

void ThumbBuilder::write(uint16_t hw)
{
    if(ptr != endPtr)
        *ptr++ = hw;
    else
        error = true;
}

uint32_t ThumbBuilder::encodeModifiedImmediate(uint32_t val)
{
    if(val <= 0xFF)
        return val;

    if((val & 0xFFFF) == (val >> 16)) // repeated halfwords
    {
        if(!(val & 0xFF00)) // 00 xx 00 xx
            return 1 << 12 | (val & 0xFF);

        if(!(val & 0xFF)) // xx 00 xx 00
            return 2 << 12 | (val & 0xFF00) >> 8;

        if((val & 0xFF) == ((val >> 8) & 0xFF)) // xx xx xx xx
            return 3 << 12 | (val & 0xFF);
    }

    int shift = __builtin_ctz(val);

    if(val >> shift <= 0xFF) // shifted
    {
        shift -= __builtin_clz(val >> shift) % 8;
        val = (val >> shift) & 0x7F; // shift down and clear top bit

        shift = 32 - shift;

        return (shift & 0x10) << 22 | (shift & 0xE) << 11 | (shift & 1) << 7 | val;
    }

    assert(false && "invalid modified immediate value");
    return ~0;
}

uint32_t ThumbBuilder::encodeShiftedRegister(ShiftType type, int shift)
{
    assert(shift < 32);
    return (shift & 0x1C) << 10 | (shift & 3) << 6  | static_cast<int>(type) << 4;
}