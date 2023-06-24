#include <cassert>
#include <cstring>

#include "X86Builder.h"

enum REX
{
    REX_B = 1 << 0,
    REX_X = 1 << 1,
    REX_R = 1 << 2,
    REX_W = 1 << 3,
};

// reg -> reg
void X86Builder::add(Reg32 dst, Reg32 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x01); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 16 bit
void X86Builder::add(Reg16 dst, Reg16 src)
{
    write(0x66); // 16 bit override
    add(static_cast<Reg32>(dst), static_cast<Reg32>(src));
}

// reg -> reg, 8bit
void X86Builder::add(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x00); // opcode
    encodeModRMReg8(dstReg, srcReg);
}

// imm -> reg, 32 bit
void X86Builder::add(Reg32 dst, uint32_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x81); // opcode, w = 1, s = 0
    encodeModRM(dstReg);

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

// imm -> reg, 8 bit
void X86Builder::add(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, w = 0, s = 0
    encodeModRM(dstReg);
    write(imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::add(Reg64 dst, int8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(true, 0, 0, dstReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(dstReg);
    write(imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::add(Reg32 dst, int8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(dstReg);
    write(imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::add(Reg16 dst, int8_t imm)
{
    write(0x66); // 16 bit override
    add(static_cast<Reg32>(dst), imm);
}

// imm -> mem, 32 bit mem, 8 bit sign extended
void X86Builder::addD(int8_t imm, Reg64 base, int disp)
{
    auto baseReg = static_cast<int>(base);

    encodeREX(false, 0, 0, baseReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(0, baseReg, disp);
    write(imm);
}

// imm -> mem, 16 bit mem, 8 bit sign extended
void X86Builder::addW(int8_t imm, Reg64 base, int disp)
{
    write(0x66); // 16 bit override
    addD(imm, base, disp);
}

// reg -> reg, 8bit
void X86Builder::adc(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x10); // opcode
    encodeModRMReg8(dstReg, srcReg);
}

// imm -> reg, 8 bit
void X86Builder::adc(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, w = 0, s = 0
    encodeModRM(dstReg, 2);
    write(imm);
}

// reg -> reg
void X86Builder::and_(Reg32 dst, Reg32 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x21); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 16 bit
void X86Builder::and_(Reg16 dst, Reg16 src)
{
    write(0x66); // 16 bit override
    and_(static_cast<Reg32>(dst), static_cast<Reg32>(src));
}

// reg -> reg, 8 bit
void X86Builder::and_(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x20); // opcode, w = 0
    encodeModRMReg8(dstReg, srcReg);
}

// imm -> reg
void X86Builder::and_(Reg32 dst, uint32_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x81); // opcode, s = 0, w = 1
    encodeModRM(dstReg, 4);

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

// imm -> reg, 8 bit
void X86Builder::and_(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, s = 0, w = 0
    encodeModRM(dstReg, 4);
    write(imm); // imm
}

// direct
void X86Builder::call(int disp)
{
    // adjust for opcode len
    if(disp < 0)
        disp -= 5;

    write(0xE8); // opcode

    write(disp);
    write(disp >> 8);
    write(disp >> 16);
    write(disp >> 24);
}

// indirect
void X86Builder::call(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xFF); // opcode
    encodeModRM(reg, 2);
}

// reg -> reg
void X86Builder::cmp(Reg32 dst, Reg32 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x39); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 8 bit
void X86Builder::cmp(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x38); // opcode, w = 0
    encodeModRMReg8(dstReg, srcReg);
}

// imm -> reg
void X86Builder::cmp(Reg32 dst, uint32_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x81); // opcode, w = 0, s = 0
    encodeModRM(dstReg, 7);

    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

// imm -> reg, 8 bit
void X86Builder::cmp(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, w = 0, s = 0
    encodeModRM(dstReg, 7);
    write(imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::cmp(Reg32 dst, int8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(dstReg, 7);
    write(imm);
}

// imm -> mem, 8 bit
void X86Builder::cmp(uint8_t imm, Reg64 base, int disp)
{
    auto baseReg = static_cast<int>(base);

    encodeREX(false, 0, 0, baseReg);
    write(0x80); // opcode, w = 0
    encodeModRM(7, baseReg, disp);
    write(imm);
}

// reg, 16 bit
void X86Builder::dec(Reg16 r)
{
    auto reg = static_cast<int>(r);

    write(0x66); // 16 bit override
    encodeREX(false, 0, 0, reg);
    write(0xFF); // opcode, w = 1
    encodeModRM(reg, 1);
}

// reg, 8 bit
void X86Builder::dec(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xFE); // opcode, w = 0
    encodeModRM(reg, 1);
}

// reg, 16 bit
void X86Builder::inc(Reg16 r)
{
    auto reg = static_cast<int>(r);

    write(0x66); // 16 bit override
    encodeREX(false, 0, 0, reg);
    write(0xFF); // opcode, w = 1
    encodeModRM(reg, 0);
}

// reg, 8 bit
void X86Builder::inc(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xFE); // opcode, w = 0
    encodeModRM(reg, 0);
}

void X86Builder::jcc(Condition cc, int disp)
{
    if(disp < 128 && disp >= -126) // 8 bit disp
    {
        // adjust for opcode len
        if(disp < 0)
            disp -= 2;

        write(0x70 | static_cast<int>(cc)); // opcode
        write(disp);
    }
    else
    {
        // adjust for opcode len
        if(disp < 0)
            disp -= 6;

        write(0x0F); // two byte opcode
        write(0x80 | static_cast<int>(cc)); // opcode

        write(disp);
        write(disp >> 8);
        write(disp >> 16);
        write(disp >> 24);
    }
}

void X86Builder::jmp(int disp, bool forceLong)
{
    if(disp < 128 && disp >= -126 && !forceLong) // 8 bit disp
    {
        // adjust for opcode len
        if(disp < 0)
            disp -= 2;

        write(0xEB); // opcode
        write(disp);
    }
    else
    {
        // adjust for opcode len
        if(disp < 0)
            disp -= 5;

        write(0xE9); // opcode

        write(disp);
        write(disp >> 8);
        write(disp >> 16);
        write(disp >> 24);
    }
}

// indirect
void X86Builder::jmp(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xFF); // opcode
    encodeModRM(reg, 4);
}

void X86Builder::lea(Reg32 r, Reg64 base, int disp)
{
    auto reg = static_cast<int>(r);
    auto baseReg = static_cast<int>(base);

    encodeREX(false, reg, 0, baseReg);
    write(0x8D); // opcode
    encodeModRM(reg, baseReg, disp);
}

// reg -> reg, 64 bit
void X86Builder::mov(Reg64 dst, Reg64 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(true, srcReg, 0, dstReg);
    write(0x89); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg
void X86Builder::mov(Reg32 dst, Reg32 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x89); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 8bit
void X86Builder::mov(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x88); // opcode, w = 0
    encodeModRMReg8(dstReg, srcReg);
}

// reg <-> mem, 64 bit
void X86Builder::mov(Reg64 r, Reg64 base, bool isStore, int disp)
{
    auto reg = static_cast<int>(r);
    auto baseReg = static_cast<int>(base);

    encodeREX(true, reg, 0, baseReg);

    if(isStore)
        write(0x89); // opcode, w = 1
    else
        write(0x8B); // opcode, w = 1

    encodeModRM(reg, baseReg, disp);
}

// reg <-> mem
void X86Builder::mov(Reg32 r, Reg64 base, bool isStore, int disp)
{
    auto reg = static_cast<int>(r);
    auto baseReg = static_cast<int>(base);

    encodeREX(false, reg, 0, baseReg);

    if(isStore)
        write(0x89); // opcode, w = 1
    else
        write(0x8B); // opcode, w = 1

    encodeModRM(reg, baseReg, disp);
}

// reg <-> mem, 16 bit
void X86Builder::mov(Reg16 r, Reg64 base, bool isStore, int disp)
{
    write(0x66); // 16 bit override
    mov(static_cast<Reg32>(r), base, isStore, disp);
}

// imm -> reg, 64 bit
void X86Builder::mov(Reg64 r, uint64_t imm)
{
    auto reg = static_cast<int>(r);

    encodeREX(true, 0, 0, reg);
    write(0xB8 | (reg & 7)); // opcode

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
    write(imm >> 32);
    write(imm >> 40);
    write(imm >> 48);
    write(imm >> 56);
}

// imm -> reg
void X86Builder::mov(Reg32 r, uint32_t imm)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xB8 | (reg & 7)); // opcode

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

// imm -> reg, 8 bit
void X86Builder::mov(Reg8 r, uint8_t imm)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xB0 | (reg & 7)); // opcode

    // immediate
    write(imm);
}

// imm -> mem, 8 bit
void X86Builder::mov(uint8_t imm, Reg64 base, int disp)
{
    auto baseReg = static_cast<int>(base);

    encodeREX(false, 0, 0, baseReg);

    write(0xC6); // opcode, w = 0

    encodeModRM(0, baseReg, disp);

    // immediate
    write(imm);
}

// zero extend, reg -> reg, 16 bit
void X86Builder::movzx(Reg32 dst, Reg16 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, dstReg, 0, srcReg);
    write(0x0F); // two byte opcode
    write(0xB7); // opcode, w = 1
    encodeModRM(srcReg, dstReg);
}

// zero extend, reg -> reg, 8 bit
void X86Builder::movzx(Reg32 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, dstReg, 0, srcReg);
    write(0x0F); // two byte opcode
    write(0xB6); // opcode, w = 0
    encodeModRMReg8(srcReg, dstReg);
}

// zero extend, mem -> reg, 16 bit
void X86Builder::movzxW(Reg32 r, Reg64 base, int disp)
{
    auto reg = static_cast<int>(r);
    auto baseReg = static_cast<int>(base);

    encodeREX(false, reg, 0, baseReg);
    write(0x0F); // two byte opcode
    write(0xB7); // opcode, w = 1
    encodeModRM(reg, baseReg, disp);
}

// reg
void X86Builder::not_(Reg32 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xF7); // opcode, w = 1
    encodeModRM(reg, 2);
}

// reg, 8 bit
void X86Builder::not_(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xF6); // opcode, w = 0
    encodeModRM(reg, 2);
}

// reg -> reg
void X86Builder::or_(Reg32 dst, Reg32 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x09); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 16 bit
void X86Builder::or_(Reg16 dst, Reg16 src)
{
    write(0x66); // 16 bit override
    or_(static_cast<Reg32>(dst), static_cast<Reg32>(src));
}

// reg -> reg, 8 bit
void X86Builder::or_(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x08); // opcode, w = 0
    encodeModRMReg8(dstReg, srcReg);
}

// imm -> reg
void X86Builder::or_(Reg32 dst, uint32_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x81); // opcode, s = 0, w = 1
    encodeModRM(dstReg, 1);

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

// imm -> reg, 8 bit
void X86Builder::or_(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, s = 0, w = 0
    encodeModRM(dstReg, 1);
    write(imm); // imm
}

void X86Builder::pop(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0x58 | (reg & 0x7)); // opcode
}

void X86Builder::push(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0x50 | (reg & 0x7)); // opcode
}

// reg by CL, 8bit
void X86Builder::rclCL(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD2); // opcode, w = 0
    encodeModRM(reg, 2);
}

// reg, 8 bit
void X86Builder::rcl(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg, 2);
    write(count);
}

// reg by CL, 8bit
void X86Builder::rcrCL(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD2); // opcode, w = 0
    encodeModRM(reg, 3);
}

// reg, 8 bit
void X86Builder::rcr(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg, 3);
    write(count);
}

void X86Builder::ret()
{
    write(0xC3); // opcode
}

// reg by CL, 8bit
void X86Builder::rolCL(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD2); // opcode, w = 0
    encodeModRM(reg);
}

// reg, 8 bit
void X86Builder::rol(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg);
    write(count);
}

// reg by CL, 8bit
void X86Builder::rorCL(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD2); // opcode, w = 0
    encodeModRM(reg, 1);
}

// reg, 8 bit
void X86Builder::ror(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg, 1);
    write(count);
}

// reg by CL
void X86Builder::sarCL(Reg32 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD3); // opcode, w = 1
    encodeModRM(reg, 7);
}

// reg by CL, 8bit
void X86Builder::sarCL(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD2); // opcode
    encodeModRM(reg, 7);
}

// reg
void X86Builder::sar(Reg32 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    // TODO: 0xD0 for 1

    encodeREX(false, 0, 0, reg);
    write(0xC1); // opcode, w = 1
    encodeModRM(reg, 7);
    write(count);
}

// reg, 8 bit
void X86Builder::sar(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    // TODO: 0xD0 for 1

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg, 7);
    write(count);
}

// reg -> reg, 8bit
void X86Builder::sbb(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x18); // opcode
    encodeModRMReg8(dstReg, srcReg);
}

// imm -> reg, 8 bit
void X86Builder::sbb(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, w = 0, s = 0
    encodeModRMReg8(dstReg, 3);
    write(imm);
}

// -> reg
void X86Builder::setcc(Condition cc, Reg8 dst)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x0F); // two byte opcode
    write(0x90 | static_cast<int>(cc)); // opcode
    encodeModRM(dstReg);
}

// reg by CL
void X86Builder::shrCL(Reg32 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD3); // opcode, w = 1
    encodeModRM(reg, 5);
}

// reg by CL, 8bit
void X86Builder::shrCL(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD2); // opcode
    encodeModRM(reg, 5);
}

// reg
void X86Builder::shr(Reg32 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    // TODO: 0xD0 for 1

    encodeREX(false, 0, 0, reg);
    write(0xC1); // opcode, w = 1
    encodeModRM(reg, 5);
    write(count);
}

// reg, 8 bit
void X86Builder::shr(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    // TODO: 0xD0 for 1

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg, 5);
    write(count);
}

// reg by CL
void X86Builder::shlCL(Reg32 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD3); // opcode, w = 1
    encodeModRM(reg, 4);
}

// reg by CL, 8bit
void X86Builder::shlCL(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xD2); // opcode, w = 0
    encodeModRM(reg, 4);
}

// reg
void X86Builder::shl(Reg32 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    // TODO: 0xD0 for 1

    encodeREX(false, 0, 0, reg);
    write(0xC1); // opcode, w = 1
    encodeModRM(reg, 4);
    write(count);
}

// reg, 8 bit
void X86Builder::shl(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    // TODO: 0xD0 for 1

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg, 4);
    write(count);
}

void X86Builder::stc()
{
    write(0xF9); // opcode
}

// reg -> reg
void X86Builder::sub(Reg32 dst, Reg32 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x29); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 16 bit
void X86Builder::sub(Reg16 dst, Reg16 src)
{
    write(0x66); // 16 bit override
    sub(static_cast<Reg32>(dst), static_cast<Reg32>(src));
}

// reg -> reg, 8bit
void X86Builder::sub(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x28); // opcode
    encodeModRMReg8(dstReg, srcReg);
}

// imm -> reg, 32 bit
void X86Builder::sub(Reg32 dst, uint32_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x81); // opcode, w = 1, s = 0
    encodeModRM(dstReg, 5);

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

// imm -> reg, 8 bit
void X86Builder::sub(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, w = 0, s = 0
    encodeModRM(dstReg, 5);
    write(imm);
}

// imm -> reg, 64 bit reg, 8 bit sign extended
void X86Builder::sub(Reg64 dst, int8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(true, 0, 0, dstReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(dstReg, 5);
    write(imm);
}

// imm -> reg, 8 bit sign extended
void X86Builder::sub(Reg32 dst, int8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(dstReg, 5);
    write(imm);
}

// imm -> reg, 8 bit
void X86Builder::test(Reg8 r, uint8_t imm)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xF6); // opcode, s = 0, w = 0
    encodeModRM(reg);
    write(imm); // imm
}

// reg -> reg, 8bit
void X86Builder::xchg(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x86); // opcode, w = 0
    encodeModRMReg8(dstReg, srcReg);
}

// reg -> reg
void X86Builder::xor_(Reg32 dst, Reg32 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x31); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 8 bit
void X86Builder::xor_(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x30); // opcode, w = 0
    encodeModRMReg8(dstReg, srcReg);
}

// imm -> reg
void X86Builder::xor_(Reg32 dst, uint32_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x81); // opcode, s = 0, w = 1
    encodeModRM(dstReg, 6);

    // immediate
    write(imm);
    write(imm >> 8);
    write(imm >> 16);
    write(imm >> 24);
}

// imm -> reg, 8 bit
void X86Builder::xor_(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, s = 0, w = 0
    encodeModRM(dstReg, 6);
    write(imm); // imm
}

void X86Builder::resetPtr(uint8_t *oldPtr)
{
    assert(oldPtr <= ptr);
    ptr = oldPtr;
    error = false;
}

void X86Builder::patch(uint8_t *patchPtr, uint8_t *patchEndPtr)
{
    assert(patchPtr < ptr);
    assert(patchEndPtr < ptr);

    savedPtr = ptr;
    savedEndPtr = endPtr;

    ptr = patchPtr;
    endPtr = patchEndPtr;
}

void X86Builder::endPatch()
{
    ptr = savedPtr;
    endPtr = savedEndPtr;
}

void X86Builder::removeRange(uint8_t *startPtr, uint8_t *endPtr)
{
    if(error)
        return;

    // making sure this doesn't break any jumps is the user's problem

    assert(startPtr < ptr && endPtr <= ptr);
    assert(endPtr > startPtr);

    // move any code after the end of the removed range
    if(endPtr != ptr)
        memmove(startPtr, endPtr, ptr - endPtr);

    this->ptr -= (endPtr - startPtr);
}

void X86Builder::write(uint8_t b)
{
    if(ptr != endPtr)
        *ptr++ = b;
    else
        error = true;
}

void X86Builder::encodeModRM(int reg, int baseReg, int disp)
{
    // FIXME: make sure base isn't ESP or EBP

    // mod r/m
    if(!disp)
        write(((reg & 7) << 3) | (baseReg & 7)); // mod = 0
    else if(disp < 128 && disp >= -128) // 8 bit disp
    {
        write(0x40 | ((reg & 7) << 3) | (baseReg & 7)); // mod = 1
        write(disp);
    }
    else
    {
        write(0x80 | ((reg & 7) << 3) | (baseReg & 7)); // mod = 2

        write(disp);
        write(disp >> 8);
        write(disp >> 16);
        write(disp >> 24);
    }
}

void X86Builder::encodeModRM(int reg1, int reg2Op)
{
    write(0xC0 | (reg2Op & 7) << 3 | (reg1 & 7)); // mod = 3, reg 2 or sub-opcode, reg 1
}

void X86Builder::encodeModRMReg8(int reg1, int reg2)
{
    // if one reg is >= 8, the other can't be xH (4-7)
    assert((reg2 < 8 || reg1 >= 8 || reg1 < 4) && (reg1 < 8 || reg2 >= 8 || reg2 < 4));

    encodeModRM(reg1, reg2);
}

void X86Builder::encodeREX(bool w, int reg, int index, int base)
{
    if(!w && reg <= 7 && index <= 7 && base <= 7)
        return;

    write(0x40 | (w ? REX_W : 0)
               | (reg & 8 ? REX_R : 0)
               | (index & 8 ? REX_X : 0)
               | (base & 8 ? REX_B : 0));
}