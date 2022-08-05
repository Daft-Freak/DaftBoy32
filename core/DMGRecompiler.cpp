#include <cassert>
#include <cstdio>
#include <variant>

#include <sys/mman.h>
#include <unistd.h>

#include "DMGRecompiler.h"

#include "DMGCPU.h"
#include "DMGMemory.h"

// this is currently linux/x86-64 specific


enum class Reg8
{
    AL = 0,
    CL,
    DL,
    BL,
    AH,
    CH,
    DH,
    BH,

    R8B,
    R9B,
    R10B,
    R11B,
    R12B,
    R13B,
    R14B,
    R15B,

    // these replace AH-DH with a REX prefix
    SPL = 4 | 0x10,
    BPL,
    SIL,
    DIL
};

enum class Reg16
{
    AX = 0,
    CX,
    DX,
    BX,
    SP,
    BP,
    SI,
    DI,

    R8W,
    R9W,
    R10W,
    R11W,
    R12W,
    R13W,
    R14W,
    R15W
};

enum class Reg32
{
    EAX = 0,
    ECX,
    EDX,
    EBX,
    ESP,
    EBP,
    ESI,
    EDI,

    R8D,
    R9D,
    R10D,
    R11D,
    R12D,
    R13D,
    R14D,
    R15D
};

enum class Reg64
{
    RAX = 0,
    RCX,
    RDX,
    RBX,
    RSP,
    RBP,
    RSI,
    RDI,

    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    R15
};

enum class Condition
{
    O = 0,
    NO,
    B,
    AE,
    E,
    NE,
    BE,
    A,
    S,
    NS,
    P,
    NP,
    L,
    GE,
    LE,
    G
};

enum REX
{
    REX_B = 1 << 0,
    REX_X = 1 << 1,
    REX_R = 1 << 2,
    REX_W = 1 << 3,
};

class X86Builder
{
public:
    X86Builder(uint8_t *ptr, uint8_t *endPtr) : ptr(ptr), endPtr(endPtr){}

    void add(Reg32 dst, Reg32 src);
    void add(Reg16 dst, Reg16 src);
    void add(Reg8 dst, Reg8 src);
    void add(Reg8 dst, uint8_t imm);
    void add(Reg64 dst, int8_t imm);
    void add(Reg16 dst, int8_t imm);
    void addD(int8_t imm, Reg64 base, int disp = 0);
    void addW(int8_t imm, Reg64 base, int disp = 0);

    void adc(Reg8 dst, Reg8 src);
    void adc(Reg8 dst, uint8_t imm);

    void and_(Reg16 dst, Reg16 src);
    void and_(Reg8 dst, Reg8 src);
    void and_(Reg32 dst, uint32_t imm);
    void and_(Reg8 dst, uint8_t imm);

    void call(int disp);
    void call(Reg64 r);

    void cmp(Reg8 dst, Reg8 src);
    void cmp(Reg32 dst, uint32_t imm);
    void cmp(Reg8 dst, int8_t imm);
    void cmp(uint8_t imm, Reg64 base, int disp = 0);

    void dec(Reg16 r);
    void dec(Reg8 r);

    void inc(Reg16 r);
    void inc(Reg8 r);

    void jcc(Condition cc, int disp);

    void jmp(int disp, bool forceLong = false);
    void jmp(Reg64 r);

    void lea(Reg32 r, Reg64 base, int disp = 0);

    void mov(Reg64 dst, Reg64 src);
    void mov(Reg32 dst, Reg32 src);
    void mov(Reg8 dst, Reg8 src);
    void mov(Reg64 r, Reg64 base, bool isStore, int disp = 0);
    void mov(Reg32 r, Reg64 base, bool isStore = false, int disp = 0);
    void mov(Reg16 r, Reg64 base, bool isStore = false, int disp = 0);
    void mov(Reg64 r, uint64_t imm);
    void mov(Reg32 r, uint32_t imm);
    void mov(Reg8 r, uint8_t imm);
    void mov(uint8_t imm, Reg64 base, int disp = 0);

    void movzx(Reg32 dst, Reg16 src);
    void movzx(Reg32 dst, Reg8 src);
    void movzxW(Reg32 r, Reg64 base, int disp = 0);

    void not_(Reg8 r);

    void or_(Reg16 dst, Reg16 src);
    void or_(Reg8 dst, Reg8 src);
    void or_(Reg32 dst, uint32_t imm);
    void or_(Reg8 dst, uint8_t imm);

    void pop(Reg64 r);

    void push(Reg64 r);

    void rcl(Reg8 r, uint8_t count);

    void rcr(Reg8 r, uint8_t count);

    void ret();

    void rol(Reg8 r, uint8_t count);

    void ror(Reg8 r, uint8_t count);

    void sar(Reg8 r, uint8_t count);

    void sbb(Reg8 dst, Reg8 src);
    void sbb(Reg8 dst, uint8_t imm);

    void setcc(Condition cc, Reg8 dst);

    void shr(Reg32 r, uint8_t count);
    void shr(Reg8 r, uint8_t count);

    void shl(Reg32 r, uint8_t count);
    void shl(Reg8 r, uint8_t count);

    void stc();

    void sub(Reg8 dst, Reg8 src);
    void sub(Reg8 dst, uint8_t imm);
    void sub(Reg64 dst, int8_t imm);
    void sub(Reg32 dst, int8_t imm);

    void test(Reg8 dst, uint8_t imm);

    void xor_(Reg8 dst, Reg8 src);
    void xor_(Reg8 dst, uint8_t imm);

    uint8_t *getPtr() const {return ptr;}

    void resetPtr(uint8_t *oldPtr);

    bool getError() const {return error;}

private:
    void write(uint8_t b);

    void encodeModRM(int reg, int baseReg, int disp); // mod 0-2
    void encodeModRM(int reg1, int reg2Op = 0); // mod 3
    void encodeREX(bool w, int reg, int index, int base);

    uint8_t *ptr, *endPtr;

    bool error = false; // super basic error handling
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
    // FIXME: check no xH reg with REX (low byte of BP, SP, DI, SI instead )
    write(0x00); // opcode
    encodeModRM(dstReg, srcReg);
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
void X86Builder::add(Reg16 dst, int8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    write(0x66); // 16 bit override

    encodeREX(false, 0, 0, dstReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(dstReg);
    write(imm);
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
    encodeModRM(dstReg, srcReg);
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

// reg -> reg, 16 bit
void X86Builder::and_(Reg16 dst, Reg16 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    write(0x66); // 16 bit override

    encodeREX(false, srcReg, 0, dstReg);
    write(0x21); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 8 bit
void X86Builder::and_(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x20); // opcode, w = 0
    encodeModRM(dstReg, srcReg);
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
};

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

// reg -> reg, 8 bit
void X86Builder::cmp(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x38); // opcode, w = 0
    encodeModRM(dstReg, srcReg);
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
void X86Builder::cmp(Reg8 dst, int8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, w = 0, s = 0
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
    encodeModRM(dstReg, srcReg);
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
};

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
};

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
    encodeModRM(srcReg, dstReg);
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

// reg, 8 bit
void X86Builder::not_(Reg8 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xF6); // opcode, w = 0
    encodeModRM(reg, 2);
}

// reg -> reg, 16 bit
void X86Builder::or_(Reg16 dst, Reg16 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    write(0x66); // 16 bit override

    encodeREX(false, srcReg, 0, dstReg);
    write(0x09); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
}

// reg -> reg, 8 bit
void X86Builder::or_(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x08); // opcode, w = 0
    encodeModRM(dstReg, srcReg);
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

// reg, 8 bit
void X86Builder::rcl(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg, 2);
    write(count);
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

// reg, 8 bit
void X86Builder::rol(Reg8 r, uint8_t count)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xC0); // opcode, w = 0
    encodeModRM(reg);
    write(count);
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
    encodeModRM(dstReg, srcReg);
}

// imm -> reg, 8 bit
void X86Builder::sbb(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, w = 0, s = 0
    encodeModRM(dstReg, 3);
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

// reg -> reg, 8bit
void X86Builder::sub(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x28); // opcode
    encodeModRM(dstReg, srcReg);
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

// reg -> reg, 8 bit
void X86Builder::xor_(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x30); // opcode, w = 0
    encodeModRM(dstReg, srcReg);
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
    assert(oldPtr < ptr);
    ptr = oldPtr;
    error = false;
}

void X86Builder::write(uint8_t b)
{
    if(ptr + 1 != endPtr)
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

void X86Builder::encodeREX(bool w, int reg, int index, int base)
{
    if(!w && reg <= 7 && index <= 7 && base <= 7)
        return;

    write(0x40 | (w ? REX_W : 0)
               | (reg & 8 ? REX_R : 0)
               | (index & 8 ? REX_X : 0)
               | (base & 8 ? REX_B : 0));
}

enum OpFlags
{
    Op_ReadC = 1 << 0,
    Op_ReadH = 1 << 1,
    Op_ReadN = 1 << 2,
    Op_ReadZ = 1 << 3,
    Op_ReadFlags = Op_ReadC | Op_ReadH | Op_ReadN | Op_ReadZ,

    // these match the bits in the F register
    Op_WriteC = 1 << 4,
    Op_WriteH = 1 << 5,
    Op_WriteN = 1 << 6,
    Op_WriteZ = 1 << 7,
    Op_WriteFlags = Op_WriteC | Op_WriteH | Op_WriteN | Op_WriteZ,

    Op_Branch = 1 << 8,
    Op_BranchTarget = 1 << 9,
    Op_Exit = 1 << 10,

    Op_Load = 1 << 11,
    Op_Store = 1 << 12,

    Op_Last = 1 << 13, // last instruction in compiled block, compiler can't see this itself
};

enum RegFlags
{
    Reg_A = 1 << 0,
    Reg_SP = 1 << 1, // this would be F, but we track flags seperately
    Reg_B = 1 << 2,
    Reg_C = 1 << 3,
    Reg_D = 1 << 4,
    Reg_E = 1 << 5,
    Reg_H = 1 << 6,
    Reg_L = 1 << 7
};

// size of the code to call these functions
static const int cycleExecutedCallSize = 23;
static const int readMemRegCallSize = 30;
static const int writeMemRegImmCallSize = 36; // this is only accurate for call (addr = SP)

static const int cycleExecutedInlineSize = 5;

// reg helpers
static const Reg32 pcReg32 = Reg32::R12D;
static const Reg16 pcReg16 = Reg16::R12W;

static const Reg32 spReg32 = Reg32::R13D;
static const Reg16 spReg16 = Reg16::R13W;
static const Reg8 spReg8 = Reg8::R13B; // mostly for the adds with the strange flags

inline constexpr Reg8 reg(DMGCPU::Reg r)
{
    const Reg8 regMap8[]
    {
        Reg8::AH, // A
        Reg8::AL, // F
        Reg8::CH, // B
        Reg8::CL, // C
        Reg8::DH, // D
        Reg8::DL, // E
        Reg8::BH, // H
        Reg8::BL  // L
    };
    return regMap8[static_cast<int>(r)];
}

inline constexpr Reg16 reg(DMGCPU::WReg r)
{
    const Reg16 regMap16[]
    {
        Reg16::AX, // AF
        Reg16::CX, // BC
        Reg16::DX, // DE
        Reg16::BX  // HL
    };
    return regMap16[static_cast<int>(r)];
}

// call helpers
static void callSave(X86Builder &builder)
{
    builder.push(Reg64::RAX);
    builder.push(Reg64::RCX);
    builder.push(Reg64::RDX);
    builder.push(Reg64::RDI);
    // builder.sub(Reg64::RSP, 8); // align stack
}

static void callSaveOrSkip(X86Builder &builder)
{
    auto tmpPtr = builder.getPtr() - 4;

    // skip pop/push if we just popped
    // only safe if values from RAX, RCX, RDX, RDI are not moved to args
    if(tmpPtr[0] == 0x5F/*pop rdi*/ && tmpPtr[1] == 0x5A/*pop rdx*/ && tmpPtr[2] == 0x59/*pop rcx*/ && tmpPtr[3] == 0x58/*pop rax*/)
        builder.resetPtr(tmpPtr);
    else if(tmpPtr[1] == 0x5A/*pop rdx*/ && tmpPtr[2] == 0x59/*pop rcx*/ && tmpPtr[3] == 0x58/*pop rax*/)
    {
        // skip 3/4 of the pops
        // (should be after a writeMem)
        builder.resetPtr(tmpPtr + 1);
        builder.push(Reg64::RDI);
    }
    else
        callSave(builder);
}

static void callRestore(X86Builder &builder)
{
    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);
    builder.pop(Reg64::RAX);
}

static void callRestore(X86Builder &builder, Reg32 dstReg)
{
    // mov ret val (this is never used for the emulated regs)
    assert(dstReg != Reg32::EAX && dstReg != Reg32::EDX && dstReg != Reg32::ECX);

    builder.mov(dstReg, Reg32::EAX);

    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(dstReg == Reg32::EDI ? Reg64::RSI : Reg64::RDI); // use RSI to pop the unneeded value
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);
    builder.pop(Reg64::RAX);
}

static void callRestore(X86Builder &builder, Reg8 dstReg)
{
    assert(dstReg != Reg8::DIL); // no

    // move before popping if possible
    bool isPoppedReg = dstReg == Reg8::AL || dstReg == Reg8::CL || dstReg == Reg8::DL || dstReg == Reg8::AH || dstReg == Reg8::CH || dstReg == Reg8::DH;

    if(!isPoppedReg)
        builder.mov(dstReg, Reg8::AL);
        
    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);

    // mov ret val (if not going to RAX)
    if(isPoppedReg && dstReg != Reg8::AL && dstReg != Reg8::AH)
    {
        builder.mov(dstReg, Reg8::AL);
        builder.pop(Reg64::RAX);
    }
    else if(dstReg == Reg8::AH) // TODO: having a worse case for A is not great
    {
        builder.mov(Reg8::AH, Reg8::AL);
        builder.pop(Reg64::R10);
        builder.mov(Reg8::AL, Reg8::R10B); // restore low byte
    }
    else if(dstReg == Reg8::AL)// ... though this is the worst case... (AL == F, so unlikely)
    {
        // EAX = EAX + (R10D & 0xFF00)
        builder.pop(Reg64::R10);
        builder.and_(Reg32::R10D, 0xFF00);
        builder.movzx(Reg32::EAX, Reg8::AL);
        builder.add(Reg32::EAX, Reg32::R10D); // TODO: OR? (haven't added that to builder yet)
    }
    else // we already did it
        builder.pop(Reg64::RAX);
}

DMGRecompiler::DMGRecompiler(DMGCPU &cpu) : cpu(cpu)
{
    // allocate some memory
    auto pageSize = sysconf(_SC_PAGE_SIZE);
    int numPages = 256;

    // FIXME: alloc RW, switch to RX
    codeBufSize = pageSize * numPages;
    codeBuf = reinterpret_cast<uint8_t *>(mmap(0, codeBufSize, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));

    if(codeBuf == MAP_FAILED)
        perror("failed to allocate code buffer (mmap failed)");

    curCodePtr = codeBuf;

    for(auto &saved : savedExits)
        saved = {nullptr, 0};
}

void DMGRecompiler::handleBranch()
{
    if(!codeBuf)
        return;

    while(true)
    {
        // calculate cycles to run
        int cycles = std::min(cpu.cyclesToRun, cpu.getDisplay().getCyclesToNextUpdate());

        if(cpu.nextTimerInterrupt)
            cycles = std::min(cycles, static_cast<int>(cpu.nextTimerInterrupt - cpu.cycleCount));

        if(cycles <= 0)
            break;

        // about to hit a bus conflict
        if(cpu.pc < 0xFF00 && (cpu.oamDMADelay || cpu.oamDMACount))
            break;

        uint8_t *codePtr = nullptr;

        auto mappedAddr = cpu.mem.makeBankedAddress(cpu.pc);

        // attempt to re-enter previous code
        int savedIndex = curSavedExit - 1;
        for(int i = 0; i < savedExitsSize; i++, savedIndex--)
        {
            // wrap
            if(savedIndex < 0)
                savedIndex += savedExitsSize;

            uint8_t *ptr;
            uint32_t pc;
            std::tie(ptr, pc) = savedExits[savedIndex];

            if(pc == mappedAddr && ptr)
            {
                codePtr = ptr;
                curSavedExit = savedIndex;
                savedExits[savedIndex] = {nullptr, 0};
                break;
            }
        }

        if(!codePtr)
        {
            // lookup compiled code
            auto it = compiled.find(mappedAddr);

            if(it == compiled.end())
            {
                if(!entryFunc)
                    compileEntry();

                // attempt compile
                auto ptr = curCodePtr;
                auto startPtr = ptr;
                auto pc = cpu.pc;

                BlockInfo blockInfo;
                analyse(pc, blockInfo);
                printf("analysed %04X-%04X (%zi instructions)\n", cpu.pc, pc, blockInfo.instructions.size());

                FuncInfo info{};

                if(compile(ptr, cpu.pc, blockInfo))
                {
                    info.startPtr = startPtr;
                    info.endPtr = curCodePtr = ptr;
                    info.endPC = cpu.mem.makeBankedAddress(pc);
                }
                
                it = compiled.emplace(mappedAddr, info).first;

                // running code from RAM other than HRAM seems uncommon
                // so track the min address used for code
                if(cpu.pc > 0x8000/*in RAM*/ && cpu.pc < minRAMCode)
                    minRAMCode = cpu.pc;
            }
            codePtr = it->second.startPtr;
        }

        auto startCycleCount = cpu.cycleCount;
        bool inHRAM = cpu.pc >= 0xFF00;

        // run the code if valid, or stop
        if(codePtr)
            entryFunc(cycles, cpu.regs, cpu.pc, cpu.sp, codePtr);
        else
            break;

        // update cyclesToRun if we inlined cycleExecuted and skipped updating it
        if(!inHRAM)
            cpu.cyclesToRun -= (cpu.cycleCount - startCycleCount);

        // code exited with a saved address for re-entry, store PC for later
        if(tmpSavedPtr)
        {
            auto savedPC = cpu.pc;

            if(exitCallFlag)
            {
                // get return address from stack
                auto &mem = cpu.getMem();
                savedPC = mem.read(cpu.sp) | mem.read(cpu.sp + 1) << 8;
            }

            savedExits[curSavedExit++] = {tmpSavedPtr, cpu.getMem().makeBankedAddress(savedPC)};
            curSavedExit %= savedExitsSize;

            tmpSavedPtr = nullptr;
            exitCallFlag = false;
        }

        // CPU not running, stop
        if(cpu.halted || cpu.stopped)
            break;

        if(cpu.gdmaTriggered)
            cpu.doGDMA();

        // we ran something, do the usual CPU update stuff
        // TODO: duplicated from CPU

        // sync timer if interrupts enabled
        if(cpu.nextTimerInterrupt)
            cpu.updateTimer();

        // sync display if interrupts enabled
        cpu.display.updateForInterrupts();

        if(cpu.serviceableInterrupts)
            cpu.serviceInterrupts();
    }
}

void DMGRecompiler::analyse(uint16_t &pc, BlockInfo &blockInfo)
{
    bool done = false;

    auto &mem = cpu.getMem();

    auto startPC = pc;
    auto maxBranch = pc;

    // opcodes with a 3 bit reg encode in this order
    static const int regMap8[]
    {
        Reg_B,
        Reg_C,
        Reg_D,
        Reg_E,
        Reg_H,
        Reg_L,
        0, // (HL)
        Reg_A
    };

    // order for 16bit regs
    static const int regMap16[]
    {
        Reg_B | Reg_C,
        Reg_D | Reg_E,
        Reg_H | Reg_L,
        Reg_SP
    };

    static const int condFlags[]
    {
        Op_ReadZ, // NZ
        Op_ReadZ, // Z
        Op_ReadC, // NC
        Op_ReadC, // C
    };

    auto updateEnd = [&startPC, &maxBranch](OpInfo &info, uint16_t target)
    {
        maxBranch = std::max(maxBranch, target);
    };

    while(!done)
    {
        uint8_t opcode = mem.read(pc++);

        OpInfo info{};
        info.opcode[0] = opcode;
        info.len = 1;

        switch(opcode)
        {
            case 0x00: // NOP
                break;
            case 0x01: // LD BC,nn
            case 0x11: // LD DE,nn
            case 0x21: // LD HL,nn
            case 0x31: // LD SP,nn
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsWritten = regMap16[opcode >> 4];
                break;
            case 0x02: // LD (BC),A
            case 0x12: // LD (DE),A
                info.regsRead = regMap16[opcode >> 4] | Reg_A;
                info.flags |= Op_Store;
                break;
            case 0x22: // LDI (HL),A
            case 0x32: // LDD (HL),A
                info.regsRead = Reg_H | Reg_L | Reg_A;
                info.regsWritten = Reg_H | Reg_L; // these inc/dec unlike the other two
                info.flags |= Op_Store;
                break;
            case 0x03: // INC BC
            case 0x0B: // DEC BC
            case 0x13: // INC DE
            case 0x1B: // DEC DE
            case 0x23: // INC HL
            case 0x2B: // DEC HL
            case 0x33: // INC SP
            case 0x3B: // DEC SP
                info.regsRead = info.regsWritten = regMap16[opcode >> 4];
                break;
            case 0x04: // INC B
            case 0x05: // DEC B
            case 0x0C: // INC C
            case 0x0D: // DEC C
            case 0x14: // INC D
            case 0x15: // DEC D
            case 0x1C: // INC E
            case 0x1D: // DEC E
            case 0x24: // INC H
            case 0x25: // DEC H
            case 0x2C: // INC L
            case 0x2D: // DEC L
            case 0x3C: // INC A
            case 0x3D: // DEC A
                info.regsRead = info.regsWritten = regMap8[opcode >> 3];
                info.flags = Op_WriteH | Op_WriteN | Op_WriteZ; // C preserved
                break;
            case 0x34: // INC (HL)
            case 0x35: // DEC (HL)
                info.regsRead = Reg_H | Reg_L;
                info.flags = Op_WriteH | Op_WriteN | Op_WriteZ | Op_Load | Op_Store;
                break;
            case 0x06: // LD B,n
            case 0x0E: // LD C,n
            case 0x16: // LD D,n
            case 0x1E: // LD E,n
            case 0x26: // LD H,n
            case 0x2E: // LD L,n
            case 0x3E: // LD A,n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsWritten = regMap8[opcode >> 3];
                break;
            case 0x36: // LD (HL),n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = Reg_H | Reg_L;
                info.flags = Op_Store;
                break;
            case 0x07: // RLCA
            case 0x0F: // RRCA
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags; // zeroes everything other than C
                break;
            case 0x08: // LD (nn),SP
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsRead = Reg_SP;
                info.flags = Op_Store;
                break;
            case 0x09: // ADD HL,BC
            case 0x19: // ADD HL,DE
            case 0x29: // ADD HL,HL
            case 0x39: // ADD HL,SP
                info.regsRead = regMap16[opcode >> 4] | Reg_H | Reg_L;
                info.regsWritten = Reg_H | Reg_L;
                info.flags = Op_WriteFlags;
                break;
            case 0x0A: // LD A,(BC)
            case 0x1A: // LD A,(DE)
                info.regsRead = regMap16[opcode >> 4];
                info.regsWritten = Reg_A;
                info.flags |= Op_Load;
                break;
            case 0x2A: // LDI A,(HL)
            case 0x3A: // LDD A,(HL)
                info.regsRead = Reg_H | Reg_L;
                info.regsWritten = Reg_H | Reg_L | Reg_A; // these inc/dec unlike the other two
                info.flags |= Op_Load;
                break;
            case 0x10: // STOP
                done = true;
                break;
            case 0x17: // RLA
            case 0x1F: // RRA
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_ReadC; // zeroes everything other than C
                break;
            case 0x18: // JR m
            {
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.flags = Op_Branch;

                auto target = pc + static_cast<int8_t>(info.opcode[1]);
                updateEnd(info, target);

                // can't reach past here
                if(maxBranch < pc && target < pc)
                    done = true;

                break;
            }
            case 0x20: // JR NZ,n
            case 0x28: // JR Z,n
            case 0x30: // JR NC,n
            case 0x38: // JR C,n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.flags = Op_Branch | condFlags[(opcode >> 3) & 3];

                updateEnd(info, pc + static_cast<int8_t>(info.opcode[1]));
                break;
            case 0x27: // DAA
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteC | Op_WriteH | Op_WriteZ | Op_ReadC | Op_ReadH | Op_ReadN; 
                break;
            case 0x2F: // CPL
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteH | Op_WriteN;
                break;
            case 0x37: // SCF
            case 0x3F: // CCF
                info.flags = Op_WriteC | Op_WriteH | Op_WriteN;
                break;
            case 0x40: // LD B,B
            case 0x41: // LD B,C
            case 0x42: // LD B,D
            case 0x43: // LD B,E
            case 0x44: // LD B,H
            case 0x45: // LD B,L
            case 0x47: // LD B,A
            case 0x48: // LD C,B
            case 0x49: // LD C,C
            case 0x4A: // LD C,D
            case 0x4B: // LD C,E
            case 0x4C: // LD C,H
            case 0x4D: // LD C,L
            case 0x4F: // LD C,A
            case 0x50: // LD D,B
            case 0x51: // LD D,C
            case 0x52: // LD D,D
            case 0x53: // LD D,E
            case 0x54: // LD D,H
            case 0x55: // LD D,L
            case 0x57: // LD D,A
            case 0x58: // LD E,B
            case 0x59: // LD E,C
            case 0x5A: // LD E,D
            case 0x5B: // LD E,E
            case 0x5C: // LD E,H
            case 0x5D: // LD E,L
            case 0x5F: // LD E,A
            case 0x60: // LD H,B
            case 0x61: // LD H,C
            case 0x62: // LD H,D
            case 0x63: // LD H,E
            case 0x64: // LD H,H
            case 0x65: // LD H,L
            case 0x67: // LD H,A
            case 0x68: // LD L,B
            case 0x69: // LD L,C
            case 0x6A: // LD L,D
            case 0x6B: // LD L,E
            case 0x6C: // LD L,H
            case 0x6D: // LD L,L
            case 0x6F: // LD L,A
            case 0x78: // LD A,B
            case 0x79: // LD A,C
            case 0x7A: // LD A,D
            case 0x7B: // LD A,E
            case 0x7C: // LD A,H
            case 0x7D: // LD A,L
            case 0x7F: // LD A,A
                info.regsRead = regMap8[opcode & 7];
                info.regsWritten = regMap8[(opcode >> 3) & 7];
                // if these are the same, this is effectively a nop
                break;
            case 0x46: // LD B,(HL)
            case 0x4E: // LD C,(HL)
            case 0x56: // LD D,(HL)
            case 0x5E: // LD E,(HL)
            case 0x66: // LD H,(HL)
            case 0x6E: // LD L,(HL)
            case 0x7E: // LD A,(HL)
                info.regsRead = Reg_H | Reg_L;
                info.regsWritten = regMap8[(opcode >> 3) & 7];
                info.flags = Op_Load;
                break;
            case 0x70: // LD (HL),B
            case 0x71: // LD (HL),C
            case 0x72: // LD (HL),D
            case 0x73: // LD (HL),E
            case 0x74: // LD (HL),H
            case 0x75: // LD (HL),L
            case 0x77: // LD (HL),A
                info.regsRead = Reg_H | Reg_L | regMap8[opcode & 7];
                info.flags = Op_Store;
                break;
            case 0x76: // HALT
                break;
            case 0x80: // ADD B
            case 0x81: // ADD C
            case 0x82: // ADD D
            case 0x83: // ADD E
            case 0x84: // ADD H
            case 0x85: // ADD L
            case 0x87: // ADD A
            case 0x90: // SUB B
            case 0x91: // SUB C
            case 0x92: // SUB D
            case 0x93: // SUB E
            case 0x94: // SUB H
            case 0x95: // SUB L
            case 0x97: // SUB A
            case 0xA0: // AND B
            case 0xA1: // AND C
            case 0xA2: // AND D
            case 0xA3: // AND E
            case 0xA4: // AND H
            case 0xA5: // AND L
            case 0xA7: // AND A
            case 0xA8: // XOR B
            case 0xA9: // XOR C
            case 0xAA: // XOR D
            case 0xAB: // XOR E
            case 0xAC: // XOR H
            case 0xAD: // XOR L
            case 0xAF: // XOR A
            case 0xB0: // OR B
            case 0xB1: // OR C
            case 0xB2: // OR D
            case 0xB3: // OR E
            case 0xB4: // OR H
            case 0xB5: // OR L
            case 0xB7: // OR A
                info.regsRead = regMap8[opcode & 7] | Reg_A;
                info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags; // bit ops mostly write 0
                break;
            case 0x86: // ADD (HL)
            case 0x96: // SUB (HL)
            case 0xA6: // AND (HL)
            case 0xAE: // XOR (HL)
            case 0xB6: // OR (HL)
                info.regsRead = Reg_H | Reg_L | Reg_A;
                info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_Load; // bit ops mostly write 0
                break;
            case 0x88: // ADC B
            case 0x89: // ADC C
            case 0x8A: // ADC D
            case 0x8B: // ADC E
            case 0x8C: // ADC H
            case 0x8D: // ADC L
            case 0x8F: // ADC A
            case 0x98: // SBC B
            case 0x99: // SBC C
            case 0x9A: // SBC D
            case 0x9B: // SBC E
            case 0x9C: // SBC H
            case 0x9D: // SBC L
            case 0x9F: // SBC A
                info.regsRead = regMap8[opcode & 7] | Reg_A;
                info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_ReadC;
                break;
            case 0x8E: // ADC (HL)
            case 0x9E: // SBC (HL)
                info.regsRead = Reg_H | Reg_L | Reg_A;
                info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_ReadC | Op_Load; 
                break;
            case 0xB8: // CP B
            case 0xB9: // CP C
            case 0xBA: // CP D
            case 0xBB: // CP E
            case 0xBC: // CP H
            case 0xBD: // CP L
            case 0xBF: // CP A
                info.regsRead = regMap8[opcode & 7] | Reg_A;
                info.flags = Op_WriteFlags;
                break;
            case 0xBE: // CP (HL)
                info.regsRead = regMap8[opcode & 7] | Reg_A;
                info.flags = Op_WriteFlags | Op_Load;
                break;
            case 0xC0: // RET NZ
            case 0xC8: // RET Z
            case 0xD0: // RET NC
            case 0xD8: // RET C
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_Exit | condFlags[(opcode >> 3) & 3];
                break;
            case 0xC9: // RET
            case 0xD9: // RETI
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_Exit;

                if(pc > maxBranch)
                    done = true;
                break;
            case 0xC1: // POP BC
            case 0xD1: // POP DE
            case 0xE1: // POP HL
                info.regsRead = Reg_SP;
                info.regsWritten = regMap16[(opcode >> 4) & 3] | Reg_SP;
                info.flags = Op_Load;
                break;
            case 0xF1: // POP AF
                // this is the one thing that writes F
                info.regsRead = Reg_SP;
                info.regsWritten = Reg_A | Reg_SP;
                info.flags = Op_Load | Op_WriteFlags; // flags through writing F
                break;
            case 0xC2: // JP NZ,nn
            case 0xCA: // JP Z,nn
            case 0xD2: // JP NC,nn
            case 0xDA: // JP C,nn
            {
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.flags = Op_Branch | condFlags[(opcode >> 3) & 3];
                break;
            }
            case 0xC3: // JP nn
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.flags = Op_Branch;

                if(pc > maxBranch)
                    done = true;
                break;
            case 0xC4: // CALL NZ,nn
            case 0xCC: // CALL Z,nn
            case 0xD4: // CALL NC,nn
            case 0xDC: // CALL C,nn
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                // TODO: re-entry for return
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_Exit | condFlags[(opcode >> 3) & 3];
                break;
            case 0xCD: // CALL nn
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_Exit;
                break;
            case 0xC5: // PUSH BC
            case 0xD5: // PUSH DE
            case 0xE5: // PUSH HL
                info.regsRead = regMap16[(opcode >> 4) & 3] | Reg_SP;
                info.regsWritten = Reg_SP;
                info.flags = Op_Store;
                break;
            case 0xF5: // PUSH AF
                info.regsRead = Reg_A | Reg_SP;
                info.regsWritten = Reg_SP;
                info.flags = Op_Store | Op_ReadC | Op_ReadH | Op_ReadN | Op_ReadZ; // pushes F
                break;
            case 0xC6: // ADD n
            case 0xD6: // SUB n
            case 0xE6: // AND n
            case 0xEE: // XOR n
            case 0xF6: // OR n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags; // bit ops mostly write 0
                break;
            case 0xCE: // ADC n
            case 0xDE: // SBC n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_ReadC;
                break;
            case 0xFE: // CP n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = Reg_A;
                info.flags = Op_WriteFlags;
                break;
            case 0xC7: // RST 00
            case 0xCF: // RST 08
            case 0xD7: // RST 10
            case 0xDF: // RST 18
            case 0xE7: // RST 20
            case 0xEF: // RST 28
            case 0xF7: // RST 30
            case 0xFF: // RST 38
                info.flags = Op_Exit;

                if(pc > maxBranch)
                    done = true;
                break;
            case 0xCB: // extended ops
            {
                info.opcode[1] = mem.read(pc++);
                info.len = 2;

                // these all have a reg field, all of them read that reg and most write back
                bool writes = opcode < 0x40 || opcode >= 0x80; // not BIT
                
                if((opcode & 7) == 6) // (HL)
                {
                    info.regsRead = Reg_H | Reg_L;
                    info.flags = Op_Load | (writes ? Op_Store : 0);
                }
                else
                {
                    info.regsRead = regMap8[opcode & 7];
                    if(writes)
                        info.regsWritten = info.regsWritten;
                }

                if(info.opcode[1] < 0x10) // RLC, RRC
                    info.flags |= Op_WriteFlags;
                else if(info.opcode[1] < 0x20) // RL, RR
                    info.flags |= Op_WriteFlags | Op_ReadC;
                else if(info.opcode[1] < 0x40) // SLA, SRA, SWAP, SRL
                    info.flags |= Op_WriteFlags;
                else if(info.opcode[1] < 0x80) // BIT
                    info.flags |= Op_WriteH | Op_WriteN | Op_WriteZ;
                // RES/SET don't affect F
                break;
            }
            case 0xE0: // LDH (n),A
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = Reg_A;
                info.flags = Op_Store;
                break;
            case 0xE2: // LDH (C),A
                info.regsRead = Reg_A | Reg_C;
                info.flags = Op_Store;
                break;
            case 0xE8: // ADD SP,n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_WriteFlags; // Z is always 0, H/C are... unusual
                break;
            case 0xE9: // JP (HL)
                info.regsRead = Reg_H | Reg_L;
                info.flags = Op_Exit;

                if(pc > maxBranch)
                    done = true;
                break;
            case 0xEA: // LD (nn),A
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsRead = Reg_A;
                info.flags = Op_Store;
                break;
            case 0xF0: // LDH A,(n)
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsWritten = Reg_A;
                info.flags = Op_Load;
                break;
            case 0xF2: // LDH A,(C)
                info.regsRead = Reg_C;
                info.regsWritten = Reg_A;
                info.flags = Op_Load;
                break;
            case 0xF3: // DI
            case 0xFB: // EI
                break;
            case 0xF8: // LDHL SP,n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = Reg_SP;
                info.regsWritten = Reg_H | Reg_L;
                info.flags = Op_WriteFlags; // Z is always 0, H/C are... unusual
                break;
            case 0xF9: // LD SP,HL
                info.regsRead = Reg_H | Reg_L;
                info.regsWritten = Reg_SP;
                break;
            case 0xFA: // LD A,(nn)
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsWritten = Reg_A;
                info.flags = Op_Load;
                break;

            default:
                printf("invalid op in analysis %02X\n", opcode);
                info.len = 0;
                done = true;
        }

        if(info.len)
            blockInfo.instructions.push_back(info);
    }

    auto endPC = pc;

    // cleanup
    pc = startPC;

    // TODO: we end up scanning for branch targets a lot
    auto findBranchTarget = [&blockInfo](uint16_t pc, uint16_t target, std::vector<OpInfo>::iterator it)
    {
        auto searchPC = pc;
        if(target < pc)
        {
            auto prevIt = std::make_reverse_iterator(it + 1);
    
            for(; prevIt != blockInfo.instructions.rend() && searchPC >= target; ++prevIt)
            {
                searchPC -= prevIt->len;
                if(searchPC == target)
                    return prevIt.base() - 1;
            }
        }
        else
        {
            for(auto nextIt = it + 1; nextIt != blockInfo.instructions.end() && searchPC <= target; searchPC += nextIt->len, ++nextIt)
            {
                if(searchPC == target)
                    return nextIt;
            }
        }

        return blockInfo.instructions.end();
    };

    std::vector<uint8_t> origFlags; // there aren't enough bits in ->flags...
    origFlags.resize(blockInfo.instructions.size());

    for(auto it = blockInfo.instructions.begin(); it != blockInfo.instructions.end(); ++it)
    {
        auto &instr = *it;
        pc += instr.len;

        if(instr.flags & Op_WriteFlags)
        {
            // find actually used flags
            int read = 0;

            int falseBranchFlags = 0; // flags used by the "other" branch
            bool inBranch = false;

            // save flags
            origFlags[it - blockInfo.instructions.begin()] = instr.flags & Op_WriteFlags;

            // look ahead until we have no flags wrtiten that are not read
            auto nextPC = pc;
            for(auto next = it + 1; next != blockInfo.instructions.end() && (instr.flags & Op_WriteFlags) != read << 4;)
            {
                nextPC += next->len;

                // collect read flags
                read |= next->flags & Op_ReadFlags;

                if(next->flags & Op_Exit)
                    break; // give up

                if(next->flags & Op_Branch)
                {
                    // don't go too deep
                    if(inBranch)
                        break;

                    inBranch = true;

                    bool isConditional = next->flags & Op_ReadFlags;

                    uint16_t target;
                    if(next->opcode[0] >= 0xC2) // JP
                        target = next->opcode[1] | next->opcode[2] << 8;
                    else // JR
                        target = nextPC + static_cast<int8_t>(next->opcode[1]);

                    auto targetInstr = findBranchTarget(nextPC, target, next);

                    // bad branch, give up
                    if(targetInstr == blockInfo.instructions.end())
                        break;

                    if(!isConditional)
                    {
                        // follow it
                        next = targetInstr;
                        nextPC = target;
                        continue;
                    }
                    else
                    {
                        // follow false branch until we hit another branch
                        falseBranchFlags = instr.flags & Op_WriteFlags;
                        for(auto falseInstr = next + 1; falseInstr != blockInfo.instructions.end() && falseBranchFlags != read << 4; ++falseInstr)
                        {
                            if(falseInstr->flags & (Op_Branch | Op_Exit))
                                break;

                            read |= falseInstr->flags & Op_ReadFlags;
                            falseBranchFlags = (falseBranchFlags & ~falseInstr->flags) | read << 4;
                        }

                        // follow the true branch
                        next = targetInstr;
                        nextPC = target;
                        continue;
                    }
                }

                auto written = next->flags & Op_WriteFlags;
                if(next < it) // backwards jump, restore old flags
                    written = origFlags[next - blockInfo.instructions.begin()];

                // clear overriden flags (keep any that are used)
                instr.flags = (instr.flags & ~(written)) | read << 4 | falseBranchFlags;

                ++next;
            }
        }

        if(instr.flags & Op_Branch)
        {
            // get target (should be JR or JP)
            uint16_t target;
            if(instr.opcode[0] >= 0xC2) // JP
                target = instr.opcode[1] | instr.opcode[2] << 8;
            else
                target = pc + static_cast<int8_t>(instr.opcode[1]);

            // not reachable, convert to exit
            if(target < startPC || target >= endPC)
            {
                instr.flags &= ~Op_Branch;
                instr.flags |= Op_Exit;
            }
            else
            {
                // find and mark target
                auto targetInstr = findBranchTarget(pc, target, it);

                if(targetInstr != blockInfo.instructions.end())
                    targetInstr->flags |= Op_BranchTarget;
                else
                {
                    // failed to find target
                    // may happen if there's a jump over some data
                    instr.flags &= ~Op_Branch;
                    instr.flags |= Op_Exit;
                }
            }
        }
    }

    blockInfo.instructions.back().flags |= Op_Last;
}

void DMGRecompiler::printInfo(BlockInfo &blockInfo)
{
    for(auto &instr : blockInfo.instructions)
    {
        int i = 0;
        for(; i < instr.len; i++)
            printf("%02X ", instr.opcode[i]);

        for(;i < 3; i++)
            printf("   ");

        if(instr.regsRead || instr.regsWritten)
        {
            const char *regNames[]{"A", "SP", "B", "C", "D", "E", "H", "L"};

            // read
            for(int i = 0; i < 8; i ++)
            {
                if(instr.regsRead & (1 << i))
                    printf("%s ", regNames[i]);
            }

            if(instr.flags & Op_Load)
                printf("mem ");

            printf("-> ");

            // written
            for(int i = 0; i < 8; i ++)
            {
                if(instr.regsWritten & (1 << i))
                    printf("%s ", regNames[i]);
            }

            if(instr.flags & Op_Store)
                printf("mem ");
        }

        // flags
        if(instr.flags & 0xF)
        {
            printf("Fi %s%s%s%s ", 
                    instr.flags & Op_ReadC ? "C" : "",
                    instr.flags & Op_ReadH ? "H" : "",
                    instr.flags & Op_ReadN ? "N" : "",
                    instr.flags & Op_ReadZ ? "Z" : "");
        }

        if(instr.flags & 0xF0)
        {
            printf("Fo %s%s%s%s ", 
                    instr.flags & Op_WriteC ? "C" : "",
                    instr.flags & Op_WriteH ? "H" : "",
                    instr.flags & Op_WriteN ? "N" : "",
                    instr.flags & Op_WriteZ ? "Z" : "");
        }

        if(instr.flags & Op_Branch)
            printf("Br --> ");

        if(instr.flags & Op_Exit)
            printf("Exit ");

        if(instr.flags & Op_BranchTarget)
            printf("<-- Br");

        printf("\n");
    }
}

bool DMGRecompiler::compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo)
{
    X86Builder builder(codePtr, codeBuf + codeBufSize);

    auto startPC = pc;

    // do instructions
    int numInstructions = 0;

    for(auto &instr : blockInfo.instructions)
    {
        if(!recompileInstruction(pc, instr, builder))
            break;

        numInstructions++;
    }

    lastInstrCycleCheck = nullptr;
    branchTargets.clear();
    forwardBranchesToPatch.clear();

    if(builder.getError())
    {
        printf("recompile @%04X failed due to error (out of space?)\n", startPC);
        return false;
    }

    if(numInstructions == 0)
    {
        printf("recompile @%04X failed to handle any instructions\n", startPC);
        return false;
    }

    auto endPtr = builder.getPtr();

    int len = endPtr - codePtr;

    //debug
    printf("recompile @%04X generated %i bytes (%i instructions)\ncode:", startPC, len, numInstructions);

    for(auto p = codePtr; p != endPtr; p++)
        printf(" %02X", *p);

    printf("\n");
    printf("(addr %p->%p)\n", codePtr, endPtr);

    codePtr = endPtr;

    return true;
}

bool DMGRecompiler::recompileInstruction(uint16_t &pc, OpInfo &instr, X86Builder &builder)
{
    using Reg = DMGCPU::Reg;
    using WReg = DMGCPU::WReg;

    auto &mem = cpu.getMem();
    uint8_t opcode = instr.opcode[0];

    int cyclesThisInstr = 0;
    int delayedCyclesExecuted = 0;

    bool checkInterrupts = false;

    // AF = EAX
    // BC = ECX
    // DE = EDX
    // HL = EBX
    // PC = R12D
    // SP = R13D
    // cycles = EDI

    // mapping from opcodes
    static const Reg8 regMap8[]
    {
        reg(Reg::B),
        reg(Reg::C),
        reg(Reg::D),
        reg(Reg::E),
        reg(Reg::H),
        reg(Reg::L),
        Reg8::R10B, // where we load to for (HL)
        reg(Reg::A),
    };

    static const Reg16 regMap16[]
    {
        reg(WReg::BC),
        reg(WReg::DE),
        reg(WReg::HL),
        spReg16, // unless it's push/pop, then it's AF
    };

    bool inHRAM = pc >= 0xFF00;

    // TODO: shared trampolines?
    auto cycleExecuted = [this, &builder, inHRAM, &cyclesThisInstr, &delayedCyclesExecuted]()
    {
        cyclesThisInstr += 4;

        if(!inHRAM)
        {
            // since we refuse to compile when OAM DMA is active we can just inline cycleExecuted (-updateOAMDMA) when not running from HRAM
            // ...and we can also do it slightly later
            delayedCyclesExecuted += 4;
            return;
        }

        // safe to skip here as we take no args
        callSaveOrSkip(builder);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::cycleExecuted)); // function ptr
        builder.mov(Reg64::RDI, Reg64::R14); // cpu/this ptr
        builder.call(Reg64::RAX); // do call

        callRestore(builder);
    };

    auto syncCyclesExecuted = [this, &builder, &delayedCyclesExecuted]()
    {
        if(!delayedCyclesExecuted)
            return;

        assert(delayedCyclesExecuted < 127);
        auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

        int8_t i8Cycles = delayedCyclesExecuted;

        // we don't update cyclesToRun here, do it after returning instead
        builder.addD(i8Cycles, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.cycleCount) - cpuPtr);

        delayedCyclesExecuted = 0;
    };

    auto setupMemAddr = [&builder, &syncCyclesExecuted](std::variant<Reg16, uint16_t> addr)
    {
        if(std::holds_alternative<Reg16>(addr))
        {
            syncCyclesExecuted();
            builder.movzx(Reg32::ESI, std::get<Reg16>(addr));
        }
        else
        {
            // accessing most ram shouldn't cause anything to be updated, so we don't need an accurate cycle count
            auto immAddr = std::get<uint16_t>(addr);
            if(immAddr >> 8 == 0xFF)
                syncCyclesExecuted();

            builder.mov(Reg32::ESI, immAddr);
        }
    };

    auto readMem = [&builder, &cycleExecuted, &syncCyclesExecuted, &setupMemAddr](std::variant<Reg16, uint16_t> addr, Reg8 dstReg, bool postInc = false)
    {
        callSave(builder);

        setupMemAddr(addr);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::readMem)); // function ptr
        builder.mov(Reg64::RDI, Reg64::R14); // cpu/this ptr
        
        builder.call(Reg64::RAX); // do call

        if(postInc && std::holds_alternative<Reg16>(addr))
            builder.inc(std::get<Reg16>(addr));

        callRestore(builder, dstReg);

        cycleExecuted();
    };

    auto writeMem = [&builder, &cycleExecuted, &syncCyclesExecuted, &setupMemAddr](std::variant<Reg16, uint16_t> addr, std::variant<Reg8, uint8_t> data, bool preDec = false)
    {
        callSave(builder);

        if(preDec && std::holds_alternative<Reg16>(addr))
            builder.dec(std::get<Reg16>(addr));

        setupMemAddr(addr);
    
        if(std::holds_alternative<Reg8>(data))
            builder.movzx(Reg32::EDX, std::get<Reg8>(data));
        else
            builder.mov(Reg32::EDX, std::get<uint8_t>(data));

        builder.mov(Reg32::ECX, Reg32::EDI); // cycle count

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::writeMem)); // function ptr
        builder.mov(Reg64::RDI, Reg64::R14); // cpu/this ptr
        builder.call(Reg64::RAX); // do call

        callRestore(builder, Reg32::EDI);

        cycleExecuted();
    };

    // neededFlags is what we need to output
    // updateFlags is what we can output from the result
    // oneFlags is is what is alwyas set to 1
    // preservedFlags is what is... preserved
    const auto setFlags = [&builder](uint8_t &neededFlags, uint8_t updateFlags, uint8_t oneFlags = 0, uint8_t preservedFlags = 0)
    {
        auto f = reg(Reg::F);

        updateFlags &= neededFlags;  // mask out unneeded

        // preserved flags should have been preserved already
        // so F should have all other bits zero

        bool setF = preservedFlags != 0;
        bool haveOpFlags = true; // assuming nothing has destroyed flags before this

        if(haveOpFlags && (updateFlags & DMGCPU::Flag_C))
        {
            // copy + shift if not set
            if(!setF)
            {
                builder.setcc(Condition::B, f);
                builder.shl(f, 4); // C is bit 4
                setF = true;
            }
            else
            {
                builder.jcc(Condition::AE, 3); // if !carry
                builder.or_(f, DMGCPU::Flag_C); // set C
            }

            haveOpFlags = false;
            neededFlags &= ~DMGCPU::Flag_C;
        }

        if(haveOpFlags && (updateFlags & DMGCPU::Flag_Z))
        {
            if(!setF)
            {
                builder.setcc(Condition::E, f);
                builder.shl(f, 7); // Z is bit 7
                setF = true;
            }
            else
            {
                builder.jcc(Condition::NE, 3); // if != 0
                builder.or_(f, DMGCPU::Flag_Z); // set Z
            }

            haveOpFlags = false;
            neededFlags &= ~DMGCPU::Flag_Z;
        }

        if(neededFlags && !setF)// some flags left and no write, make sure constant flags are set
        {
            builder.mov(f, oneFlags);
            neededFlags &= ~oneFlags;
            setF = true;
        }

        // still haven't set always one flags
        if(neededFlags & oneFlags)
        {
            builder.or_(f, oneFlags);
            neededFlags &= ~oneFlags;
        }
    };

    // ADC/SBC, rotates
    const auto carryIn = [&builder]()
    {
        builder.test(reg(Reg::F), DMGCPU::Flag_C); // sets CF to 0
        builder.jcc(Condition::E, 1); // not set
        builder.stc(); // CF = 1
    };

    // used by the rotates
    const auto carryOut = [&instr, &builder]()
    {
        if(instr.flags & DMGCPU::Flag_C)
        {
            // copy carry out
            // (shortcut as this is the only flag)
            builder.setcc(Condition::B, reg(Reg::F));
            builder.shl(reg(Reg::F), 4); // C is bit 4
        }
    };

    const auto push = [&builder, &cycleExecuted, &writeMem](Reg16 r)
    {
        assert(r == Reg16::AX || reg == Reg16::CX || reg == Reg16::DX || reg == Reg16::BX);
        auto lowReg = static_cast<Reg8>(r); // AX == AL, CX == CL, ...
        auto highReg = static_cast<Reg8>(static_cast<int>(lowReg) + 4); // AH == AL + 4

        cycleExecuted(); // delay

        writeMem(spReg16, highReg, true);
        writeMem(spReg16, lowReg, true);
    };

    const auto pop = [&builder, &readMem](Reg16 r)
    {
        assert(r == Reg16::AX || reg == Reg16::CX || reg == Reg16::DX || reg == Reg16::BX);
        auto lowReg = static_cast<Reg8>(r); // AX == AL, CX == CL, ...
        auto highReg = static_cast<Reg8>(static_cast<int>(lowReg) + 4); // AH == AL + 4

        readMem(spReg16, lowReg, true);
        readMem(spReg16, highReg, true);

        // low bits in F can never be set
        if(r == reg(WReg::AF))
            builder.and_(reg(Reg::F), 0xF0);
    };

    const auto prepareForHCalc = [&builder](Reg8 a, std::variant<Reg8, uint8_t> b, bool saveF)
    {
        // shuffles around regs to get a,b in the F reg and R10B (masked to the low half)
        // ready to do whatever op needs to be done to calculate the H flag
        // also optionally saves F in R11B
        auto f = reg(Reg::F);

        if(saveF)
            builder.mov(Reg8::R11B, f);

        // copy src/dst (using flags reg as temp)
        if(std::holds_alternative<Reg8>(b))
        {
            auto rb = std::get<Reg8>(b);

            if(rb == Reg8::AH || rb == Reg8::CH || rb == Reg8::DH || rb == Reg8::BH)
            {
                // legacy regs, extra copy
                builder.mov(f, rb);
                builder.mov(Reg8::R10B, f);
            }
            else
                builder.mov(Reg8::R10B, rb);
        }
        else // yeah this case could be optimised further
            builder.mov(Reg8::R10B, std::get<uint8_t>(b));

        builder.mov(f, a);

        // mask and do half add
        builder.and_(f, 0xF);
        builder.and_(Reg8::R10B, 0xF);
    };

    const auto add = [&instr, &builder, &setFlags, &carryIn, &prepareForHCalc](std::variant<Reg8, uint8_t> b, bool withCarry = false)
    {
        auto a = reg(Reg::A);
        auto f = reg(Reg::F);

        bool bIsReg = std::holds_alternative<Reg8>(b);
        bool bIsF = bIsReg && std::get<Reg8>(b) == f;

        if(instr.flags & DMGCPU::Flag_H)
        {
            // preserve F if it was was used as tmp (affects ADD (HL))
            // also preserve F for ADC
            prepareForHCalc(a, b, bIsF || withCarry);

            // do half add
            builder.add(Reg8::R10B, f);

            // put tmp/flags reg back
            if(bIsF || withCarry)
                builder.mov(f, Reg8::R11B);

            if(withCarry)
            {
                // add carry to half add
                builder.test(f, DMGCPU::Flag_C); // sets CF to 0
                builder.jcc(Condition::E, 3); // not set
                builder.inc(Reg8::R10B);
            }
        }

        if(withCarry)
        {
            carryIn();

            if(bIsReg)
            {
                auto rb = std::get<Reg8>(b);

                if(rb == Reg8::SIL) // ADC (HL)
                {
                    // more reg shuffling...
                    builder.mov(f, rb);
                    builder.adc(a, f);
                }
                else
                    builder.adc(a, rb);
            }
            else
                builder.adc(a, std::get<uint8_t>(b));
        }
        else if(bIsReg)// do add
            builder.add(a, std::get<Reg8>(b));
        else
            builder.add(a, std::get<uint8_t>(b));

        // flags
        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_Z); // without any preserved flags, this will always set C (if needed)

        if(flags & DMGCPU::Flag_H)
        {
            // half carry flag
            // r10b > 0xF ? H : 0
            builder.cmp(Reg8::R10B, 0xF);
            builder.jcc(Condition::BE, 3); // <= 0xF
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        if(flags & DMGCPU::Flag_Z)
        {
            // zero flag
            builder.and_(a, a); // TODO: TEST?
            builder.jcc(Condition::NE, 3); // if != 0
            builder.or_(f, DMGCPU::Flag_Z); // set Z
        }

        return true;
    };

    const auto addWithCarry = [&add](std::variant<Reg8, uint8_t> b)
    {
        return add(b, true);
    };

    const auto sub = [&instr, &builder, &setFlags, &carryIn, &prepareForHCalc](std::variant<Reg8, uint8_t> b, bool withCarry = false)
    {
        auto a = reg(Reg::A);
        auto f = reg(Reg::F);

        bool bIsReg = std::holds_alternative<Reg8>(b);
        bool bIsF = bIsReg && std::get<Reg8>(b) == f;

        if(instr.flags & DMGCPU::Flag_H)
        {
            // preserve F if it was was used as tmp (affects SUB (HL))
            // also preserve F for SBC
            prepareForHCalc(a, b, bIsF || withCarry);

            // do half sub
            builder.sub(f, Reg8::R10B);
            builder.mov(Reg8::R10B, f);

            // put tmp/flags reg back
            if(bIsF || withCarry)
                builder.mov(f, Reg8::R11B);

            if(withCarry)
            {
                // sub carry from half sub
                builder.test(f, DMGCPU::Flag_C); // sets CF to 0
                builder.jcc(Condition::E, 3); // not set
                builder.dec(Reg8::R10B);
            }
        }

        if(withCarry)
        {
            carryIn();

            if(bIsReg)
            {
                auto rb = std::get<Reg8>(b);

                if(rb == Reg8::SIL) // SBC (HL)
                {
                    // more reg shuffling...
                    builder.mov(f, rb);
                    builder.sbb(a, f);
                }
                else
                    builder.sbb(a, rb);
            }
            else
                builder.sbb(a, std::get<uint8_t>(b));
        }
        else if(bIsReg) // do sub
            builder.sub(a, std::get<Reg8>(b));
        else
            builder.sub(a, std::get<uint8_t>(b));

        // flags
        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_Z, DMGCPU::Flag_N);

        if(flags & DMGCPU::Flag_H)
        {
            // half carry flag
            // r10b < 0 ? H : 0
            builder.cmp(Reg8::R10B, 0);
            builder.jcc(Condition::GE, 3); // >= 0
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        if(flags & DMGCPU::Flag_Z)
        {
            // zero flag
            builder.and_(a, a); // TODO: TEST?
            builder.jcc(Condition::NE, 3); // if != 0
            builder.or_(f, DMGCPU::Flag_Z); // set Z
        }
    };

    const auto subWithCarry = [&sub](std::variant<Reg8, uint8_t> b)
    {
        return sub(b, true);
    };

    const auto bitAnd = [&instr, &builder, &setFlags](std::variant<Reg8, uint8_t> b)
    {
        if(std::holds_alternative<Reg8>(b))
            builder.and_(reg(Reg::A), std::get<Reg8>(b));
        else
            builder.and_(reg(Reg::A), std::get<uint8_t>(b));

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z, DMGCPU::Flag_H);
    };

    const auto bitOr = [&instr, &builder, &setFlags](std::variant<Reg8, uint8_t> b)
    {
        if(std::holds_alternative<Reg8>(b))
            builder.or_(reg(Reg::A), std::get<Reg8>(b));
        else
            builder.or_(reg(Reg::A), std::get<uint8_t>(b));

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z);
    };

    const auto bitXor = [&instr, &builder, &setFlags, &pc/*!!*/](std::variant<Reg8, uint8_t> b)
    {
        if(std::holds_alternative<Reg8>(b))
            builder.xor_(reg(Reg::A), std::get<Reg8>(b));
        else
            builder.xor_(reg(Reg::A), std::get<uint8_t>(b));

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z);
    };

    const auto cmp = [&instr, &builder, &setFlags, &prepareForHCalc](std::variant<Reg8, uint8_t> b)
    {
        auto a = reg(Reg::A);
        auto f = reg(Reg::F);

        bool bIsReg = std::holds_alternative<Reg8>(b);
        bool bIsF = bIsReg && std::get<Reg8>(b) == f;

        if(instr.flags & DMGCPU::Flag_H)
        {
            // preserve F for (CP (HL))
            prepareForHCalc(a, b, bIsF);

            //  do half cmp
            builder.cmp(f, Reg8::R10B);
            builder.setcc(Condition::B, Reg8::R10B); // save carry

            // put tmp reg back
            if(bIsF)
                builder.mov(f, Reg8::R11B);
        }

        // do cmp
        if(bIsReg)
            builder.cmp(a, std::get<Reg8>(b));
        else
            builder.cmp(a, std::get<uint8_t>(b));

        if(instr.flags & DMGCPU::Flag_Z)
            builder.setcc(Condition::E, Reg8::R11B); // save zero

        // flags
        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_Z, DMGCPU::Flag_N);

        if(flags & DMGCPU::Flag_H)
        {
            // half carry flag
            builder.and_(Reg8::R10B, Reg8::R10B); // TODO: TEST?
            builder.jcc(Condition::E, 3); // carry not set
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        // zero flag
        if(flags & DMGCPU::Flag_Z)
        {
            builder.and_(Reg8::R11B, Reg8::R11B); // TODO: TEST?
            builder.jcc(Condition::E, 3); // if != 0
            builder.or_(f, DMGCPU::Flag_Z); // set Z
        }
    };

    const auto inc = [&instr, &builder, &setFlags](Reg8 r)
    {
        auto f = reg(Reg::F);

        if(instr.flags & Op_WriteFlags)
            builder.and_(f, DMGCPU::Flag_C); // preserve C

        // H flag
        if(instr.flags & DMGCPU::Flag_H)
        {
            builder.mov(Reg8::R10B, f); // save F

            builder.mov(f, r); // & 0xF == 0xF (use F as tmp)
            builder.and_(f, 0xF);
            builder.cmp(f, 0xF);
            builder.mov(f, Reg8::R10B); // restore F
            builder.jcc(Condition::NE, 3); // != 0xF
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        // do inc
        builder.inc(r);

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z, 0, DMGCPU::Flag_C); // only works because preserve C
    };

    const auto dec = [&instr, &builder, &setFlags](Reg8 r)
    {
        auto f = reg(Reg::F);

        if(instr.flags & Op_WriteFlags)
            builder.and_(f, DMGCPU::Flag_C); // preserve C

        // H flag
        if(instr.flags & DMGCPU::Flag_H)
        {
            builder.mov(Reg8::R10B, f); // save F

            builder.mov(f, r); // & 0xF == 0 (use F as tmp)
            builder.and_(f, 0xF);
            builder.mov(f, Reg8::R10B); // restore F
            builder.jcc(Condition::NE, 3); // != 0x0
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        // do dec
        builder.dec(r);

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z, DMGCPU::Flag_N, DMGCPU::Flag_C); // only works because preserve C
    };

    const auto add16 = [&instr, &builder, &setFlags, &cycleExecuted](Reg16 b)
    {
        auto a = reg(WReg::HL);
        auto f = reg(Reg::F);

        if(instr.flags & DMGCPU::Flag_H)
        {
            // copy src/dst
            builder.mov(Reg32::R10D, static_cast<Reg32>(a));
            builder.mov(Reg32::R11D, static_cast<Reg32>(b));

            // mask and do "half"(3/4?) add
            builder.and_(Reg32::R10D, 0xFFF);
            builder.and_(Reg32::R11D, 0xFFF);
            builder.add(Reg32::R10D, Reg32::R11D);
        }

        // preserve Z flag
        if(instr.flags & Op_WriteFlags)
            builder.and_(f, DMGCPU::Flag_Z);

        // do add
        builder.add(a, b);

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_N, 0, DMGCPU::Flag_Z);

        // half carry flag
        if(flags & Op_WriteH)
        {
            // r10d > 0xFFF ? H : 0
            builder.cmp(Reg32::R10D, 0xFFF);
            builder.jcc(Condition::BE, 3); // <= 0xFFF
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        cycleExecuted();
    };

    const auto doJump = [this, &instr, &pc, &builder, inHRAM, &cycleExecuted, &syncCyclesExecuted, &cyclesThisInstr](uint16_t addr, int flag = 0, bool set = true)
    {
        auto it = branchTargets.find(addr);

        // condition
        if(flag)
        {
            syncCyclesExecuted();
            if(!(instr.flags & Op_Branch)) // branch flag means the jump is in range, so we should be able to handle it
                builder.mov(pcReg32, pc); // otherwise we're going to exit, so save PC

            builder.test(reg(Reg::F), flag);
            int len = ((instr.flags & Op_Branch) ? 3/*sub*/ : 6/*mov*/) + 5/*jmp*/;

            if(inHRAM)
                len += cycleExecutedCallSize;
            else
                len += cycleExecutedInlineSize;

            builder.jcc(set ? Condition::E : Condition::NE, len);
        }

        cycleExecuted();
        syncCyclesExecuted();

        // sub cycles early (we jump past the usual code that does this)
        if(instr.flags & Op_Branch)
            builder.sub(Reg32::EDI, cyclesThisInstr);
        else // or set PC if we're just going to exit
            builder.mov(pcReg32, addr);

        // don't update twice for unconditional branches (if it doesn't have the branch flag it's an exit anyway)
        if(!flag)
            cyclesThisInstr = 0;

        if(it != branchTargets.end()) 
            builder.jmp(it->second - builder.getPtr(), flag != 0);
        else
        {
            if(instr.flags & Op_Branch)
                forwardBranchesToPatch.emplace(addr, builder.getPtr());

            // will be patched later if possible
            bool forceLong = flag != 0 && (instr.flags & Op_Branch);
            builder.jmp(exitPtr - builder.getPtr(), forceLong);
        }

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
            builder.jmp(exitPtr - builder.getPtr());
    };

    const auto jump = [&instr, &cycleExecuted, &doJump](int flag = 0, bool set = true)
    {
        uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
        cycleExecuted();
        cycleExecuted();

        doJump(addr, flag, set);
    };

    const auto jumpRel = [&instr, &pc, &cycleExecuted, &doJump](int flag = 0, bool set = true)
    {
        int8_t off = instr.opcode[1];
        cycleExecuted();

        doJump(pc + off, flag, set);
    };

    const auto call = [this, &instr, &pc, &builder, inHRAM, &cycleExecuted, &syncCyclesExecuted, &writeMem](int flag = 0, bool set = true)
    {
        uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
        cycleExecuted();
        cycleExecuted();

        // condition
        if(flag)
        {
            syncCyclesExecuted();
            builder.mov(pcReg32, pc);
            builder.test(reg(Reg::F), flag);

            int len = 19 + writeMemRegImmCallSize * 2;
            if(inHRAM)
                len += cycleExecutedCallSize * 3 - 6 * 2;
            else
                len += cycleExecutedInlineSize * 3;

            builder.jcc(set ? Condition::E : Condition::NE, len);
        }

        cycleExecuted(); // delay

        writeMem(spReg16, static_cast<uint8_t>(pc >> 8), true);
        writeMem(spReg16, static_cast<uint8_t>(pc), true);
        syncCyclesExecuted();

        builder.mov(pcReg32, addr);

        // exit but flag as a call so we can save the return addr
        builder.call(exitForCallPtr - builder.getPtr());

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
            builder.jmp(exitPtr - builder.getPtr());
    };

    const auto reset = [this, &pc, &builder, &cycleExecuted, &syncCyclesExecuted, &writeMem](int addr)
    {
        cycleExecuted(); // delay

        writeMem(spReg16, static_cast<uint8_t>(pc >> 8), true);
        writeMem(spReg16, static_cast<uint8_t>(pc), true);
        syncCyclesExecuted();

        builder.mov(pcReg32, addr);

        builder.jmp(exitPtr - builder.getPtr()); // exit
    };

    const auto ret = [this, &instr, &pc, &builder, inHRAM, &cycleExecuted, &syncCyclesExecuted, &readMem](int flag = 0, bool set = true)
    {
        // condition
        if(flag)
        {
            cycleExecuted(); // delay
            syncCyclesExecuted();

            builder.mov(pcReg32, pc);
            builder.test(reg(Reg::F), flag);

            int len = 27 + readMemRegCallSize * 2;
            if(inHRAM)
                len += cycleExecutedCallSize * 3 - 8 * 2/*adjacent calls*/;
            else
                len += cycleExecutedInlineSize * 2;

            builder.jcc(set ? Condition::E : Condition::NE, len);
        }

        auto pcReg8 = static_cast<Reg8>(pcReg16);

        builder.mov(pcReg32, 0);
        readMem(spReg16, pcReg8, true);
        readMem(spReg16, Reg8::R10B, true);

        builder.shl(Reg32::R10D, 8);
        builder.or_(pcReg16, Reg16::R10W);
        cycleExecuted();
        syncCyclesExecuted();

        assert(exitPtr - builder.getPtr() < -126);
        builder.jmp(exitPtr - builder.getPtr()); // it's > 128 bytes to here from the start of the op, so should always be 5 bytes

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
            builder.jmp(exitPtr - builder.getPtr());
    };

    // handle branch targets
    if(instr.flags & Op_BranchTarget)
    {
        // store for backwards jumps
        if(lastInstrCycleCheck)
            branchTargets.emplace(pc, lastInstrCycleCheck); // after adjusting cycle count but before the jump
        else
        {
            // this is the first instruction, so make a cycle check for the branch to go to
            builder.jmp(13);
            lastInstrCycleCheck = builder.getPtr();

            // if <= 0 exit
            builder.jcc(Condition::G, 11);
            builder.mov(pcReg32, pc);
            builder.call(saveAndExitPtr - builder.getPtr());

            // TODO: this is the first instruction
            branchTargets.emplace(pc, lastInstrCycleCheck);
        } 

        // patch forwards jumps
        // can't hit this for the first instruction, so the lastInstrCycleCheck will be valid
        auto jumps = forwardBranchesToPatch.equal_range(pc);
        for(auto it = jumps.first; it != jumps.second; ++it)
        {
            // overrite 4 byte disp
            auto off = lastInstrCycleCheck - (it->second + 5);

            it->second[1] = off;
            it->second[2] = off >> 8;
            it->second[3] = off >> 16;
            it->second[4] = off >> 24;
        }
        forwardBranchesToPatch.erase(jumps.first, jumps.second);
    }

    pc += instr.len;

    // cycle for opcode read
    auto oldPtr = builder.getPtr();
    cycleExecuted();

    // previous op was EI
    if(mem.read(pc - (instr.len + 1)) == 0xFB /*EI*/)
    {
        // enable interrupts for EI
        auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

        // if(enableInterruptsNextCycle)
        // probably don't need this check... might get a false positive in some extreme case though
        builder.cmp(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle) - cpuPtr);
        builder.jcc(Condition::E, 10);

        // masterInterruptEnable = true
        builder.mov(1, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtr);

        // enableInterruptsNextCycle = false
        builder.mov(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle) - cpuPtr);

        checkInterrupts = true;
    }

    switch(opcode)
    {
        case 0x00: // NOP
            break;
        case 0x01: // LD BC,nn
        case 0x11: // LD DE,nn
        case 0x21: // LD HL,nn
        case 0x31: // LD SP,nn
        {
            uint16_t v = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            builder.mov(static_cast<Reg32>(regMap16[opcode >> 4]), v);
            break;
        }
        case 0x02: // LD (BC),A
        case 0x12: // LD (DE),A
            writeMem(regMap16[opcode >> 4], reg(Reg::A));
            break;

        case 0x03: // INC BC
        case 0x13: // INC DE
        case 0x23: // INC HL
        case 0x33: // INC SP
            builder.inc(regMap16[opcode >> 4]);
            cycleExecuted();
            break;

        case 0x04: // INC B
        case 0x0C: // INC C
        case 0x14: // INC D
        case 0x1C: // INC E
        case 0x24: // INC H
        case 0x2C: // INC L
        case 0x3C: // INC A
            inc(regMap8[opcode >> 3]);
            break;

        case 0x05: // DEC B
            dec(reg(Reg::B));
            break;
        case 0x0D: // DEC C
        case 0x15: // DEC D
        case 0x1D: // DEC E
        case 0x25: // DEC H
        case 0x2D: // DEC L
        case 0x3D: // DEC A
            dec(regMap8[opcode >> 3]);
            break;

        case 0x0B: // DEC BC
        case 0x1B: // DEC DE
        case 0x2B: // DEC HL
        case 0x3B: // DEC SP
            builder.dec(regMap16[opcode >> 4]);
            cycleExecuted();
            break;

        case 0x06: // LD B,n
        case 0x0E: // LD C,n
        case 0x16: // LD D,n
        case 0x1E: // LD E,n
        case 0x26: // LD H,n
        case 0x2E: // LD L,n
        case 0x3E: // LD A,n
            cycleExecuted();
            builder.mov(regMap8[opcode >> 3], instr.opcode[1]);
            break;

        case 0x07: // RLCA
            builder.rol(reg(Reg::A), 1);
            carryOut();
            break;
        case 0x08: // LD (nn),SP
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            writeMem(addr++, spReg8); // low byte

            // SP >> 8
            builder.mov(Reg32::R10D, spReg32);
            builder.shr(Reg32::R10D, 8);

            writeMem(addr, Reg8::R10B); // low byte
            break;
        }

        case 0x09: // ADD HL,BC
        case 0x19: // ADD HL,DE
        case 0x29: // ADD HL,HL
        case 0x39: // ADD HL,SP
            add16(regMap16[opcode >> 4]);
            break;

        case 0x0A: // LD A,(BC)
        case 0x1A: // LD A,(DE)
            readMem(regMap16[opcode >> 4], reg(Reg::A));
            break;

        case 0x0F: // RRCA
            builder.ror(reg(Reg::A), 1);
            carryOut();
            break;

        case 0x17: // RLA
            carryIn();
            builder.rcl(reg(Reg::A), 1);
            carryOut();
            break;

        case 0x18: // JR m
            jumpRel();
            break;

        case 0x1F: // RRA
            carryIn();
            builder.rcr(reg(Reg::A), 1);
            carryOut();
            break;

        case 0x20: // JR NZ,n
        case 0x28: // JR Z,n
        case 0x30: // JR NC,n
        case 0x38: // JR C,n
        {
            int flag = opcode & (1 << 4) ? DMGCPU::Flag_C : DMGCPU::Flag_Z;
            jumpRel(flag, opcode & (1 << 3));
            break;
        }
    
        case 0x22: // LDI (HL),A
            writeMem(reg(WReg::HL), reg(Reg::A));
            builder.inc(reg(WReg::HL));
            break;

        case 0x27: // DAA
        {
            auto f = reg(Reg::F);
            auto a = reg(Reg::A);

            // clear Z flag (will add back later)
            builder.and_(f, DMGCPU::Flag_Z ^ 0xFF);

            builder.test(f, DMGCPU::Flag_N);
            builder.jcc(Condition::E, 18); // N not set

            // negative

            // if (Flag_C) A -= 0x60
            builder.test(f, DMGCPU::Flag_C);
            builder.jcc(Condition::E, 03); // C not set
            builder.sub(a, 0x60);

            // if (Flag_H) A -= 0x06
            builder.test(f, DMGCPU::Flag_H);
            builder.jcc(Condition::E, 3); // C not set
            builder.sub(a, 0x06);
    
            builder.jmp(43); // skip

            // positive

            // if (Flag_C ...
            builder.test(f, DMGCPU::Flag_C);
            builder.jcc(Condition::NE, 5); // C set
            // ... || A > 0x99) ...
            builder.cmp(a, 0x99);
            builder.jcc(Condition::BE, 6);
            // ... A += 0x60
            builder.add(a, 0x60);
            builder.or_(f, DMGCPU::Flag_C); // set C flag

            // if (Flag_H ...
            builder.test(f, DMGCPU::Flag_H);
            builder.jcc(Condition::NE, 19); // H set
            // ... || A & (0x0F) > 0x09) ...
            builder.mov(Reg32::R10D, static_cast<Reg32>(reg(WReg::AF))); // move the whole thing as we can't easily mov just the high byte
            builder.and_(Reg32::R10D, 0xF00);
            builder.cmp(Reg32::R10D, 0x0900);
            builder.jcc(Condition::BE, 3);
            // .. A += 0x06
            builder.add(a, 0x06);

            // Z flag
            builder.and_(a, a);
            builder.jcc(Condition::NE, 3);
            builder.or_(f, DMGCPU::Flag_Z);

            // clear H flag
            builder.and_(f, ~DMGCPU::Flag_H);

            break;
        }

        case 0x2A: // LDI A,(HL)
            readMem(reg(WReg::HL), reg(Reg::A));
            builder.inc(reg(WReg::HL));
            break;

        case 0x2F: // CPL
            builder.not_(reg(Reg::A));
            builder.or_(reg(Reg::F), DMGCPU::Flag_H | DMGCPU::Flag_N);
            break;

        case 0x32: // LDD (HL),A
            writeMem(reg(WReg::HL), reg(Reg::A));
            builder.dec(reg(WReg::HL));
            break;

        case 0x34: // INC (HL)
        {
            auto tmp = Reg8::R11B;

            readMem(reg(WReg::HL), tmp);
            inc(tmp);
            writeMem(reg(WReg::HL), tmp);

            break;
        }
        case 0x35: // DEC (HL)
        {
            auto tmp = Reg8::R11B;

            readMem(reg(WReg::HL), tmp);
            dec(tmp);
            writeMem(reg(WReg::HL), tmp);

            break;
        }
        case 0x36: // LD (HL),n
        {
            cycleExecuted();
            writeMem(reg(WReg::HL), instr.opcode[1]);
            break;
        }
        case 0x37: // SCF
        {
            builder.and_(reg(Reg::F), DMGCPU::Flag_Z); // H/N are cleared
            builder.or_(reg(Reg::F), DMGCPU::Flag_C);
            break;
        }

        case 0x3A: // LDD A,(HL)
            readMem(reg(WReg::HL), reg(Reg::A));
            builder.dec(reg(WReg::HL));
            break;

        case 0x3F: // CCF
        {
            builder.and_(reg(Reg::F), DMGCPU::Flag_C | DMGCPU::Flag_Z); // H/N are cleared
            builder.xor_(reg(Reg::F), DMGCPU::Flag_C);
            break;
        }

        case 0x40: // LD B,B
        case 0x41: // LD B,C
        case 0x42: // LD B,D
        case 0x43: // LD B,E
        case 0x44: // LD B,H
        case 0x45: // LD B,L
        case 0x47: // LD B,A
        case 0x48: // LD C,B
        case 0x49: // LD C,C
        case 0x4A: // LD C,D
        case 0x4B: // LD C,E
        case 0x4C: // LD C,H
        case 0x4D: // LD C,L
        case 0x4F: // LD C,A
        case 0x50: // LD D,B
        case 0x51: // LD D,C
        case 0x52: // LD D,D
        case 0x53: // LD D,E
        case 0x54: // LD D,H
        case 0x55: // LD D,L
        case 0x57: // LD D,A
        case 0x58: // LD E,B
        case 0x59: // LD E,C
        case 0x5A: // LD E,D
        case 0x5B: // LD E,E
        case 0x5C: // LD E,H
        case 0x5D: // LD E,L
        case 0x5F: // LD E,A
        case 0x60: // LD H,B
        case 0x61: // LD H,C
        case 0x62: // LD H,D
        case 0x63: // LD H,E
        case 0x64: // LD H,H
        case 0x65: // LD H,L
        case 0x67: // LD H,A
        case 0x68: // LD L,B
        case 0x69: // LD L,C
        case 0x6A: // LD L,D
        case 0x6B: // LD L,E
        case 0x6C: // LD L,H
        case 0x6D: // LD L,L
        case 0x6F: // LD L,A
        case 0x78: // LD A,B
        case 0x79: // LD A,C
        case 0x7A: // LD A,D
        case 0x7B: // LD A,E
        case 0x7C: // LD A,H
        case 0x7D: // LD A,L
        case 0x7F: // LD A,A
            builder.mov(regMap8[(opcode >> 3) & 7], regMap8[opcode & 7]);
            break;
        case 0x46: // LD B,(HL)
        case 0x4E: // LD C,(HL)
        case 0x56: // LD D,(HL)
        case 0x5E: // LD E,(HL)
        case 0x66: // LD H,(HL)
        case 0x6E: // LD L,(HL)
        case 0x7E: // LD A,(HL)
            readMem(reg(WReg::HL), regMap8[(opcode >> 3) & 7]);
            break;
        case 0x70: // LD (HL),B
        case 0x71: // LD (HL),C
        case 0x72: // LD (HL),D
        case 0x73: // LD (HL),E
        case 0x74: // LD (HL),H
        case 0x75: // LD (HL),L
        case 0x77: // LD (HL),A
            writeMem(reg(WReg::HL), regMap8[opcode & 7]);
            break;
        
        case 0x76: // HALT
        {
            // halted = true
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
            builder.mov(1, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.halted) - cpuPtr);

            // if(!masterInterruptEnable && serviceableInterrupts)
            builder.cmp(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtr);
            builder.jcc(Condition::NE, 12);
            builder.cmp(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu. serviceableInterrupts) - cpuPtr);
            builder.jcc(Condition::E, 5);
            // haltBug = true
            builder.mov(1, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.haltBug) - cpuPtr);

            // exit
            syncCyclesExecuted();
            builder.mov(pcReg32, pc); // exits need to set PC themselves
            builder.call(saveAndExitPtr - builder.getPtr());
            break;
        }

        case 0x80: // ADD A,B
        case 0x81: // ADD A,C
        case 0x82: // ADD A,D
        case 0x83: // ADD A,E
        case 0x84: // ADD A,H
        case 0x85: // ADD A,L
        case 0x87: // ADD A,A
            add(regMap8[opcode & 7]);
            break;
        case 0x86: // ADD (HL)
        {
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            add(tmp);
            break;
        }

        case 0x88: // ADC A,B
        case 0x89: // ADC A,C
        case 0x8A: // ADC A,D
        case 0x8B: // ADC A,E
        case 0x8C: // ADC A,F
        case 0x8D: // ADC A,H
        case 0x8F: // ADC A,A
            addWithCarry(regMap8[opcode & 7]);
            break;
        case 0x8E: // ADC (HL)
        {
            auto tmp = Reg8::SIL; // can't use F here so use something wierd
            readMem(reg(WReg::HL), tmp);
            addWithCarry(tmp);
            break;
        }

        case 0x90: // SUB B
        case 0x91: // SUB C
        case 0x92: // SUB D
        case 0x93: // SUB E
        case 0x94: // SUB H
        case 0x95: // SUB L
        case 0x97: // SUB A
            sub(regMap8[opcode & 7]);
            break;
        case 0x96: // SUB (HL)
        {
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            sub(tmp);
            break;
        }

        case 0x98: // SBC B
        case 0x99: // SBC C
        case 0x9A: // SBC D
        case 0x9B: // SBC E
        case 0x9C: // SBC H
        case 0x9D: // SBC L
        case 0x9F: // SBC A
            subWithCarry(regMap8[opcode & 7]);
            break;
        case 0x9E: // SBC (HL)
        {
            auto tmp = Reg8::SIL; // can't use F here so use something wierd
            readMem(reg(WReg::HL), tmp);
            subWithCarry(tmp);
            break;
        }

        case 0xA0: // AND B
        case 0xA1: // AND C
        case 0xA2: // AND D
        case 0xA3: // AND E
        case 0xA4: // AND H
        case 0xA5: // AND L
        case 0xA7: // AND A
            bitAnd(regMap8[opcode & 7]);
            break;
        case 0xA6: // AND (HL)
        {
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            bitAnd(tmp);
            break;
        }

        case 0xA8: // XOR B
        case 0xA9: // XOR C
        case 0xAA: // XOR D
        case 0xAB: // XOR E
        case 0xAC: // XOR H
        case 0xAD: // XOR L
            bitXor(regMap8[opcode & 7]);
            break;
        case 0xAE: // XOR (HL)
        {
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            bitXor(tmp);
            break;
        }
        case 0xAF: // XOR A
            if(instr.flags & Op_WriteFlags)
                builder.mov(Reg32::EAX, DMGCPU::Flag_Z); // A = 0, F = Z
            else
                builder.mov(reg(Reg::A), 0);
            break;
        case 0xB0: // OR B
        case 0xB1: // OR C
        case 0xB2: // OR D
        case 0xB3: // OR E
        case 0xB4: // OR H
        case 0xB5: // OR L
        case 0xB7: // OR A
            bitOr(regMap8[opcode & 7]);
            break;
        case 0xB6: // OR (HL)
        {
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            bitOr(tmp);
            break;
        }

        case 0xB8: // CP B
        case 0xB9: // CP C
        case 0xBA: // CP D
        case 0xBB: // CP E
        case 0xBC: // CP H
        case 0xBD: // CP L
        case 0xBF: // CP A
            cmp(regMap8[opcode & 7]);
            break;
        case 0xBE: // CP (HL)
        {
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            cmp(tmp);
            break;
        }

        case 0xC0: // RET NZ
        case 0xC8: // RET Z
        case 0xD0: // RET NC
        case 0xD8: // RET C
        {
            int flag = opcode & (1 << 4) ? DMGCPU::Flag_C : DMGCPU::Flag_Z;
            ret(flag, opcode & (1 << 3));
            break;
        }

        case 0xC1: // POP BC
        case 0xD1: // POP DE
        case 0xE1: // POP HL
            pop(regMap16[(opcode >> 4) & 3]);
            break;

        case 0xC2: // JP NZ,nn
        case 0xCA: // JP Z,nn
        case 0xDA: // JP C,nn
        case 0xD2: // JP NC,nn
        {
            int flag = opcode & (1 << 4) ? DMGCPU::Flag_C : DMGCPU::Flag_Z;
            jump(flag, opcode & (1 << 3));
            break;
        }
        case 0xC3: // JP nn
            jump();
            break;

        case 0xC4: // CALL NZ,nn
        case 0xCC: // CALL Z,nn
        case 0xD4: // CALL NC,nn
        case 0xDC: // CALL C,nn
        {
            int flag = opcode & (1 << 4) ? DMGCPU::Flag_C : DMGCPU::Flag_Z;
            call(flag, opcode & (1 << 3));
            break;
        }

        case 0xC5: // PUSH BC
        case 0xD5: // PUSH DE
        case 0xE5: // PUSH HL
            push(regMap16[(opcode >> 4) & 3]);
            break;

        case 0xC6: // ADD n
        {
            cycleExecuted();
            add(instr.opcode[1]);
            break;
        }

        case 0xC7: // RST 0
        case 0xCF: // RST 08
        case 0xD7: // RST 10
        case 0xDF: // RST 18
        case 0xE7: // RST 20
        case 0xEF: // RST 28
        case 0xF7: // RST 30
        case 0xFF: // RST 38
            reset(opcode & 0x38);
            break;

        case 0xC9: // RET
            ret();
            break;

        case 0xCB:
        {
            uint8_t exOpcode = instr.opcode[1];

            cycleExecuted();

            bool isMem = (exOpcode & 7) == 6; // (HL)

            if(isMem)
                readMem(reg(WReg::HL), Reg8::R10B);

            recompileExInstruction(instr, builder);

            if(isMem && (exOpcode >= 0x80 || exOpcode < 0x40)) // BIT doesn't write back
                writeMem(reg(WReg::HL), Reg8::R10B);
            break;
        }

        case 0xCD: // CALL nn
            call();
            break;

        case 0xCE: // ADC n
        {
            cycleExecuted();
            addWithCarry(instr.opcode[1]);
            break;
        }

        case 0xD6: // SUB n
        {
            cycleExecuted();
            sub(instr.opcode[1]);
            break;
        }

        case 0xD9: // RETI
            // masterInterruptEnable = true
            builder.mov(1, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - reinterpret_cast<uintptr_t>(&cpu));
            ret();
            break;

        case 0xDE: // SBC n
        {
            cycleExecuted();
            subWithCarry(instr.opcode[1]);
            break;
        }

        case 0xE0: // LDH (n),A
        {
            uint16_t addr = 0xFF00 | instr.opcode[1];
            cycleExecuted();
            writeMem(addr, reg(Reg::A));
            break;
        }

        case 0xE2: // LDH (C),A
        {
            builder.movzx(Reg32::R10D, reg(Reg::C));
            builder.or_(Reg32::R10D, 0xFF00);
            writeMem(Reg16::R10W, reg(Reg::A));
            break;
        }

        case 0xE6: // AND n
        {
            cycleExecuted();
            bitAnd(instr.opcode[1]);
            break;
        }

        case 0xE8: // ADD SP,n
        case 0xF8: // LDHL SP,n
        {
            auto f = reg(Reg::F);

            auto b = instr.opcode[1];
            cycleExecuted();

            // flags are set as if this is an 8 bit op
            builder.mov(f, 0);

            // 8 bit add
            builder.mov(Reg8::R10B, spReg8);
            builder.add(Reg8::R10B, b);

            // carry flag
            builder.jcc(Condition::AE, 3); // if !carry
            builder.or_(f, DMGCPU::Flag_C); // set C

            // half add
            builder.mov(Reg8::R10B, spReg8);
            builder.and_(Reg8::R10B, 0xF);
            builder.add(Reg8::R10B, b & 0xF);

            // half carry flag if > 0xF
            builder.cmp(Reg8::R10B, 0xF);
            builder.jcc(Condition::BE, 3); // <= 0xF
            builder.or_(f, DMGCPU::Flag_H); // set H

            // real add
            if(opcode == 0xE8) // ADD SP,n
            {
                builder.add(spReg16, static_cast<int8_t>(b));

                // 2x delay
                cycleExecuted();
            }
            else // LDHL SP,n
            {
                builder.mov(static_cast<Reg32>(reg(WReg::HL)), spReg32);
                builder.add(reg(WReg::HL), static_cast<int8_t>(b));
            }

            cycleExecuted();

            break;
        }
        case 0xE9: // JP (HL)
            syncCyclesExecuted();
            builder.movzx(pcReg32, reg(WReg::HL)); // PC = HL
            builder.jmp(exitPtr - builder.getPtr()); // exit
            break;
        case 0xEA: // LD (nn),A
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            writeMem(addr, reg(Reg::A));
            break;
        }

        case 0xEE: // XOR n
        {
            cycleExecuted();
            bitXor(instr.opcode[1]);
            break;
        }

        case 0xF0: // LDH A,(n)
        {
            uint16_t addr = 0xFF00 | instr.opcode[1];
            cycleExecuted();
            readMem(addr, reg(Reg::A));
            break;
        }
        case 0xF1: // POP AF
            pop(reg(WReg::AF));
            break;
        case 0xF2: // LDH A,(C)
        {
            builder.movzx(Reg32::R10D, reg(Reg::C));
            builder.or_(Reg32::R10D, 0xFF00);
            readMem(Reg16::R10W, reg(Reg::A));
            break;
        }
        case 0xF3: // DI
            // masterInterruptEnable = false
            // TODO: after next instruction (DMGCPU also has this TODO)
            builder.mov(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - reinterpret_cast<uintptr_t>(&cpu));
            break;

        case 0xF5: // PUSH AF
            push(reg(WReg::AF));
            break;
        case 0xF6: // OR n
        {
            cycleExecuted();
            bitOr(instr.opcode[1]);
            break;
        }

        case 0xF9: // LD SP,HL
            cycleExecuted();
            builder.mov(spReg32, static_cast<Reg32>(reg(WReg::HL)));
            break;
        case 0xFA: // LD A,(nn)
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            readMem(addr, reg(Reg::A));
            break;
        }
        case 0xFB: // EI
            // enableInterruptsNextCycle = true
            // TODO: if we store the CPU ptr then we can use a disp here
            builder.mov(Reg64::R10, reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle));
            builder.mov(1, Reg64::R10);
            break;

        case 0xFE: // CP n
        {
            cycleExecuted();
            cmp(instr.opcode[1]);
            break;
        }

        default:
            printf("unhandled op in recompile %02X\n", opcode);
            builder.resetPtr(oldPtr);
            builder.mov(pcReg32, pc - 1);
            builder.jmp(exitPtr - builder.getPtr());
            return false;
    }

    syncCyclesExecuted();

    if(!(instr.flags & Op_Last)) // TODO: also safe to omit if there's an unconditional exit
    {
        // cycles -= executed
        if(cyclesThisInstr) // 0 means we already did the sub
            builder.sub(Reg32::EDI, cyclesThisInstr);

        lastInstrCycleCheck = builder.getPtr(); // save in case the next instr is a branch target

        // if <= 0 exit
        builder.jcc(Condition::G, 11);
        builder.mov(pcReg32, pc);
        builder.call(saveAndExitPtr - builder.getPtr());

        // interrupt check after EI
        if(checkInterrupts)
        {
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

            // if servicableInterrupts != 0
            builder.cmp(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.serviceableInterrupts) - cpuPtr);
            builder.jcc(Condition::E, 11);
            builder.mov(pcReg32, pc);
            builder.call(saveAndExitPtr - builder.getPtr());
        }
    }

    return true;
}

void DMGRecompiler::recompileExInstruction(OpInfo &instr, X86Builder &builder)
{
    uint8_t opcode = instr.opcode[1];

    using Reg = DMGCPU::Reg;

    static const Reg8 regMap8[]
    {
        reg(Reg::B),
        reg(Reg::C),
        reg(Reg::D),
        reg(Reg::E),
        reg(Reg::H),
        reg(Reg::L),
        Reg8::R10B, // where we load to for (HL)
        reg(Reg::A),
    };

    // the flags here are more simple
    // all the shifts/rotates set C and Z
    // SWAP only sets Z
    const auto setFlags = [&instr, &builder](Reg8 r, bool isRotate = false, bool setC = true)
    {
        auto f = reg(Reg::F);

        if(setC && (instr.flags & DMGCPU::Flag_C))
        {
            builder.setcc(Condition::B, f);
            builder.shl(f, 4); // C is bit 4

            // also want the other one
            if(instr.flags & DMGCPU::Flag_Z)
            {
                builder.cmp(r, 0);
                builder.jcc(Condition::NE, 3); // if != 0
                builder.or_(f, DMGCPU::Flag_Z); // set Z
            }
        }
        else if(instr.flags & DMGCPU::Flag_Z)
        {
            if(isRotate)
                builder.cmp(r, 0); // rotates don't set ZF

            builder.setcc(Condition::E, f);
            builder.shl(f, 7); // Z is bit 7
        }
        else if(instr.flags & Op_WriteFlags)
            builder.mov(f, 0); // something relying on an unset flag
    };

    auto r = regMap8[opcode & 7];

    if(opcode < 0x40) // shifts/rotates
    {
        switch(opcode & ~7)
        {
            case 0x00: // RLC
                builder.rol(r, 1);
                setFlags(r, true);
                break;

            case 0x08: // RRC
                builder.ror(r, 1);
                setFlags(r, true);
                break;

            case 0x10: // RL
            {
                auto f = reg(Reg::F);

                // copy carry flag
                builder.test(f, DMGCPU::Flag_C); // sets CF to 0
                builder.jcc(Condition::E, 1); // not set
                builder.stc(); // CF = 1

                builder.rcl(r, 1);
                setFlags(r, true);
                break;
            }

            case 0x18: // RR
            {
                auto f = reg(Reg::F);

                // copy carry flag
                builder.test(f, DMGCPU::Flag_C); // sets CF to 0
                builder.jcc(Condition::E, 1); // not set
                builder.stc(); // CF = 1

                builder.rcr(r, 1);
                setFlags(r, true);
                break;
            }

            case 0x20: // SLA
                builder.shl(r, 1);
                setFlags(r);
                break;

            case 0x28: // SRA
                builder.sar(r, 1);
                setFlags(r);
                break;

            case 0x30: // SWAP
                builder.rol(r, 4); // v = (v >> 4) | (v << 4);
                setFlags(r, true, false);
                break;
            
            case 0x38: // SRL
                builder.shr(r, 1);
                setFlags(r);
                break;
        }
    }
    else if(opcode < 0x80) // BIT
    {
        int bit = (opcode >> 3) & 7;
        auto f = reg(Reg::F);

        if(instr.flags & Op_WriteFlags)
            builder.and_(f, DMGCPU::Flag_C); // preserve C
        
        if(instr.flags & DMGCPU::Flag_H)
            builder.or_(f, DMGCPU::Flag_H); // set H

        builder.test(r, 1 << bit);

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.jcc(Condition::NE, 3); // if != 0
            builder.or_(f, DMGCPU::Flag_Z); // set Z
        }
    }
    else if(opcode < 0xC0) // RES
    {
        int bit = (opcode >> 3) & 7;
        builder.and_(r, ~(1 << bit));
    }
    else // SET
    {
        int bit = (opcode >> 3) & 7;
        builder.or_(r, 1 << bit);
    }
}

void DMGRecompiler::compileEntry()
{
    X86Builder builder(codeBuf, codeBuf + codeBufSize);

    // prologue
    builder.push(Reg64::RBP);
    builder.mov(Reg64::RBP, Reg64::RSP);

    // save
    builder.push(Reg64::R12);
    builder.push(Reg64::R13);
    builder.push(Reg64::R14);
    builder.push(Reg64::RBX);
    builder.push(Reg64::RDX);
    builder.push(Reg64::RCX);
    builder.push(Reg64::RSI);
    builder.sub(Reg64::RSP, 8); // align stack

    // load emu sp
    builder.movzxW(spReg32, Reg64::RCX);

    // load emu regs
    builder.movzxW(Reg32::EAX, Reg64::RSI);
    builder.movzxW(Reg32::ECX, Reg64::RSI, 2);
    builder.movzxW(Reg32::EDX, Reg64::RSI, 4);
    builder.movzxW(Reg32::EBX, Reg64::RSI, 6);

    // store pointer to CPU
    builder.mov(Reg64::R14, reinterpret_cast<uintptr_t>(&cpu));

    // jump to code
    builder.jmp(Reg64::R8);

    // exit setting the call flag ... and saving ip
    exitForCallPtr = builder.getPtr();
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(&exitCallFlag));
    builder.mov(1, Reg64::R11);

    // exit saving ip
    saveAndExitPtr = builder.getPtr();
    builder.pop(Reg64::R10); // ret address (this is called)
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(&tmpSavedPtr));
    builder.mov(Reg64::R10, Reg64::R11, true);

    // just exit
    exitPtr = builder.getPtr();

    builder.add(Reg64::RSP, 8); // alignment

    // restore regs ptr
    builder.pop(Reg64::RSI);

    // save emu regs
    builder.mov(Reg16::AX, Reg64::RSI, true);
    builder.mov(Reg16::CX, Reg64::RSI, true, 2);
    builder.mov(Reg16::DX, Reg64::RSI, true, 4);
    builder.mov(Reg16::BX, Reg64::RSI, true, 6);

    // restore
    builder.pop(Reg64::RCX);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RBX);

    // save emu pc/sp
    builder.mov(pcReg16, Reg64::RDX, true);
    builder.mov(spReg16, Reg64::RCX, true);

    builder.pop(Reg64::R14);
    builder.pop(Reg64::R13);
    builder.pop(Reg64::R12);

    // epilogue
    builder.pop(Reg64::RBP);
    builder.ret();

    entryFunc = reinterpret_cast<CompiledFunc>(codeBuf);

    int len = builder.getPtr() - codeBuf;
    curCodePtr = builder.getPtr();

    //debug
    printf("generated %i bytes for entry/exit\ncode:", len);

    for(auto p = codeBuf; p !=curCodePtr; p++)
        printf(" %02X", *p);

    printf("\n");
}

// wrappers around member funcs
void DMGRecompiler::cycleExecuted(DMGCPU *cpu)
{
    cpu->cycleExecuted();
}

uint8_t DMGRecompiler::readMem(DMGCPU *cpu, uint16_t addr)
{
    return cpu->readMem(addr);
}

int DMGRecompiler::writeMem(DMGCPU *cpu, uint16_t addr, uint8_t data, int cyclesToRun)
{
    auto &compiler = cpu->compiler; // oh right, this is a static func

    // invalidate code on mem write
    if(addr >= compiler.minRAMCode)
    {
        auto mappedAddr = cpu->mem.makeBankedAddress(addr);

        // skip anything not in RAM
        for(auto it = compiler.compiled.lower_bound(1 << 28 /*VRAM as a mapped address */); it != compiler.compiled.end();)
        {
            if(mappedAddr >= it->first && mappedAddr < it->second.endPC)
            {
                printf("invalidate compiled code @%07X(%04X) in %04X-%04X\n", mappedAddr, addr, it->first, it->second.endPC);

                // rewind if last code compiled
                // TODO: reclaim memory in other cases
                if(it->second.endPtr == compiler.curCodePtr)
                    compiler.curCodePtr = it->second.startPtr;

                // invalidate any saved return addresses
                for(auto &saved : compiler.savedExits)
                {
                    if(std::get<1>(saved) >= it->first && std::get<1>(saved) <= it->second.endPC)
                        saved = {nullptr, 0};
                }

                it = compiler.compiled.erase(it);

                continue; // might have compiled the same code more than once
            }

            // past this address, stop
            if(it->first > mappedAddr)
                break;

            ++it;
        }
    }

    cpu->writeMem(addr, data);

    // if we wrote a reg, timings may have changed
    // ... and we have a convenient return value to pass the new value back

    // ... also, there could be an interrupt/DMA pending RIGHT NOW
    bool inHRAM = cpu->pc >= 0xFF00; // PC is not up-to-date here, but should be close
    if((cpu->masterInterruptEnable && cpu->serviceableInterrupts) || cpu->gdmaTriggered || (!inHRAM && (cpu->oamDMADelay || cpu->oamDMACount)))
    {
        // we're going to miss the first updateOAMDMA call
        if(cpu->oamDMADelay == 2)
            cpu->oamDMADelay--;

        return 0;
    }

    int cycles = std::min(cyclesToRun, cpu->getDisplay().getCyclesToNextUpdate());

    if(cpu->nextTimerInterrupt)
        cycles = std::min(cycles, static_cast<int>(cpu->nextTimerInterrupt - cpu->cycleCount));

    return cycles;
}