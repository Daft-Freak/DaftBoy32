#include <cassert>
#include <cstdio>

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
    R15B
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
    void add(Reg64 dst, int8_t src);

    void and_(Reg8 dst, Reg8 src);
    void and_(Reg16 dst, Reg16 src);
    void and_(Reg32 dst, uint32_t imm);
    void and_(Reg8 dst, uint8_t imm);

    void call(Reg64 r);

    void cmp(Reg8 dst, Reg8 src);
    void cmp(Reg32 dst, uint32_t imm);
    void cmp(Reg8 dst, int8_t imm);

    void dec(Reg16 r);
    void dec(Reg8 r);

    void inc(Reg16 r);
    void inc(Reg8 r);

    void jcc(Condition cc, int8_t disp);
    void jmp(int8_t disp);

    void lea(Reg32 r, Reg64 base, int disp = 0);

    void mov(Reg64 dst, Reg64 src);
    void mov(Reg32 dst, Reg32 src);
    void mov(Reg8 dst, Reg8 src);
    void mov(Reg32 r, Reg64 base, bool isStore = false, int disp = 0);
    void mov(Reg16 r, Reg64 base, bool isStore = false, int disp = 0);
    void mov(Reg64 r, uint64_t imm);
    void mov(Reg32 r, uint32_t imm);
    void mov(Reg8 r, uint8_t imm);

    void movzx(Reg32 dst, Reg16 src);
    void movzx(Reg32 dst, Reg8 src);
    void movzxW(Reg32 r, Reg64 base, int disp = 0);

    void or_(Reg8 dst, Reg8 src);
    void or_(Reg8 dst, uint8_t imm);

    void pop(Reg64 r);

    void push(Reg64 r);

    void ret();

    void setcc(Condition cc, Reg8 dst);

    void sub(Reg64 dst, int8_t src);

    void test(Reg8 dst, uint8_t imm);

    uint8_t *getPtr() const {return ptr;}

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
};

// reg -> reg, 16 bit
void X86Builder::add(Reg16 dst, Reg16 src)
{
    write(0x66); // 16 bit override
    add(static_cast<Reg32>(dst), static_cast<Reg32>(src));
};

// reg -> reg, 8bit
void X86Builder::add(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    // FIXME: check no xH reg with REX (low byte of BP, SP, DI, SI instead )
    write(0x00); // opcode
    encodeModRM(dstReg, srcReg);
};

// imm -> reg, 8 bit sign extended
void X86Builder::add(Reg64 dst, int8_t src)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(true, 0, 0, dstReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(dstReg);
    write(src);
};

// reg -> reg, 8 bit
void X86Builder::and_(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x20); // opcode, w = 0
    encodeModRM(dstReg, srcReg);
};

// reg -> reg, 16 bit
void X86Builder::and_(Reg16 dst, Reg16 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    write(0x66); // 16 bit override

    encodeREX(false, srcReg, 0, dstReg);
    write(0x21); // opcode, w = 1
    encodeModRM(dstReg, srcReg);
};

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
};

// indirect
void X86Builder::call(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0xFF); // opcode
    encodeModRM(reg, 2);
};

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
};

// imm -> reg, 8 bit
void X86Builder::cmp(Reg8 dst, int8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, w = 0, s = 0
    encodeModRM(dstReg, 7);
    write(imm);
};

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

void X86Builder::jcc(Condition cc, int8_t disp)
{
    write(0x70 | static_cast<int>(cc)); // opcode
    write(disp);
}

void X86Builder::jmp(int8_t disp)
{
    write(0xEB); // opcode
    write(disp);
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
};

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
};

// reg -> reg, 8 bit
void X86Builder::or_(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);
    write(0x08); // opcode, w = 0
    encodeModRM(dstReg, srcReg);
};

// imm -> reg, 8 bit
void X86Builder::or_(Reg8 dst, uint8_t imm)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(false, 0, 0, dstReg);
    write(0x80); // opcode, s = 0, w = 0
    encodeModRM(dstReg, 1);
    write(imm); // imm
};

void X86Builder::pop(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0x58 | (reg & 0x7)); // opcode
};

void X86Builder::push(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);
    write(0x50 | (reg & 0x7)); // opcode
}

void X86Builder::ret()
{
    write(0xC3); // opcode
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

// imm -> reg, 8 bit sign extended
void X86Builder::sub(Reg64 dst, int8_t src)
{
    auto dstReg = static_cast<int>(dst);

    encodeREX(true, 0, 0, dstReg);
    write(0x83); // opcode, w = 1, s = 1
    encodeModRM(dstReg, 5);
    write(src);
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
               | (reg > 7 ? REX_R : 0)
               | (index > 7 ? REX_X : 0)
               | (base > 7 ? REX_B : 0));
}

// reg helpers
static const Reg8 regMap8[]
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

static const Reg16 regMap16[]
{
    Reg16::AX, // AF
    Reg16::CX, // BC
    Reg16::DX, // DE
    Reg16::BX  // HL
};

inline Reg8 reg(DMGCPU::Reg r)
{
    return regMap8[static_cast<int>(r)];
};

inline Reg16 reg(DMGCPU::WReg r)
{
    return regMap16[static_cast<int>(r)];
};

// call helpers
static void callSave(X86Builder &builder)
{
    builder.push(Reg64::RAX);
    builder.push(Reg64::RCX);
    builder.push(Reg64::RDX);
    builder.push(Reg64::R8);
    builder.push(Reg64::R9);
    builder.push(Reg64::RSI);
    builder.sub(Reg64::RSP, 8); // align stack
}

static void callRestore(X86Builder &builder)
{
    builder.add(Reg64::RSP, 8); // alignment
    builder.pop(Reg64::RSI);
    builder.pop(Reg64::R9);
    builder.pop(Reg64::R8);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);
    builder.pop(Reg64::RAX);
}

static void callRestore(X86Builder &builder, Reg8 dstReg)
{
    builder.add(Reg64::RSP, 8); // alignment
    builder.pop(Reg64::RSI);
    builder.pop(Reg64::R9);
    builder.pop(Reg64::R8);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);

    // mov ret val (if not going to RAX)
    if(dstReg != Reg8::AL && dstReg != Reg8::AH)
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
    else // ... though this is the worst case... (AL == F, so unlikely)
    {
        // EAX = EAX + (R10D & 0xFF00)
        builder.pop(Reg64::R10);
        builder.and_(Reg32::R10D, 0xFF00);
        builder.add(Reg32::EAX, Reg32::R10D); // TODO: OR? (haven't added that to builder yet)
    }
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
}

void DMGRecompiler::handleBranch()
{
    if(!codeBuf)
        return;

    auto it = compiled.find(cpu.pc);

    if(it == compiled.end())
    {
        auto ptr = curCodePtr;
        auto startPtr = ptr;

        if(compile(ptr, cpu.pc))
        {
            it = compiled.emplace(cpu.pc, reinterpret_cast<CompiledFunc>(startPtr)).first;
            curCodePtr = ptr;
        }
        else
            it = compiled.emplace(cpu.pc, nullptr).first;
    }

    if(it->second)
    {
        int cycles = cpu.cyclesToRun; // FIXME: min interrupts
        it->second(cycles, cpu.regs, cpu.pc, cpu.sp);
    }
}

bool DMGRecompiler::compile(uint8_t *&codePtr, uint16_t pc)
{
    X86Builder builder(codePtr, codeBuf + codeBufSize);

    auto startPC = pc;

    // prologue
    builder.push(Reg64::RBP);
    builder.mov(Reg64::RBP, Reg64::RSP);

    // TODO: reuse prologe/epilogue
    // save
    builder.push(Reg64::RBX);
    builder.push(Reg64::RDX);
    builder.push(Reg64::RCX);
    builder.push(Reg64::RSI);
    builder.push(Reg64::RDI);

    // load emu pc/sp
    // TODO: we know what PC is, dont bother loading/updating/saving it
    builder.movzxW(Reg32::R8D, Reg64::RDX);
    builder.movzxW(Reg32::R9D, Reg64::RCX);

    // load emu regs
    builder.movzxW(Reg32::EAX, Reg64::RSI);
    builder.movzxW(Reg32::ECX, Reg64::RSI, 2);
    builder.movzxW(Reg32::EDX, Reg64::RSI, 4);
    builder.movzxW(Reg32::EBX, Reg64::RSI, 6);

    // load cycle count
    builder.mov(Reg32::ESI, Reg64::RDI);

    // do instructions
    int numInstructions = 0;
    
    while(true)
    {
        if(!recompileInstruction(pc, builder))
            break;

        numInstructions++;
    }

    // store cycle count
    builder.pop(Reg64::RDI);
    builder.mov(Reg32::ESI, Reg64::RDI, true); // do we need this?

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
    builder.mov(Reg16::R8W, Reg64::RDX, true);
    builder.mov(Reg16::R9W, Reg64::RCX, true);

    // epilogue
    builder.pop(Reg64::RBP);
    builder.ret();

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

bool DMGRecompiler::recompileInstruction(uint16_t &pc, X86Builder &builder)
{
    auto &mem = cpu.getMem();
    uint8_t opcode = mem.read(pc++);

    // AF = EAX
    // BC = ECX
    // DE = EDX
    // HL = EBX
    // PC = R8D
    // SP = R9D
    // cycles = ESI

    // TODO: should be able to avoid this
    const auto incPC = [&builder]()
    {
        builder.lea(Reg32::R8D, Reg64::R8, 1);
    };

    // TODO: shared trampolines?
    auto cycleExecuted = [&builder, this]()
    {
        callSave(builder);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::cycleExecuted)); // function ptr
        builder.mov(Reg64::RDI, reinterpret_cast<uintptr_t>(&cpu)); // cpu/this ptr (TODO: store cpu pointer in a reg?)
        builder.call(Reg64::RAX); // do call

        callRestore(builder);
    };

    auto readMem = [&builder, this](Reg16 addrReg, Reg8 dstReg)
    {
        callSave(builder);

        builder.movzx(Reg32::ESI, addrReg);
        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::readMem)); // function ptr
        builder.mov(Reg64::RDI, reinterpret_cast<uintptr_t>(&cpu)); // cpu/this ptr (TODO: store cpu pointer in a reg?)
        builder.call(Reg64::RAX); // do call

        callRestore(builder, dstReg);
    };

    auto readMemImmAddr = [&builder, this](uint16_t addr, Reg8 dstReg)
    {
        callSave(builder);

        builder.mov(Reg32::ESI, addr);
        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::readMem)); // function ptr
        builder.mov(Reg64::RDI, reinterpret_cast<uintptr_t>(&cpu)); // cpu/this ptr (TODO: store cpu pointer in a reg?)
        builder.call(Reg64::RAX); // do call

        callRestore(builder, dstReg);
    };

    auto writeMem = [&builder, this](Reg16 addrReg, Reg8 dataReg)
    {
        callSave(builder);

        builder.movzx(Reg32::ESI, addrReg);
        builder.movzx(Reg32::EDX, dataReg);
        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::writeMem)); // function ptr
        builder.mov(Reg64::RDI, reinterpret_cast<uintptr_t>(&cpu)); // cpu/this ptr (TODO: store cpu pointer in a reg?)
        builder.call(Reg64::RAX); // do call

        callRestore(builder);
    };

    auto writeMemImmAddr = [&builder, this](uint16_t addr, Reg8 dataReg)
    {
        callSave(builder);

        builder.mov(Reg32::ESI, addr);
        builder.movzx(Reg32::EDX, dataReg);
        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::writeMem)); // function ptr
        builder.mov(Reg64::RDI, reinterpret_cast<uintptr_t>(&cpu)); // cpu/this ptr (TODO: store cpu pointer in a reg?)
        builder.call(Reg64::RAX); // do call

        callRestore(builder);
    };

    using Reg = DMGCPU::Reg;
    using WReg = DMGCPU::WReg;

    const auto load8 = [this, &pc, &builder, &incPC, &cycleExecuted](Reg r)
    {
        incPC();
        cycleExecuted();
        uint8_t v = cpu.readMem(pc++);
        builder.mov(reg(r), v);
        incPC();
        cycleExecuted();
        return true;
    };

    const auto copy8 = [&builder, &incPC, &cycleExecuted](Reg dst, Reg src)
    {
        incPC();
        cycleExecuted();
        builder.mov(reg(dst), reg(src));
        return true;
    };

    const auto load16 = [this, &pc, &builder, &incPC, &cycleExecuted](WReg r)
    {
        auto lowReg = static_cast<Reg8>(reg(r)); // AX == AL, CX == CL, ...
        auto highReg = static_cast<Reg8>(static_cast<int>(lowReg) + 4); // AH == AL + 4

        incPC();
        cycleExecuted();

        uint8_t v = cpu.readMem(pc++);
        builder.mov(lowReg, v);
        incPC();
        cycleExecuted();

        v = cpu.readMem(pc++);
        builder.mov(highReg, v);
        incPC();
        cycleExecuted();
        return true;
    };

    const auto push = [&builder, &incPC, &cycleExecuted, &writeMem](WReg r)
    {
        auto lowReg = static_cast<Reg8>(reg(r)); // AX == AL, CX == CL, ...
        auto highReg = static_cast<Reg8>(static_cast<int>(lowReg) + 4); // AH == AL + 4

        auto sp = Reg16::R9W;

        incPC();
        cycleExecuted();

        cycleExecuted(); // delay

        builder.dec(sp);
        writeMem(sp, highReg);
        cycleExecuted();

        builder.dec(sp);
        writeMem(sp, lowReg);
        cycleExecuted();
        return true;
    };

    const auto pop = [&builder, &incPC, &cycleExecuted, &readMem](WReg r)
    {
        auto lowReg = static_cast<Reg8>(reg(r)); // AX == AL, CX == CL, ...
        auto highReg = static_cast<Reg8>(static_cast<int>(lowReg) + 4); // AH == AL + 4

        auto sp = Reg16::R9W;

        incPC();
        cycleExecuted();

        readMem(sp, lowReg);
        builder.inc(sp);
        cycleExecuted();

        readMem(sp, highReg);
        builder.inc(sp);
        cycleExecuted();

        // low bits in F can never be set
        if(r == WReg::AF)
            builder.and_(reg(Reg::F), 0xF0);

        return true;
    };

    const auto add = [&builder, &incPC, &cycleExecuted](Reg8 b)
    {
        auto a = reg(Reg::A);
        auto f = reg(Reg::F);

        // TODO: all of this ...

        // copy src/dst (using flags reg as temp)
        if(b == Reg8::AH || b == Reg8::CH || b == Reg8::DH || b == Reg8::BH)
        {
            // legacy regs, extra copy
            builder.mov(f, b);
            builder.mov(Reg8::R10B, f);
        }
        else
            builder.mov(Reg8::R10B, b);

        // F was used as tmp, make a second copy (affects ADD (HL), ADD n)
        if(b == f)
            builder.mov(Reg8::R11B, b);

        builder.mov(f, a);

        // mask and do half add
        builder.and_(f, 0xF);
        builder.and_(Reg8::R10B, 0xF);
        builder.add(Reg8::R10B, f);

        // put tmp reg back
        if(b == f)
            builder.mov(b, Reg8::R11B);

        // TODO: ... can be avoided if we don't need the H flag

        // do add
        builder.add(a, b);

        // flags
        builder.mov(f, 0);

        // carry flag
        builder.jcc(Condition::AE, 3); // if !carry
        builder.or_(f, DMGCPU::Flag_C); // set C

        // half carry flag
        // r10b > 0xF ? H : 0
        builder.cmp(Reg8::R10B, 0xF);
        builder.jcc(Condition::BE, 3); // <= 0xF
        builder.or_(f, DMGCPU::Flag_H); // set H

        // zero flag
        builder.and_(a, a); // TODO: TEST?
        builder.jcc(Condition::NE, 3); // if != 0
        builder.or_(f, DMGCPU::Flag_Z); // set Z

        return true;
    };

    const auto bitAnd = [&builder, &incPC, &cycleExecuted](Reg8 r)
    {
        builder.and_(reg(Reg::A), r);

        // F = Flag_H | (res == 0 ? Flag_Z : 0)
        builder.jcc(Condition::E, 2 + 2); // if == 0
        builder.mov(reg(Reg::F), DMGCPU::Flag_H); // not zero
        builder.jmp(2);
        builder.mov(reg(Reg::F), DMGCPU::Flag_H | DMGCPU::Flag_Z); // zero

        return true;
    };

    const auto bitOr = [&builder](Reg8 r)
    {
        builder.or_(reg(Reg::A), r);

        builder.mov(reg(Reg::F), 0);
        builder.jcc(Condition::NE, 3); // if != 0
        builder.or_(reg(Reg::F), DMGCPU::Flag_Z); // zero

        return true;
    };

    const auto cmp = [&builder, &incPC, &cycleExecuted](Reg8 b)
    {
        auto a = reg(Reg::A);
        auto f = reg(Reg::F);

        // TODO: all of this ...

        // copy src/dst (using flags reg as temp)
        if(b == Reg8::AH || b == Reg8::CH || b == Reg8::DH || b == Reg8::BH)
        {
            // legacy regs, extra copy
            builder.mov(f, b);
            builder.mov(Reg8::R10B, f);
        }
        else
            builder.mov(Reg8::R10B, b);

        // F was used as tmp, make a second copy (affects ADD (HL), ADD n)
        if(b == f)
            builder.mov(Reg8::R11B, b);

        builder.mov(f, a);

        // mask and do half cmp
        builder.and_(f, 0xF);
        builder.and_(Reg8::R10B, 0xF);
        builder.cmp(f, Reg8::R10B);
        builder.setcc(Condition::B, Reg8::R10B); // save carry

        // put tmp reg back
        if(b == f)
            builder.mov(b, Reg8::R11B);

        // TODO: ... can be avoided if we don't need the H flag

        // do cmp
        builder.cmp(a, b);

        // flags
        builder.mov(f, DMGCPU::Flag_N);

        builder.setcc(Condition::E, Reg8::R11B); // save zero

        // carry flag
        builder.jcc(Condition::AE, 3); // if !carry
        builder.or_(f, DMGCPU::Flag_C); // set C

        // half carry flag
        builder.and_(Reg8::R10B, Reg8::R10B); // TODO: TEST?
        builder.jcc(Condition::E, 3); // carry not set
        builder.or_(f, DMGCPU::Flag_H); // set H

        // zero flag
        builder.and_(Reg8::R11B, Reg8::R11B); // TODO: TEST?
        builder.jcc(Condition::E, 3); // if != 0
        builder.or_(f, DMGCPU::Flag_Z); // set Z

        return true;
    };

    const auto inc = [&builder, &incPC, &cycleExecuted](Reg8 r)
    {
        auto f = reg(Reg::F);
        builder.and_(f, DMGCPU::Flag_C); // preserve C

        // H flag
        builder.mov(Reg8::R10B, f); // save F

        builder.mov(f, r); // & 0xF == 0xF (use F as tmp)
        builder.and_(f, 0xF);
        builder.cmp(f, 0xF);
        builder.mov(f, Reg8::R10B); // restore F
        builder.jcc(Condition::NE, 3); // != 0xF
        builder.or_(f, DMGCPU::Flag_H); // set H

        // do inc and set Z
        builder.inc(r);
        builder.jcc(Condition::NE, 3); // if != 0
        builder.or_(f, DMGCPU::Flag_Z); // set Z

        return true;
    };

    const auto dec = [&builder, &incPC, &cycleExecuted](Reg8 r)
    {
        auto f = reg(Reg::F);
        builder.and_(f, DMGCPU::Flag_C); // preserve C

        // H flag
        builder.mov(Reg8::R10B, f); // save F

        builder.mov(f, r); // & 0xF == 0 (use F as tmp)
        builder.and_(f, 0xF);
        builder.mov(f, Reg8::R10B); // restore F
        builder.jcc(Condition::NE, 3); // != 0x0
        builder.or_(f, DMGCPU::Flag_H); // set H

        builder.or_(f, DMGCPU::Flag_N); // set N

        // do dec and set Z
        builder.dec(r);
        builder.jcc(Condition::NE, 3); // if != 0
        builder.or_(f, DMGCPU::Flag_Z); // set Z

        return true;
    };

    const auto add16 = [&builder, &incPC, &cycleExecuted](Reg16 b)
    {
        auto a = reg(WReg::HL);
        auto f = reg(Reg::F);

        // TODO: all of this ...

        // copy src/dst
        builder.mov(Reg32::R10D, static_cast<Reg32>(a));
        builder.mov(Reg32::R11D, static_cast<Reg32>(b));

        // mask and do "half"(3/4?) add
        builder.and_(Reg32::R10D, 0xFFF);
        builder.and_(Reg32::R11D, 0xFFF);
        builder.add(Reg32::R10D, Reg32::R11D);

        // TODO: ... can be avoided if we don't need the H flag

        // preserve Z flag
        builder.and_(f, DMGCPU::Flag_Z);

        // do add
        builder.add(a, b);

        // carry flag
        builder.jcc(Condition::AE, 3); // if !carry
        builder.or_(f, DMGCPU::Flag_C); // set C

        // half carry flag
        // r10d > 0xFFF ? H : 0
        builder.cmp(Reg32::R10D, 0xFFF);
        builder.jcc(Condition::BE, 3); // <= 0xFFF
        builder.or_(f, DMGCPU::Flag_H); // set H

        cycleExecuted();

        return true;
    };

    const auto inc16 = [&builder, &incPC, &cycleExecuted](WReg r)
    {
        incPC();
        cycleExecuted();

        builder.inc(reg(r));
        cycleExecuted();
        return true;
    };

    const auto dec16 = [&builder, &incPC, &cycleExecuted](WReg r)
    {
        incPC();
        cycleExecuted();

        builder.dec(reg(r));
        cycleExecuted();
        return true;
    };

    switch(opcode)
    {
        case 0x01: // LD BC,nn
            return load16(WReg::BC);
        case 0x02: // LD (BC),A
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::BC), reg(Reg::A));
            cycleExecuted();
            break;
        case 0x03: // INC BC
            return inc16(WReg::BC);
        case 0x04: // INC B
            incPC();
            cycleExecuted();
            return inc(reg(Reg::B));
        case 0x05: // DEC B
            incPC();
            cycleExecuted();
            return dec(reg(Reg::B));
        case 0x06: // LD B,n
            return load8(Reg::B);

        case 0x09: // ADD HL,BC
            incPC();
            cycleExecuted();
            return add16(reg(WReg::BC));
        case 0x0A: // LD A,(BC)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::BC), reg(Reg::A));
            cycleExecuted();
            break;
        case 0x0B: // DEC BC
            return dec16(WReg::BC);
        case 0x0C: // INC C
            incPC();
            cycleExecuted();
            return inc(reg(Reg::C));
        case 0x0D: // DEC C
            incPC();
            cycleExecuted();
            return dec(reg(Reg::C));
        case 0x0E: // LD C,n
            return load8(Reg::C);

        case 0x11: // LD DE,nn
            return load16(WReg::DE);
        case 0x12: // LD (DE),A
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::DE), reg(Reg::A));
            cycleExecuted();
            break;
        case 0x13: // INC DE
            return inc16(WReg::DE);
        case 0x14: // INC D
            incPC();
            cycleExecuted();
            return inc(reg(Reg::D));
        case 0x15: // DEC D
            incPC();
            cycleExecuted();
            return dec(reg(Reg::D));
        case 0x16: // LD D,n
            return load8(Reg::D);

        case 0x19: // ADD HL,DE
            incPC();
            cycleExecuted();
            return add16(reg(WReg::DE));
        case 0x1A: // LD A,(DE)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::DE), reg(Reg::A));
            cycleExecuted();
            break;
        case 0x1B: // DEC DE
            return dec16(WReg::DE);
        case 0x1C: // INC E
            incPC();
            cycleExecuted();
            return inc(reg(Reg::E));
        case 0x1D: // DEC E
            incPC();
            cycleExecuted();
            return dec(reg(Reg::E));
        case 0x1E: // LD E,n
            return load8(Reg::E);

        case 0x21: // LD HL,nn
            return load16(WReg::HL);
        case 0x22: // LDI (HL),A
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::A));
            builder.inc(reg(WReg::HL));
            cycleExecuted();
            break;
        case 0x23: // INC HL
            return inc16(WReg::HL);
        case 0x24: // INC H
            incPC();
            cycleExecuted();
            return inc(reg(Reg::H));
        case 0x25: // DEC H
            incPC();
            cycleExecuted();
            return dec(reg(Reg::H));
        case 0x26: // LD H,n
            return load8(Reg::H);

        case 0x29: // ADD HL,HL
            incPC();
            cycleExecuted();
            return add16(reg(WReg::HL));
        case 0x2A: // LDI A,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::A));
            builder.inc(reg(WReg::HL));
            cycleExecuted();
            break;
        case 0x2B: // DEC HL
            return dec16(WReg::HL);
        case 0x2C: // INC L
            incPC();
            cycleExecuted();
            return inc(reg(Reg::L));
        case 0x2D: // DEC L
            incPC();
            cycleExecuted();
            return dec(reg(Reg::L));
        case 0x2E: // LD L,n
            return load8(Reg::L);

        case 0x32: // LDD (HL),A
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::A));
            builder.dec(reg(WReg::HL));
            cycleExecuted();
            break;

        case 0x34: // INC (HL)
        {
            incPC();
            cycleExecuted();
            auto tmp = Reg8::R11B;

            readMem(reg(WReg::HL), tmp);
            cycleExecuted();

            inc(tmp);

            writeMem(reg(WReg::HL), tmp);
            cycleExecuted();            

            break;
        }
        case 0x35: // DEC (HL)
        {
            incPC();
            cycleExecuted();
            auto tmp = Reg8::R11B;

            readMem(reg(WReg::HL), tmp);
            cycleExecuted();

            dec(tmp);

            writeMem(reg(WReg::HL), tmp);
            cycleExecuted();            

            break;
        }
        case 0x36: // LD (HL),n
        {
            incPC();
            cycleExecuted();
            auto val = cpu.readMem(pc++);
            incPC();
            cycleExecuted();
            builder.mov(Reg8::R10B, val); // TODO: yeah, too lazy to add a third writeMem just for this
            writeMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            break;
        }

        case 0x3A: // LDD A,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::A));
            builder.dec(reg(WReg::HL));
            cycleExecuted();
            break;

        case 0x3C: // INC A
            incPC();
            cycleExecuted();
            return inc(reg(Reg::A));
        case 0x3D: // DEC A
            incPC();
            cycleExecuted();
            return dec(reg(Reg::A));
        case 0x3E: // LD A,n
            return load8(Reg::A);

        case 0x40: // LD B,B
            return copy8(Reg::B, Reg::B);
        case 0x41: // LD B,C
            return copy8(Reg::B, Reg::C);
        case 0x42: // LD B,D
            return copy8(Reg::B, Reg::D);
        case 0x43: // LD B,E
            return copy8(Reg::B, Reg::E);
        case 0x44: // LD B,H
            return copy8(Reg::B, Reg::H);
        case 0x45: // LD B,L
            return copy8(Reg::B, Reg::L);
        case 0x46: // LD B,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::B));
            cycleExecuted();
            break;
        case 0x47: // LD B,A
            return copy8(Reg::B, Reg::A);
        case 0x48: // LD C,B
            return copy8(Reg::C, Reg::B);
        case 0x49: // LD C,C
            return copy8(Reg::C, Reg::C);
        case 0x4A: // LD C,D
            return copy8(Reg::C, Reg::D);
        case 0x4B: // LD C,E
            return copy8(Reg::C, Reg::E);
        case 0x4C: // LD C,H
            return copy8(Reg::C, Reg::H);
        case 0x4D: // LD C,L
            return copy8(Reg::C, Reg::L);
        case 0x4E: // LD C,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::C));
            cycleExecuted();
            break;
        case 0x4F: // LD C,A
            return copy8(Reg::C, Reg::A);
        case 0x50: // LD D,B
            return copy8(Reg::D, Reg::B);
        case 0x51: // LD D,C
            return copy8(Reg::D, Reg::C);
        case 0x52: // LD D,D
            return copy8(Reg::D, Reg::D);
        case 0x53: // LD D,E
            return copy8(Reg::D, Reg::E);
        case 0x54: // LD D,H
            return copy8(Reg::D, Reg::H);
        case 0x55: // LD D,L
            return copy8(Reg::D, Reg::L);
        case 0x56: // LD D,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::D));
            cycleExecuted();
            break;
        case 0x57: // LD D,A
            return copy8(Reg::D, Reg::A);
        case 0x58: // LD E,B
            return copy8(Reg::E, Reg::B);
        case 0x59: // LD E,C
            return copy8(Reg::E, Reg::C);
        case 0x5A: // LD E,D
            return copy8(Reg::E, Reg::D);
        case 0x5B: // LD E,E
            return copy8(Reg::E, Reg::E);
        case 0x5C: // LD E,H
            return copy8(Reg::E, Reg::H);
        case 0x5D: // LD E,L
            return copy8(Reg::E, Reg::L);
        case 0x5E: // LD E,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::E));
            cycleExecuted();
            break;
        case 0x5F: // LD E,A
            return copy8(Reg::E, Reg::A);
        case 0x60: // LD H,B
            return copy8(Reg::H, Reg::B);
        case 0x61: // LD H,C
            return copy8(Reg::H, Reg::C);
        case 0x62: // LD H,D
            return copy8(Reg::H, Reg::D);
        case 0x63: // LD H,E
            return copy8(Reg::H, Reg::E);
        case 0x64: // LD H,H
            return copy8(Reg::H, Reg::H);
        case 0x65: // LD H,L
            return copy8(Reg::H, Reg::L);
        case 0x66: // LD H,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::H));
            cycleExecuted();
            break;
        case 0x67: // LD H,A
            return copy8(Reg::H, Reg::A);
        case 0x68: // LD L,B
            return copy8(Reg::L, Reg::B);
        case 0x69: // LD L,C
            return copy8(Reg::L, Reg::C);
        case 0x6A: // LD L,D
            return copy8(Reg::L, Reg::D);
        case 0x6B: // LD L,E
            return copy8(Reg::L, Reg::E);
        case 0x6C: // LD L,H
            return copy8(Reg::L, Reg::H);
        case 0x6D: // LD L,L
            return copy8(Reg::L, Reg::L);
        case 0x6E: // LD L,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::L));
            cycleExecuted();
            break;
        case 0x6F: // LD L,A
            return copy8(Reg::L, Reg::A);
        case 0x70: // LD (HL),B
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::B));
            cycleExecuted();
            break;
        case 0x71: // LD (HL),C
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::C));
            cycleExecuted();
            break;
        case 0x72: // LD (HL),D
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::D));
            cycleExecuted();
            break;
        case 0x73: // LD (HL),E
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::E));
            cycleExecuted();
            break;
        case 0x74: // LD (HL),H
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::H));
            cycleExecuted();
            break;
        case 0x75: // LD (HL),L
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::L));
            cycleExecuted();
            break;

        case 0x77: // LD (HL),A
            incPC();
            cycleExecuted();
            writeMem(reg(WReg::HL), reg(Reg::A));
            cycleExecuted();
            break;
        case 0x78: // LD A,B
            return copy8(Reg::A, Reg::B);
        case 0x79: // LD A,C
            return copy8(Reg::A, Reg::C);
        case 0x7A: // LD A,D
            return copy8(Reg::A, Reg::D);
        case 0x7B: // LD A,E
            return copy8(Reg::A, Reg::E);
        case 0x7C: // LD A,H
            return copy8(Reg::A, Reg::H);
        case 0x7D: // LD A,L
            return copy8(Reg::A, Reg::L);
        case 0x7E: // LD A,(HL)
            incPC();
            cycleExecuted();
            readMem(reg(WReg::HL), reg(Reg::A));
            cycleExecuted();
            break;
        case 0x7F: // LD A,A
            return copy8(Reg::A, Reg::A);

        case 0x80: // ADD A,B
            incPC();
            cycleExecuted();
            return add(reg(Reg::B));
        case 0x81: // ADD A,C
            incPC();
            cycleExecuted();
            return add(reg(Reg::C));
        case 0x82: // ADD A,D
            incPC();
            cycleExecuted();
            return add(reg(Reg::D));
        case 0x83: // ADD A,E
            incPC();
            cycleExecuted();
            return add(reg(Reg::E));
        case 0x84: // ADD A,H
            incPC();
            cycleExecuted();
            return add(reg(Reg::H));
        case 0x85: // ADD A,L
            incPC();
            cycleExecuted();
            return add(reg(Reg::L));
        case 0x86: // ADD (HL)
        {
            incPC();
            cycleExecuted();
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            add(tmp);
            cycleExecuted();
            break;
        }
        case 0x87: // ADD A,A
            incPC();
            cycleExecuted();
            return add(reg(Reg::A));

        case 0xA0: // AND B
            incPC();
            cycleExecuted();
            return bitAnd(reg(Reg::B));
        case 0xA1: // AND C
            incPC();
            cycleExecuted();
            return bitAnd(reg(Reg::C));
        case 0xA2: // AND D
            incPC();
            cycleExecuted();
            return bitAnd(reg(Reg::D));
        case 0xA3: // AND E
            incPC();
            cycleExecuted();
            return bitAnd(reg(Reg::E));
        case 0xA4: // AND H
            incPC();
            cycleExecuted();
            return bitAnd(reg(Reg::H));
        case 0xA5: // AND L
            incPC();
            cycleExecuted();
            return bitAnd(reg(Reg::L));
        case 0xA6: // AND (HL)
        {
            incPC();
            cycleExecuted();
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            bitAnd(tmp);
            cycleExecuted();
            break;
        }
        case 0xA7: // AND A
            incPC();
            cycleExecuted();
            return bitAnd(reg(Reg::A));

        case 0xAF: // XOR A
            incPC();
            cycleExecuted();
            builder.mov(Reg32::EAX, DMGCPU::Flag_Z); // A = 0, F = Z
            break;
        case 0xB0: // OR B
            incPC();
            cycleExecuted();
            return bitOr(reg(Reg::B));
        case 0xB1: // OR C
            incPC();
            cycleExecuted();
            return bitOr(reg(Reg::C));
        case 0xB2: // OR D
            incPC();
            cycleExecuted();
            return bitOr(reg(Reg::D));
        case 0xB3: // OR E
            incPC();
            cycleExecuted();
            return bitOr(reg(Reg::E));
        case 0xB4: // OR H
            incPC();
            cycleExecuted();
            return bitOr(reg(Reg::H));
        case 0xB5: // OR L
            incPC();
            cycleExecuted();
            return bitOr(reg(Reg::L));
        case 0xB6: // OR (HL)
        {
            incPC();
            cycleExecuted();
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            bitOr(tmp);
            cycleExecuted();
            break;
        }
        case 0xB7: // OR A
            incPC();
            cycleExecuted();
            return bitOr(reg(Reg::A));
        case 0xB8: // CP B
            incPC();
            cycleExecuted();
            return cmp(reg(Reg::B));
        case 0xB9: // CP C
            incPC();
            cycleExecuted();
            return cmp(reg(Reg::C));
        case 0xBA: // CP D
            incPC();
            cycleExecuted();
            return cmp(reg(Reg::D));
        case 0xBB: // CP E
            incPC();
            cycleExecuted();
            return cmp(reg(Reg::E));
        case 0xBC: // CP H
            incPC();
            cycleExecuted();
            return cmp(reg(Reg::H));
        case 0xBD: // CP L
            incPC();
            cycleExecuted();
            return cmp(reg(Reg::L));
        case 0xBE: // CP (HL)
        {
            incPC();
            cycleExecuted();
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            cmp(tmp);
            cycleExecuted();
            break;
        }
        case 0xBF: // CP A
            incPC();
            cycleExecuted();
            return cmp(reg(Reg::A));

        case 0xC1: // POP BC
            return pop(WReg::BC);

        case 0xC5: // PUSH BC
            return push(WReg::BC);
        case 0xC6: // ADD n
        {
            // TODO: use imm?
            incPC();
            cycleExecuted();
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            builder.mov(tmp, cpu.readMem(pc++));
            incPC();
            add(tmp);
            cycleExecuted();
            break;
        }

        case 0xCB:
            return recompileExInstruction(pc, builder);

        case 0xD1: // POP DE
            return pop(WReg::DE);

        case 0xD5: // PUSH DE
            return push(WReg::DE);

        case 0xE0: // LDH (n),A
        {
            incPC();
            cycleExecuted();
            auto addr = 0xFF00 | cpu.readMem(pc++);
            incPC();
            cycleExecuted();
            writeMemImmAddr(addr, reg(Reg::A));
            cycleExecuted();
            break;
        }
        case 0xE1: // POP HL
            return pop(WReg::HL);

        case 0xE5: // PUSH HL
            return push(WReg::HL);
        case 0xE6: // AND n
        {
            // TODO: use AND with immediate?
            incPC();
            cycleExecuted();
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            builder.mov(tmp, cpu.readMem(pc++));
            incPC();
            bitAnd(tmp);
            cycleExecuted();
            break;
        }

        case 0xEA: // LD (nn),A
        {
            incPC();
            cycleExecuted();
            uint16_t addr = cpu.readMem(pc++);
            incPC();
            cycleExecuted();
            addr |= (cpu.readMem(pc++) << 8);
            incPC();
            cycleExecuted();

            writeMemImmAddr(addr, reg(Reg::A));
            cycleExecuted();
            break;
        }

        case 0xF0: // LDH A,(n)
        {
            incPC();
            cycleExecuted();
            uint16_t addr = 0xFF00 | cpu.readMem(pc++);
            incPC();
            cycleExecuted();
            readMemImmAddr(addr, reg(Reg::A));
            cycleExecuted();
            break;
        }
        case 0xF1: // POP AF
            return pop(WReg::AF);

        case 0xF5: // PUSH AF
            return push(WReg::AF);
        case 0xF6: // OR n
        {
            // TODO: use imm?
            incPC();
            cycleExecuted();
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            builder.mov(tmp, cpu.readMem(pc++));
            incPC();
            bitOr(tmp);
            cycleExecuted();
            break;
        }

        case 0xFA: // LD A,(nn)
        {
            incPC();
            cycleExecuted();
            uint16_t addr = cpu.readMem(pc++);
            incPC();
            cycleExecuted();
            addr |= (cpu.readMem(pc++) << 8);
            incPC();
            cycleExecuted();

            readMemImmAddr(addr, reg(Reg::A));
            cycleExecuted();
            break;
        }

        case 0xFE: // CP n
        {
            // TODO: use imm?
            incPC();
            cycleExecuted();
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            builder.mov(tmp, cpu.readMem(pc++));
            incPC();
            cmp(tmp);
            cycleExecuted();
            break;
        }

        default:
            printf("unhandled op in recompile %02X\n", opcode);
            return false;
    }

    return true;
}

bool DMGRecompiler::recompileExInstruction(uint16_t &pc, X86Builder &builder)
{
    auto &mem = cpu.getMem();
    uint8_t opcode = mem.read(pc++);

    // FIXME: copy/paste

    // TODO: should be able to avoid this
    const auto incPC = [&builder]()
    {
        builder.lea(Reg32::R8D, Reg64::R8, 1);
    };

    // TODO: shared trampolines?
    auto cycleExecuted = [&builder, this]()
    {
        callSave(builder);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::cycleExecuted)); // function ptr
        builder.mov(Reg64::RDI, reinterpret_cast<uintptr_t>(&cpu)); // cpu/this ptr (TODO: store cpu pointer in a reg?)
        builder.call(Reg64::RAX); // do call

        callRestore(builder);
    };

    auto readMem = [&builder, this](Reg16 addrReg, Reg8 dstReg)
    {
        callSave(builder);

        builder.movzx(Reg32::ESI, addrReg);
        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::readMem)); // function ptr
        builder.mov(Reg64::RDI, reinterpret_cast<uintptr_t>(&cpu)); // cpu/this ptr (TODO: store cpu pointer in a reg?)
        builder.call(Reg64::RAX); // do call

        callRestore(builder, dstReg);
    };

    using Reg = DMGCPU::Reg;
    using WReg = DMGCPU::WReg;

    // early out for unhandled
    if(opcode >= 0x80 || opcode < 0x40)
    {
        printf("unhandled op in recompile CB%02X\n", opcode);
        return false;
    }

    // shifts...

    const auto testBit = [&builder](Reg8 r, int bit)
    {
        auto f = reg(Reg::F);

        builder.and_(f, DMGCPU::Flag_C); // preserve C
        builder.or_(f, DMGCPU::Flag_H); // set H

        builder.test(r, 1 << bit);

        builder.jcc(Condition::NE, 3); // if != 0
        builder.or_(f, DMGCPU::Flag_Z); // set Z

        return true;
    };

    // set/reset

    incPC();
    cycleExecuted();
    incPC();
    cycleExecuted();

    switch(opcode)
    {
        case 0x40: // BIT 0,B
            return testBit(reg(Reg::B), 0);
        case 0x41: // BIT 0,C
            return testBit(reg(Reg::C), 0);
        case 0x42: // BIT 0,D
            return testBit(reg(Reg::D), 0);
        case 0x43: // BIT 0,E
            return testBit(reg(Reg::E), 0);
        case 0x44: // BIT 0,H
            return testBit(reg(Reg::H), 0);
        case 0x45: // BIT 0,L
            return testBit(reg(Reg::L), 0);
        case 0x46: // BIT 0,(HL)
            readMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            return testBit(Reg8::R10B, 0);
        case 0x47: // BIT 0,A
            return testBit(reg(Reg::A), 0);
        case 0x48: // BIT 1,B
            return testBit(reg(Reg::B), 1);
        case 0x49: // BIT 1,C
            return testBit(reg(Reg::C), 1);
        case 0x4A: // BIT 1,D
            return testBit(reg(Reg::D), 1);
        case 0x4B: // BIT 1,E
            return testBit(reg(Reg::E), 1);
        case 0x4C: // BIT 1,H
            return testBit(reg(Reg::H), 1);
        case 0x4D: // BIT 1,L
            return testBit(reg(Reg::L), 1);
        case 0x4E: // BIT 1,(HL)
            readMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            return testBit(Reg8::R10B, 1);
        case 0x4F: // BIT 1,A
            return testBit(reg(Reg::A), 1);
        case 0x50: // BIT 2,B
            return testBit(reg(Reg::B), 2);
        case 0x51: // BIT 2,C
            return testBit(reg(Reg::C), 2);
        case 0x52: // BIT 2,D
            return testBit(reg(Reg::D), 2);
        case 0x53: // BIT 2,E
            return testBit(reg(Reg::E), 2);
        case 0x54: // BIT 2,H
            return testBit(reg(Reg::H), 2);
        case 0x55: // BIT 2,L
            return testBit(reg(Reg::L), 2);
        case 0x56: // BIT 2,(HL)
            readMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            return testBit(Reg8::R10B, 2);
        case 0x57: // BIT 2,A
            return testBit(reg(Reg::A), 2);
        case 0x58: // BIT 3,B
            return testBit(reg(Reg::B), 3);
        case 0x59: // BIT 3,C
            return testBit(reg(Reg::C), 3);
        case 0x5A: // BIT 3,D
            return testBit(reg(Reg::D), 3);
        case 0x5B: // BIT 3,E
            return testBit(reg(Reg::E), 3);
        case 0x5C: // BIT 3,H
            return testBit(reg(Reg::H), 3);
        case 0x5D: // BIT 3,L
            return testBit(reg(Reg::L), 3);
        case 0x5E: // BIT 3,(HL)
            readMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            return testBit(Reg8::R10B, 3);
        case 0x5F: // BIT 3,A
            return testBit(reg(Reg::A), 3);
        case 0x60: // BIT 4,B
            return testBit(reg(Reg::B), 4);
        case 0x61: // BIT 4,C
            return testBit(reg(Reg::C), 4);
        case 0x62: // BIT 4,D
            return testBit(reg(Reg::D), 4);
        case 0x63: // BIT 4,E
            return testBit(reg(Reg::E), 4);
        case 0x64: // BIT 4,H
            return testBit(reg(Reg::H), 4);
        case 0x65: // BIT 4,L
            return testBit(reg(Reg::L), 4);
        case 0x66: // BIT 4,(HL)
            readMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            return testBit(Reg8::R10B, 4);
        case 0x67: // BIT 4,A
            return testBit(reg(Reg::A), 4);
        case 0x68: // BIT 5,B
            return testBit(reg(Reg::B), 5);
        case 0x69: // BIT 5,C
            return testBit(reg(Reg::C), 5);
        case 0x6A: // BIT 5,D
            return testBit(reg(Reg::D), 5);
        case 0x6B: // BIT 5,E
            return testBit(reg(Reg::E), 5);
        case 0x6C: // BIT 5,H
            return testBit(reg(Reg::H), 5);
        case 0x6D: // BIT 5,L
            return testBit(reg(Reg::L), 5);
        case 0x6E: // BIT 5,(HL)
            readMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            return testBit(Reg8::R10B, 5);
        case 0x6F: // BIT 5,A
            return testBit(reg(Reg::A), 5);
        case 0x70: // BIT 6,B
            return testBit(reg(Reg::B), 6);
        case 0x71: // BIT 6,C
            return testBit(reg(Reg::C), 6);
        case 0x72: // BIT 6,D
            return testBit(reg(Reg::D), 6);
        case 0x73: // BIT 6,E
            return testBit(reg(Reg::E), 6);
        case 0x74: // BIT 6,H
            return testBit(reg(Reg::H), 6);
        case 0x75: // BIT 6,L
            return testBit(reg(Reg::L), 6);
        case 0x76: // BIT 6,(HL)
            readMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            return testBit(Reg8::R10B, 6);
        case 0x77: // BIT 6,A
            return testBit(reg(Reg::A), 6);
        case 0x78: // BIT 7,B
            return testBit(reg(Reg::B), 7);
        case 0x79: // BIT 7,C
            return testBit(reg(Reg::C), 7);
        case 0x7A: // BIT 7,D
            return testBit(reg(Reg::D), 7);
        case 0x7B: // BIT 7,E
            return testBit(reg(Reg::E), 7);
        case 0x7C: // BIT 7,H
            return testBit(reg(Reg::H), 7);
        case 0x7D: // BIT 7,L
            return testBit(reg(Reg::L), 7);
        case 0x7E: // BIT 7,(HL)
            readMem(reg(WReg::HL), Reg8::R10B);
            cycleExecuted();
            return testBit(Reg8::R10B, 7);
        case 0x7F: // BIT 7,A
            return testBit(reg(Reg::A), 7);
    }

    return true;
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

void DMGRecompiler::writeMem(DMGCPU *cpu, uint16_t addr, uint8_t data)
{
    cpu->writeMem(addr, data);
}