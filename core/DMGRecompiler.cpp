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
    void add(Reg8 dst, Reg8 src);

    void call(Reg64 r);

    void lea(Reg32 r, Reg64 base, int disp = 0);

    void mov(Reg64 dst, Reg64 src);
    void mov(Reg32 r, Reg64 base, bool isStore = false, int disp = 0);
    void mov(Reg16 r, Reg64 base, bool isStore = false, int disp = 0);
    void mov(Reg32 r, uint32_t imm);
    void mov(Reg64 r, uint64_t imm);

    void movzxW(Reg32 r, Reg64 base, int disp = 0);

    void pop(Reg64 r);

    void push(Reg64 r);

    void ret();

    uint8_t *getPtr() const {return ptr;}

    bool getError() const {return error;}

private:
    void write(uint8_t b);

    void encodeModRM(int reg, int baseReg, int disp);
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

    write(0x01); // opcode

    write(0xC0 | (srcReg & 7) << 3 | (dstReg & 7));
};

// reg -> reg, 8bit
void X86Builder::add(Reg8 dst, Reg8 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(false, srcReg, 0, dstReg);

    // FIXME: check no xH reg with REX (low byte of BP, SP, DI, SI instead )

    write(0x00); // opcode

    write(0xC0 | (srcReg & 7) << 3 | (dstReg & 7));
};

// indirect
void X86Builder::call(Reg64 r)
{
    auto reg = static_cast<int>(r);

    encodeREX(false, 0, 0, reg);

    write(0xFF); // opcode
    write(0xD0 | (reg & 7));
};

void X86Builder::lea(Reg32 r, Reg64 base, int disp)
{
    auto reg = static_cast<int>(r);
    auto baseReg = static_cast<int>(base);

    encodeREX(false, reg, 0, baseReg);

    write(0x8D); // opcode

    encodeModRM(reg, baseReg, disp);
};

// reg -> reg
void X86Builder::mov(Reg64 dst, Reg64 src)
{
    auto dstReg = static_cast<int>(dst);
    auto srcReg = static_cast<int>(src);

    encodeREX(true, srcReg, 0, dstReg);
    write(0x89); // opcode, w = 1
    write(0xC0 | ((srcReg & 7) << 3) | (dstReg & 7)); // mod 3, regs
};

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
};

// reg <-> mem, 16 bit
void X86Builder::mov(Reg16 r, Reg64 base, bool isStore, int disp)
{
    write(0x66); // 16 bit override
    mov(static_cast<Reg32>(r), base, isStore, disp);
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
};

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

// sign extend, mem -> reg, 16 bit
void X86Builder::movzxW(Reg32 r, Reg64 base, int disp)
{
    auto reg = static_cast<int>(r);
    auto baseReg = static_cast<int>(base);

    encodeREX(false, reg, 0, baseReg);

    write(0x0F); // two byte opcode
    write(0xB7); // opcode, w = 1

    encodeModRM(reg, baseReg, disp);
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

    if(reg > 7)
        write(0x41); // REX.B

    write(0x50 | (reg & 0x7)); // opcode
};

void X86Builder::ret()
{
    write(0xC3); // opcode
};

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

void X86Builder::encodeREX(bool w, int reg, int index, int base)
{
    if(!w && reg <= 7 && index <= 7 && base <= 7)
        return;

    write(0x40 | (w ? REX_W : 0)
               | (reg > 7 ? REX_R : 0)
               | (index > 7 ? REX_X : 0)
               | (base > 7 ? REX_B : 0));
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

    // prologue
    builder.push(Reg64::RBP);
    builder.mov(Reg64::RBP, Reg64::RSP);

    // TODO: reuse prologe/epilogue
    // save
    builder.push(Reg64::RBX);
    builder.push(Reg64::RDX);
    builder.push(Reg64::RCX);
    builder.push(Reg64::RSI);

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
        printf("recompile @%04X failed due to error (out of space?)\n", pc);
        return false;
    }

    auto endPtr = builder.getPtr();

    int len = endPtr - codePtr;

    //debug
    printf("recompile @%04X generated %i bytes (%i instructions)\ncode:", pc, len, numInstructions);

    for(auto p = codePtr; p != endPtr; p++)
        printf(" %02X", *p);

    printf("\n");

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

    // TODO: shared trampolines?
    auto cycleExecuted = [&builder, this]()
    {
        // save
        builder.push(Reg64::RAX);
        builder.push(Reg64::RCX);
        builder.push(Reg64::RDX);
        builder.push(Reg64::R8);
        builder.push(Reg64::R9);
        builder.push(Reg64::RSI);
        builder.push(Reg64::RDI);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompiler::cycleExecuted)); // function ptr
        builder.mov(Reg64::RDI, reinterpret_cast<uintptr_t>(&cpu)); // cpu/this ptr (TODO: store cpu pointer in a reg?)
        builder.call(Reg64::RAX); // do call

        // restore
        builder.pop(Reg64::RDI);
        builder.pop(Reg64::RSI);
        builder.pop(Reg64::R9);
        builder.pop(Reg64::R8);
        builder.pop(Reg64::RDX);
        builder.pop(Reg64::RCX);
        builder.pop(Reg64::RAX);
    };

    switch(opcode)
    {
        case 0xAF: // XOR A
            cycleExecuted();
            builder.mov(Reg32::EAX, DMGCPU::Flag_Z); // A = 0, F = Z
            break;

        default:
            printf("unhandled op in recompile %02X\n", opcode);
            return false;
    }

    return true;
}

// wrappers around member funcs
void DMGRecompiler::cycleExecuted(DMGCPU *cpu)
{
    cpu->cycleExecuted();
}