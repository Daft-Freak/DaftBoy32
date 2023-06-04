#include <cassert>
#include <cstdio>

#include "X86Target.h"
#include "X86Builder.h"

// reg helpers
static const Reg64 cpuPtrReg = Reg64::R14;

// only an output at the end, could be anything
static const Reg32 pcReg32 = Reg32::R12D;
static const Reg16 pcReg16 = Reg16::R12W;

static const Reg32 spReg32 = Reg32::R13D;
static const Reg16 spReg16 = Reg16::R13W;
static const Reg8 spReg8 = Reg8::R13B; // mostly for the adds with the strange flags

#ifdef _WIN32
static const Reg64 argumentRegs64[]{Reg64::RCX, Reg64::RDX, Reg64::R8, Reg64::R9};
static const Reg32 argumentRegs32[]{Reg32::ECX, Reg32::EDX, Reg32::R8D, Reg32::R9D};
#else
static const Reg64 argumentRegs64[]{Reg64::RDI, Reg64::RSI, Reg64::RDX, Reg64::RCX}; // + R8/9 but we don't use > 4 args
static const Reg32 argumentRegs32[]{Reg32::EDI, Reg32::ESI, Reg32::EDX, Reg32::ECX};
#endif

void X86Target::init(SourceInfo sourceInfo, void *cpuPtr)
{
    static const Reg32 regList[]
    {
        // from DMGRecompilerX86
        Reg32::EAX,
        Reg32::ECX,
        Reg32::EDX,
        Reg32::EBX,
        Reg32::R13D,
    };
    // also free R11 R15

    // alloc registers
    unsigned int allocOff = 0;

    regAlloc.emplace(0, int(Reg32::R10D)); // temp

    int i = 1;
    for(auto it = sourceInfo.registers.begin() + 1; it != sourceInfo.registers.end(); ++it, i++)
    {
        if(it->alias || it->type != SourceRegType::General)
            continue;

        regAlloc.emplace(i, static_cast<int>(regList[allocOff]));

        allocOff++;

        if(allocOff == std::size(regList))
            break;
    }

    this->sourceInfo = std::move(sourceInfo);
    this->cpuPtr = cpuPtr;
}

bool X86Target::compile(uint8_t *&codePtr, uint16_t pc, GenBlockInfo &blockInfo)
{
    return false;
}

void X86Target::compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize)
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

#ifdef _WIN32
    builder.push(Reg64::RSI);
    builder.push(Reg64::RDI);

    builder.mov(Reg64::RDI, argumentRegs64[0]); // move cycle count
    builder.mov(Reg64::RSI, argumentRegs64[1]); // code ptr
#endif

    // store pointer to CPU
    auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);
    builder.mov(cpuPtrReg, cpuPtrInt);


    // load emu regs
    uint8_t i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16);

            // TODO: helper
            auto alloc = regAlloc.find(i);
            if(alloc != regAlloc.end())
                builder.movzxW(static_cast<Reg32>(alloc->second), cpuPtrReg, reg.cpuOffset); // FIXME: assumes 16 bit regs
        }

        i++;
    }

    // jump to code
    builder.jmp(Reg64::RSI);

    // exit setting the call flag ... and saving ip
    exitForCallPtr = builder.getPtr();
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(sourceInfo.exitCallFlag));
    builder.mov(1, Reg64::R11);

    // exit saving ip
    saveAndExitPtr = builder.getPtr();
    builder.pop(Reg64::R10); // ret address (this is called)
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(sourceInfo.savedExitPtr));
    builder.mov(Reg64::R10, Reg64::R11, true);

    // just exit
    exitPtr = builder.getPtr();

    // save emu regs
    i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16);

            // TODO: helper
            auto alloc = regAlloc.find(i);
            if(alloc != regAlloc.end())
                builder.mov(static_cast<Reg16>(alloc->second), cpuPtrReg, true, reg.cpuOffset); // FIXME: assumes 16 bit regs
        }

        i++;
    }

    // save emu pc
    builder.mov(pcReg16, cpuPtrReg, true, sourceInfo.pcOffset);

    // restore

#ifdef _WIN32
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RSI);
#endif

    builder.pop(Reg64::RBX);
    builder.pop(Reg64::R14);
    builder.pop(Reg64::R13);
    builder.pop(Reg64::R12);

    // epilogue
    builder.pop(Reg64::RBP);
    builder.ret();

#ifdef RECOMPILER_DEBUG
    int len = builder.getPtr() - codeBuf;

    //debug
    printf("generated %i bytes for entry/exit\ncode:", len);

    for(auto p = codeBuf; p != codeBuf + len; p++)
        printf(" %02X", *p);

    printf("\n");
#endif

    codeBuf = builder.getPtr();
}