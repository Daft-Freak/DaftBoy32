#pragma once
#include <cstdint>
#include <map>

class DMGCPU;
class X86Builder;

class DMGRecompiler
{
public:
    DMGRecompiler(DMGCPU &cpu);

    void handleBranch();

private:
    DMGCPU &cpu;

    bool compile(uint8_t *&codePtr, uint16_t pc);

    bool recompileInstruction(uint16_t &pc, X86Builder &builder);
    bool recompileExInstruction(uint16_t &pc, X86Builder &builder);

    static void cycleExecuted(DMGCPU *cpu);
    static uint8_t readMem(DMGCPU *cpu, uint16_t addr);
    static void writeMem(DMGCPU *cpu, uint16_t addr, uint8_t data);

    uint8_t *codeBuf, *curCodePtr;
    unsigned int codeBufSize;

    // cycles, regs[4], pc, sp
    using CompiledFunc = void(*)(int &, uint16_t *, uint16_t &, uint16_t &);

    std::map<uint16_t, CompiledFunc> compiled;
};