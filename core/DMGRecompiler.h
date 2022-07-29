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

    bool compile(uint8_t *&codePtr, uint16_t &pc);

    bool recompileInstruction(uint16_t &pc, X86Builder &builder, bool &exited);
    void recompileExInstruction(uint16_t &pc, X86Builder &builder, int &cyclesThisInstr);

    void compileEntry();

    static void cycleExecuted(DMGCPU *cpu);
    static uint8_t readMem(DMGCPU *cpu, uint16_t addr);
    static int writeMem(DMGCPU *cpu, uint16_t addr, uint8_t data);

    uint8_t *codeBuf, *curCodePtr;
    unsigned int codeBufSize;

    // cycles, regs[4], pc, sp
    using CompiledFunc = void(*)(int &, uint16_t *, uint16_t &, uint16_t &, uint8_t *entryAddr);

    struct FuncInfo
    {
        uint8_t *startPtr;
        uint8_t *endPtr;
        uint32_t endPC;
    };

    std::map<uint32_t, FuncInfo> compiled;

    // common code
    CompiledFunc entryFunc = nullptr;
    uint8_t *exitPtr = nullptr, *saveAndExitPtr = nullptr;

    // saved pointer on exit
    uint8_t *savedPtr = nullptr;
    uint16_t savedPC = 0;
};