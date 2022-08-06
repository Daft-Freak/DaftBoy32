#include <cassert>
#include <cstdio>
#include <variant>

#include "DMGRecompilerThumb.h"

#include "DMGCPU.h"
#include "ThumbBuilder.h"

bool DMGRecompilerThumb::compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo)
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codePtr);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBuf + codeBufSize));

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

    int len = endPtr - codePtr16;

    //debug
    printf("recompile @%04X generated %i halfwords (%i instructions)\ncode:", startPC, len, numInstructions);

    for(auto p = codePtr16; p != endPtr; p++)
        printf(" %02X %02X", *p & 0xFF, *p >> 8);

    printf("\n");
    printf("(addr %p->%p)\n", codePtr, endPtr);

    codePtr = reinterpret_cast<uint8_t *>(endPtr);

    // need to clear the cache
    // TODO: don't do the entire range
    __builtin___clear_cache(codeBuf, codeBuf + codeBufSize);

    return true;
}

bool DMGRecompilerThumb::recompileInstruction(uint16_t &pc, OpInfo &instr, ThumbBuilder &builder)
{
    //using Reg = DMGCPU::Reg;
    using WReg = DMGCPU::WReg;

    auto &mem = cpu.getMem();
    uint8_t opcode = instr.opcode[0];

    auto getOff = [&builder](uint8_t *ptr)
    {
        return ptr - reinterpret_cast<uint8_t *>(builder.getPtr());
    };

    pc += instr.len;

    auto oldPtr = builder.getPtr();
    //cycleExecuted

    switch(opcode)
    {
        //case 0x00: // NOP
        //    break;

        default:
            printf("unhandled op in recompile %02X\n", opcode);
            builder.resetPtr(oldPtr);

            builder.mov(Reg::R1, (pc - 1) & 0xFF);
            builder.mov(Reg::R0, (pc - 1) >> 8);
            builder.lsl(Reg::R0, Reg::R0, 8);
            builder.orr(Reg::R1, Reg::R0);
            
            builder.bl(getOff(exitPtr));
            return false;
    }

    //syncCyclesExecuted();

    // cycle check

    return true;
}

void DMGRecompilerThumb::compileEntry()
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codeBuf);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBuf + codeBufSize));

    auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

    //FIXME: actually implement this
    
    builder.mov(Reg::R2, Reg::R8);
    builder.mov(Reg::R3, Reg::R9);
    builder.push((1 << 2) | (1 << 3), true); // R2-3, LR
    
    // set the low bit so we stay in thumb mode
    builder.mov(Reg::R2, 1);
    builder.orr(Reg::R1, Reg::R2);

    // load cpu pointer
    builder.ldr(Reg::R2, 20);
    builder.mov(Reg::R8, Reg::R2);

    builder.bx(Reg::R1);

    exitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    // exit
    // store PC
    int pcOff = reinterpret_cast<uintptr_t>(&cpu.pc) - cpuPtr;
    assert(pcOff <= 0xFF);
    builder.mov(Reg::R2, Reg::R8); // cpu ptr
    builder.mov(Reg::R0, pcOff);
    builder.strh(Reg::R1, Reg::R2, Reg::R0);

    builder.pop((1 << 2) | (1 << 3), false); // R2-3
    builder.mov(Reg::R8, Reg::R2);
    builder.mov(Reg::R9, Reg::R3);

    builder.pop(0, true); // PC

    entryFunc = reinterpret_cast<CompiledFunc>(codeBuf + 1/*thumb*/);

    int len = builder.getPtr() - codePtr16;

    // write cpu addr
    auto ptr = builder.getPtr();
    *ptr++ = 0; // align
    *ptr++ = cpuPtr;
    *ptr++ = cpuPtr >> 16;

    curCodePtr = reinterpret_cast<uint8_t *>(ptr);

    //debug
    printf("generated %i halfwords for entry/exit\ncode:", len);

    for(auto p = codeBuf; p !=curCodePtr; p++)
        printf(" %02X", *p);

    printf("\n");
}
