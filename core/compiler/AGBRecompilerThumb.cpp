#include <cassert>
#include <cstdio>
#include <variant>

#include "AGBRecompilerThumb.h"

#include "AGBCPU.h"
#include "ThumbBuilder.h"

bool AGBRecompilerThumb::compileTHUMB(uint8_t *&codePtr, uint32_t pc, BlockInfo &blockInfo)
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

        if(builder.getError())
            break;

        numInstructions++;
    }

    if(builder.getError())
    {
        ldrLiteralInstrs.clear();
        printf("recompile @%08X failed due to error (out of space?)\n", startPC);
        return false;
    }

    if(numInstructions == 0)
    {
#ifdef RECOMPILER_DEBUG
        printf("recompile @%04X failed to handle any instructions\n", startPC);
#endif
        return false;
    }

    auto endPtr = builder.getPtr();

#ifdef RECOMPILER_DEBUG
    int len = endPtr - codePtr16;

    //debug
    printf("recompile @%04X generated %i halfwords (%i instructions)\ncode:", startPC, len, numInstructions);

    for(auto p = codePtr16; p != endPtr; p++)
        printf(" %02X %02X", *p & 0xFF, *p >> 8);

    printf("\n");
    printf("(addr %p->%p)\n", codePtr, endPtr);
#endif

    codePtr = reinterpret_cast<uint8_t *>(endPtr);

    // need to clear the cache
    // TODO: don't do the entire range
    __builtin___clear_cache(codeBuf, codeBuf + codeBufSize);

    return true;
}

bool AGBRecompilerThumb::recompileInstruction(uint32_t &pc, OpInfo &instr, ThumbBuilder &builder)
{
    pc += 2;

    // literal helpers
    auto loadLiteral = [this, &builder](Reg reg, uint32_t val)
    {
        unsigned int index;
        for(index = 0; index < curLiteral; index++)
        {
            if(literals[index] == val)
                break;
        }

        if(index == curLiteral)
        {
            assert(curLiteral < std::size(literals));
            literals[curLiteral++] = val;
        }

        ldrLiteralInstrs.push_back(builder.getPtr());
        builder.ldr(reg, index << 2); //write literal index, patched later
    };

    auto outputLiterals = [this, &builder]()
    {
        if(!ldrLiteralInstrs.empty())
        {
            auto dataPtr = builder.getPtr() + 1/*B*/;
            if(reinterpret_cast<uintptr_t>(dataPtr) & 2)
            {
                // not aligned
                builder.b(curLiteral * 4 + 2);
                builder.data(0);
                dataPtr++;
            }
            else // aligned
                builder.b(curLiteral * 4);

            for(unsigned int i = 0; i < curLiteral; i++)
            {
                builder.data(literals[i]);
                builder.data(literals[i] >> 16);
            }

            // ran out of space
            if(builder.getError())
                return;

            // patch
            for(auto ptr : ldrLiteralInstrs)
            {
                bool t2 = (*ptr & 0xFF7F) == 0xF85F;

                auto start = ptr + 1;

                if(reinterpret_cast<uintptr_t>(start) & 2)
                    start++;

                if(t2)
                    ptr++;

                // update LDR imm
                auto index = *ptr & 0xFF;
                assert(index < 2);

                auto off = ((dataPtr + index * 2) - start);
                
                if(t2)
                {
                    off <<= 1;
                    assert(off < 0xFFF);
                    *ptr = (*ptr & 0xF000) | off;
                }
                else
                {
                    off >>= 1;
                    assert(off <= 0xFF);
                    *ptr = (*ptr & 0xFF00) | off;
                }
            }
        }

        // reset
        ldrLiteralInstrs.clear();
        curLiteral = 0;

        for(auto &literal : literals)
            literal = 0;
    };

    auto getOff = [&builder](uint8_t *ptr)
    {
        return ptr - reinterpret_cast<uint8_t *>(builder.getPtr());
    };

    auto oldPtr = builder.getPtr();

    switch(instr.opcode >> 12)
    {

        default:
            printf("unhandled op>>12 in recompile %X\n", instr.opcode >> 12);
            builder.resetPtr(oldPtr);
            loadLiteral(Reg::R12, pc); // -2 to ignore this instr, +2 for prefetch
            builder.bl(getOff(exitPtr));
            outputLiterals();
            return false;
    }

    // output literals if ~close to the limit or this is the last instruction
    // TODO: adjust this? (T2 encoding has x4 range forwards and can index backwards)
    if((instr.flags & Op_Last) || (!ldrLiteralInstrs.empty() && builder.getPtr() - ldrLiteralInstrs[0] > 460))
        outputLiterals();

    return true;
}

void AGBRecompilerThumb::compileEntry()
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codeBuf);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBuf + codeBufSize));

    auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

    builder.push(0b0100111111110000); // R4-11, LR
    
    // set the low bit so we stay in thumb mode
    builder.orr(Reg::R12, Reg::R1, 1);

    // load cpu pointer
    builder.ldr(Reg::R2, 28);
    builder.mov(Reg::R9, Reg::R2);

    builder.mov(Reg::R10, Reg::R0); // cycle count

    // load emu regs
    // the first 8 are never banked
    int regsOff = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtr;
    builder.add(Reg::R2, regsOff); // add to cpu ptr
    builder.mov(Reg::R8, Reg::R2); // store regs ptr
    builder.ldm(0xFF, Reg::R2, false);

    builder.bx(Reg::R12);

    // exit setting the call flag ... and saving LR
    /*exitForCallPtr = reinterpret_cast<uint8_t *>(builder.getPtr());
    builder.mov(Reg::R0, 1);
    builder.ldr(Reg::R2, 16);
    builder.strb(Reg::R0, Reg::R2, 0);

    // exit saving LR
    saveAndExitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    builder.mov(Reg::R0, Reg::LR);
    builder.ldr(Reg::R2, 16);
    builder.str(Reg::R0, Reg::R2, 0);*/

    // exit
    exitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    // save emu regs
    builder.stm(0xFF, Reg::R8, false);

    // store PC
    builder.str(Reg::R12, Reg::R8, 4 * 15);

    // restore regs and return
    builder.pop(0b0000111111110000); // R4-11
    builder.pop(0, true); // PC

    entryFunc = reinterpret_cast<CompiledFunc>(codeBuf + 1/*thumb*/);

    // write cpu addr
    auto ptr = builder.getPtr();
    *ptr++ = 0; // align
    *ptr++ = cpuPtr;
    *ptr++ = cpuPtr >> 16;

    // write addr of exitCallFlag
    auto addr = reinterpret_cast<uintptr_t>(&exitCallFlag);
    *ptr++ = addr;
    *ptr++ = addr >> 16;

    // write addr of tmpSavedPtr
    addr = reinterpret_cast<uintptr_t>(&tmpSavedPtr);
    *ptr++ = addr;
    *ptr++ = addr >> 16;

    curCodePtr = reinterpret_cast<uint8_t *>(ptr);

#ifdef RECOMPILER_DEBUG
    int len = builder.getPtr() - codePtr16;

    //debug
    printf("generated %i halfwords for entry/exit\ncode:", len);

    for(auto p = codeBuf; p !=curCodePtr; p++)
        printf(" %02X", *p);

    printf("\n");
#endif
}