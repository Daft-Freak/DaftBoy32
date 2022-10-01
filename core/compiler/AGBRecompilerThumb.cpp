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

    // cycle counting
    int instrCycles = 0;
    bool hasLoadStore = false;

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

    auto outputLiterals = [this, &builder](bool reachable = true)
    {
        if(!ldrLiteralInstrs.empty())
        {
            auto dataPtr = builder.getPtr();

            if(!reachable)
            {
                // don't need to jump over
                if(reinterpret_cast<uintptr_t>(dataPtr) & 2)
                {
                    // still need to align though
                    builder.data(0);
                    dataPtr++;
                }
            }
            else if(reinterpret_cast<uintptr_t>(dataPtr + 1) & 2)
            {
                // not aligned
                builder.b(curLiteral * 4 + 2);
                builder.data(0);
                dataPtr += 2;
            }
            else // aligned
            {
                builder.b(curLiteral * 4);
                dataPtr++;
            }

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


                // get back the index (we encoded it in the immediate value)
                unsigned int index;

                if(t2)
                    index = (*(ptr + 1) & 0xFFF) >> 2;
                else
                    index = *ptr & 0xFF;

                assert(index < std::size(literals));

                auto off = ((dataPtr + index * 2) - start);
                
                // update LDR imm
                if(t2)
                {
                    off <<= 1;
                    assert(off < 0xFFF);
                    ptr++;
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

    // function call helpers
    auto readMem = [&builder, &instrCycles, &hasLoadStore, &loadLiteral](Reg base, std::variant<Reg, uint32_t> offset, Reg dst, int width, bool sequential = false)
    {
        // R0-3, but not the dest reg
        int regSaveMask = 0b1111 & ~(1 << static_cast<int>(dst));
        builder.push(regSaveMask, false);

        // address
        if(std::holds_alternative<Reg>(offset))
            builder.add(Reg::R1, base, std::get<Reg>(offset));
        else if(base == Reg::PC) // precalculated
            loadLiteral(Reg::R1, std::get<uint32_t>(offset));
        else
        {
            auto offVal = std::get<uint32_t>(offset);
            if(offVal)
                builder.add(Reg::R1, base, offVal);
            else
                builder.mov(Reg::R1, base);
        }

        int spOff = regSaveMask == 0b1111 ? 16 : 12;

        // store initial cycles at first load/store
        if(!hasLoadStore)
        {
            builder.mov(Reg::R0, instrCycles);
            builder.str(Reg::R0, spOff);
            hasLoadStore = true;
        }

        builder.mov(Reg::R0, Reg::R9); // CPU ptr
        builder.add(Reg::R2, Reg::SP, spOff); // cycles out
        builder.mov(Reg::R3, sequential);

        // get func pointer
        // TODO: use aligned if address known and aligned
        uintptr_t funcPtr;
        if(width == 8)
            funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::readMem8);
        else if(width == 16)
            funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::readMem16);
        else
        {
            assert(width == 32);
            funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::readMem32);
        }

        // call
        loadLiteral(Reg::R12, funcPtr);
        builder.blx(Reg::R12);

        // move to dst
        if(dst != Reg::R0)
            builder.mov(dst, Reg::R0);

        builder.pop(regSaveMask, false);
    };

    // output helpers
    auto exit = [&builder](uint8_t *ptr)
    {
        auto off = ptr - reinterpret_cast<uint8_t *>(builder.getPtr());
        builder.bl(off);
    };

    auto flagsIn = [&builder]()
    {
        builder.msr(Reg::R11, 2, 0); // CPSR
    };

    auto passThrough = [&instr, &builder]()
    {
        builder.data(instr.opcode);

        if(instr.flags & Op_WriteFlags)
            builder.mrs(Reg::R11, 0); // CPSR
    };

    auto oldPtr = builder.getPtr();

    auto fail = [&]()
    {
        builder.resetPtr(oldPtr);
        loadLiteral(Reg::R12, pc); // -2 to ignore this instr, +2 for prefetch
        exit(exitPtr);
        outputLiterals(false);
        return false;
    };

    auto pcSCycles = cpu.pcSCycles, pcNCycles = cpu.pcNCycles;

    switch(instr.opcode >> 12)
    {
        case 0x0: // format 1, move shifted
        case 0x1: // formats 1-2
        {
            auto instOp = (instr.opcode >> 11) & 0x3;
            if(instOp != 3 && (instr.flags & Op_WriteFlags))
                flagsIn(); // MOV (shift), preserve V (and C for LSL 0)
            
            passThrough();
            instrCycles = pcSCycles;
            break;
        }

        case 0x2: // format 3, mov/cmp immediate
        case 0x3: // format 3, add/sub immediate
        {
            auto instOp = (instr.opcode >> 11) & 0x3;
            if(instOp == 0 && (instr.flags & Op_WriteFlags))
                flagsIn(); // MOV, preverve C/V

            passThrough();
            instrCycles = pcSCycles;
            break;
        }

        case 0x4: // formats 4-6
        {
            if(instr.opcode & (1 << 11)) // format 6, PC-relative load
            {
                auto dstReg = static_cast<Reg>((instr.opcode >> 8) & 7);
                uint8_t word = instr.opcode & 0xFF;

                auto addr = ((pc + 2) & ~2) + (word << 2);

                instrCycles = pcSCycles + 1;
                
                // TODO: could inline this?
                readMem(Reg::PC, addr, dstReg, 32);
            }
            else if(instr.opcode & (1 << 10)) // format 5, Hi reg/branch exchange
            {
                auto op = (instr.opcode >> 8) & 3;
                bool h1 = instr.opcode & (1 << 7);
                bool h2 = instr.opcode & (1 << 6);

                if(op == 3/*BX*/)
                {
                    printf("unhandled BX in recompile \n");
                    return fail();
                }

                if(h1 || h2)
                {
                    auto srcReg = static_cast<Reg>(((instr.opcode >> 3) & 7) + (h2 ? 8 : 0));
                    auto dstReg = static_cast<Reg>((instr.opcode & 7) + (h1 ? 8 : 0));

                    int dstRegIndex = 0;

                    if(srcReg == Reg::PC || dstReg == Reg::PC)
                    {
                        printf("unhandled format 5 in recompile (PC access)\n");
                        return fail();
                    }

                    // remap regs
                    // (need to load anything > 8)
                    if(h1)
                    {
                        dstRegIndex = static_cast<int>(cpu.mapReg(static_cast<AGBCPU::Reg>(dstReg)));

                        if(op != 2/*MOV*/)
                            builder.ldr(Reg::R12, Reg::R8/*regs*/, dstRegIndex * 4);

                        dstReg = Reg::R12;
                    }

                    if(h2)
                    {
                        int regIndex = static_cast<int>(cpu.mapReg(static_cast<AGBCPU::Reg>(srcReg)));

                        if(op == 2/*MOV*/ && !h1) // ship the mov
                            builder.ldr(dstReg, Reg::R8/*regs*/, regIndex * 4);
                        else
                            builder.ldr(Reg::R14, Reg::R8/*regs*/, regIndex * 4);

                        srcReg = Reg::R14;
                    }

                    // then output ~the same instruction
                    if(op == 0) // ADD
                    {
                        builder.add(dstReg, srcReg);

                        if(h1) // store back
                            builder.str(dstReg, Reg::R8/*regs*/, dstRegIndex * 4);
                    }
                    else if(op == 1) // CMP
                        builder.cmp(dstReg, srcReg);
                    else if(op == 2) // MOV
                    {
                        if(h1) // skip the mov
                            builder.str(srcReg, Reg::R8/*regs*/, dstRegIndex * 4);
                    }

                    if(instr.flags & Op_WriteFlags) // only CMP
                        builder.mrs(Reg::R11, 0); // CPSR
                }
                else
                    passThrough();

                instrCycles = pcSCycles;
            }
            else // format 4, alu
            {
                auto op = (instr.opcode >> 6) & 0xF;
                // most ops here either use or preserve some flags
                // TODO: NEG/CMP/CMN don't
                if((instr.flags & (Op_ReadFlags | Op_WriteFlags)) && (op < 9 || op > 0xB)/*NEG, CMP, CMN*/)
                    flagsIn();

                passThrough();

                if((op >= 2 && op <= 4) /*LSL, LSR, ASR*/ || op == 7/*ROR*/)
                    instrCycles = pcSCycles + 1;
                else if(op == 0xD/*MUL*/)
                    instrCycles = pcSCycles + 1; // TODO: more cycles (dst 0/1 bytes)
                else
                    instrCycles = pcSCycles;
            }

            break;
        }

        case 0x5: // formats 7-8
        {
            auto offReg = static_cast<Reg>((instr.opcode >> 6) & 7);
            auto baseReg = static_cast<Reg>((instr.opcode >> 3) & 7);
            auto dstReg = static_cast<Reg>(instr.opcode & 7);

            if(instr.opcode & (1 << 9)) // format 8, load/store sign-extended byte/halfword
            {
                bool hFlag = instr.opcode & (1 << 11);
                bool signEx = instr.opcode & (1 << 10);
                if(signEx)
                {
                    instrCycles = pcSCycles + 1;
                
                    if(hFlag) // LDRSH... or SB if misaligned
                    {
                        // alignment check
                        builder.add(Reg::R12, baseReg, offReg);

                        builder.tst(Reg::R12, 1);
                        auto branchPtr = builder.getPtr();
                        builder.b(Condition::NE, 0);

                        // aligned load
                        readMem(Reg::R12, 0u, dstReg, 16);
                        builder.sxth(dstReg, dstReg);
                        builder.b(0);

                        if(!builder.getError())
                        {
                            // patch branch
                            int off = (builder.getPtr() - (branchPtr + 1));
                            *branchPtr = (*branchPtr & 0xFF00) | (off - 1);

                            branchPtr = builder.getPtr() - 1;
                        }

                        // unaligned load
                        // TODO: could de-dup a lot between these two
                        readMem(Reg::R12, 0u, dstReg, 8);
                        builder.sxtb(dstReg, dstReg);

                        if(!builder.getError())
                        {
                            // patch branch
                            int off = (builder.getPtr() - (branchPtr + 1));
                            *branchPtr = (*branchPtr & 0xF800) | (off - 1);
                        }
                    }
                    else // LDRSB
                    {
                        readMem(baseReg, offReg, dstReg, 8);
                        builder.sxtb(dstReg, dstReg); // TODO: replace the mov from the call
                    }
                }
                else
                {
                    if(hFlag) // LDRH
                    {
                        instrCycles = pcSCycles + 1;
                        readMem(baseReg, offReg, dstReg, 16);
                    }
                    else
                    {
                        printf("unhandled format 8 in recompile (store)\n");
                        return fail();
                    }
                }
            }
            else // format 7, load/store with reg offset
            {
                bool isLoad = instr.opcode & (1 << 11);
                bool isByte = instr.opcode & (1 << 10);

                if(isLoad)
                {
                    instrCycles = pcSCycles + 1;
                    readMem(baseReg, offReg, dstReg, isByte ? 8 : 32);
                }
                else
                {
                    printf("unhandled format 7 in recompile (store)\n");
                    return fail();
                }
            }
            break;
        }

        case 0x6: // format 9, load/store with imm offset (words)
        case 0x7: // ... (bytes)
        case 0x8: // format 10, load/store halfword
        {
            int width;
            if((instr.opcode >> 12) == 0x6)
                width = 32;
            else if((instr.opcode >> 12) == 0x8)
                width = 16;
            else
                width = 8;

            auto baseReg = static_cast<Reg>((instr.opcode >> 3) & 7);
            auto dstReg = static_cast<Reg>(instr.opcode & 7);
            auto offset = ((instr.opcode >> 6) & 0x1F);

            offset <<= (width / 16);

            if(instr.flags & Op_Load)
            {
                instrCycles = pcSCycles + 1;
                readMem(baseReg, offset, dstReg, width);
            }
            else
            {
                printf("unhandled op>>12 in recompile %X (store)\n", instr.opcode >> 12);
                return fail();
            }
            break;
        }

        case 0x9: // format 11, SP-relative load/store
        {
            auto dstReg = static_cast<Reg>((instr.opcode >> 8) & 7);
            auto offset = (instr.opcode & 0xFF) << 2;

            int regIndex = static_cast<int>(cpu.mapReg(AGBCPU::Reg::SP));
            builder.ldr(Reg::R12, Reg::R8/*regs*/, regIndex * 4);

            if(instr.flags & Op_Load)
            {
                instrCycles = pcSCycles + 1;
                readMem(Reg::R12, offset, dstReg, 32);
            }
            else
            {
                printf("unhandled op>>12 in recompile %X (store)\n", instr.opcode >> 12);
                return fail();
            }
            break;
        }

        default:
            printf("unhandled op>>12 in recompile %X\n", instr.opcode >> 12);
            return fail();
    }

    // cycles
    auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
    auto cycleCountOff = reinterpret_cast<uintptr_t>(&cpu.cycleCount) - cpuPtr;
    assert(cycleCountOff <= 4095);

    builder.ldr(Reg::R12, Reg::R9, cycleCountOff);

    if(hasLoadStore)
    {
        builder.ldr(Reg::R14, Reg::SP, 0);
        builder.add(Reg::R12, Reg::R14);
    }
    else
        builder.add(Reg::R12, Reg::R12, instrCycles);

    builder.str(Reg::R12, Reg::R9, cycleCountOff);

    // TODO: cycles to run check

    // output literals if ~close to the limit or this is the last instruction
    // TODO: adjust this? (T2 encoding has x4 range forwards and can index backwards)
    const int minLiterals = 2; // load can use 2
    if((instr.flags & Op_Last) || (!ldrLiteralInstrs.empty() && builder.getPtr() - ldrLiteralInstrs[0] > 460) || curLiteral + minLiterals > std::size(literals))
        outputLiterals();

    return true;
}

void AGBRecompilerThumb::compileEntry()
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codeBuf);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBuf + codeBufSize));

    auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

    builder.push(0b0100111111110000); // R4-11, LR

    builder.sub(Reg::SP, 4); // make room for cycles temp (for read/write)
    
    // set the low bit so we stay in thumb mode
    builder.orr(Reg::R12, Reg::R1, 1);

    // load cpu pointer
    builder.ldr(Reg::R2, 52);
    builder.mov(Reg::R9, Reg::R2);

    builder.mov(Reg::R10, Reg::R0); // cycle count

    // load CPSR
    int cpsrOff = reinterpret_cast<uintptr_t>(&cpu.cpsr) - cpuPtr;
    builder.ldr(Reg::R11, Reg::R9, cpsrOff);

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

    // update CPSR
    builder.ldr(Reg::R0, Reg::R9, cpsrOff); // load old
    builder.mov(Reg::R1, 0xFF);
    builder.and_(Reg::R0, Reg::R1);

    builder.orr(Reg::R11, Reg::R11, Reg::R0);
    builder.str(Reg::R11, Reg::R9, cpsrOff);

    builder.add(Reg::SP, 4);

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
