#include <cassert>
#include <cstdio>
#include <variant>

#include "AGBRecompilerThumb.h"

#include "AGBCPU.h"
#include "ThumbBuilder.h"

const Reg cycleCountReg = Reg::R8;
const Reg cpuPtrReg = Reg::R9;
const Reg cyclesToRunReg = Reg::R10;
const Reg cpsrReg = Reg::R11;
// R12 used as tmp

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

    lastInstrCycleCheck = nullptr;
    branchTargets.clear();
    forwardBranchesToPatch.clear();

    if(builder.getError())
    {
        ldrLiteralInstrs.clear();
        curLiteral = 0;
        printf("recompile @%08X failed due to error (out of space?)\n", startPC);
        return false;
    }

    if(numInstructions == 0)
    {
#ifdef RECOMPILER_DEBUG
        printf("recompile @%08X failed to handle any instructions\n", startPC);
#endif
        return false;
    }

    auto endPtr = builder.getPtr();

#ifdef RECOMPILER_DEBUG
    int len = endPtr - codePtr16;

    //debug
    printf("recompile @%08X generated %i halfwords (%i instructions)\ncode:", startPC, len, numInstructions);

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
    bool updatedCycleCount = false;

    // literal helpers
    auto loadLiteral = [this, &builder](Reg reg, uint32_t val)
    {
        unsigned int index;
        for(index = 0; index < curLiteral; index++)
        {
            if(literals[index] == val)
                break;
        }

        bool isHigh = static_cast<int>(reg) >= 8;

        // use a MOV if the value will fit... and we would be using a 32bit instruction anyway
        if(isHigh && (builder.isValidModifiedImmediate(val) || val <= 0xFFFF))
        {
            builder.mov(reg, val);
            return;
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

    auto syncCyclesExecuted = [this, &builder, &instrCycles, &hasLoadStore, &updatedCycleCount]()
    {
        if(!instrCycles || updatedCycleCount)
            return;

        if(hasLoadStore)
        {
            builder.ldr(Reg::R14, Reg::SP, 0);
            builder.add(cycleCountReg, Reg::R14);
        }
        else
            builder.add(cycleCountReg, cycleCountReg, instrCycles);

        updatedCycleCount = true;
    };

    auto storeCycleCount = [this, &builder]()
    {
        auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
        auto cycleCountOff = reinterpret_cast<uintptr_t>(&cpu.cycleCount) - cpuPtr;
        assert(cycleCountOff <= 4095);

        builder.str(cycleCountReg, cpuPtrReg, cycleCountOff);
    };

    // function call helpers
    auto readMem = [&builder, &instrCycles, &hasLoadStore, &storeCycleCount, &loadLiteral](Reg base, std::variant<Reg, uint32_t> offset, Reg dst, int width, bool sequential = false, int cyclesVarOff = 0)
    {
        // R0-3, but not the dest reg
        int regSaveMask = 0b1111 & ~(1 << static_cast<int>(dst));
        builder.push(regSaveMask, false);

        storeCycleCount();

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

        int spOff = (regSaveMask == 0b1111 ? 16 : 12) + cyclesVarOff;

        // store initial cycles at first load/store
        if(!hasLoadStore)
        {
            builder.mov(Reg::R0, instrCycles);
            builder.str(Reg::R0, spOff);
            hasLoadStore = true;
        }

        builder.mov(Reg::R0, cpuPtrReg); // CPU ptr
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

    auto writeMem = [&builder, &instrCycles, &hasLoadStore, &storeCycleCount, &loadLiteral](Reg base, std::variant<Reg, uint32_t> offset, Reg data, int width, bool sequential = false, int cyclesVarOff = 0)
    {
        builder.push(0b1111, false); // R0-3
        builder.sub(Reg::SP, 8); // args 5/6

        storeCycleCount();

        // prevent data getting overriden with addr
        if(data == Reg::R1)
        {
            // move directly to R2 if base/offset aren't there
            // TODO: check offset reg
            if(base != Reg::R2 && !std::holds_alternative<Reg>(offset))
            {
                builder.mov(Reg::R2, data);
                data = Reg::R2;
            }
            else
            {
                builder.mov(Reg::R12, data);
                data = Reg::R12;
            }
        }

        // address
        if(std::holds_alternative<Reg>(offset))
            builder.add(Reg::R1, base, std::get<Reg>(offset));
        else
        {
            auto offVal = std::get<uint32_t>(offset);
            if(offVal)
                builder.add(Reg::R1, base, offVal);
            else
                builder.mov(Reg::R1, base);
        }

        // data
        if(data != Reg::R2)
            builder.mov(Reg::R2, data);

        int cyclesSPOff = 24 + cyclesVarOff;

        // store initial cycles at first load/store
        if(!hasLoadStore)
        {
            builder.mov(Reg::R0, instrCycles);
            builder.str(Reg::R0, cyclesSPOff);
            hasLoadStore = true;
        }

        builder.mov(Reg::R0, sequential);
        builder.str(Reg::R0, 0);

        builder.str(cyclesToRunReg, Reg::SP, 4); // cycle count in
        builder.mov(Reg::R0, cpuPtrReg); // CPU ptr
        builder.add(Reg::R3, Reg::SP, cyclesSPOff); // cycles out

        // get func pointer
        uintptr_t funcPtr;
        if(width == 8)
            funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::writeMem8);
        else if(width == 16)
            funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::writeMem16);
        else
        {
            assert(width == 32);
            funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::writeMem32);
        }

        // call
        loadLiteral(Reg::R12, funcPtr);
        builder.blx(Reg::R12);

        builder.mov(cyclesToRunReg, Reg::R0); // new cycle count

        builder.add(Reg::SP, 8);
        builder.pop(0b1111, false); // R0-3
    };

    auto writePC = [&builder, &instrCycles, &hasLoadStore, &loadLiteral](std::variant<Reg, uint32_t> addr, bool interworked = false)
    {
        builder.push(0b1111, false); // R0-3

        // address
        if(std::holds_alternative<Reg>(addr))
            builder.mov(Reg::R1, std::get<Reg>(addr));
        else
            loadLiteral(Reg::R1, std::get<uint32_t>(addr));

        builder.mov(Reg::R0, cpuPtrReg); // CPU ptr

        // get func pointer
        uintptr_t funcPtr;
        if(interworked)
            funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::updatePCInterworked);
        else
            funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::updatePCTHUMB);

        // call
        loadLiteral(Reg::R12, funcPtr);
        builder.blx(Reg::R12);

        builder.pop(0b1111, false); // R0-3
    };

    // output helpers
    auto exit = [&builder](uint8_t *ptr)
    {
        auto off = ptr - reinterpret_cast<uint8_t *>(builder.getPtr());
        builder.bl(off);
    };

    auto flagsIn = [&builder]()
    {
        builder.msr(cpsrReg, 2, 0); // CPSR
    };

    auto passThrough = [&instr, &builder]()
    {
        builder.data(instr.opcode);

        if(instr.flags & Op_WriteFlags)
            builder.mrs(cpsrReg, 0); // CPSR
    };

    auto loadHiReg = [this, &builder](Reg dst, Reg r)
    {
        int regIndex = static_cast<int>(cpu.mapReg(static_cast<AGBCPU::Reg>(r)));
        int regsOff = reinterpret_cast<uintptr_t>(&cpu.regs) - reinterpret_cast<uintptr_t>(&cpu);
        builder.ldr(dst, cpuPtrReg, regsOff + regIndex * 4);
        return dst;
    };

    auto storeHiReg = [this, &builder](Reg src, Reg r)
    {
        int regIndex = static_cast<int>(cpu.mapReg(static_cast<AGBCPU::Reg>(r)));
        int regsOff = reinterpret_cast<uintptr_t>(&cpu.regs) - reinterpret_cast<uintptr_t>(&cpu);
        builder.str(src, cpuPtrReg, regsOff + regIndex * 4);
    };

    auto oldPtr = builder.getPtr();

    auto fail = [&]()
    {
        builder.resetPtr(oldPtr);

        auto pcOff = pc - cpu.loReg(AGBCPU::Reg::PC); // -2 to ignore this instr, +2 for prefetch
        loadLiteral(Reg::R12, pcOff);
        exit(exitPtr);

        outputLiterals(false);
        return false;
    };

    // handle branch targets
    if(instr.flags & Op_BranchTarget)
    {
        // store for backwards jumps
        if(lastInstrCycleCheck)
            branchTargets.emplace(pc - 2, lastInstrCycleCheck); // after adjusting cycle count but before the jump
        else
        {
            // this is the first instruction, so make a cycle check for the branch to go to
            builder.b(4 + 4 + 2);
            lastInstrCycleCheck = reinterpret_cast<uint8_t *>(builder.getPtr());

            // if <= 0 exit
            builder.b(Condition::GT, 4 + 4);

            auto pcOff = pc - cpu.loReg(AGBCPU::Reg::PC);
            loadLiteral(Reg::R12, pcOff);
            exit(saveAndExitPtr);

            branchTargets.emplace(pc - 2, lastInstrCycleCheck);
        }

        // patch forwards jumps
        // can't hit this for the first instruction, so the lastInstrCycleCheck will be valid
        auto jumps = forwardBranchesToPatch.equal_range(pc - 2);
        for(auto it = jumps.first; it != jumps.second; ++it)
        {
            auto off = lastInstrCycleCheck - (it->second + 2);

            auto ptr = reinterpret_cast<uint16_t *>(it->second);

            // patch in new branch
            builder.patch(ptr, ptr + 2);
            builder.b(off);
            builder.endPatch();
        }
        forwardBranchesToPatch.erase(jumps.first, jumps.second);
    }

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
                    auto srcReg = static_cast<Reg>(((instr.opcode >> 3) & 7) + (h2 ? 8 : 0));

                    if(srcReg == Reg::PC)
                        writePC(pc + 2, true); // hmm, always ARM switch
                    else
                    {
                        if(h2)
                            srcReg = loadHiReg(Reg::R12, srcReg);

                        writePC(srcReg, true);
                    }

                    instrCycles = pcSCycles * 2 + pcNCycles;
                    syncCyclesExecuted();

                    exit(exitNoPCPtr);
                    break;
                }

                if(h1 || h2)
                {
                    auto srcReg = static_cast<Reg>(((instr.opcode >> 3) & 7) + (h2 ? 8 : 0));
                    auto dstReg = static_cast<Reg>((instr.opcode & 7) + (h1 ? 8 : 0));

                    auto origDstReg = dstReg;

                    // remap regs
                    // (need to load anything > 8)
                    if(dstReg == Reg::PC)
                    {
                        if(op != 2)
                        {
                            loadLiteral(Reg::R12, pc + 2);
                            dstReg = Reg::R12;
                        }
                    }
                    else if(h1)
                    {
                        if(op != 2/*MOV*/)
                            dstReg = loadHiReg(Reg::R12, dstReg);
                    }

                    if(srcReg == Reg::PC)
                    {
                        if(op == 2/*MOV*/ && !h1)
                            loadLiteral(dstReg, pc + 2);
                        else
                            loadLiteral(Reg::R14, pc + 2);

                        srcReg = Reg::R14;
                    }
                    else if(h2)
                    {
                        if(op == 2/*MOV*/ && !h1) // ship the mov
                            loadHiReg(dstReg, srcReg);
                        else
                            srcReg = loadHiReg(Reg::R14, srcReg);
                    }

                    // then output ~the same instruction
                    if(op == 0) // ADD
                    {
                        builder.add(dstReg, srcReg);

                        if(origDstReg == Reg::PC)
                            writePC(dstReg);
                        else if(h1) // store back
                            storeHiReg(dstReg, origDstReg);
                    }
                    else if(op == 1) // CMP
                        builder.cmp(dstReg, srcReg);
                    else if(op == 2) // MOV
                    {
                        if(dstReg == Reg::PC)
                            writePC(srcReg);
                        else if(h1) // skip the mov
                            storeHiReg(srcReg, origDstReg);
                    }

                    if(origDstReg == Reg::PC)
                    {
                        instrCycles = pcSCycles * 2 + pcNCycles;
                        syncCyclesExecuted();

                        exit(exitNoPCPtr);
                        break;
                    }

                    if(instr.flags & Op_WriteFlags) // only CMP
                        builder.mrs(cpsrReg, 0); // CPSR
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
                        instrCycles = pcNCycles;
                        writeMem(baseReg, offReg, dstReg, 16);
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
                    instrCycles = pcNCycles;
                    writeMem(baseReg, offReg, dstReg, isByte ? 8 : 32);
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
                instrCycles = pcNCycles;
                writeMem(baseReg, offset, dstReg, width);
            }
            break;
        }

        case 0x9: // format 11, SP-relative load/store
        {
            auto dstReg = static_cast<Reg>((instr.opcode >> 8) & 7);
            auto offset = (instr.opcode & 0xFF) << 2;

            loadHiReg(Reg::R12, Reg::SP);

            if(instr.flags & Op_Load)
            {
                instrCycles = pcSCycles + 1;
                readMem(Reg::R12, offset, dstReg, 32);
            }
            else
            {
                instrCycles = pcNCycles;
                writeMem(Reg::R12, offset, dstReg, 32);
            }
            break;
        }

        case 0xA: // format 12, load address
        {
            bool isSP = instr.opcode & (1 << 11);
            auto dstReg = static_cast<Reg>((instr.opcode >> 8) & 7);
            auto word = (instr.opcode & 0xFF) << 2;

            if(isSP)
            {
                loadHiReg(Reg::R12, Reg::SP);
                builder.add(dstReg, Reg::R12, word);
            }
            else
                loadLiteral(dstReg, ((pc + 2) & ~2) + word);

            instrCycles = pcSCycles;
            break;
        }

        case 0xB: // formats 13-14
        {
            if(instr.opcode & (1 << 10)) // format 14, push/pop
            {
                bool isLoad = instr.opcode & (1 << 11);
                bool pclr = instr.opcode & (1 << 8); // store LR/load PC
                uint8_t regList = instr.opcode & 0xFF;

                // TODO: output here is nasty

                if(isLoad) // POP
                {
                    instrCycles = pcSCycles + 1;

                    uint32_t offset = 0;

                    for(int i = 0; i < 8; i++)
                    {
                        if(regList & (1 << i))
                        {
                            auto reg = static_cast<Reg>(i);
                            loadHiReg(Reg::R12, Reg::SP);
                            builder.bic(Reg::R12, Reg::R12, 3);
                            readMem(Reg::R12, offset, reg, 32, true); // first N?
                            offset += 4;
                        }
                    }

                    if(pclr)
                    {
                        loadHiReg(Reg::R12, Reg::SP);
                        builder.bic(Reg::R12, Reg::R12, 3);
                        readMem(Reg::R12, offset, Reg::R12, 32, true);
                        writePC(Reg::R12);
                        offset += 4;

                        // TODO: cycles for branch (not implemented in CPU either)
                    }

                    // update SP
                    loadHiReg(Reg::R12, Reg::SP);
                    builder.add(Reg::R12, Reg::R12, offset);
                    storeHiReg(Reg::R12, Reg::SP);

                    // exit if we set PC
                    if(pclr)
                    {
                        syncCyclesExecuted();
                        exit(exitNoPCPtr);
                    }
                }
                else // PUSH
                {
                    instrCycles = pcNCycles;

                    uint32_t offset = 0;

                    if(pclr)
                        offset += 4;

                    for(int i = 0; i < 8; i++)
                    {
                        if(regList & (1 << i))
                            offset += 4;
                    }

                    // update SP
                    // TODO: should be done last but...
                    loadHiReg(Reg::R12, Reg::SP);
                    builder.sub(Reg::R12, Reg::R12, offset);
                    storeHiReg(Reg::R12, Reg::SP);

                    offset = 0;

                    for(int i = 0; i < 8; i++)
                    {
                        if(regList & (1 << i))
                        {
                            auto reg = static_cast<Reg>(i);
                            loadHiReg(Reg::R12, Reg::SP);
                            builder.bic(Reg::R12, Reg::R12, 3);
                            writeMem(Reg::R12, offset, reg, 32, true); // first N?
                            offset += 4;
                        }
                    }

                    if(pclr) // store LR
                    {
                        loadHiReg(Reg::R12, Reg::SP);
                        builder.bic(Reg::R12, Reg::R12, 3);

                        loadHiReg(Reg::R14, Reg::LR);
 
                        writeMem(Reg::R12, offset, Reg::R14, 32, true);
                    }
                }
            }
            else // format 13, add offset to SP
            {
                bool isNeg = instr.opcode & (1 << 7);
                int off = (instr.opcode & 0x7F) << 2;

                loadHiReg(Reg::R12, Reg::SP);

                if(isNeg)
                    builder.sub(Reg::R12, Reg::R12, off);
                else
                    builder.add(Reg::R12, Reg::R12, off);

                storeHiReg(Reg::R12, Reg::SP);

                instrCycles = pcSCycles;
            }
            break;
        }

        case 0xC: // format 15, multiple load/store
        {
            bool isLoad = instr.opcode & (1 << 11);
            auto baseReg = static_cast<Reg>((instr.opcode >> 8) & 7);
            uint8_t regList = instr.opcode & 0xFF;

            if(isLoad)
                instrCycles = pcSCycles + 1;
            else
                instrCycles = pcNCycles;

            if(!regList) // empty list load/stores PC (bug)
            {
                builder.mov(Reg::R12, baseReg);
                builder.bic(Reg::R12, Reg::R12, 3);

                if(isLoad)
                {
                    readMem(Reg::R12, 0u, Reg::R12, 32);
                    writePC(Reg::R12);

                    builder.add(baseReg, 0x40);

                    // TODO: cycles for branch (not implemented in CPU either)

                    syncCyclesExecuted();
                    exit(exitNoPCPtr);
                }
                else
                {
                    loadLiteral(Reg::R14, pc + 4);
                    writeMem(Reg::R12, 0u, Reg::R14, 32);
                    builder.add(baseReg, 0x40);
                }
            }
            else
            {
                // TODO: output here could also be improved

                int endOff = 0;
                for(int i = 0; i < 8; i++)
                {
                    if(regList & (1 << i))
                        endOff += 4;
                }

                bool first = true, seq = false;
                uint32_t offset = 0;

                // prevent overriding base for loads
                // "A LDM will always overwrite the updated base if the base is in the list."
                if(isLoad && (regList & (1 << static_cast<int>(baseReg))))
                    first = false;

                builder.push(1 << 8); // need somewhere to store the address

                builder.mov(Reg::R8, baseReg);
                builder.bic(Reg::R8, Reg::R8, 3);

                for(int i = 0; i < 8; i++)
                {
                    if(!(regList & (1 << i)))
                        continue;

                    auto reg = static_cast<Reg>(i);

                    if(isLoad)
                        readMem(Reg::R8, offset, reg, 32, seq, 4);
                    else
                        writeMem(Reg::R8, offset, reg, 32, seq, 4);

                    if(first) // write-back
                        builder.add(baseReg, endOff);

                    first = false;
                    seq = true;
                    offset += 4;
                }

                builder.pop(1 << 8);
            }
            break;
        }

        case 0xD:
        {
            auto cond = (instr.opcode >> 8) & 0xF;

            if(cond == 0xF) // format 17, SWI
            {
                // call helper func
                builder.push(0b1111, false); // R0-3

                builder.mov(Reg::R0, cpuPtrReg); // CPU ptr
                builder.mov(Reg::R1, cpsrReg); // CPSR

                auto funcPtr = reinterpret_cast<uintptr_t>(AGBRecompiler::handleSWI);
                loadLiteral(Reg::R12, funcPtr);
                builder.blx(Reg::R12);

                builder.pop(0b1111, false); // R0-3

                // set LR
                loadLiteral(Reg::R12, pc);

                // not using storeReg, need to write reg from another bank
                int regIndex = static_cast<int>(AGBCPU::Reg::R14_svc);
                int regsOff = reinterpret_cast<uintptr_t>(&cpu.regs) - reinterpret_cast<uintptr_t>(&cpu);
                builder.str(Reg::R12, cpuPtrReg, regsOff + regIndex * 4);

                // exit
                instrCycles = pcSCycles * 2 + pcNCycles;
                syncCyclesExecuted();
                exit(exitNoPCPtr);
            }
            else // format 16, conditional branch
            {
                int offset = static_cast<int8_t>(instr.opcode & 0xFF);

                uint16_t *branchPtr = nullptr;

                switch(cond)
                {
                    case 0x0: // BEQ
                    case 0x1: // BNE
                    case 0x2: // BCS
                    case 0x3: // BCC
                    case 0x4: // BMI
                    case 0x5: // BPL
                    case 0x6: // BVS
                    case 0x7: // BVC
                        builder.tst(cpsrReg, (instr.flags & Op_ReadFlags) << 28);

                        branchPtr = builder.getPtr();
                        builder.b(cond & 1 ? Condition::NE : Condition::EQ, 0);
                        break;

                    case 0x8: // BHI
                    case 0x9: // BLS
                        // C & !Z
                        builder.bic(Reg::R12, cpsrReg, cpsrReg, false, ShiftType::LSR, 1); // cpsr & ~(cpsr >> 1) (Z is the bit above C)
                        builder.tst(Reg::R12, AGBCPU::Flag_C);

                        branchPtr = builder.getPtr();
                        builder.b(cond & 1 ? Condition::NE : Condition::EQ, 0);
                        break;

                    case 0xA: // BGE
                    case 0xB: // BLT
                        // N != V (inverted)
                        builder.eor(Reg::R12, cpsrReg, cpsrReg, false, ShiftType::LSR, 3); // cpsr ^ (cpsr >> 3) (N is 3 bits above V)
                        builder.tst(Reg::R12, AGBCPU::Flag_V);

                        branchPtr = builder.getPtr();
                        builder.b(cond & 1 ? Condition::EQ : Condition::NE, 0);
                        break;

                    case 0xC: // BGT
                    case 0xD: // BLE
                        // N != V || Z (inverted)
                        builder.eor(Reg::R12, cpsrReg , cpsrReg, false, ShiftType::LSR, 3); // cpsr ^ (cpsr >> 3) (N is 3 bits above V)
                        builder.orr(Reg::R12, Reg::R12, cpsrReg, false, ShiftType::LSR, 2); // ...  | (cpsr >> 2) (Z is 2 bits above V)
                        builder.tst(Reg::R12, AGBCPU::Flag_V);

                        branchPtr = builder.getPtr();
                        builder.b(cond & 1 ? Condition::EQ : Condition::NE, 0);
                        break;

                    default:
                        printf("invalid format 16 in recompile (cond %i)\n", cond);
                        return fail();
                }

                instrCycles = pcSCycles * 2 + pcNCycles;
                syncCyclesExecuted();

                if(instr.flags & Op_Branch)
                {
                    builder.sub(cyclesToRunReg, cyclesToRunReg, instrCycles);

                    auto addr = pc + 2 + offset * 2;
                    auto it = branchTargets.find(addr);

                    if(it != branchTargets.end())
                    {
                        int off = it->second - reinterpret_cast<uint8_t *>(builder.getPtr());
                        builder.b(off);
                    }
                    else
                    {
                        forwardBranchesToPatch.emplace(addr, reinterpret_cast<uint8_t *>(builder.getPtr()));
                        // leave some space for the branch
                        builder.data(0xBF00); // NOP
                        builder.data(0xBF00);
                    }
                }
                else
                {
                    writePC(pc + 2 + offset * 2);
                    exit(exitNoPCPtr);
                }

                if(branchPtr && !builder.getError())
                {
                    // update branch
                    int off = (builder.getPtr() - (branchPtr + 1));
                    *branchPtr = (*branchPtr & 0xFF00) | (off - 1);
                }

                // cycles for branch not taken?
                instrCycles = pcSCycles;
                updatedCycleCount = false;
            }

            break;
        }

        case 0xE: // format 18, unconditional branch
        {
            uint32_t offset = static_cast<int16_t>(instr.opcode << 5) >> 4; // sign extend and * 2

            if(instr.flags & Op_Branch)
            {
                auto addr = pc + 2 + offset;
                auto it = branchTargets.find(addr);

                instrCycles = pcSCycles * 2 + pcNCycles;
                syncCyclesExecuted();

                // about to jump over this
                builder.sub(cyclesToRunReg, cyclesToRunReg, instrCycles);
                instrCycles = 0;

                if(it != branchTargets.end())
                {
                    int off = it->second - reinterpret_cast<uint8_t *>(builder.getPtr());
                    builder.b(off);
                }
                else
                {
                    forwardBranchesToPatch.emplace(addr, reinterpret_cast<uint8_t *>(builder.getPtr()));
                    // leave some space for the branch
                    builder.data(0xBF00); // NOP
                    builder.data(0xBF00);
                }
            }
            else
            {
                writePC(pc + 2 + offset);

                instrCycles = pcSCycles * 2 + pcNCycles;
                syncCyclesExecuted();

                exit(exitNoPCPtr);
            }
            break;
        }

        case 0xF: // format 19, long branch with link
        {
            bool high = instr.opcode & (1 << 11);
            uint32_t offset = instr.opcode & 0x7FF;

            // TODO: could optimise if we handle both halves together?
            if(!high)
            {
                offset <<= 12;
                if(offset & (1 << 22))
                    offset |= 0xFF800000; //sign extend

                loadLiteral(Reg::R12, pc + 2 + offset);

                storeHiReg(Reg::R12, Reg::LR);

                instrCycles = pcSCycles;
            }
            else
            {
                // calc PC
                loadHiReg(Reg::R12, Reg::LR);
                builder.add(Reg::R12, Reg::R12, offset << 1);

                // set LR
                loadLiteral(Reg::R14, pc | 1);
                storeHiReg(Reg::R14, Reg::LR);

                // set PC
                writePC(Reg::R12);

                instrCycles = pcNCycles + pcSCycles * 2;
                syncCyclesExecuted();

                exit(exitForCallPtr);
            }

            break;
        }
    }

    // cycles
    syncCyclesExecuted();

    if(!(instr.flags & Op_Last)) // TODO: also safe to omit if there's an unconditional exit
    {
        // cycles -= executed
        
        if(hasLoadStore)
            builder.sub(cyclesToRunReg, cyclesToRunReg, Reg::R14);
        else if(instrCycles)
            builder.sub(cyclesToRunReg, cyclesToRunReg, instrCycles);

        lastInstrCycleCheck = reinterpret_cast<uint8_t *>(builder.getPtr()); // save in case the next instr is a branch target

        // if <= 0 exit
        builder.b(Condition::GT, 4 + 4);

        auto pcOff = pc + 2 - cpu.loReg(AGBCPU::Reg::PC);
        loadLiteral(Reg::R12, pcOff);
        exit(saveAndExitPtr);
    }

    // output literals if ~close to the limit or this is the last instruction
    // TODO: adjust this? (T2 encoding has x4 range forwards and can index backwards)
    const int minLiterals = 3; // load can use 2 + 1 for exit from cycle check
    if((instr.flags & Op_Last) || (!ldrLiteralInstrs.empty() && builder.getPtr() - ldrLiteralInstrs[0] > 400) || curLiteral + minLiterals > std::size(literals))
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
    builder.ldr(Reg::R2, 96);
    builder.mov(cpuPtrReg, Reg::R2);

    builder.mov(cyclesToRunReg, Reg::R0); // cycle count

    // load CPSR
    int cpsrOff = reinterpret_cast<uintptr_t>(&cpu.cpsr) - cpuPtr;
    builder.ldr(cpsrReg, cpuPtrReg, cpsrOff);
    builder.and_(cpsrReg, cpsrReg, 0xF0000000); // only keep the condition flags

    // load cycle count
    auto cycleCountOff = reinterpret_cast<uintptr_t>(&cpu.cycleCount) - cpuPtr;
    assert(cycleCountOff <= 4095);

    builder.ldr(cycleCountReg, cpuPtrReg, cycleCountOff);

    // load emu regs
    // the first 8 are never banked
    int regsOff = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtr;
    builder.add(Reg::R2, regsOff); // add to cpu ptr
    builder.ldm(0xFF, Reg::R2, false);

    builder.bx(Reg::R12);

    // exit setting the call flag ... and saving LR
    exitForCallPtr = reinterpret_cast<uint8_t *>(builder.getPtr());
    builder.mov(Reg::R12, 1);
    builder.ldr(Reg::R10, 68);
    builder.strb(Reg::R12, Reg::R10, 0);
    builder.mov(Reg::R12, 0);

    // exit saving LR
    saveAndExitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    builder.ldr(Reg::R10, 60);
    builder.str(Reg::LR, Reg::R10, 0);

    // exit
    exitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    // update PC
    builder.ldr(Reg::R10, cpuPtrReg, regsOff + 4 * 15);
    builder.add(Reg::R10, Reg::R12);
    builder.str(Reg::R10, cpuPtrReg, regsOff + 4 * 15);

    // skip PC store
    exitNoPCPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    // store cycle count
    builder.str(cycleCountReg, cpuPtrReg, cycleCountOff);

    // save emu regs
    builder.add(Reg::R10, cpuPtrReg, regsOff);
    builder.stm(0xFF, Reg::R10, false);

    // update CPSR
    builder.ldr(Reg::R0, cpuPtrReg, cpsrOff); // load old
    builder.mov(Reg::R1, 0xFF);
    builder.and_(Reg::R0, Reg::R1);

    builder.orr(cpsrReg, cpsrReg, Reg::R0);
    builder.str(cpsrReg, cpuPtrReg, cpsrOff);

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
