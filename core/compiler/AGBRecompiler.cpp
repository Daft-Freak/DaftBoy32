#include <cassert>
#include <cstdio>
#include <utility>

#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#include "AGBRecompiler.h"

#include "AGBCPU.h"
#include "AGBMemory.h"


AGBRecompiler::AGBRecompiler(AGBCPU &cpu) : cpu(cpu)
{
    SourceInfo sourceInfo{};

    auto cpuPtrInt = reinterpret_cast<uintptr_t>(&cpu);

    uint16_t regsOffset = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtrInt;

    sourceInfo.registers.emplace_back(SourceRegInfo{"tmp", 32, SourceRegType::Temp, 0, 0, 0xFFFF});

    // main regs
    sourceInfo.registers.emplace_back(SourceRegInfo{"R0 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R1 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R2 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R3 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R4 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R5 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R6 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R7 ", 32, SourceRegType::General, 0, 0, regsOffset});
    
    // >= R8 may require mapping
    sourceInfo.registers.emplace_back(SourceRegInfo{"R8 ", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R9 ", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R10", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R11", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R12", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R13", 32, SourceRegType::General, 0, 0, 0xFFFF}); // SP
    sourceInfo.registers.emplace_back(SourceRegInfo{"R14", 32, SourceRegType::General, 0, 0, 0xFFFF}); // LR
    // R15 is PC

    regsOffset = reinterpret_cast<uintptr_t>(&cpu.cpsr) - cpuPtrInt;
    sourceInfo.registers.emplace_back(SourceRegInfo{"PSR", 32, SourceRegType::Flags, 0, 0, regsOffset});

    // SPSR?

    // condition flags
    sourceInfo.flags.emplace_back(SourceFlagInfo{'V', 28, SourceFlagType::Overflow});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'C', 29, SourceFlagType::Carry});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'Z', 30, SourceFlagType::Zero});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'N', 31, SourceFlagType::Negative});

    sourceInfo.pcSize = 32;
    sourceInfo.pcPrefetch = 2;
    sourceInfo.pcOffset = reinterpret_cast<uintptr_t>(&cpu.regs[15]) - cpuPtrInt;

    sourceInfo.cycleMul = 1;

    sourceInfo.exitCallFlag = &exitCallFlag;
    sourceInfo.savedExitPtr = &tmpSavedPtr;
    sourceInfo.cycleCount = &cpu.cycleCount;

    //sourceInfo.readMem = reinterpret_cast<uint8_t (*)(void *, uint16_t)>(AGBRecompiler::readMem);
    //sourceInfo.writeMem = reinterpret_cast<int (*)(void *, uint16_t, uint8_t, int)>(AGBRecompiler::writeMem);

    target.init(sourceInfo, &cpu);

#if defined(__linux__)
    // allocate some memory
    auto pageSize = sysconf(_SC_PAGE_SIZE);
    int numPages = 256;

    // FIXME: alloc RW, switch to RX
    codeBufSize = pageSize * numPages;
    codeBuf = reinterpret_cast<uint8_t *>(mmap(0, codeBufSize, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));

    if(codeBuf == MAP_FAILED)
        perror("failed to allocate code buffer (mmap failed)");
#elif defined(_WIN32)
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    auto pageSize = sysInfo.dwPageSize;
    int numPages = 256;

    codeBufSize = pageSize * numPages;
    codeBuf = reinterpret_cast<uint8_t *>(VirtualAlloc(nullptr, codeBufSize, MEM_COMMIT, PAGE_EXECUTE_READWRITE));
#endif

    curCodePtr = codeBuf;

    for(auto &saved : savedExits)
        saved = {nullptr, 0, 0};
}

int AGBRecompiler::run(int cyclesToRun)
{
    if(!codeBuf)
        return 0;

    int cyclesExecuted = 0;

    while(true)
    {
        bool isThumb = cpu.cpsr & AGBCPU::Flag_T;

        // TODO
        if(!isThumb)
            break;

        // calculate cycles to run
        int cycles = std::min(cyclesToRun - cyclesExecuted, static_cast<int>(cpu.nextUpdateCycle - cpu.cycleCount));

        if(cycles <= 0)
            break;

        if(cpu.dmaTriggered)
        {
            int dmaCycles = cpu.updateDMA();
            cpu.cycleCount += dmaCycles;
            cyclesExecuted += dmaCycles;
        }
        else if(!attemptToRun(cycles, cyclesExecuted))
            break;

        // CPU not running, stop
        if(cpu.halted)
            break;

        if(cpu.currentInterrupts)
        {
            assert(cpu.interruptDelay <= cyclesExecuted);

            int intrCycles = cpu.serviceInterrupts();
            cyclesExecuted += intrCycles;
            cpu.cycleCount += intrCycles;

            if(cpu.interruptDelay)
            {
                cpu.interruptDelay = 0;
                cpu.calculateNextUpdate(cpu.cycleCount);
            }
        }

        // CPU update stuff
        if(static_cast<int>(cpu.nextUpdateCycle - cpu.cycleCount) <= 0)
        {
            cpu.updateTimers();
            cpu.display.update();

            cpu.calculateNextUpdate(cpu.cycleCount);
        }
    }

    // if we executed anything, we need to refill the pipeline
    if(cyclesExecuted && (cpu.cpsr & AGBCPU::Flag_T))
    {
        auto pc = cpu.loReg(AGBCPU::Reg::PC) - 2;
        auto thumbPCPtr = reinterpret_cast<const uint16_t *>(cpu.pcPtr + pc);
        cpu.decodeOp = *thumbPCPtr++;
        cpu.fetchOp = *thumbPCPtr;
    }

    return cyclesExecuted;
}

bool AGBRecompiler::attemptToRun(int cyclesToRun, int &cyclesExecuted)
{
    uint8_t *codePtr = nullptr;

    bool isThumb = cpu.cpsr & AGBCPU::Flag_T;
    auto cpuPC = cpu.loReg(AGBCPU::Reg::PC) - (isThumb ? 2 : 4);
    auto blockStartPC = cpuPC;

    // attempt to re-enter previous code
    int savedIndex = curSavedExit - 1;
    for(int i = 0; i < savedExitsSize; i++, savedIndex--)
    {
        // wrap
        if(savedIndex < 0)
            savedIndex += savedExitsSize;

        uint8_t *ptr;
        uint32_t pc;
        uint32_t startPC;
        std::tie(ptr, pc, startPC) = savedExits[savedIndex];

        if(pc == cpuPC && ptr)
        {
            codePtr = ptr;
            curSavedExit = savedIndex;

            cpu.loReg(AGBCPU::Reg::PC) = startPC + (isThumb ? 2 : 4); // compiled code depends on PC pointing to the start of the block
            blockStartPC = startPC;

            savedExits[savedIndex] = {nullptr, 0, 0};
            break;
        }
    }

    if(!codePtr)
    {
        // lookup compiled code
        auto it = compiled.find(cpuPC);

        if(it == compiled.end())
        {
            if(!entryFunc)
                compileEntry();

            // attempt compile
            auto ptr = curCodePtr;
            auto startPtr = ptr;
            auto pc = cpuPC;

            GenBlockInfo genBlock;
            genBlock.flags = 0;

            convertTHUMBToGeneric(pc, genBlock);

            analyseGenBlock(cpuPC, pc, genBlock, target.getSourceInfo());

#ifdef RECOMPILER_DEBUG
            printf("analysed %04X-%04X (%zi instructions)\n", cpuPC, pc, genBlock.instructions.size());
            printGenBlock(cpuPC, genBlock, target.getSourceInfo());
            printf("\n\n");
#endif

            FuncInfo info{};

            info.cpsrMode = cpu.cpsr & 0x1F;

            if(target.compile(ptr, codeBuf + codeBufSize, cpuPC, genBlock))
            {
                info.startPtr = startPtr;
                info.endPtr = curCodePtr = ptr;
                info.endPC = pc;
            }
            
            it = compiled.emplace(cpuPC, info).first;
        }

        // reject code if compiled for a different CPU mode
        if(it->second.cpsrMode == (cpu.cpsr & 0x1F))
            codePtr = it->second.startPtr;
    }

    auto startCycleCount = cpu.cycleCount;

    // run the code if valid, or stop
    if(codePtr)
        entryFunc(cyclesToRun, codePtr);
    else
        return false;

    cyclesExecuted += (cpu.cycleCount - startCycleCount);

    // code exited with a saved address for re-entry, store PC for later
    if(tmpSavedPtr)
    {
        auto savedPC = cpu.loReg(AGBCPU::Reg::PC) - (isThumb ? 2 : 4);

        // get return address
        if(exitCallFlag)
            savedPC = cpu.reg(AGBCPU::Reg::LR) & ~1;

        savedExits[curSavedExit++] = {tmpSavedPtr, savedPC, blockStartPC};
        curSavedExit %= savedExitsSize;

        tmpSavedPtr = nullptr;
        exitCallFlag = false;
    }

    return true;
}


void AGBRecompiler::convertTHUMBToGeneric(uint32_t &pc, GenBlockInfo &genBlock)
{
    bool done = false;

    auto &mem = cpu.getMem();

    auto startPC = pc;
    auto maxBranch = pc;

    // reg 0 is special tmp register
    enum class GenReg
    {
        Temp = 0, // special temp

        R0,
        R1,
        R2,
        R3,
        R4,
        R5,
        R6,
        R7,
        R8,
        R9,
        R10,
        R11,
        R12,
        R13,
        R14,
        // R15 = PC

        CPSR,
    };

    auto lowReg = [](int reg)
    {
        assert(reg < 8);

        return static_cast<GenReg>(reg + 1);
    };

    /*static const GenCondition condMap[]
    {
        GenCondition::NotEqual,
        GenCondition::Equal,
        GenCondition::CarryClear,
        GenCondition::CarrySet
    };

    static const GenCondition invCondMap[]
    {
        GenCondition::Equal,
        GenCondition::NotEqual,
        GenCondition::CarrySet,
        GenCondition::CarryClear
    };*/


    auto updateEnd = [&startPC, &maxBranch](uint32_t target)
    {
        maxBranch = std::max(maxBranch, target);
    };

    auto addInstruction = [&genBlock](GenOpInfo op, uint8_t len = 0, uint16_t flags = 0)
    {
        if(flags & GenOp_Exit)
            assert(op.opcode == GenOpcode::Jump);

        // move branch target flag up to the start of the original instruction
        if((flags & GenOp_BranchTarget) && !genBlock.instructions.empty() && !genBlock.instructions.back().len)
        {
            flags &= ~GenOp_BranchTarget;
            auto rit = genBlock.instructions.rbegin();
            for(;rit != genBlock.instructions.rend(); ++rit)
            {
                if(rit->len) // end of previous instr
                {
                    rit--; // the one after
                    break;
                }
            }

            // must be the first instruction
            if(rit == genBlock.instructions.rend())
                rit--;

            rit->flags |= GenOp_BranchTarget;
        }

        op.len = len;
        op.flags = flags;
        genBlock.instructions.emplace_back(std::move(op));
    };

    // helpers
    auto loadImm = [](uint32_t imm, int cycles = 1)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::LoadImm;
        ret.cycles = cycles;
        ret.imm = imm;

        return ret;
    };

    auto move = [](GenReg src, GenReg dst, int cycles = 1)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Move;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(src);
        ret.dst[0] = static_cast<uint8_t>(dst);

        return ret;
    };

    auto load = [](GenReg addr, GenReg dst, int cycles = 1)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Load;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(addr);
        ret.dst[0] = static_cast<uint8_t>(dst);

        return ret;
    };

    auto store = [](GenReg addr, GenReg data, int cycles = 1)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Store;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(addr);
        ret.src[1] = static_cast<uint8_t>(data);

        return ret;
    };

    auto alu = [](GenOpcode op, GenReg src0, GenReg src1, GenReg dst, int cycles = 1)
    {
        GenOpInfo ret{};
        ret.opcode = op;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(src0);
        ret.src[1] = static_cast<uint8_t>(src1);
        ret.dst[0] = static_cast<uint8_t>(dst);

        return ret;
    };

    auto compare = [](GenReg src0, GenReg src1, int cycles = 1)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Compare;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(src0);
        ret.src[1] = static_cast<uint8_t>(src1);

        return ret;
    };

    auto jump = [](GenCondition cond = GenCondition::Always, GenReg src = GenReg::Temp, int cycles = 2)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Jump;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(cond);
        ret.src[1] = static_cast<uint8_t>(src);

        return ret;
    };

    const int preserveV = AGBCPU::Flag_V >> 28;
    const int preserveC = AGBCPU::Flag_C >> 28;

    const int writeV = AGBCPU::Flag_V >> 24;
    const int writeC = AGBCPU::Flag_C >> 24;
    const int writeZ = AGBCPU::Flag_Z >> 24;
    const int writeN = uint32_t(AGBCPU::Flag_N) >> 24;

    auto pcPtr = reinterpret_cast<const uint16_t *>(std::as_const(mem).mapAddress(pc));

    // we don't handle prefetch here, hmm
    auto pcSCycles = cpu.pcSCycles, pcNCycles = cpu.pcNCycles;

    while(!done)
    {
        uint16_t opcode = *pcPtr++;
        pc += 2;

        switch(opcode >> 12)
        {
            case 0x0: // format 1, move shifted
            case 0x1: // formats 1-2
            {
                auto instOp = (opcode >> 11) & 0x3;
                auto srcReg = lowReg((opcode >> 3) & 7);
                auto dstReg = lowReg(opcode & 7);

                if(instOp == 3) // format 2, add/sub
                {
                    bool isImm = opcode & (1 << 10);
                    bool isSub = opcode & (1 << 9);

                    auto src1Reg = GenReg::Temp;

                    if(isImm)
                        addInstruction(loadImm((opcode >> 6) & 7, 0));
                    else
                        src1Reg = lowReg((opcode >> 6) & 7);

                    if(isSub)
                        addInstruction(alu(GenOpcode::Subtract, srcReg, src1Reg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                    else
                        addInstruction(alu(GenOpcode::Add, srcReg, src1Reg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                }
                else // format 1, move shifted register
                {
                    auto offset = (opcode >> 6) & 0x1F;

                    switch(instOp)
                    {
                        case 0: // LSL
                        {
                            int cFlag = offset == 0 ? preserveC : writeC; // C preserved if shift by 0
                            addInstruction(loadImm(offset, 0));
                            addInstruction(alu(GenOpcode::ShiftLeft, srcReg, GenReg::Temp, dstReg, pcSCycles), 2, preserveV | cFlag | writeZ | writeN);
                            break;
                        }

                        case 1: // LSR
                            addInstruction(loadImm(offset ? offset : 32, 0));
                            addInstruction(alu(GenOpcode::ShiftRightLogic, srcReg, GenReg::Temp, dstReg, pcSCycles), 2, preserveV | writeC | writeZ | writeN);
                            break;

                        case 2: // ASR
                            addInstruction(loadImm(offset ? offset : 32, 0));
                            addInstruction(alu(GenOpcode::ShiftRightArith, srcReg, GenReg::Temp, dstReg, pcSCycles), 2, preserveV | writeC | writeZ | writeN);
                            break;
                    }
                }
                
                break;
            }

            case 0x2: // format 3, mov/cmp immediate
            case 0x3: // format 3, add/sub immediate
            {
                auto instOp = (opcode >> 11) & 0x3;
                auto dstReg = lowReg((opcode >> 8) & 7);

                addInstruction(loadImm(opcode & 0xFF, 0));

                switch(instOp)
                {
                    case 0: // MOV
                        addInstruction(move(GenReg::Temp, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                        break;
                    case 1: // CMP
                        addInstruction(compare(dstReg, GenReg::Temp, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                        break;
                    case 2: // ADD
                        addInstruction(alu(GenOpcode::Add, dstReg, GenReg::Temp, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                        break;
                    case 3: // SUB
                        addInstruction(alu(GenOpcode::Subtract, dstReg, GenReg::Temp, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                        break;
                }

                break;
            }

            case 0x4: // formats 4-6
            {
                if(opcode & (1 << 11)) // format 6, PC-relative load
                {
                    // this is almost certainly going to cause more timing problems
                    auto dstReg = lowReg((opcode >> 8) & 7);
                    uint8_t word = opcode & 0xFF;
                    auto addr = ((pc + 2) & ~2) + (word << 2);

                    int cycles = pcSCycles + 1;

                    addInstruction(loadImm(mem.read<uint32_t>(addr, cycles, false), 0));
                    addInstruction(move(GenReg::Temp, dstReg, cycles), 2);
                }
                else if(opcode & (1 << 10)) // format 5, Hi reg/branch exchange
                {
                    printf("unhandled op in convertToGeneric %04X\n", opcode & 0xFC00);
                    done = true;
                }
                else // format 4, alu
                {
                    auto instOp = (opcode >> 6) & 0xF;
                    auto srcReg = lowReg((opcode >> 3) & 7);
                    auto dstReg = lowReg(opcode & 7);

                    switch(instOp)
                    {
                        case 0x0: // AND
                            addInstruction(alu(GenOpcode::And, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0x1: // EOR
                            addInstruction(alu(GenOpcode::Xor, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        //case 0x2: // LSL
                        //case 0x3: // LSR
                        //case 0x4: // ASR
                            // preserves C if src == 0
                        case 0x5: // ADC
                            addInstruction(alu(GenOpcode::AddWithCarry, dstReg, srcReg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0x6: // SBC
                            addInstruction(alu(GenOpcode::SubtractWithCarry, dstReg, srcReg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        //case 0x7: // ROR
                            // preserves C if src == 0
                        case 0x8: // TST
                            addInstruction(alu(GenOpcode::And, dstReg, srcReg, GenReg::Temp, pcSCycles), 2, preserveV | writeC | writeZ | writeN);
                            break;
                        case 0x9: // NEG
                            addInstruction(loadImm(0, 0));
                            addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, srcReg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xA: // CMP
                            addInstruction(compare(dstReg, srcReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xB: // CMN
                            addInstruction(alu(GenOpcode::Add, dstReg, srcReg, GenReg::Temp, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xC: // ORR
                            addInstruction(alu(GenOpcode::Or, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        //case 0xD:
                            // variable cycles
                        case 0xE: // BIC
                        {
                            GenOpInfo notOp{};
                            notOp.opcode = GenOpcode::Not;
                            notOp.src[0] = static_cast<uint8_t>(srcReg);
                            notOp.dst[0] = static_cast<uint8_t>(GenReg::Temp);
                            addInstruction(notOp);

                            addInstruction(alu(GenOpcode::And, dstReg, GenReg::Temp, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        }
                        case 0xF: // MVN
                        {
                            GenOpInfo notOp{};
                            notOp.opcode = GenOpcode::Not;
                            notOp.cycles = pcSCycles;
                            notOp.src[0] = static_cast<uint8_t>(srcReg);
                            notOp.dst[0] = static_cast<uint8_t>(dstReg);
                            addInstruction(notOp, 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        }

                        default:
                            printf("unhandled alu op in convertToGeneric %x\n", instOp);
                            done = true;
                    }
                    
                }
                break;
            }

            case 0x5: // formats 7-8
            {
                printf("unhandled op in convertToGeneric %04X\n", opcode & 0xFE00);
                done = true;
                break;
            }

            case 0x6: // format 9, load/store with imm offset (words)
            case 0x7: // ... (bytes)
            case 0x8: // format 10, load/store halfword
            case 0x9: // format 11, SP-relative load/store
            case 0xA: // format 12, load address
            {
                printf("unhandled op in convertToGeneric %04X\n", opcode & 0xF800);
                done = true;
                break;
            }

            case 0xB: // formats 13-14
            {
                if(opcode & (1 << 10)) // format 14, push/pop
                {
                    printf("unhandled op in convertToGeneric %04X\n", opcode & 0xFC00);
                    done = true;
                }
                else // format 13, add offset to SP
                {
                    printf("unhandled op in convertToGeneric %04X\n", opcode & 0xF800);
                    done = true;
                }
                break;
            }


            case 0xC: // format 15, multiple load/store
            {
                printf("unhandled op in convertToGeneric %04X\n", opcode & 0xF800);
                done = true;

                break;
            }

            case 0xD: // formats 16-17, conditional branch + SWI
            {
                auto cond = (opcode >> 8) & 0xF;
                int offset = static_cast<int8_t>(opcode & 0xFF);

                if(cond == 15)
                {
                    printf("unhandled SWI in convertToGeneric %04X\n", opcode & 0xF000);
                    done = true;
                }
                else
                {
                    auto genCond = static_cast<GenCondition>(cond); // they happen to match
                    auto addr = pc + 2 + offset * 2;
                    addInstruction(loadImm(addr, pcSCycles));
                    addInstruction(jump(genCond, GenReg::Temp, pcSCycles + pcNCycles), 2);

                    updateEnd(addr);
                }

                break;
            }

            case 0xE: // format 18, unconditional branch
            {
                uint32_t offset = static_cast<int16_t>(opcode << 5) >> 4; // sign extend and * 2
                addInstruction(loadImm(pc + 2 + offset, 0));
                addInstruction(jump(GenCondition::Always, GenReg::Temp, pcSCycles * 2 + pcNCycles), 2);

                if(pc > maxBranch)
                    done = true;
                break;
            }

            case 0xF: // format 19, long branch with link
            {
                printf("unhandled op in convertToGeneric %04X\n", opcode & 0xF000);
                done = true;

                break;
            }

            default:
            {
                printf("invalid op in convertToGeneric %Xxxx\n", opcode >> 12);
                done = true;
            }
        }
    }
}

void AGBRecompiler::compileEntry()
{
    auto entryPtr = target.compileEntry(curCodePtr, codeBufSize);
    entryFunc = reinterpret_cast<CompiledFunc>(entryPtr);
}

// wrappers around member funcs
uint8_t AGBRecompiler::readMem8(AGBCPU *cpu, uint32_t addr, int &cycles, bool sequential)
{
    return cpu->readMem8(addr, cycles, sequential);
}

uint32_t AGBRecompiler::readMem16(AGBCPU *cpu, uint32_t addr, int &cycles, bool sequential)
{
    return cpu->readMem16(addr, cycles, sequential);
}

uint32_t AGBRecompiler::readMem32(AGBCPU *cpu, uint32_t addr, int &cycles, bool sequential)
{
    return cpu->readMem32(addr, cycles, sequential);
}

int AGBRecompiler::writeMem8(AGBCPU *cpu, uint32_t addr, uint8_t data, int &cycles, bool sequential, int cyclesToRun)
{
    // TODO: invalidate code
    cpu->writeMem8(addr, data, cycles, sequential);
    return updateCyclesForWrite(cpu, cyclesToRun);
}

int AGBRecompiler::writeMem16(AGBCPU *cpu, uint32_t addr, uint16_t data, int &cycles, bool sequential, int cyclesToRun)
{
    // TODO: invalidate code
    cpu->writeMem16(addr, data, cycles, sequential);
    return updateCyclesForWrite(cpu, cyclesToRun);
}

int AGBRecompiler::writeMem32(AGBCPU *cpu, uint32_t addr, uint32_t data, int &cycles, bool sequential, int cyclesToRun)
{
    // TODO: invalidate code
    cpu->writeMem32(addr, data, cycles, sequential);
    return updateCyclesForWrite(cpu, cyclesToRun);
}

int AGBRecompiler::updateCyclesForWrite(AGBCPU *cpu, int cyclesToRun)
{
    if(cpu->dmaTriggered || cpu->currentInterrupts)
        return 0;

    return std::min(cyclesToRun, static_cast<int>(cpu->nextUpdateCycle - cpu->cycleCount));
}
