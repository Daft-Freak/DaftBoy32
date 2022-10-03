#include <cstdio>

#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#endif

#include "AGBRecompiler.h"

#include "AGBCPU.h"
#include "AGBMemory.h"

// this is currently linux/x86-64 specific

AGBRecompiler::AGBRecompiler(AGBCPU &cpu) : cpu(cpu)
{
#if defined(__linux__)
    // allocate some memory
    auto pageSize = sysconf(_SC_PAGE_SIZE);
    int numPages = 256;

    // FIXME: alloc RW, switch to RX
    codeBufSize = pageSize * numPages;
    codeBuf = reinterpret_cast<uint8_t *>(mmap(0, codeBufSize, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));

    if(codeBuf == MAP_FAILED)
        perror("failed to allocate code buffer (mmap failed)");
#endif

    curCodePtr = codeBuf;

    for(auto &saved : savedExits)
        saved = {nullptr, 0, 0};
}

int AGBRecompiler::handleBranch(int cyclesToRun)
{
    if(!codeBuf)
        return 0;

    int cyclesExecuted = 0;

    while(true)
    {
        // calculate cycles to run
        int cycles = std::min(cyclesToRun - cyclesExecuted, static_cast<int>(cpu.nextUpdateCycle - cpu.cycleCount));

        if(cycles <= 0)
            break;

        bool isThumb = cpu.cpsr & AGBCPU::Flag_T;

        // TODO
        if(!isThumb)
            break;
   
        uint8_t *codePtr = nullptr;

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

                BlockInfo blockInfo;
                analyseTHUMB(pc, blockInfo);

#ifdef RECOMPILER_DEBUG
                printf("analysed %04X-%04X (%zi instructions)\n", cpuPC, pc, blockInfo.instructions.size());
#endif

                FuncInfo info{};

                info.cpsrMode = cpu.cpsr & 0x1F;

                if(compileTHUMB(ptr, cpuPC, blockInfo))
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
            entryFunc(cycles, codePtr);
        else
            break;

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

        // CPU not running, stop
        if(cpu.halted)
            break;

        // TODO: CPU update stuff
        if(static_cast<int>(cpu.nextUpdateCycle - cpu.cycleCount) <= 0)
        {
            cpu.updateTimers();
            cpu.display.update();

            cpu.calculateNextUpdate(cpu.cycleCount);
        }

        if(cpu.currentInterrupts || cpu.dmaTriggered)
            break;
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

void AGBRecompiler::analyseTHUMB(uint32_t &pc, BlockInfo &blockInfo)
{
    bool done = false;

    auto &mem = cpu.getMem();

    auto startPC = pc;
    auto maxBranch = pc;

    auto updateEnd = [&startPC, &maxBranch](OpInfo &info, uint32_t target)
    {
        maxBranch = std::max(maxBranch, target);
    };

    auto exit = [&pc, &done, &maxBranch](OpInfo &info)
    {
        info.flags |= Op_Exit;

        if(pc > maxBranch)
            done = true;
    };

    auto pcPtr = reinterpret_cast<const uint16_t *>(std::as_const(mem).mapAddress(pc));

    while(!done)
    {
        uint16_t opcode = *pcPtr++;
        pc += 2;

        OpInfo info{};
        info.opcode = opcode;

        switch(opcode >> 12)
        {
            case 0x0: // format 1, move shifted
            case 0x1: // formats 1-2
            {
                auto instOp = (opcode >> 11) & 0x3;
                auto srcReg = (opcode >> 3) & 7;
                auto dstReg = opcode & 7;

                info.regsRead = 1 << srcReg;
                info.regsWritten = 1 << dstReg;

                if(instOp == 3) // format 2, add/sub
                {
                    bool isImm = opcode & (1 << 10);
                    if(!isImm)
                        info.regsRead |= 1 << ((opcode >> 6) & 7);

                    info.flags = Op_WriteN | Op_WriteZ | Op_WriteC | Op_WriteV;
                }
                else // format 1, move shifted register
                {
                    auto offset = (opcode >> 6) & 0x1F;
                    info.flags = Op_WriteN | Op_WriteZ;

                    if(offset || instOp != 0) // LSL 0 (op 0) doesn't write C
                        info.flags |= Op_WriteC;
                }
                break;
            }

            case 0x2: // format 3, mov/cmp immediate
            case 0x3: // format 3, add/sub immediate
            {
                auto instOp = (opcode >> 11) & 0x3;
                auto dstReg = (opcode >> 8) & 7;

                if(instOp == 0) // MOV
                {
                    info.regsWritten = 1 << dstReg;
                    info.flags = Op_WriteN | Op_WriteZ; // N is cleared
                }
                else
                {
                    info.regsRead = 1 << dstReg;

                    if(instOp != 1) // CMP
                        info.regsWritten = 1 << dstReg;

                    info.flags |= Op_WriteN | Op_WriteZ | Op_WriteC | Op_WriteV;
                }
                break;
            }

            case 0x4: // formats 4-6
            {
                if(opcode & (1 << 11)) // format 6, PC-relative load
                {
                    auto dstReg = (opcode >> 8) & 7;

                    info.regsRead = 1 << 15;
                    info.regsWritten = 1 << dstReg;
                    info.flags = Op_Load;
                }
                else if(opcode & (1 << 10)) // format 5, Hi reg/branch exchange
                {
                    auto op = (opcode >> 8) & 3;
                    bool h1 = opcode & (1 << 7);
                    bool h2 = opcode & (1 << 6);

                    auto srcReg = ((opcode >> 3) & 7) + (h2 ? 8 : 0);
                    auto dstReg = (opcode & 7) + (h1 ? 8 : 0);

                    info.regsRead = 1 << srcReg;

                    if(op == 0 || op == 2) // ADD/ MOV
                    {
                        info.regsWritten = 1 << dstReg;

                        if(op == 0) // ADD
                            info.regsRead |= 1 << dstReg;

                        if(dstReg == 15) // writing PC, exit
                            exit(info);
                    }
                    else if(op == 1) // CMP
                    {
                        info.regsRead |= 1 << dstReg;
                        info.flags = Op_WriteN | Op_WriteZ | Op_WriteC | Op_WriteV; 
                    }
                    else if(op == 3) // BX
                        exit(info);
                }
                else // format 4, alu
                {
                    auto instOp = (opcode >> 6) & 0xF;
                    auto srcReg = (opcode >> 3) & 7;
                    auto dstReg = opcode & 7;

                    info.regsRead = 1 << srcReg | 1 << dstReg;
                    info.regsWritten = 1 << dstReg;

                    switch(instOp)
                    {
                        case 0x0: // AND
                        case 0x1: // EOR
                        case 0xC: // ORR
                        case 0xE: // BIC
                            info.flags = Op_WriteN | Op_WriteZ;
                            break;

                        case 0x2: // LSL
                        case 0x3: // LSR
                        case 0x4: // ASR
                        case 0x7: // ROR
                            info.flags = Op_ReadC | Op_WriteN | Op_WriteZ | Op_WriteC; // FIXME: doesn't write C if shift by 0 (ReadC as workaround)
                            break;

                        case 0x5: // ADC
                        case 0x6: // SBC
                            info.flags = Op_ReadC | Op_WriteN | Op_WriteZ | Op_WriteC | Op_WriteV;
                            break;

                        case 0x8: // TST
                            info.regsWritten = 0; // doesn't write
                            info.flags = Op_WriteN | Op_WriteZ;
                            break;

                        case 0x9: // NEG
                            info.regsRead = 1 << srcReg; // dst is not read
                            info.flags = Op_WriteN | Op_WriteZ | Op_WriteC | Op_WriteV;
                            break;

                        case 0xA: // CMP
                        case 0xB: // CMN
                            info.regsWritten = 0; // doesn't write
                            info.flags = Op_WriteN | Op_WriteZ | Op_WriteC | Op_WriteV;
                            break;

                        case 0xD: // MUL
                            info.flags = Op_WriteN | Op_WriteZ | Op_WriteC; // writes C, but value is "meaningless"
                            break;

                        case 0xF: // MVN
                            info.regsRead = 1 << srcReg; // dst is not read
                            info.flags = Op_WriteN | Op_WriteZ;
                            break;
                    }
                }
                break;
            }

            case 0x5: // formats 7-8
            {
                auto offReg = (opcode >> 6) & 7;
                auto baseReg = (opcode >> 3) & 7;
                auto dstReg = opcode & 7;
                
                info.regsRead = (1 << baseReg) | (1 << offReg);

                if(opcode & (1 << 9)) // format 8, load/store sign-extended byte/halfword
                {
                    bool hFlag = opcode & (1 << 11);
                    bool signEx = opcode & (1 << 10);

                    if(signEx || hFlag) // LDRSH/LDRSB or LDRH
                    {
                        info.flags = Op_Load;
                        info.regsWritten = 1 << dstReg;
                    }
                    else // STRH
                    {
                        info.flags = Op_Store;
                        info.regsRead |= 1 << dstReg;
                    }
                }
                else // format 7, load/store with reg offset
                {
                    bool isLoad = opcode & (1 << 11);

                    if(isLoad)
                    {
                        info.flags = Op_Load;
                        info.regsWritten = 1 << dstReg;
                    }
                    else
                    {
                        info.flags = Op_Store;
                        info.regsRead |= 1 << dstReg;
                    }
                }
                break;
            }

            case 0x6: // format 9, load/store with imm offset (words)
            case 0x7: // ... (bytes)
            case 0x8: // format 10, load/store halfword
            {
                bool isLoad = opcode & (1 << 11);
                auto baseReg = (opcode >> 3) & 7;
                auto dstReg = opcode & 7;

                info.regsRead = 1 << baseReg;

                if(isLoad)
                {
                    info.flags = Op_Load;
                    info.regsWritten = 1 << dstReg;
                }
                else
                {
                    info.flags = Op_Store;
                    info.regsRead |= 1 << dstReg;
                }

                break;
            }

            case 0x9: // format 11, SP-relative load/store
            {
                bool isLoad = opcode & (1 << 11);
                auto dstReg = (opcode >> 8) & 7;

                info.regsRead = 1 << 13; // SP

                if(isLoad)
                {
                    info.flags = Op_Load;
                    info.regsWritten = 1 << dstReg;
                }
                else
                {
                    info.flags = Op_Store;
                    info.regsRead |= 1 << dstReg;
                }

                break;
            }

            case 0xA: // format 12, load address
            {
                bool isSP = opcode & (1 << 11);
                auto dstReg = (opcode >> 8) & 7;

                info.regsRead = isSP ? (1 << 13) : (1 << 15);
                info.regsWritten = 1 << dstReg;

                break;
            }

            case 0xB: // formats 13-14
            {
                if(opcode & (1 << 10)) // format 14, push/pop
                {
                    bool isLoad = opcode & (1 << 11);
                    bool pclr = opcode & (1 << 8); // store LR/load PC
                    uint8_t regList = opcode & 0xFF;

                    info.regsRead = info.regsWritten = 1 << 13; // SP

                    if(isLoad)
                    {
                        info.regsWritten |= regList;
                        info.flags = Op_Load;

                        if(pclr)
                            exit(info);
                    }
                    else
                    {
                        info.regsRead |= regList;
                        if(pclr)
                            info.regsRead |= 1 << 14;

                        info.flags = Op_Store;
                    }
                }
                else // format 13, add offset to SP
                    info.regsRead = info.regsWritten = 1 << 13; // SP
                break;
            }

            case 0xC: // format 15, multiple load/store
            {
                bool isLoad = opcode & (1 << 11);
                auto baseReg = (opcode >> 8) & 7;
                uint8_t regList = opcode & 0xFF;

                info.regsRead = info.regsWritten = 1 << baseReg;

                if(isLoad)
                {
                    info.regsWritten |= regList;
                    info.flags = Op_Load;
                }
                else
                {
                    info.regsRead |= regList;
                    info.flags = Op_Store;
                }
                break;
            }

            case 0xD: // formats 16-17, conditional branch + SWI
            {
                auto cond = (opcode >> 8) & 0xF;
                int offset = static_cast<int8_t>(opcode & 0xFF);
                switch(cond & 0xE) // ignore lowest bit
                {
                    case 0x0: // BEQ/BNE
                        info.flags = Op_ReadZ;
                        break;
                    case 0x2: // BCS/BCC
                        info.flags = Op_ReadC;
                        break;
                    case 0x4: // BMI/BPL
                        info.flags = Op_ReadN;
                        break;
                    case 0x6: // BVS/BVC
                        info.flags = Op_ReadV;
                        break;
                    case 0x8: // BHI/BLS
                        info.flags = Op_ReadZ | Op_ReadC;
                        break;
                    case 0xA: // BGE/BLT
                        info.flags = Op_ReadN | Op_ReadV;
                        break;
                    case 0xC: // BGT/BLE
                        info.flags = Op_ReadN | Op_ReadZ | Op_ReadC;
                        break;
                    
                    case 0xE: // undef/SWI
                        info.regsWritten = (1 << 14); // LR
                        info.flags = Op_Exit;
                        break;
                }

                if(cond != 0xF)
                {
                    auto targetAddr = pc + 4 + offset * 2;
                    info.flags |= Op_Branch;
                    updateEnd(info, targetAddr);
                }

                break;
            }

            case 0xE: // format 18, unconditional branch
            {
                info.flags = Op_Branch;
                if(pc > maxBranch)
                    done = true;
                break;
            }

            case 0xF: // format 19, long branch with link
            {
                bool high = opcode & (1 << 11);

                if(!high)
                    info.regsWritten = (1 << 14); // LR
                else
                    info.regsRead = info.regsWritten = (1 << 14); // LR

                info.flags = Op_Exit;
                break;
            }
        }

        blockInfo.instructions.push_back(info);
    }

    auto endPC = pc;

    if(blockInfo.instructions.empty())
        return;

    // cleanup
    pc = startPC;

    auto getBranchTarget = [](uint32_t pc, const OpInfo &instr)
    {
        if((instr.opcode >> 12) == 0xD) // conditional
            return pc + 2 + static_cast<int8_t>(instr.opcode & 0xFF) * 2;
        else // == 0xE, unconditional
            return pc + 2 + (static_cast<int16_t>(instr.opcode << 5) >> 4); // sign extend and * 2
    };

    std::vector<uint8_t> origFlags; // there aren't enough bits in ->flags...
    origFlags.resize(blockInfo.instructions.size());

    for(auto it = blockInfo.instructions.begin(); it != blockInfo.instructions.end(); ++it)
    {
        auto &instr = *it;
        pc += 2;

        if(instr.flags & Op_WriteFlags)
        {
            // find actually used flags
            int read = 0;

            int falseBranchFlags = 0; // flags used by the "other" branch
            bool inBranch = false;

            // save flags
            origFlags[it - blockInfo.instructions.begin()] = instr.flags & Op_WriteFlags;

            // look ahead until we have no flags wrtiten that are not read
            auto nextPC = pc;
            for(auto next = it + 1; next != blockInfo.instructions.end() && (instr.flags & Op_WriteFlags) != read << 4;)
            {
                nextPC += 2;

                // collect read flags
                read |= next->flags & Op_ReadFlags;

                if(next->flags & Op_Exit)
                    break; // give up

                if(next->flags & Op_Branch)
                {
                    // don't go too deep
                    if(inBranch)
                        break;

                    inBranch = true;

                    bool isConditional = next->flags & Op_ReadFlags;

                    // TODO:
                    uint32_t target = getBranchTarget(nextPC, *next);

                    // bad branch, give up
                    if(target < startPC || target >= endPC)
                        break;

                    auto targetInstr = next + (target - nextPC) / 2 + 1;

                    if(!isConditional)
                    {
                        // follow it
                        next = targetInstr;
                        nextPC = target;
                        continue;
                    }
                    else
                    {
                        // follow false branch until we hit another branch
                        falseBranchFlags = instr.flags & Op_WriteFlags;
                        for(auto falseInstr = next + 1; falseInstr != blockInfo.instructions.end() && falseBranchFlags != read << 4; ++falseInstr)
                        {
                            if(falseInstr->flags & (Op_Branch | Op_Exit))
                                break;

                            read |= falseInstr->flags & Op_ReadFlags;
                            falseBranchFlags = (falseBranchFlags & ~falseInstr->flags) | read << 4;
                        }

                        // follow the true branch
                        next = targetInstr;
                        nextPC = target;
                        continue;
                    }
                }

                auto written = next->flags & Op_WriteFlags;
                if(next < it) // backwards jump, restore old flags
                    written = origFlags[next - blockInfo.instructions.begin()];

                // clear overriden flags (keep any that are used)
                instr.flags = (instr.flags & ~(written)) | read << 4 | falseBranchFlags;

                ++next;
            }
        }

        if(instr.flags & Op_Branch)
        {
            uint32_t target = getBranchTarget(pc, instr);

            // not reachable, convert to exit
            if(target < startPC || target >= endPC)
            {
                instr.flags &= ~Op_Branch;
                instr.flags |= Op_Exit;
            }
            else
            {
                // find and mark target
                auto targetInstr = it + (target - pc) / 2 + 1;
                targetInstr->flags |= Op_BranchTarget;
            }
        }
    }

    blockInfo.instructions.back().flags |= Op_Last;
}

void AGBRecompiler::printInfo(BlockInfo &blockInfo)
{
    for(auto &instr : blockInfo.instructions)
    {
        printf("%04X ", instr.opcode);

        if(instr.regsRead || instr.regsWritten)
        {
            // read
            for(int i = 0; i < 16; i ++)
            {
                if(instr.regsRead & (1 << i))
                    printf("R%i ", i);
            }

            if(instr.flags & Op_Load)
                printf("mem ");

            printf("-> ");

            // written
            for(int i = 0; i < 16; i ++)
            {
                if(instr.regsWritten & (1 << i))
                    printf("R%i ", i);
            }

            if(instr.flags & Op_Store)
                printf("mem ");
        }

        // flags
        if(instr.flags & 0xF)
        {
            printf("Fi %s%s%s%s ", 
                    instr.flags & Op_ReadV ? "V" : "",
                    instr.flags & Op_ReadC ? "C" : "",
                    instr.flags & Op_ReadZ ? "Z" : "",
                    instr.flags & Op_ReadN ? "N" : "");
        }

        if(instr.flags & 0xF0)
        {
            printf("Fo %s%s%s%s ", 
                    instr.flags & Op_WriteV ? "V" : "",
                    instr.flags & Op_WriteC ? "C" : "",
                    instr.flags & Op_WriteZ ? "Z" : "",
                    instr.flags & Op_WriteN ? "N" : "");
        }

        if(instr.flags & Op_Branch)
            printf("Br --> ");

        if(instr.flags & Op_Exit)
            printf("Exit ");

        if(instr.flags & Op_BranchTarget)
            printf("<-- Br");

        printf("\n");
    }
}

// wrappers around member funcs
uint8_t AGBRecompiler::readMem8(AGBCPU *cpu, uint32_t addr, int &cycles, bool sequential)
{
    return cpu->readMem8(addr, cycles, sequential);
}

uint16_t AGBRecompiler::readMem16(AGBCPU *cpu, uint32_t addr, int &cycles, bool sequential)
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
    return cyclesToRun; // TODO: update
}

int AGBRecompiler::writeMem16(AGBCPU *cpu, uint32_t addr, uint16_t data, int &cycles, bool sequential, int cyclesToRun)
{
    // TODO: invalidate code
    cpu->writeMem16(addr, data, cycles, sequential);
    return cyclesToRun; // TODO: update
}

int AGBRecompiler::writeMem32(AGBCPU *cpu, uint32_t addr, uint32_t data, int &cycles, bool sequential, int cyclesToRun)
{
    // TODO: invalidate code
    cpu->writeMem32(addr, data, cycles, sequential);
    return cyclesToRun; // TODO: update
}

void AGBRecompiler::updatePCTHUMB(AGBCPU *cpu, uint32_t addr)
{
    cpu->updateTHUMBPC(addr & ~1);
}

void AGBRecompiler::updatePCInterworked(AGBCPU *cpu, uint32_t addr)
{
    if(addr & 1)
    {
        cpu->cpsr |= AGBCPU::Flag_T;
        cpu->updateTHUMBPC(addr & ~1);
    }
    else
    {
        cpu->cpsr &= ~AGBCPU::Flag_T;
        cpu->updateARMPC(addr);
    }
}

// doesn't set LR
void AGBRecompiler::handleSWI(AGBCPU *cpu, uint32_t cpsrCond)
{
    cpu->spsr[1/*svc*/] = (cpu->cpsr & ~0xF0000000) | cpsrCond;

    cpu->cpsr = (cpu->cpsr & ~(0x1F | AGBCPU::Flag_T)) | AGBCPU::Flag_I | 0x13; //supervisor mode

    cpu->modeChanged();
    cpu->updateARMPC(8);
}