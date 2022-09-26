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
        saved = {nullptr, 0};
}

void AGBRecompiler::handleBranch()
{
    if(!codeBuf)
        return;

    while(true)
    {
        // calculate cycles to run
        int cycles = 1; // TODO

        if(cycles <= 0)
            break;

        bool isThumb = cpu.cpsr & AGBCPU::Flag_T;

        // TODO
        if(!isThumb)
            break;
   
        uint8_t *codePtr = nullptr;

        auto cpuPC = cpu.loReg(AGBCPU::Reg::PC) - (isThumb ? 4 : 8);

        // attempt to re-enter previous code
        int savedIndex = curSavedExit - 1;
        for(int i = 0; i < savedExitsSize; i++, savedIndex--)
        {
            // wrap
            if(savedIndex < 0)
                savedIndex += savedExitsSize;

            uint8_t *ptr;
            uint32_t pc;
            std::tie(ptr, pc) = savedExits[savedIndex];

            if(pc == cpuPC && ptr)
            {
                codePtr = ptr;
                curSavedExit = savedIndex;
                savedExits[savedIndex] = {nullptr, 0};
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

                if(compileTHUMB(ptr, cpuPC, blockInfo))
                {
                    info.startPtr = startPtr;
                    info.endPtr = curCodePtr = ptr;
                    info.endPC = pc;
                }
                
                it = compiled.emplace(cpuPC, info).first;
            }
            codePtr = it->second.startPtr;
        }

        auto startCycleCount = cpu.cycleCount;

        // run the code if valid, or stop
        if(codePtr)
            entryFunc(cycles, codePtr);
        else
            break;

        // TODO: cycle count?
        //cpu.cyclesToRun -= (cpu.cycleCount - startCycleCount);

        // code exited with a saved address for re-entry, store PC for later
        if(tmpSavedPtr)
        {
            auto savedPC = cpu.loReg(AGBCPU::Reg::PC) - (isThumb ? 4 : 8);

            // TODO: call exit
            /*if(exitCallFlag)
            {
                // get return address from stack
                auto &mem = cpu.getMem();
                savedPC = mem.read(cpu.sp) | mem.read(cpu.sp + 1) << 8;
            }*/

            savedExits[curSavedExit++] = {tmpSavedPtr, savedPC};
            curSavedExit %= savedExitsSize;

            tmpSavedPtr = nullptr;
            exitCallFlag = false;
        }

        // CPU not running, stop
        if(cpu.halted)
            break;

        // TODO: CPU update stuff
        cpu.updateTimers();
        cpu.display.update();

        if(cpu.currentInterrupts)
            break;
    }
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

    auto pcPtr = std::as_const(mem).mapAddress(pc);

    while(!done)
    {
        uint16_t opcode = *pcPtr++;

        OpInfo info{};
        info.opcode = opcode;

        switch(opcode)
        {
            default:
                printf("invalid op in analysis %04X\n", opcode);
                info.opcode = ~0u;
                done = true;
        }

        if(info.opcode != ~0u)
            blockInfo.instructions.push_back(info);
    }

    auto endPC = pc;

    if(blockInfo.instructions.empty())
        return;

    // cleanup
    pc = startPC;

    // TODO: we end up scanning for branch targets a lot
    auto findBranchTarget = [&blockInfo](uint16_t pc, uint16_t target, std::vector<OpInfo>::iterator it)
    {
        auto searchPC = pc;
        if(target < pc)
        {
            auto prevIt = std::make_reverse_iterator(it + 1);
    
            for(; prevIt != blockInfo.instructions.rend() && searchPC >= target; ++prevIt)
            {
                searchPC -= 2;
                if(searchPC == target)
                    return prevIt.base() - 1;
            }
        }
        else
        {
            for(auto nextIt = it + 1; nextIt != blockInfo.instructions.end() && searchPC <= target; searchPC += 2, ++nextIt)
            {
                if(searchPC == target)
                    return nextIt;
            }
        }

        return blockInfo.instructions.end();
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
                    uint32_t target = 0;
                    //auto targetInstr = findBranchTarget(nextPC, target, next);
                    auto targetInstr = blockInfo.instructions.end();

                    // bad branch, give up
                    if(targetInstr == blockInfo.instructions.end())
                        break;

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
            // TODO: get target
            uint32_t target = startPC - 1;

            // not reachable, convert to exit
            if(target < startPC || target >= endPC)
            {
                instr.flags &= ~Op_Branch;
                instr.flags |= Op_Exit;
            }
            else
            {
                // find and mark target
                auto targetInstr = findBranchTarget(pc, target, it);

                if(targetInstr != blockInfo.instructions.end())
                    targetInstr->flags |= Op_BranchTarget;
                else
                {
                    // failed to find target
                    // may happen if there's a jump over some data
                    instr.flags &= ~Op_Branch;
                    instr.flags |= Op_Exit;
                }
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

// TODO: wrappers around member funcs