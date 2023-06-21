#include <cassert>
#include <cstdio>

#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#include "DMGRecompiler.h"

#include "DMGCPU.h"
#include "DMGMemory.h"

static bool shouldSyncForAddress(uint16_t addr)
{
    // accessing most ram shouldn't cause anything to be updated, so we don't need an accurate cycle count
    return addr >= 0xFF00 && addr < 0xFF80/*HRAM start*/ && addr != 0xFFFF/*IE*/;
}

static bool shouldSyncForRegIndex(uint8_t reg, const GenBlockInfo &block)
{
    // skip sync for stack read/write
    // unless stack could be pointing somewhere silly
    return reg != 5/*SP*/ || (block.flags & GenBlock_DMGSPWrite);
}

DMGRecompiler::DMGRecompiler(DMGCPU &cpu) : cpu(cpu)
{
    SourceInfo sourceInfo;

    auto cpuPtrInt = reinterpret_cast<uintptr_t>(&cpu);

    uint16_t regsOffset = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtrInt;

    sourceInfo.registers.emplace_back(SourceRegInfo{"tmp", 16, SourceRegType::Temp, 0, 0, 0xFFFF});

    // 16 bit
    sourceInfo.registers.emplace_back(SourceRegInfo{"AF ", 16, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 2;
    sourceInfo.registers.emplace_back(SourceRegInfo{"BC ", 16, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 2;
    sourceInfo.registers.emplace_back(SourceRegInfo{"DE ", 16, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 2;
    sourceInfo.registers.emplace_back(SourceRegInfo{"HL ", 16, SourceRegType::General, 0, 0, regsOffset});
    regsOffset = reinterpret_cast<uintptr_t>(&cpu.sp) - cpuPtrInt;
    sourceInfo.registers.emplace_back(SourceRegInfo{"SP ", 16, SourceRegType::General, 0, 0, regsOffset});

    // 8 bit aliases
    sourceInfo.registers.emplace_back(SourceRegInfo{"A  ", 8, SourceRegType::General, 1, 0xFF00, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"F  ", 8, SourceRegType::Flags  , 1, 0x00FF, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"B  ", 8, SourceRegType::General, 2, 0xFF00, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"C  ", 8, SourceRegType::General, 2, 0x00FF, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"D  ", 8, SourceRegType::General, 3, 0xFF00, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"E  ", 8, SourceRegType::General, 3, 0x00FF, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"H  ", 8, SourceRegType::General, 4, 0xFF00, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"L  ", 8, SourceRegType::General, 4, 0x00FF, 0xFFFF});

    // extra temps
    sourceInfo.registers.emplace_back(SourceRegInfo{"t2h", 16, SourceRegType::Temp, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"t2b",  8, SourceRegType::Temp, 14, 0xFF, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"tm3", 16, SourceRegType::Temp, 0, 0, 0xFFFF});

    sourceInfo.flags.emplace_back(SourceFlagInfo{'C', 4, SourceFlagType::Carry});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'H', 5, SourceFlagType::HalfCarry});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'N', 6, SourceFlagType::WasSub});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'Z', 7, SourceFlagType::Zero});

    sourceInfo.pcSize = 16;
    sourceInfo.pcOffset = reinterpret_cast<uintptr_t>(&cpu.pc) - cpuPtrInt;

    sourceInfo.extraCPUOffsets[0] = reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtrInt; // DI, RETI, HALT
    sourceInfo.extraCPUOffsets[1] = reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle) - cpuPtrInt; // EI
    sourceInfo.extraCPUOffsets[2] = reinterpret_cast<uintptr_t>(&cpu.halted) - cpuPtrInt; // HALT
    sourceInfo.extraCPUOffsets[3] = reinterpret_cast<uintptr_t>(&cpu.serviceableInterrupts) - cpuPtrInt; // HALT
    sourceInfo.extraCPUOffsets[4] = reinterpret_cast<uintptr_t>(&cpu.haltBug) - cpuPtrInt; // HALT

    sourceInfo.shouldSyncForAddress = shouldSyncForAddress;
    sourceInfo.shouldSyncForRegIndex = shouldSyncForRegIndex;

    sourceInfo.exitCallFlag = &exitCallFlag;
    sourceInfo.savedExitPtr = &tmpSavedPtr;

    sourceInfo.cycleCount = &cpu.cycleCount;
    sourceInfo.cycleExecuted = reinterpret_cast<void (*)(void *)>(DMGRecompiler::cycleExecuted);
    sourceInfo.readMem = reinterpret_cast<uint8_t (*)(void *, uint16_t)>(DMGRecompiler::readMem);
    sourceInfo.writeMem = reinterpret_cast<int (*)(void *, uint16_t, uint8_t, int)>(DMGRecompiler::writeMem);

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
        saved = {nullptr, 0};
}

void DMGRecompiler::handleBranch()
{
    if(!codeBuf)
        return;

    while(true)
    {
        // calculate cycles to run
        int cycles = std::min(cpu.cyclesToRun, cpu.getDisplay().getCyclesToNextUpdate());

        if(cpu.nextTimerInterrupt)
            cycles = std::min(cycles, static_cast<int>(cpu.nextTimerInterrupt - cpu.cycleCount));

        if(cpu.nextSerialBitCycle)
            cycles = std::min(cycles, static_cast<int>(cpu.nextSerialBitCycle - cpu.cycleCount));

        if(cycles <= 0)
            break;

        // about to hit a bus conflict
        if(cpu.pc < 0xFF00 && (cpu.oamDMADelay || cpu.oamDMACount))
            break;

        // stack pointing to regs is... unusual
        if(cpu.sp >= 0xFF00 && cpu.sp < 0xFF80)
            break;

        uint8_t *codePtr = nullptr;

        auto mappedAddr = cpu.mem.makeBankedAddress(cpu.pc);

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

            if(pc == mappedAddr && ptr)
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
            auto it = compiled.find(mappedAddr);

            if(it == compiled.end())
            {
                if(!entryFunc)
                    compileEntry();

                // attempt compile
                auto ptr = curCodePtr;
                auto startPtr = ptr;
                auto pc = cpu.pc;

                BlockInfo blockInfo;
                gatherBlock(pc, blockInfo);
                analyse(cpu.pc, pc, blockInfo);

#ifdef RECOMPILER_DEBUG
                printf("analysed %04X-%04X (%zi instructions)\n", cpu.pc, pc, blockInfo.instructions.size());
#endif

                FuncInfo info{};

                if(compile(ptr, cpu.pc, blockInfo))
                {
                    info.startPtr = startPtr;
                    info.endPtr = curCodePtr = ptr;
                    info.endPC = cpu.mem.makeBankedAddress(pc);
                }
                
                it = compiled.emplace(mappedAddr, info).first;

                // running code from RAM other than HRAM seems uncommon
                // so track the min address used for code
                if(cpu.pc > 0x8000/*in RAM*/ && cpu.pc < minRAMCode)
                    minRAMCode = cpu.pc;
            }
            codePtr = it->second.startPtr;
        }

        auto startCycleCount = cpu.cycleCount;
        bool inHRAM = cpu.pc >= 0xFF00;

        // run the code if valid, or stop
        if(codePtr)
            entryFunc(cycles, codePtr);
        else
            break;

        // update cyclesToRun if we inlined cycleExecuted and skipped updating it
        if(!inHRAM)
            cpu.cyclesToRun -= (cpu.cycleCount - startCycleCount);

        // code exited with a saved address for re-entry, store PC for later
        if(tmpSavedPtr)
        {
            auto savedPC = cpu.pc;

            if(exitCallFlag)
            {
                // get return address from stack
                auto &mem = cpu.getMem();
                savedPC = mem.read(cpu.sp) | mem.read(cpu.sp + 1) << 8;
            }

            savedExits[curSavedExit++] = {tmpSavedPtr, cpu.getMem().makeBankedAddress(savedPC)};
            curSavedExit %= savedExitsSize;

            tmpSavedPtr = nullptr;
            exitCallFlag = false;
        }

        // CPU not running, stop
        if(cpu.halted || cpu.stopped)
            break;

        if(cpu.gdmaTriggered)
            cpu.doGDMA();

        // we ran something, do the usual CPU update stuff
        // TODO: duplicated from CPU

        // sync timer if needed
        if(cpu.nextTimerInterrupt && (static_cast<int>(cpu.nextTimerInterrupt - cpu.cycleCount) <= 0 || cpu.timerReload))
            cpu.updateTimer();

        if(cpu.nextSerialBitCycle)
            cpu.updateSerial();

        // sync display if interrupts enabled
        cpu.display.updateForInterrupts();

        if(cpu.serviceableInterrupts)
            cpu.serviceInterrupts();

        if(cpu.enableInterruptsNextCycle)
            break; // might happen if we exited right after an EI, make it the interpreter's problem

    }
}

void DMGRecompiler::gatherBlock(uint16_t &pc, BlockInfo &blockInfo)
{
    bool done = false;

    auto &mem = cpu.getMem();

    auto startPC = pc;
    auto maxBranch = pc;

    // opcodes with a 3 bit reg encode in this order
    static const int regMap8[]
    {
        Reg_B,
        Reg_C,
        Reg_D,
        Reg_E,
        Reg_H,
        Reg_L,
        0, // (HL)
        Reg_A
    };

    // order for 16bit regs
    static const int regMap16[]
    {
        Reg_B | Reg_C,
        Reg_D | Reg_E,
        Reg_H | Reg_L,
        Reg_SP
    };

    static const int condFlags[]
    {
        Op_ReadZ, // NZ
        Op_ReadZ, // Z
        Op_ReadC, // NC
        Op_ReadC, // C
    };

    auto updateEnd = [&startPC, &maxBranch](OpInfo &info, uint16_t target)
    {
        maxBranch = std::max(maxBranch, target);
    };

    while(!done)
    {
        uint8_t opcode = mem.read(pc++);

        OpInfo info{};
        info.opcode[0] = opcode;
        info.len = 1;

        switch(opcode)
        {
            case 0x00: // NOP
                break;
            case 0x01: // LD BC,nn
            case 0x11: // LD DE,nn
            case 0x21: // LD HL,nn
            case 0x31: // LD SP,nn
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsWritten = regMap16[opcode >> 4];
                break;
            case 0x02: // LD (BC),A
            case 0x12: // LD (DE),A
                info.regsRead = regMap16[opcode >> 4] | Reg_A;
                info.flags |= Op_Store;
                break;
            case 0x22: // LDI (HL),A
            case 0x32: // LDD (HL),A
                info.regsRead = Reg_H | Reg_L | Reg_A;
                info.regsWritten = Reg_H | Reg_L; // these inc/dec unlike the other two
                info.flags |= Op_Store;
                break;
            case 0x03: // INC BC
            case 0x0B: // DEC BC
            case 0x13: // INC DE
            case 0x1B: // DEC DE
            case 0x23: // INC HL
            case 0x2B: // DEC HL
            case 0x33: // INC SP
            case 0x3B: // DEC SP
                info.regsRead = info.regsWritten = regMap16[opcode >> 4];
                break;
            case 0x04: // INC B
            case 0x05: // DEC B
            case 0x0C: // INC C
            case 0x0D: // DEC C
            case 0x14: // INC D
            case 0x15: // DEC D
            case 0x1C: // INC E
            case 0x1D: // DEC E
            case 0x24: // INC H
            case 0x25: // DEC H
            case 0x2C: // INC L
            case 0x2D: // DEC L
            case 0x3C: // INC A
            case 0x3D: // DEC A
                info.regsRead = info.regsWritten = regMap8[opcode >> 3];
                info.flags = Op_WriteH | Op_WriteN | Op_WriteZ; // C preserved
                break;
            case 0x34: // INC (HL)
            case 0x35: // DEC (HL)
                info.regsRead = Reg_H | Reg_L;
                info.flags = Op_WriteH | Op_WriteN | Op_WriteZ | Op_Load | Op_Store;
                break;
            case 0x06: // LD B,n
            case 0x0E: // LD C,n
            case 0x16: // LD D,n
            case 0x1E: // LD E,n
            case 0x26: // LD H,n
            case 0x2E: // LD L,n
            case 0x3E: // LD A,n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsWritten = regMap8[opcode >> 3];
                break;
            case 0x36: // LD (HL),n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = Reg_H | Reg_L;
                info.flags = Op_Store;
                break;
            case 0x07: // RLCA
            case 0x0F: // RRCA
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags; // zeroes everything other than C
                break;
            case 0x08: // LD (nn),SP
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsRead = Reg_SP;
                info.flags = Op_Store;
                break;
            case 0x09: // ADD HL,BC
            case 0x19: // ADD HL,DE
            case 0x29: // ADD HL,HL
            case 0x39: // ADD HL,SP
                info.regsRead = regMap16[opcode >> 4] | Reg_H | Reg_L;
                info.regsWritten = Reg_H | Reg_L;
                info.flags = Op_WriteC | Op_WriteH | Op_WriteN; // Z preserved
                break;
            case 0x0A: // LD A,(BC)
            case 0x1A: // LD A,(DE)
                info.regsRead = regMap16[opcode >> 4];
                info.regsWritten = Reg_A;
                info.flags |= Op_Load;
                break;
            case 0x2A: // LDI A,(HL)
            case 0x3A: // LDD A,(HL)
                info.regsRead = Reg_H | Reg_L;
                info.regsWritten = Reg_H | Reg_L | Reg_A; // these inc/dec unlike the other two
                info.flags |= Op_Load;
                break;
            case 0x10: // STOP
                done = true;
                break;
            case 0x17: // RLA
            case 0x1F: // RRA
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_ReadC; // zeroes everything other than C
                break;
            case 0x18: // JR m
            {
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.flags = Op_Branch;

                auto target = pc + static_cast<int8_t>(info.opcode[1]);
                updateEnd(info, target);

                // can't reach past here
                if(maxBranch < pc && target < pc)
                    done = true;

                break;
            }
            case 0x20: // JR NZ,n
            case 0x28: // JR Z,n
            case 0x30: // JR NC,n
            case 0x38: // JR C,n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.flags = Op_Branch | condFlags[(opcode >> 3) & 3];

                updateEnd(info, pc + static_cast<int8_t>(info.opcode[1]));
                break;
            case 0x27: // DAA
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteC | Op_WriteH | Op_WriteZ | Op_ReadC | Op_ReadH | Op_ReadN; 
                break;
            case 0x2F: // CPL
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteH | Op_WriteN;
                break;
            case 0x37: // SCF
                info.flags = Op_WriteC | Op_WriteH | Op_WriteN;
                break;
            case 0x3F: // CCF
                info.flags = Op_WriteC | Op_WriteH | Op_WriteN | Op_ReadC;
                break;
            case 0x40: // LD B,B
            case 0x41: // LD B,C
            case 0x42: // LD B,D
            case 0x43: // LD B,E
            case 0x44: // LD B,H
            case 0x45: // LD B,L
            case 0x47: // LD B,A
            case 0x48: // LD C,B
            case 0x49: // LD C,C
            case 0x4A: // LD C,D
            case 0x4B: // LD C,E
            case 0x4C: // LD C,H
            case 0x4D: // LD C,L
            case 0x4F: // LD C,A
            case 0x50: // LD D,B
            case 0x51: // LD D,C
            case 0x52: // LD D,D
            case 0x53: // LD D,E
            case 0x54: // LD D,H
            case 0x55: // LD D,L
            case 0x57: // LD D,A
            case 0x58: // LD E,B
            case 0x59: // LD E,C
            case 0x5A: // LD E,D
            case 0x5B: // LD E,E
            case 0x5C: // LD E,H
            case 0x5D: // LD E,L
            case 0x5F: // LD E,A
            case 0x60: // LD H,B
            case 0x61: // LD H,C
            case 0x62: // LD H,D
            case 0x63: // LD H,E
            case 0x64: // LD H,H
            case 0x65: // LD H,L
            case 0x67: // LD H,A
            case 0x68: // LD L,B
            case 0x69: // LD L,C
            case 0x6A: // LD L,D
            case 0x6B: // LD L,E
            case 0x6C: // LD L,H
            case 0x6D: // LD L,L
            case 0x6F: // LD L,A
            case 0x78: // LD A,B
            case 0x79: // LD A,C
            case 0x7A: // LD A,D
            case 0x7B: // LD A,E
            case 0x7C: // LD A,H
            case 0x7D: // LD A,L
            case 0x7F: // LD A,A
                info.regsRead = regMap8[opcode & 7];
                info.regsWritten = regMap8[(opcode >> 3) & 7];
                // if these are the same, this is effectively a nop
                break;
            case 0x46: // LD B,(HL)
            case 0x4E: // LD C,(HL)
            case 0x56: // LD D,(HL)
            case 0x5E: // LD E,(HL)
            case 0x66: // LD H,(HL)
            case 0x6E: // LD L,(HL)
            case 0x7E: // LD A,(HL)
                info.regsRead = Reg_H | Reg_L;
                info.regsWritten = regMap8[(opcode >> 3) & 7];
                info.flags = Op_Load;
                break;
            case 0x70: // LD (HL),B
            case 0x71: // LD (HL),C
            case 0x72: // LD (HL),D
            case 0x73: // LD (HL),E
            case 0x74: // LD (HL),H
            case 0x75: // LD (HL),L
            case 0x77: // LD (HL),A
                info.regsRead = Reg_H | Reg_L | regMap8[opcode & 7];
                info.flags = Op_Store;
                break;
            case 0x76: // HALT
                break;
            case 0x80: // ADD B
            case 0x81: // ADD C
            case 0x82: // ADD D
            case 0x83: // ADD E
            case 0x84: // ADD H
            case 0x85: // ADD L
            case 0x87: // ADD A
            case 0x90: // SUB B
            case 0x91: // SUB C
            case 0x92: // SUB D
            case 0x93: // SUB E
            case 0x94: // SUB H
            case 0x95: // SUB L
            case 0x97: // SUB A
            case 0xA0: // AND B
            case 0xA1: // AND C
            case 0xA2: // AND D
            case 0xA3: // AND E
            case 0xA4: // AND H
            case 0xA5: // AND L
            case 0xA7: // AND A
            case 0xA8: // XOR B
            case 0xA9: // XOR C
            case 0xAA: // XOR D
            case 0xAB: // XOR E
            case 0xAC: // XOR H
            case 0xAD: // XOR L
            case 0xAF: // XOR A
            case 0xB0: // OR B
            case 0xB1: // OR C
            case 0xB2: // OR D
            case 0xB3: // OR E
            case 0xB4: // OR H
            case 0xB5: // OR L
            case 0xB7: // OR A
                info.regsRead = regMap8[opcode & 7] | Reg_A;
                info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags; // bit ops mostly write 0
                break;
            case 0x86: // ADD (HL)
            case 0x96: // SUB (HL)
            case 0xA6: // AND (HL)
            case 0xAE: // XOR (HL)
            case 0xB6: // OR (HL)
                info.regsRead = Reg_H | Reg_L | Reg_A;
                info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_Load; // bit ops mostly write 0
                break;
            case 0x88: // ADC B
            case 0x89: // ADC C
            case 0x8A: // ADC D
            case 0x8B: // ADC E
            case 0x8C: // ADC H
            case 0x8D: // ADC L
            case 0x8F: // ADC A
            case 0x98: // SBC B
            case 0x99: // SBC C
            case 0x9A: // SBC D
            case 0x9B: // SBC E
            case 0x9C: // SBC H
            case 0x9D: // SBC L
            case 0x9F: // SBC A
                info.regsRead = regMap8[opcode & 7] | Reg_A;
                info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_ReadC;
                break;
            case 0x8E: // ADC (HL)
            case 0x9E: // SBC (HL)
                info.regsRead = Reg_H | Reg_L | Reg_A;
                info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_ReadC | Op_Load; 
                break;
            case 0xB8: // CP B
            case 0xB9: // CP C
            case 0xBA: // CP D
            case 0xBB: // CP E
            case 0xBC: // CP H
            case 0xBD: // CP L
            case 0xBF: // CP A
                info.regsRead = regMap8[opcode & 7] | Reg_A;
                info.flags = Op_WriteFlags;
                break;
            case 0xBE: // CP (HL)
                info.regsRead = regMap8[opcode & 7] | Reg_A;
                info.flags = Op_WriteFlags | Op_Load;
                break;
            case 0xC0: // RET NZ
            case 0xC8: // RET Z
            case 0xD0: // RET NC
            case 0xD8: // RET C
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_Exit | condFlags[(opcode >> 3) & 3];
                break;
            case 0xC9: // RET
            case 0xD9: // RETI
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_Exit;

                if(pc > maxBranch)
                    done = true;
                break;
            case 0xC1: // POP BC
            case 0xD1: // POP DE
            case 0xE1: // POP HL
                info.regsRead = Reg_SP;
                info.regsWritten = regMap16[(opcode >> 4) & 3] | Reg_SP;
                info.flags = Op_Load;
                break;
            case 0xF1: // POP AF
                // this is the one thing that writes F
                info.regsRead = Reg_SP;
                info.regsWritten = Reg_A | Reg_SP;
                info.flags = Op_Load | Op_WriteFlags; // flags through writing F
                break;
            case 0xC2: // JP NZ,nn
            case 0xCA: // JP Z,nn
            case 0xD2: // JP NC,nn
            case 0xDA: // JP C,nn
            {
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.flags = Op_Branch | condFlags[(opcode >> 3) & 3];
                break;
            }
            case 0xC3: // JP nn
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.flags = Op_Branch;

                if(pc > maxBranch)
                    done = true;
                break;
            case 0xC4: // CALL NZ,nn
            case 0xCC: // CALL Z,nn
            case 0xD4: // CALL NC,nn
            case 0xDC: // CALL C,nn
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                // TODO: re-entry for return
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_Exit | condFlags[(opcode >> 3) & 3];
                break;
            case 0xCD: // CALL nn
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_Exit;
                break;
            case 0xC5: // PUSH BC
            case 0xD5: // PUSH DE
            case 0xE5: // PUSH HL
                info.regsRead = regMap16[(opcode >> 4) & 3] | Reg_SP;
                info.regsWritten = Reg_SP;
                info.flags = Op_Store;
                break;
            case 0xF5: // PUSH AF
                info.regsRead = Reg_A | Reg_SP;
                info.regsWritten = Reg_SP;
                info.flags = Op_Store | Op_ReadC | Op_ReadH | Op_ReadN | Op_ReadZ; // pushes F
                break;
            case 0xC6: // ADD n
            case 0xD6: // SUB n
            case 0xE6: // AND n
            case 0xEE: // XOR n
            case 0xF6: // OR n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags; // bit ops mostly write 0
                break;
            case 0xCE: // ADC n
            case 0xDE: // SBC n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = info.regsWritten = Reg_A;
                info.flags = Op_WriteFlags | Op_ReadC;
                break;
            case 0xFE: // CP n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = Reg_A;
                info.flags = Op_WriteFlags;
                break;
            case 0xC7: // RST 00
            case 0xCF: // RST 08
            case 0xD7: // RST 10
            case 0xDF: // RST 18
            case 0xE7: // RST 20
            case 0xEF: // RST 28
            case 0xF7: // RST 30
            case 0xFF: // RST 38
                info.flags = Op_Exit;

                if(pc > maxBranch)
                    done = true;
                break;
            case 0xCB: // extended ops
            {
                info.opcode[1] = mem.read(pc++);
                info.len = 2;

                // these all have a reg field, all of them read that reg and most write back
                bool writes = opcode < 0x40 || opcode >= 0x80; // not BIT
                
                if((opcode & 7) == 6) // (HL)
                {
                    info.regsRead = Reg_H | Reg_L;
                    info.flags = Op_Load | (writes ? Op_Store : 0);
                }
                else
                {
                    info.regsRead = regMap8[opcode & 7];
                    if(writes)
                        info.regsWritten = info.regsWritten;
                }

                if(info.opcode[1] < 0x10) // RLC, RRC
                    info.flags |= Op_WriteFlags;
                else if(info.opcode[1] < 0x20) // RL, RR
                    info.flags |= Op_WriteFlags | Op_ReadC;
                else if(info.opcode[1] < 0x40) // SLA, SRA, SWAP, SRL
                    info.flags |= Op_WriteFlags;
                else if(info.opcode[1] < 0x80) // BIT
                    info.flags |= Op_WriteH | Op_WriteN | Op_WriteZ;
                // RES/SET don't affect F
                break;
            }
            case 0xE0: // LDH (n),A
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = Reg_A;
                info.flags = Op_Store;
                break;
            case 0xE2: // LDH (C),A
                info.regsRead = Reg_A | Reg_C;
                info.flags = Op_Store;
                break;
            case 0xE8: // ADD SP,n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = info.regsWritten = Reg_SP;
                info.flags = Op_WriteFlags; // Z is always 0, H/C are... unusual
                break;
            case 0xE9: // JP (HL)
                info.regsRead = Reg_H | Reg_L;
                info.flags = Op_Exit;

                if(pc > maxBranch)
                    done = true;
                break;
            case 0xEA: // LD (nn),A
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsRead = Reg_A;
                info.flags = Op_Store;
                break;
            case 0xF0: // LDH A,(n)
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsWritten = Reg_A;
                info.flags = Op_Load;
                break;
            case 0xF2: // LDH A,(C)
                info.regsRead = Reg_C;
                info.regsWritten = Reg_A;
                info.flags = Op_Load;
                break;
            case 0xF3: // DI
            case 0xFB: // EI
                break;
            case 0xF8: // LDHL SP,n
                info.opcode[1] = mem.read(pc++);
                info.len = 2;
                info.regsRead = Reg_SP;
                info.regsWritten = Reg_H | Reg_L;
                info.flags = Op_WriteFlags; // Z is always 0, H/C are... unusual
                break;
            case 0xF9: // LD SP,HL
                info.regsRead = Reg_H | Reg_L;
                info.regsWritten = Reg_SP;
                break;
            case 0xFA: // LD A,(nn)
                info.opcode[1] = mem.read(pc++);
                info.opcode[2] = mem.read(pc++);
                info.len = 3;
                info.regsWritten = Reg_A;
                info.flags = Op_Load;
                break;

            default:
                printf("invalid op in analysis %02X\n", opcode);
                info.len = 0;
                done = true;
        }

        if(info.len)
            blockInfo.instructions.push_back(info);
    }
}

void DMGRecompiler::analyse(uint16_t pc, uint16_t endPC, BlockInfo &blockInfo)
{
    auto startPC = pc;

    // TODO: we end up scanning for branch targets a lot
    auto findBranchTarget = [&blockInfo](uint16_t pc, uint16_t target, std::vector<OpInfo>::iterator it)
    {
        auto searchPC = pc;
        if(target < pc)
        {
            auto prevIt = std::make_reverse_iterator(it + 1);
    
            for(; prevIt != blockInfo.instructions.rend() && searchPC >= target; ++prevIt)
            {
                searchPC -= prevIt->len;
                if(searchPC == target)
                    return prevIt.base() - 1;
            }
        }
        else
        {
            for(auto nextIt = it + 1; nextIt != blockInfo.instructions.end() && searchPC <= target; searchPC += nextIt->len, ++nextIt)
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
        pc += instr.len;

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
                nextPC += next->len;

                // collect read flags
                read |= next->flags & Op_ReadFlags;
                read &= (instr.flags & Op_WriteFlags) >> 4; // can't have a read for a flag we're not setting

                if(next->flags & Op_Exit)
                    break; // give up

                if(next->flags & Op_Branch)
                {
                    // don't go too deep
                    if(inBranch)
                        break;

                    inBranch = true;

                    bool isConditional = next->flags & Op_ReadFlags;

                    uint16_t target;
                    if(next->opcode[0] >= 0xC2) // JP
                        target = next->opcode[1] | next->opcode[2] << 8;
                    else // JR
                        target = nextPC + static_cast<int8_t>(next->opcode[1]);

                    auto targetInstr = findBranchTarget(nextPC, target, next);

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
            // get target (should be JR or JP)
            uint16_t target;
            if(instr.opcode[0] >= 0xC2) // JP
                target = instr.opcode[1] | instr.opcode[2] << 8;
            else
                target = pc + static_cast<int8_t>(instr.opcode[1]);

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

bool DMGRecompiler::convertToGeneric(uint16_t pc, BlockInfo &block, GenBlockInfo &genBlock)
{
    // reg 0 is special tmp register
    // AF, BC, DE, HL, SP

    enum class GenReg
    {
        Temp = 0, // special temp

        AF,
        BC,
        DE,
        HL,
        SP,

        // mapped as aliases of the 16-bit regs
        A,
        F,
        B,
        C,
        D,
        E,
        H,
        L,

        Temp2, // used by ret, ld (nn) sp
        Temp2B, // byte alias (used by inc/dec (hl) and ext ops (HL))
        Temp3, // used by ret
    };

    // mapping from opcodes
    static const GenReg regMap8[]
    {
        GenReg::B,
        GenReg::C,
        GenReg::D,
        GenReg::E,
        GenReg::H,
        GenReg::L,
        GenReg::Temp2B,
        GenReg::A,
    };

    static const GenReg regMap16[]
    {
        GenReg::BC,
        GenReg::DE,
        GenReg::HL,
        GenReg::SP, // unless it's push/pop, then it's AF
    };

    static const GenReg regMap16PushPop[]
    {
        GenReg::BC,
        GenReg::DE,
        GenReg::HL,
        GenReg::AF,
    };

    static const GenCondition condMap[]
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
    };

    auto lowHalf = [](GenReg reg)
    {
        int iReg = static_cast<int>(reg);
        assert(iReg >= int(GenReg::AF) && iReg <= int(GenReg::HL));

        return static_cast<GenReg>((iReg - 1) * 2 + int(GenReg::F));
    };

    auto highHalf = [](GenReg reg)
    {
        int iReg = static_cast<int>(reg);
        assert(iReg >= int(GenReg::AF) && iReg <= int(GenReg::HL));

        return static_cast<GenReg>((iReg - 1) * 2 + int(GenReg::A));
    };

    auto addInstruction = [&genBlock](GenOpInfo op, uint8_t len = 0, uint16_t flags = 0)
    {
        if(flags & Op_Exit)
            assert(op.opcode == GenOpcode::Jump);

        // move branch target flag up to the start of the original instruction
        if((flags & Op_BranchTarget) && !genBlock.instructions.empty() && !genBlock.instructions.back().len)
        {
            flags &= ~Op_BranchTarget;
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

            rit->flags |= Op_BranchTarget;
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

    auto loadImm16 = [](uint8_t *imm, int cycles = 2, int add = 0)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::LoadImm;
        ret.cycles = cycles;
        ret.imm = ((imm[0] | imm[1] << 8) + add) & 0xFFFF;

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
    
    auto alu = [](GenOpcode op, GenReg src, GenReg dst = GenReg::A, int cycles = 1)
    {
        GenOpInfo ret{};
        ret.opcode = op;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(dst);
        ret.src[1] = static_cast<uint8_t>(src);
        ret.dst[0] = ret.src[0];

        return ret;
    };

    auto compare = [](GenReg src)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Compare;
        ret.cycles = 1;
        ret.src[0] = static_cast<uint8_t>(GenReg::A);
        ret.src[1] = static_cast<uint8_t>(src);

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

    auto preserveC = DMGCPU::Flag_C >> 4;
    auto preserveZ = DMGCPU::Flag_Z >> 4;

    bool branchTargetOnNext = false;

    for(auto &instr : block.instructions)
    {
        pc += instr.len;

        auto inFlags = instr.flags;

        // remove unused flags
        inFlags &= ~(Op_ReadFlags | Op_Branch | Op_Load | Op_Store | Op_Last);

        if(branchTargetOnNext)
        {
            // make sure the instruction after a conditional call/ret is a branch target
            inFlags |= Op_BranchTarget;
            branchTargetOnNext = false;
        }

        switch(instr.opcode[0])
        {
            case 0x00: // NOP
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::NOP;
                op.cycles = 1;
                addInstruction(op, instr.len, inFlags);
                break;
            }

            case 0x01: // LD BC,nn
            case 0x11: // LD DE,nn
            case 0x21: // LD HL,nn
            case 0x31: // LD SP,nn
                addInstruction(loadImm16(instr.opcode + 1));
                addInstruction(move(GenReg::Temp, regMap16[instr.opcode[0] >> 4]), instr.len, inFlags);

                // pointing SP at regs is very evil
                if(instr.opcode[0] == 0x31 && instr.opcode[1] < 0x80 && instr.opcode[2] == 0xFF)
                    genBlock.flags |= GenBlock_DMGSPWrite;
                break;

            case 0x02: // LD (BC),A
            case 0x12: // LD (DE),A
                addInstruction(store(regMap16[instr.opcode[0] >> 4], GenReg::A, 2), instr.len, inFlags);
                break;
            
            case 0x03: // INC BC
            case 0x13: // INC DE
            case 0x23: // INC HL
            case 0x33: // INC SP
                addInstruction(loadImm(1));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, regMap16[instr.opcode[0] >> 4]), instr.len, inFlags);
                break;

            case 0x04: // INC B
            case 0x0C: // INC C
            case 0x14: // INC D
            case 0x1C: // INC E
            case 0x24: // INC H
            case 0x2C: // INC L
            case 0x3C: // INC A
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, regMap8[instr.opcode[0] >> 3]), instr.len, inFlags | preserveC);
                break;

            case 0x05: // DEC B
            case 0x0D: // DEC C
            case 0x15: // DEC D
            case 0x1D: // DEC E
            case 0x25: // DEC H
            case 0x2D: // DEC L
            case 0x3D: // DEC A
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, regMap8[instr.opcode[0] >> 3]), instr.len, inFlags | preserveC);
                break;

            case 0x0B: // DEC BC
            case 0x1B: // DEC DE
            case 0x2B: // DEC HL
            case 0x3B: // DEC SP
                addInstruction(loadImm(1));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, regMap16[instr.opcode[0] >> 4]), instr.len, inFlags);
                break;

            case 0x06: // LD B,n
            case 0x0E: // LD C,n
            case 0x16: // LD D,n
            case 0x1E: // LD E,n
            case 0x26: // LD H,n
            case 0x2E: // LD L,n
            case 0x3E: // LD A,n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(move(GenReg::Temp, regMap8[instr.opcode[0] >> 3]), instr.len, inFlags);
                break;

            case 0x07: // RLCA
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::RotateLeft, GenReg::Temp), instr.len, inFlags | GenOp_MagicAlt1);
                break;

            case 0x08: // LD (nn),SP
            {
                // low
                addInstruction(loadImm16(instr.opcode + 1));
                addInstruction(store(GenReg::Temp, GenReg::SP));

                // high
                addInstruction(move(GenReg::SP, GenReg::Temp2, 0));
                addInstruction(loadImm(8, 0));
                addInstruction(alu(GenOpcode::ShiftRightLogic, GenReg::Temp, GenReg::Temp2, 0)); // SP >> 8
                addInstruction(loadImm16(instr.opcode + 1, 0, 1));
                addInstruction(store(GenReg::Temp, GenReg::Temp2, 2), instr.len, inFlags);

                break;
            }

            case 0x09: // ADD HL,BC
            case 0x19: // ADD HL,DE
            case 0x29: // ADD HL,HL
            case 0x39: // ADD HL,SP
                addInstruction(alu(GenOpcode::Add, regMap16[instr.opcode[0] >> 4], GenReg::HL, 2), instr.len, inFlags | preserveZ);
                break;

            case 0x0A: // LD A,(BC)
            case 0x1A: // LD A,(DE)
                addInstruction(load(regMap16[instr.opcode[0] >> 4], GenReg::A, 2), instr.len, inFlags);
                break;

            case 0x0F: // RRCA
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::RotateRight, GenReg::Temp), instr.len, inFlags | GenOp_MagicAlt1);
                break;

            case 0x10: // STOP
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::DMG_Stop;
                op.cycles = 1;
                addInstruction(op, instr.len, inFlags);
                break;
            }

            case 0x17: // RLA
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::RotateLeftCarry, GenReg::Temp), instr.len, inFlags | GenOp_MagicAlt1);
                break;

            case 0x18: // JR m
                addInstruction(loadImm(pc + static_cast<int8_t>(instr.opcode[1])));
                addInstruction(jump(), instr.len, inFlags);
                break;

            case 0x1F: // RRA
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::RotateRightCarry, GenReg::Temp), instr.len, inFlags | GenOp_MagicAlt1);
                break;

            case 0x20: // JR NZ,n
            case 0x28: // JR Z,n
            case 0x30: // JR NC,n
            case 0x38: // JR C,n
            {
                auto cond = condMap[(instr.opcode[0] >> 3) & 3];
                addInstruction(loadImm(pc + static_cast<int8_t>(instr.opcode[1])));
                addInstruction(jump(cond), instr.len, inFlags);
                break;
            }

            case 0x22: // LDI (HL),A
                addInstruction(store(GenReg::HL, GenReg::A));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::HL), instr.len, inFlags);
                break;
            
            case 0x27: // DAA
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::DMG_DAA;
                op.cycles = 1;
                addInstruction(op, instr.len, inFlags);
                break;
            }

            case 0x2A: // LDI A,(HL)
                addInstruction(load(GenReg::HL, GenReg::A));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::HL), instr.len, inFlags);
                break;

            case 0x2F: // CPL
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::Not;
                op.cycles = 1;
                op.src[0] = op.dst[0] = static_cast<uint8_t>(GenReg::A);
                addInstruction(op, instr.len, inFlags | preserveC | preserveZ);
                break;
            }

            case 0x32: // LDD (HL),A
                addInstruction(store(GenReg::HL, GenReg::A));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::HL), instr.len, inFlags);
                break;

            case 0x34: // INC (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp2B));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::Temp2B, 0), 0, inFlags | preserveC);
                addInstruction(store(GenReg::HL, GenReg::Temp2B, 2), instr.len);
                break;

            case 0x35: // DEC (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp2B));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::Temp2B, 0), 0, inFlags | preserveC);
                addInstruction(store(GenReg::HL, GenReg::Temp2B, 2), instr.len);
                break;

            case 0x36: // LD (HL),n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(store(GenReg::HL, GenReg::Temp, 2), instr.len, inFlags);
                break;
    
            case 0x37: // SCF
                addInstruction(loadImm(DMGCPU::Flag_Z, 0));
                addInstruction(alu(GenOpcode::And, GenReg::Temp, GenReg::F, 0)); // preserve Z
                addInstruction(loadImm(DMGCPU::Flag_C, 0));
                addInstruction(alu(GenOpcode::Or, GenReg::Temp, GenReg::F), instr.len, inFlags & ~Op_WriteFlags); // set C
                break;

            case 0x3A: // LDD A,(HL)
                addInstruction(load(GenReg::HL, GenReg::A));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::HL), instr.len, inFlags);
                break;

            case 0x3F: // CCF
                addInstruction(loadImm(DMGCPU::Flag_C | DMGCPU::Flag_Z, 0));
                addInstruction(alu(GenOpcode::And, GenReg::Temp, GenReg::F, 0)); // preserve Z and C
                addInstruction(loadImm(DMGCPU::Flag_C, 0));
                addInstruction(alu(GenOpcode::Xor, GenReg::Temp, GenReg::F), instr.len, inFlags & ~Op_WriteFlags); // flip C
                break;

            case 0x40: // LD B,B
            case 0x41: // LD B,C
            case 0x42: // LD B,D
            case 0x43: // LD B,E
            case 0x44: // LD B,H
            case 0x45: // LD B,L
            case 0x47: // LD B,A
            case 0x48: // LD C,B
            case 0x49: // LD C,C
            case 0x4A: // LD C,D
            case 0x4B: // LD C,E
            case 0x4C: // LD C,H
            case 0x4D: // LD C,L
            case 0x4F: // LD C,A
            case 0x50: // LD D,B
            case 0x51: // LD D,C
            case 0x52: // LD D,D
            case 0x53: // LD D,E
            case 0x54: // LD D,H
            case 0x55: // LD D,L
            case 0x57: // LD D,A
            case 0x58: // LD E,B
            case 0x59: // LD E,C
            case 0x5A: // LD E,D
            case 0x5B: // LD E,E
            case 0x5C: // LD E,H
            case 0x5D: // LD E,L
            case 0x5F: // LD E,A
            case 0x60: // LD H,B
            case 0x61: // LD H,C
            case 0x62: // LD H,D
            case 0x63: // LD H,E
            case 0x64: // LD H,H
            case 0x65: // LD H,L
            case 0x67: // LD H,A
            case 0x68: // LD L,B
            case 0x69: // LD L,C
            case 0x6A: // LD L,D
            case 0x6B: // LD L,E
            case 0x6C: // LD L,H
            case 0x6D: // LD L,L
            case 0x6F: // LD L,A
            case 0x78: // LD A,B
            case 0x79: // LD A,C
            case 0x7A: // LD A,D
            case 0x7B: // LD A,E
            case 0x7C: // LD A,H
            case 0x7D: // LD A,L
            case 0x7F: // LD A,A
                addInstruction(move(regMap8[instr.opcode[0] & 7], regMap8[(instr.opcode[0] >> 3) & 7]), instr.len, inFlags);
                break;

            case 0x46: // LD B,(HL)
            case 0x4E: // LD C,(HL)
            case 0x56: // LD D,(HL)
            case 0x5E: // LD E,(HL)
            case 0x66: // LD H,(HL)
            case 0x6E: // LD L,(HL)
            case 0x7E: // LD A,(HL)
                addInstruction(load(GenReg::HL, regMap8[(instr.opcode[0] >> 3) & 7], 2), instr.len, inFlags);
                break;

            case 0x70: // LD (HL),B
            case 0x71: // LD (HL),C
            case 0x72: // LD (HL),D
            case 0x73: // LD (HL),E
            case 0x74: // LD (HL),H
            case 0x75: // LD (HL),L
            case 0x77: // LD (HL),A
                addInstruction(store(GenReg::HL, regMap8[instr.opcode[0] & 7], 2), instr.len, inFlags);
                break;

            case 0x76: // HALT
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::DMG_Halt;
                op.cycles = 1;
                addInstruction(op, instr.len, inFlags);
                break;
            }

            case 0x80: // ADD A,B
            case 0x81: // ADD A,C
            case 0x82: // ADD A,D
            case 0x83: // ADD A,E
            case 0x84: // ADD A,H
            case 0x85: // ADD A,L
            case 0x87: // ADD A,A
                addInstruction(alu(GenOpcode::Add, regMap8[instr.opcode[0] & 7]), instr.len, inFlags);
                break;
            case 0x86: // ADD (HL)
            {
                // load -> tmp
                addInstruction(load(GenReg::HL, GenReg::Temp));

                // add dst + tmp -> dst
                addInstruction(alu(GenOpcode::Add, GenReg::Temp), instr.len, inFlags);
                break;
            }

            case 0x88: // ADC A,B
            case 0x89: // ADC A,C
            case 0x8A: // ADC A,D
            case 0x8B: // ADC A,E
            case 0x8C: // ADC A,F
            case 0x8D: // ADC A,H
            case 0x8F: // ADC A,A
                addInstruction(alu(GenOpcode::AddWithCarry, regMap8[instr.opcode[0] & 7]), instr.len, inFlags);
                break;
            case 0x8E: // ADC (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp));
                addInstruction(alu(GenOpcode::AddWithCarry, GenReg::Temp), instr.len, inFlags);
                break;

            case 0x90: // SUB B
            case 0x91: // SUB C
            case 0x92: // SUB D
            case 0x93: // SUB E
            case 0x94: // SUB H
            case 0x95: // SUB L
            case 0x97: // SUB A
                addInstruction(alu(GenOpcode::Subtract, regMap8[instr.opcode[0] & 7]), instr.len, inFlags);
                break;
            case 0x96: // SUB (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp), instr.len, inFlags);
                break;

            case 0x98: // SBC B
            case 0x99: // SBC C
            case 0x9A: // SBC D
            case 0x9B: // SBC E
            case 0x9C: // SBC H
            case 0x9D: // SBC L
            case 0x9F: // SBC A
                addInstruction(alu(GenOpcode::SubtractWithCarry, regMap8[instr.opcode[0] & 7]), instr.len, inFlags);
                break;
            case 0x9E: // SBC (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp));
                addInstruction(alu(GenOpcode::SubtractWithCarry, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xA0: // AND B
            case 0xA1: // AND C
            case 0xA2: // AND D
            case 0xA3: // AND E
            case 0xA4: // AND H
            case 0xA5: // AND L
            case 0xA7: // AND A
                addInstruction(alu(GenOpcode::And, regMap8[instr.opcode[0] & 7]), instr.len, inFlags);
                break;
            case 0xA6: // AND (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp));
                addInstruction(alu(GenOpcode::And, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xA8: // XOR B
            case 0xA9: // XOR C
            case 0xAA: // XOR D
            case 0xAB: // XOR E
            case 0xAC: // XOR H
            case 0xAD: // XOR L
            case 0xAF: // XOR A
                addInstruction(alu(GenOpcode::Xor, regMap8[instr.opcode[0] & 7]), instr.len, inFlags);
                break;
            case 0xAE: // XOR (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp));
                addInstruction(alu(GenOpcode::Xor, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xB0: // OR B
            case 0xB1: // OR C
            case 0xB2: // OR D
            case 0xB3: // OR E
            case 0xB4: // OR H
            case 0xB5: // OR L
            case 0xB7: // OR A
                addInstruction(alu(GenOpcode::Or, regMap8[instr.opcode[0] & 7]), instr.len, inFlags);
                break;
            case 0xB6: // OR (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp));
                addInstruction(alu(GenOpcode::Or, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xB8: // CP B
            case 0xB9: // CP C
            case 0xBA: // CP D
            case 0xBB: // CP E
            case 0xBC: // CP H
            case 0xBD: // CP L
            case 0xBF: // CP A
                addInstruction(compare(regMap8[instr.opcode[0] & 7]), instr.len, inFlags);
                break;
            case 0xBE: // CP (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp));
                addInstruction(compare(GenReg::Temp), instr.len, inFlags);
                break;

            case 0xC0: // RET NZ
            case 0xC8: // RET Z
            case 0xD0: // RET NC
            case 0xD8: // RET C
            {
                // same code as RET, but with an inverted conditional jump over it...
                branchTargetOnNext = true;

                auto cond = invCondMap[(instr.opcode[0] >> 3) & 3];
                addInstruction(loadImm(pc));
                addInstruction(jump(cond, GenReg::Temp, 1)); 

                // pop PC
                // low
                addInstruction(load(GenReg::SP, GenReg::Temp2, 0));
                // sp++
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP, 0));

                // high
                addInstruction(load(GenReg::SP, GenReg::Temp3));
                // addr |= high << 8
                addInstruction(loadImm(8, 0));
                addInstruction(alu(GenOpcode::ShiftLeft, GenReg::Temp, GenReg::Temp3));
                addInstruction(alu(GenOpcode::Or, GenReg::Temp3, GenReg::Temp2, 0));
                // sp++
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP, 0));

                addInstruction(jump(GenCondition::Always, GenReg::Temp2, 1), instr.len, inFlags);
                break;
            }

            case 0xC1: // POP BC
            case 0xD1: // POP DE
            case 0xE1: // POP HL
            case 0xF1: // POP AF
            {
                auto reg = regMap16PushPop[(instr.opcode[0] >> 4) & 3];

                // low
                addInstruction(load(GenReg::SP, lowHalf(reg)));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP, 0));

                // high
                addInstruction(load(GenReg::SP, highHalf(reg)));
                
                // fixup flags
                if(reg == GenReg::AF)
                {
                    addInstruction(loadImm(0xF0, 0));
                    addInstruction(alu(GenOpcode::And, GenReg::Temp, GenReg::F, 0));

                    inFlags &= ~GenOp_WriteFlags; // marked as writing all flags, but we don't want to set that on the add
                }

                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP), instr.len, inFlags);

                break;
            }

            case 0xC2: // JP NZ,nn
            case 0xCA: // JP Z,nn
            case 0xD2: // JP NC,nn
            case 0xDA: // JP C,nn
            {
                auto cond = condMap[(instr.opcode[0] >> 3) & 3];
                addInstruction(loadImm16(instr.opcode + 1));
                addInstruction(jump(cond), instr.len, inFlags);
                break;
            }
            case 0xC3: // JP nn
                addInstruction(loadImm16(instr.opcode + 1));
                addInstruction(jump(), instr.len, inFlags);
                break;

            case 0xC4: // CALL NZ,nn
            case 0xCC: // CALL Z,nn
            case 0xD4: // CALL NC,nn
            case 0xDC: // CALL C,nn
            {
                branchTargetOnNext = true;

                auto cond = invCondMap[(instr.opcode[0] >> 3) & 3];
                addInstruction(loadImm(pc, 2));
                addInstruction(jump(cond, GenReg::Temp, 1));

                // push PC
                // high
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::SP, 0));
                addInstruction(loadImm(pc >> 8, 0));
                addInstruction(store(GenReg::SP, GenReg::Temp));

                // low
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::SP, 0));
                addInstruction(loadImm(pc & 0xFF, 0));
                addInstruction(store(GenReg::SP, GenReg::Temp));

                // jump
                addInstruction(loadImm16(instr.opcode + 1, 0));
                addInstruction(jump(GenCondition::Always, GenReg::Temp, 1), instr.len, inFlags | GenOp_Call);
                break;
            }

            case 0xC5: // PUSH BC
            case 0xD5: // PUSH DE
            case 0xE5: // PUSH HL
            case 0xF5: // PUSH AF
            {
                auto reg = regMap16PushPop[(instr.opcode[0] >> 4) & 3];

                // high
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::SP));
                addInstruction(store(GenReg::SP, highHalf(reg)));

                // low
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::SP, 0));
                addInstruction(store(GenReg::SP, lowHalf(reg), 2), instr.len, inFlags);

                break;
            }

            case 0xC6: // ADD n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xC7: // RST 0
            case 0xCF: // RST 08
            case 0xD7: // RST 10
            case 0xDF: // RST 18
            case 0xE7: // RST 20
            case 0xEF: // RST 28
            case 0xF7: // RST 30
            case 0xFF: // RST 38
            {
                // push PC
                // high
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::SP, 0));
                addInstruction(loadImm(pc >> 8));
                addInstruction(store(GenReg::SP, GenReg::Temp));

                // low
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::SP, 0));
                addInstruction(loadImm(pc & 0xFF, 0));
                addInstruction(store(GenReg::SP, GenReg::Temp));

                // jump
                addInstruction(loadImm(instr.opcode[0] & 0x38, 0));
                addInstruction(jump(GenCondition::Always, GenReg::Temp, 1), instr.len, inFlags);
                break;
            }

            case 0xC9: // RET
            {
                // pop PC
                // low
                addInstruction(load(GenReg::SP, GenReg::Temp2));
                // sp++
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP, 0));

                // high
                addInstruction(load(GenReg::SP, GenReg::Temp3));
                // addr |= high << 8
                addInstruction(loadImm(8, 0));
                addInstruction(alu(GenOpcode::ShiftLeft, GenReg::Temp, GenReg::Temp3));
                addInstruction(alu(GenOpcode::Or, GenReg::Temp3, GenReg::Temp2, 0));
                // sp++
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP, 0));

                addInstruction(jump(GenCondition::Always, GenReg::Temp2, 1), instr.len, inFlags);
                break;
            }

            case 0xCB: // extended ops
            {
                // +1 cycle
                GenOpInfo op{};
                op.cycles = 1;
                addInstruction(op);

                bool isMem = (instr.opcode[1] & 7) == 6;
                auto reg = regMap8[instr.opcode[1] & 7];
                int bit = (instr.opcode[1] >> 3) & 7; // BIT/RES/SET

                if(isMem)
                    addInstruction(load(GenReg::HL, reg));

                if(instr.opcode[1] < 0x40) // shifts/rotates
                {
                    auto addShift = [&](GenOpcode op, int shift = 1)
                    {
                        addInstruction(loadImm(shift, 0));
                        if(isMem)
                            addInstruction(alu(op, GenReg::Temp, reg, 0), 0, inFlags & (GenOp_WriteFlags | GenOp_MagicAlt1 | GenOp_MagicAlt2));
                        else
                            addInstruction(alu(op, GenReg::Temp, reg), instr.len, inFlags);
                    };

                    switch(instr.opcode[1] & ~7)
                    {
                        case 0x00: // RLC
                            addShift(GenOpcode::RotateLeft);
                            break;
                        case 0x08: // RRC
                            addShift(GenOpcode::RotateRight);
                            break;
                        case 0x10: // RL
                            addShift(GenOpcode::RotateLeftCarry);
                            break;
                        case 0x18: // RR
                            addShift(GenOpcode::RotateRightCarry);
                            break;
                        case 0x20: // SLA
                            addShift(GenOpcode::ShiftLeft);
                            break;
                        case 0x28: // SRA
                            addShift(GenOpcode::ShiftRightArith);
                            break;
                        case 0x30: // SWAP
                            inFlags |= GenOp_MagicAlt2;
                            addShift(GenOpcode::RotateLeft, 4);
                            break;
                        case 0x38: // SRL
                            addShift(GenOpcode::ShiftRightLogic);
                            break;
                    }
                }
                else if(instr.opcode[1] < 0x80) // BIT
                {
                    addInstruction(loadImm(1 << bit, 0));
                    auto andOp = alu(GenOpcode::And, GenReg::Temp, reg);
                    andOp.dst[0] = andOp.src[1]; // don't need the result, set to temp
                    addInstruction(andOp, instr.len, inFlags | preserveC); // no writeback, so this is always the end
                }
                else if(instr.opcode[1] < 0xC0) // RES
                {
                    uint8_t mask = ~(1 << bit);
                    addInstruction(loadImm(mask, 0));

                    auto aluOp = alu(GenOpcode::And, GenReg::Temp, reg);

                    if(isMem)
                    {
                        // there's another op
                        aluOp.cycles = 0;
                        addInstruction(aluOp, 0, inFlags & GenOp_WriteFlags);
                    }
                    else // this is the end
                        addInstruction(aluOp, instr.len, inFlags);
                }
                else // SET
                {
                    addInstruction(loadImm(1 << bit, 0));

                    auto aluOp = alu(GenOpcode::Or, GenReg::Temp, reg);

                    if(isMem)
                    {
                        // there's another op
                        aluOp.cycles = 0;
                        addInstruction(aluOp, 0, inFlags & GenOp_WriteFlags);
                    }
                    else // this is the end
                        addInstruction(aluOp, instr.len, inFlags);
                }

                if(isMem && (instr.opcode[1] >= 0x80 || instr.opcode[1] < 0x40)) // BIT doesn't write back
                    addInstruction(store(GenReg::HL, reg, 2), instr.len, inFlags & ~GenOp_WriteFlags);

                break;
            }

            case 0xCD: // CALL nn
            {
                // push PC
                // high
                addInstruction(loadImm(1));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::SP));
                addInstruction(loadImm(pc >> 8));
                addInstruction(store(GenReg::SP, GenReg::Temp));

                // low
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::SP, 0));
                addInstruction(loadImm(pc & 0xFF, 0));
                addInstruction(store(GenReg::SP, GenReg::Temp));

                // jump
                addInstruction(loadImm16(instr.opcode + 1, 0));
                addInstruction(jump(GenCondition::Always, GenReg::Temp, 1), instr.len, inFlags | GenOp_Call);
                break;
            }

            case 0xCE: // ADC n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(alu(GenOpcode::AddWithCarry, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xD6: // SUB n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xD9: // RETI
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::DMG_EnableIntrForRet;
                addInstruction(op);

                // ... regular ret code

                // pop PC
                // low
                addInstruction(load(GenReg::SP, GenReg::Temp2));
                // sp++
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP, 0));

                // high
                addInstruction(load(GenReg::SP, GenReg::Temp3));
                // addr |= high << 8
                addInstruction(loadImm(8, 0));
                addInstruction(alu(GenOpcode::ShiftLeft, GenReg::Temp, GenReg::Temp3));
                addInstruction(alu(GenOpcode::Or, GenReg::Temp3, GenReg::Temp2, 0));
                // sp++
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP, 0));

                addInstruction(jump(GenCondition::Always, GenReg::Temp2, 1), instr.len, inFlags);
                break;
            }

            case 0xDE: // SBC n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(alu(GenOpcode::SubtractWithCarry, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xE0: // LDH (n),A
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::LoadImm;
                op.cycles = 1;
                op.imm = 0xFF00 | instr.opcode[1];
                addInstruction(op);
                addInstruction(store(GenReg::Temp, GenReg::A, 2), instr.len, inFlags);
                break;
            }

            case 0xE2: // LDH (C),A
                addInstruction(loadImm(0xFF00, 0));
                addInstruction(move(GenReg::C, GenReg::Temp2, 0));
                addInstruction(alu(GenOpcode::Or, GenReg::Temp2, GenReg::Temp, 0));
                addInstruction(store(GenReg::Temp, GenReg::A, 2), instr.len, inFlags);
                break;

            case 0xE6: // AND n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(alu(GenOpcode::And, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xE8: // ADD SP,n
            {
                // sets flags as if it's an 8-bit add... so do that first
                auto imm = instr.opcode[1];
                addInstruction(move(GenReg::SP, GenReg::Temp2B, 0));
                addInstruction(loadImm(imm));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::Temp2B), 0, (inFlags & Op_WriteFlags) | GenOp_MagicAlt1);

                // do the real add
                addInstruction(loadImm((imm & 0x80) ? (0xFF00 | imm) : imm)); // sign extend
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::SP), instr.len, inFlags & ~Op_WriteFlags);
                break;
            }

            case 0xF8: // LDHL SP,n
            {
                // sets flags as if it's an 8-bit add... so do that first
                auto imm = instr.opcode[1];
                addInstruction(move(GenReg::SP, GenReg::Temp2B, 0));
                addInstruction(loadImm(imm));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::Temp2B), 0, (inFlags & Op_WriteFlags) | GenOp_MagicAlt1);

                // do the real add
                addInstruction(move(GenReg::SP, GenReg::HL, 0));
                addInstruction(loadImm((imm & 0x80) ? (0xFF00 | imm) : imm, 0)); // sign extend
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::HL), instr.len, inFlags & ~Op_WriteFlags);
                break;
            }

            case 0xE9: // JP (HL)
                addInstruction(jump(GenCondition::Always, GenReg::HL, 1), instr.len, inFlags);
                break;


            case 0xEA: // LD (nn),A
                addInstruction(loadImm16(instr.opcode + 1));
                addInstruction(store(GenReg::Temp, GenReg::A, 2), instr.len, inFlags);
                break;

            case 0xEE: // XOR n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(alu(GenOpcode::Xor, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xF0: // LDH A,(n)
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::LoadImm;
                op.cycles = 1;
                op.imm = 0xFF00 | instr.opcode[1];
                addInstruction(op);
                addInstruction(load(GenReg::Temp, GenReg::A, 2), instr.len, inFlags);
                break;
            }

            case 0xF2: // LDH A,(C)
                addInstruction(loadImm(0xFF00, 0));
                addInstruction(move(GenReg::C, GenReg::Temp2, 0));
                addInstruction(alu(GenOpcode::Or, GenReg::Temp2, GenReg::Temp, 0));
                addInstruction(load(GenReg::Temp, GenReg::A, 2), instr.len, inFlags);
                break;

            case 0xF3: // DI
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::DMG_DI;
                op.cycles = 1;
                addInstruction(op, instr.len, inFlags);
                break;
            }

            case 0xF6: // OR n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(alu(GenOpcode::Or, GenReg::Temp), instr.len, inFlags);
                break;

            case 0xF9: // LD SP,HL
                addInstruction(move(GenReg::HL, GenReg::SP, 2), instr.len, inFlags);
                genBlock.flags |= GenBlock_DMGSPWrite;
                break;

            case 0xFA: // LD A,(nn)
                addInstruction(loadImm16(instr.opcode + 1));
                addInstruction(load(GenReg::Temp, GenReg::A, 2), instr.len, inFlags);
                break;

            case 0xFB: // EI
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::DMG_EI;
                op.cycles = 1;
                addInstruction(op, instr.len, inFlags);
                break;
            }

            case 0xFE: // CP n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(compare(GenReg::Temp), instr.len, inFlags);
                break;

            default:
            {
                printf("unhandled op %02X\n", instr.opcode[0]);
                return false;
            }
        }
    }

    return true;
}

void DMGRecompiler::printInfo(BlockInfo &blockInfo)
{
    for(auto &instr : blockInfo.instructions)
    {
        int i = 0;
        for(; i < instr.len; i++)
            printf("%02X ", instr.opcode[i]);

        for(;i < 3; i++)
            printf("   ");

        if(instr.regsRead || instr.regsWritten)
        {
            const char *regNames[]{"A", "SP", "B", "C", "D", "E", "H", "L"};

            // read
            for(int i = 0; i < 8; i ++)
            {
                if(instr.regsRead & (1 << i))
                    printf("%s ", regNames[i]);
            }

            if(instr.flags & Op_Load)
                printf("mem ");

            printf("-> ");

            // written
            for(int i = 0; i < 8; i ++)
            {
                if(instr.regsWritten & (1 << i))
                    printf("%s ", regNames[i]);
            }

            if(instr.flags & Op_Store)
                printf("mem ");
        }

        // flags
        if(instr.flags & 0xF)
        {
            printf("Fi %s%s%s%s ", 
                    instr.flags & Op_ReadC ? "C" : "",
                    instr.flags & Op_ReadH ? "H" : "",
                    instr.flags & Op_ReadN ? "N" : "",
                    instr.flags & Op_ReadZ ? "Z" : "");
        }

        if(instr.flags & 0xF0)
        {
            printf("Fo %s%s%s%s ", 
                    instr.flags & Op_WriteC ? "C" : "",
                    instr.flags & Op_WriteH ? "H" : "",
                    instr.flags & Op_WriteN ? "N" : "",
                    instr.flags & Op_WriteZ ? "Z" : "");
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

bool DMGRecompiler::compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo) 
{
    GenBlockInfo genBlock;
    genBlock.flags = 0;

    // since we refuse to compile when OAM DMA is active we can skip most of cycleExecuted when not running from HRAM
    if(pc >= 0xFF00) // in HRAM
        genBlock.flags |= GenBlock_StrictSync;

    bool success = convertToGeneric(pc, blockInfo, genBlock);

#ifdef RECOMPILER_DEBUG
    printf("gen block (%s):\n", success ? "success" : "failed");
    printGenBlock(pc, genBlock, target.getSourceInfo());
    printf("\n\n");
#endif

    if(success && target.compile(codePtr, codeBuf + codeBufSize, pc, genBlock))
        return true;

    return false;
}

void DMGRecompiler::compileEntry()
{
    auto entryPtr = target.compileEntry(curCodePtr, codeBufSize);
    entryFunc = reinterpret_cast<CompiledFunc>(entryPtr);
}

// wrappers around member funcs
void DMGRecompiler::cycleExecuted(DMGCPU *cpu)
{
    cpu->cycleExecuted();
}

uint8_t DMGRecompiler::readMem(DMGCPU *cpu, uint16_t addr)
{
    return cpu->readMem(addr);
}

int DMGRecompiler::writeMem(DMGCPU *cpu, uint16_t addr, uint8_t data, int cyclesToRun)
{
    auto &compiler = cpu->compiler; // oh right, this is a static func

    // invalidate code on mem write
    if(addr >= compiler.minRAMCode)
    {
        auto mappedAddr = cpu->mem.makeBankedAddress(addr);

        // skip anything not in RAM
        for(auto it = compiler.compiled.lower_bound(1 << 28 /*VRAM as a mapped address */); it != compiler.compiled.end();)
        {
            if(mappedAddr >= it->first && mappedAddr < it->second.endPC)
            {
#ifdef RECOMPILER_DEBUG
                printf("invalidate compiled code @%07X(%04X) in %04X-%04X\n", mappedAddr, addr, it->first, it->second.endPC);
#endif
                // rewind if last code compiled
                // TODO: reclaim memory in other cases
                if(it->second.endPtr == compiler.curCodePtr)
                    compiler.curCodePtr = it->second.startPtr;

                // invalidate any saved return addresses
                for(auto &saved : compiler.savedExits)
                {
                    if(std::get<1>(saved) >= it->first && std::get<1>(saved) <= it->second.endPC)
                        saved = {nullptr, 0};
                }

                it = compiler.compiled.erase(it);

                continue; // might have compiled the same code more than once
            }

            // past this address, stop
            if(it->first > mappedAddr)
                break;

            ++it;
        }
    }

    cpu->writeMem(addr, data);

    // if we wrote a reg, timings may have changed
    // ... and we have a convenient return value to pass the new value back

    // ... also, there could be an interrupt/DMA pending RIGHT NOW
    bool inHRAM = cpu->pc >= 0xFF00; // PC is not up-to-date here, but should be close
    if((cpu->masterInterruptEnable && cpu->serviceableInterrupts) || cpu->gdmaTriggered || (!inHRAM && (cpu->oamDMADelay || cpu->oamDMACount)) || cpu->timerReload)
    {
        // we're going to miss the first updateOAMDMA call
        if(cpu->oamDMADelay == 2)
            cpu->oamDMADelay--;

        return 0;
    }

    int cycles = std::min(cyclesToRun, cpu->getDisplay().getCyclesToNextUpdate());

    if(cpu->nextTimerInterrupt)
        cycles = std::min(cycles, static_cast<int>(cpu->nextTimerInterrupt - cpu->cycleCount));

    if(cpu->nextSerialBitCycle)
        cycles = std::min(cycles, static_cast<int>(cpu->nextSerialBitCycle - cpu->cycleCount));

    return cycles;
}