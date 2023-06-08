#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "DMGRecompilerGeneric.h"
#include "DMGCPU.h"

DMGRecompilerGeneric::DMGRecompilerGeneric(DMGCPU &cpu) : DMGRecompiler(cpu), fallback(cpu)
{
    fallback.codeBuf = codeBuf;
    fallback.curCodePtr = codeBuf;

    SourceInfo sourceInfo;

    uint16_t regsOffset = reinterpret_cast<uintptr_t>(&cpu.regs) - reinterpret_cast<uintptr_t>(&cpu);

    sourceInfo.registers.emplace_back(SourceRegInfo{"tmp", 16, SourceRegType::Temp, 0, 0, 0xFFFF});

    // 16 bit
    sourceInfo.registers.emplace_back(SourceRegInfo{"AF ", 16, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 2;
    sourceInfo.registers.emplace_back(SourceRegInfo{"BC ", 16, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 2;
    sourceInfo.registers.emplace_back(SourceRegInfo{"DE ", 16, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 2;
    sourceInfo.registers.emplace_back(SourceRegInfo{"HL ", 16, SourceRegType::General, 0, 0, regsOffset});
    regsOffset = reinterpret_cast<uintptr_t>(&cpu.sp) - reinterpret_cast<uintptr_t>(&cpu);
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
    sourceInfo.registers.emplace_back(SourceRegInfo{"tm2", 16, SourceRegType::Temp, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"tm3", 16, SourceRegType::Temp, 0, 0, 0xFFFF});

    sourceInfo.flags.emplace_back(SourceFlagInfo{'C', 4, SourceFlagType::Carry});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'H', 5, SourceFlagType::HalfCarry});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'N', 6, SourceFlagType::WasSub});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'Z', 7, SourceFlagType::Zero});

    sourceInfo.pcSize = 16;
    sourceInfo.pcOffset = reinterpret_cast<uintptr_t>(&cpu.pc) - reinterpret_cast<uintptr_t>(&cpu);

    sourceInfo.exitCallFlag = &exitCallFlag;
    sourceInfo.savedExitPtr = &tmpSavedPtr;

    sourceInfo.cycleCount = &cpu.cycleCount;
    sourceInfo.cycleExecuted = reinterpret_cast<void (*)(void *)>(DMGRecompilerGeneric::cycleExecuted);
    sourceInfo.readMem = reinterpret_cast<uint8_t (*)(void *, uint16_t)>(DMGRecompilerGeneric::readMem);
    sourceInfo.writeMem = reinterpret_cast<int (*)(void *, uint16_t, uint8_t, int)>(DMGRecompilerGeneric::writeMem);

    target.init(sourceInfo, &cpu);
}

bool DMGRecompilerGeneric::compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo) 
{
    printf("in block %04X:\n", pc);
    printInfo(blockInfo);
    printf("\n");

    GenBlockInfo genBlock;
    bool success = convertToGeneric(pc, blockInfo, genBlock);

    printf("gen block (%s):\n", success ? "success" : "failed");
    printBlock(pc, genBlock);
    printf("\n\n");

    auto oldCodePtr = codePtr;
    if(success && target.compile(codePtr, codeBuf + codeBufSize, pc, genBlock))
    {
        //fallback.curCodePtr = codePtr; //
        //return true;
    }

    codePtr = oldCodePtr;
    return fallback.compile(codePtr, pc, blockInfo);
}

void DMGRecompilerGeneric::compileEntry()
{

    auto entryPtr = curCodePtr;
    target.compileEntry(curCodePtr, codeBufSize);
    entryFunc = reinterpret_cast<CompiledFunc>(entryPtr);

    // hax
    fallback.curCodePtr = curCodePtr;
    fallback.exitPtr = target.exitPtr;
    fallback.saveAndExitPtr = target.saveAndExitPtr;
    fallback.exitForCallPtr = target.exitForCallPtr;
}

bool DMGRecompilerGeneric::convertToGeneric(uint16_t pc, BlockInfo &block, GenBlockInfo &genBlock)
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

        Temp2, // used by inc/dec (hl), ext ops (HL) and ret
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
        GenReg::Temp2,
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
        if(flags & (Op_Branch | Op_Exit))
            assert(op.opcode == GenOpcode::Jump || op.opcode == GenOpcode::NOP/*FIXME: unhandled op*/);

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

    auto loadImm16 = [](uint8_t *imm, int cycles = 2)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::LoadImm;
        ret.cycles = cycles;
        ret.imm = imm[0] | imm[1] << 8;

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

    bool ret = true;

    bool branchTargetOnNext = false;

    for(auto &instr : block.instructions)
    {
        pc += instr.len;

        auto inFlags = instr.flags;

        // remove unused flags
        inFlags &= ~(Op_ReadFlags | Op_Load | Op_Store);

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
                addInstruction(alu(GenOpcode::RotateLeft, GenReg::Temp), instr.len, inFlags);
                break;

            // ld (nn) sp

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
                addInstruction(alu(GenOpcode::RotateRight, GenReg::Temp), instr.len, inFlags);
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
                addInstruction(alu(GenOpcode::RotateLeftCarry, GenReg::Temp), instr.len, inFlags);
                break;

            case 0x18: // JR m
                addInstruction(loadImm(pc + static_cast<int8_t>(instr.opcode[1])));
                addInstruction(jump(), instr.len, inFlags);
                break;

            case 0x1F: // RLA
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::RotateRightCarry, GenReg::Temp), instr.len, inFlags);
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
                addInstruction(load(GenReg::HL, GenReg::Temp2));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Add, GenReg::Temp, GenReg::Temp2, 0), 0, inFlags | preserveC);
                addInstruction(store(GenReg::HL, GenReg::Temp2, 2), instr.len);
                break;

            case 0x35: // DEC (HL)
                addInstruction(load(GenReg::HL, GenReg::Temp2));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::Temp2, 0), 0, inFlags | preserveC);
                addInstruction(store(GenReg::HL, GenReg::Temp2, 2), instr.len);
                break;

            case 0x36: // LD (HL),n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(store(GenReg::HL, GenReg::Temp, 2), instr.len, inFlags);
                break;
    
            // SCF

            case 0x3A: // LDD A,(HL)
                addInstruction(load(GenReg::HL, GenReg::A));
                addInstruction(loadImm(1, 0));
                addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, GenReg::HL), instr.len, inFlags);
                break;

            // CCF

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
                addInstruction(jump(cond, GenReg::Temp, 1), 0, Op_Branch | (inFlags & 0xF)); // move the cond read here

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

                addInstruction(jump(GenCondition::Always, GenReg::Temp2, 1), instr.len, inFlags & ~0xF);
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
                addInstruction(jump(cond, GenReg::Temp, 1), 0, Op_Branch | (inFlags & 0xF)); // move the cond read here

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
                addInstruction(jump(GenCondition::Always, GenReg::Temp, 1), instr.len, inFlags & ~0xF);
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
                            addInstruction(alu(op, GenReg::Temp, reg, 0), 0, inFlags & GenOp_WriteFlags);
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
                    addInstruction(loadImm(~(1 << bit), 0));

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
                addInstruction(jump(GenCondition::Always, GenReg::Temp, 1), instr.len, inFlags);
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
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::LoadImm;
                op.imm = 0xFF00;
                addInstruction(op);
                addInstruction(alu(GenOpcode::Or, GenReg::C, GenReg::Temp, 0));
                addInstruction(store(GenReg::Temp, GenReg::A, 2), instr.len, inFlags);
                break;
            }

            case 0xE6: // AND n
                addInstruction(loadImm(instr.opcode[1]));
                addInstruction(alu(GenOpcode::And, GenReg::Temp), instr.len, inFlags);
                break;

            /*
            ADD SP,n
            LDHL SP,n
            */

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
            {
                GenOpInfo op{};
                op.opcode = GenOpcode::LoadImm;
                op.imm = 0xFF00;
                addInstruction(op);
                addInstruction(alu(GenOpcode::Or, GenReg::C, GenReg::Temp, 0));
                addInstruction(load(GenReg::Temp, GenReg::A, 2), instr.len, inFlags);
                break;
            }

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
                ret = false;
                GenOpInfo op{};
                op.opcode = GenOpcode::NOP; // nop with no cycles
                addInstruction(op, instr.len, inFlags);
            }
        }
    }

    return ret;
}

void DMGRecompilerGeneric::printBlock(uint16_t pc, GenBlockInfo &block)
{
    auto &sourceInfo = target.getSourceInfo();

    struct OpMeta
    {
        const char *name;
        uint8_t numSrc, numDst;
    };

    static const OpMeta opMeta[]
    {
        {"nop ", 0, 0},
        {"ldim", 0, 0},
        {"move", 1, 1},
        {"load", 1, 1},
        {"stor", 2, 0},
        {"add ", 2, 1},
        {"addc", 2, 1},
        {"and ", 2, 1},
        {"comp", 2, 0},
        {"or  ", 2, 1},
        {"sub ", 2, 1},
        {"subc", 2, 1},
        {"xor ", 2, 1},
        {"not ", 1, 1},
        {"rol ", 2, 1},
        {"rolc", 2, 1},
        {"ror ", 2, 1},
        {"rorc", 2, 1},
        {"shl ", 2, 1},
        {"shra", 2, 1},
        {"shrl", 2, 1},
        {"jump", 2, 0},
    };

    static const OpMeta specialOpMeta[]
    {
        {"_stp", 0, 0},
        {"_daa", 0, 0},
        {"_hlt", 0, 0},
        {"_eir", 0, 0},
        {"_di ", 0, 0},
        {"_ei ", 0, 0},
    };

    static const char *condName[]
    {
        "equ",
        "neq",
        "cas",
        "cac",
        "alw",
    };

    // flag info
    char flagChars[]{'X', 'X', 'X', 'X'};

    int carryMask = 0, zeroMask = 0;

    int i = 0;
    for(auto &flag : sourceInfo.flags)
    {
        if(flag.type == SourceFlagType::Carry)
            carryMask = 1 << i;
        else if(flag.type == SourceFlagType::Zero)
            zeroMask = 1 << i;

        flagChars[i++] = flag.label;
    }
    

    uint16_t lastPC = 1;

    for(auto &instr : block.instructions)
    {
        if(lastPC != pc)
        {
            lastPC = pc;
            printf("%04X: ", pc);
        }
        else
            printf("    : ");

        auto intOp = static_cast<int>(instr.opcode);

        auto &meta = intOp & 0x80 ? specialOpMeta[intOp & 0x7F] : opMeta[intOp];

        if(instr.opcode == GenOpcode::NOP && !instr.cycles)
        {
            printf("UNHANDLED\n");
            pc += instr.len;
            continue;
        }

        printf("%s", meta.name);

        if(instr.opcode == GenOpcode::LoadImm)
            printf(" %08X      ", instr.imm);
        else
        {
            // src
            int i = 0;
            
            // first jump src is the condition
            if(instr.opcode == GenOpcode::Jump)
            {
                printf(" %s", condName[instr.src[0]]);
                i++;
            }

            for(; i < meta.numSrc; i++)
            {
                if(instr.src[i] < sourceInfo.registers.size())
                    printf(" %s", sourceInfo.registers[instr.src[i]].label);
                else
                    printf(" R%02i", instr.src[i]);
            }

            // align
            for(; i < 2; i++)
                printf("    ");

            if(meta.numDst)
                printf(" ->");
            else
                printf("   ");
    
            // dst
            for(i = 0; i < meta.numDst; i++)
            {
                if(instr.dst[i] < sourceInfo.registers.size())
                    printf(" %s", sourceInfo.registers[instr.dst[i]].label);
                else
                    printf(" R%02i", instr.dst[i]);
            }
            
            // align
            for(; i < 1; i++)
                printf("    ");
        }

        // flags

        // work out flags read
        uint8_t flagsRead = 0;
        if(instr.opcode == GenOpcode::AddWithCarry || instr.opcode == GenOpcode::SubtractWithCarry || instr.opcode == GenOpcode::RotateLeftCarry || instr.opcode == GenOpcode::RotateRightCarry)
            flagsRead = carryMask;
        else if(instr.opcode == GenOpcode::Jump)
        {
            auto cond = static_cast<GenCondition>(instr.src[0]);
            if(cond == GenCondition::CarryClear || cond == GenCondition::CarrySet)
                flagsRead = carryMask;
            else if(cond != GenCondition::Always)
                flagsRead = zeroMask;
        }
        // else anything that reads a flags register directly

        if((instr.flags & 0xFF) || flagsRead)
        {
            printf(" flags %c%c%c%c -> %c%c%c%c", 
                    flagsRead   & 0x01 ? flagChars[0] : '-',
                    flagsRead   & 0x02 ? flagChars[1] : '-',
                    flagsRead   & 0x04 ? flagChars[2] : '-',
                    flagsRead   & 0x08 ? flagChars[3] : '-',
                    instr.flags & 0x10 ? flagChars[0] : '-',
                    instr.flags & 0x20 ? flagChars[1] : '-',
                    instr.flags & 0x40 ? flagChars[2] : '-',
                    instr.flags & 0x80 ? flagChars[3] : '-');
        }
        else
            printf("                   ");

        // cycle count
        if(instr.cycles)
            printf(" (%i cycles)", instr.cycles);
        else
            printf("           ");

        // branches
        // TODO: more info?
        if(instr.flags & Op_Exit)
            printf(" ex");

        if(instr.flags & Op_BranchTarget)
            printf(" bt");

        printf("\n");


        pc += instr.len;
    }
}