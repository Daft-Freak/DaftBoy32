#include <cassert>
#include <cstdio>
#include <variant>

#include "DMGRecompilerThumb.h"

#include "DMGCPU.h"
#include "ThumbBuilder.h"

// reg helpers
// we only have 32 bit regs, so A/F/AF all map to the same thing
enum class RegPart
{
    Low = 1,
    High = 2,
    Both = Low | High
};

struct RegInfo
{
    Reg reg;
    RegPart part;
};

inline constexpr RegInfo reg(DMGCPU::Reg r)
{
    const RegInfo regMap8[]
    {
        {Reg::R4, RegPart::High}, // A
        {Reg::R4, RegPart::Low }, // F
        {Reg::R5, RegPart::High}, // B
        {Reg::R5, RegPart::Low }, // C
        {Reg::R6, RegPart::High}, // D
        {Reg::R6, RegPart::Low }, // E
        {Reg::R7, RegPart::High}, // H
        {Reg::R7, RegPart::Low }  // L
    };
    return regMap8[static_cast<int>(r)];
}

inline constexpr RegInfo reg(DMGCPU::WReg r)
{
    const RegInfo regMap16[]
    {
        {Reg::R4, RegPart::Both}, // AF
        {Reg::R5, RegPart::Both}, // BC
        {Reg::R6, RegPart::Both}, // DE
        {Reg::R7, RegPart::Both}  // HL
    };
    return regMap16[static_cast<int>(r)];
}

// helpers for accessing 8/16-bit regs/values

// copies 8 bit value from upper/lower reg or imm to reg
static void get8BitValue(ThumbBuilder &builder, Reg dst, std::variant<RegInfo, uint8_t> b)
{
    if(std::holds_alternative<RegInfo>(b))
    {
        auto reg = std::get<RegInfo>(b);

        if(reg.part == RegPart::High)
            builder.lsr(dst, reg.reg, 8); // shift it down
        else
            builder.uxtb(dst, reg.reg); // clear the high half
    }
    else
        builder.mov(dst, std::get<uint8_t>(b));
}

// store to 8-bit reg, modifies src
static void  write8BitReg(ThumbBuilder &builder, RegInfo dst, Reg src, Reg tmp = Reg::R2)
{
    if(dst.part == RegPart::Low)
    {
        builder.mov(tmp, 0xFF);
        builder.bic(dst.reg, tmp);
    }
    else
    {
        builder.uxtb(dst.reg, dst.reg);
        builder.lsl(src, src, 8);
    }

    builder.orr(dst.reg, src);
}

static void load16BitValue(ThumbBuilder &builder, Reg dst, uint16_t value, Reg tmp = Reg::R2)
{
    // TODO: can emit less code if one of the bytes is zero
    // or other cases where we can shift an 8 bit value
    builder.mov(dst, value & 0xFF);
    builder.mov(tmp, value >> 8);
    builder.lsl(tmp, tmp, 8);
    builder.orr(dst, tmp);
}

static void load32BitValue(ThumbBuilder &builder, Reg dst, uint32_t value)
{
    // LDR literal + b over data + some alignment bits
    // TODO: delay writing the value
    bool aligned = reinterpret_cast<uintptr_t>(builder.getPtr()) & 2; // if we're misaligned here, we'll be aligned after the instruction 
    builder.ldr(dst, aligned ? 4 : 0);
    builder.b(aligned ? 6 : 4);

    if(aligned)
        builder.data(0);

    builder.data(value);
    builder.data(value >> 16);
}

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
    // don't handle HRAM for now
    if(pc >= 0xFF00)
        return false;

    using DMGReg = DMGCPU::Reg;
    using WReg = DMGCPU::WReg;

    auto &mem = cpu.getMem();
    uint8_t opcode = instr.opcode[0];

    int cyclesThisInstr = 0;
    int delayedCyclesExecuted = 0;

    // mapping from opcodes
    static const RegInfo regMap8[]
    {
        reg(DMGReg::B),
        reg(DMGReg::C),
        reg(DMGReg::D),
        reg(DMGReg::E),
        reg(DMGReg::H),
        reg(DMGReg::L),
        {Reg::R1, RegPart::Low}, // placeholder (HL)
        reg(DMGReg::A),
    };

    static const RegInfo regMap16[]
    {
        reg(WReg::BC),
        reg(WReg::DE),
        reg(WReg::HL),
        {Reg::R1, RegPart::Both}, // TODO: SP, unless it's push/pop, then it's AF
    };

    auto getOff = [&builder](uint8_t *ptr)
    {
        return ptr - reinterpret_cast<uint8_t *>(builder.getPtr());
    };

    auto cycleExecuted = [this, &builder, &cyclesThisInstr, &delayedCyclesExecuted]()
    {
        cyclesThisInstr += 4;

        // only the optimised not-HRAM path
        delayedCyclesExecuted += 4;
    };

    auto syncCyclesExecuted = [this, &builder, &delayedCyclesExecuted]()
    {
        if(!delayedCyclesExecuted)
            return;

        assert(delayedCyclesExecuted < 0xFF);
        auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

        uint8_t u8Cycles = delayedCyclesExecuted;

        // we don't update cyclesToRun here, do it after returning instead
        auto cycleCountOff = reinterpret_cast<uintptr_t>(&cpu.cycleCount) - cpuPtr;
        assert(cycleCountOff <= 124);

        // load to r2, add and store back
        builder.mov(Reg::R1, Reg::R8); // cpu ptr

        builder.ldr(Reg::R2, Reg::R1, cycleCountOff);
        builder.add(Reg::R2, u8Cycles);
        builder.str(Reg::R2, Reg::R1, cycleCountOff);

        delayedCyclesExecuted = 0;
    };

    auto setupMemAddr = [&builder, &syncCyclesExecuted](std::variant<Reg, uint16_t> addr)
    {
        if(std::holds_alternative<Reg>(addr))
        {
            auto reg = std::get<Reg>(addr);

            // LDH (C) does this
            if(reg == Reg::R1)
                builder.push(1 << 1, false);

            syncCyclesExecuted(); // uses R1-2

            if(reg == Reg::R1)
                builder.pop(1 << 1, false);
            else
                builder.mov(Reg::R1, reg);
        }
        else
        {
            // accessing most ram shouldn't cause anything to be updated, so we don't need an accurate cycle count
            auto immAddr = std::get<uint16_t>(addr);
            if(immAddr >> 8 == 0xFF)
                syncCyclesExecuted();

            load16BitValue(builder, Reg::R1, immAddr);
        }
    };

    auto readMem = [&builder, &cycleExecuted, &syncCyclesExecuted, &setupMemAddr](std::variant<Reg, uint16_t> addr, RegInfo dst, bool postInc = false)
    {
        builder.push(1, false); // R0

        setupMemAddr(addr);

        builder.mov(Reg::R0, Reg::R8); // cpu pointer

        // get func ptr
        load32BitValue(builder, Reg::R2, reinterpret_cast<uintptr_t>(&DMGRecompiler::readMem));

        builder.blx(Reg::R2); // do call

        // move to dest
        write8BitReg(builder, dst, Reg::R0);

        if(postInc && std::holds_alternative<Reg>(addr))
        {
            auto addrReg = std::get<Reg>(addr);
            builder.add(addrReg, 1);
            builder.uxth(addrReg, addrReg);
        }

        builder.pop(1, false);

        cycleExecuted();
    };

    auto writeMem = [&builder, &cycleExecuted, &syncCyclesExecuted, &setupMemAddr](std::variant<Reg, uint16_t> addr, std::variant<RegInfo, uint8_t> data, bool preDec = false)
    {
        builder.push(1 << 4, false); // R4

        // TODO
        /*if(preDec && std::holds_alternative<Reg>(addr))
        {
            auto addrReg = std::get<Reg>(addr);
            builder.sub(addrReg, 1);
            builder.uxth(addrReg, addrReg);
        }*/

        setupMemAddr(addr);
        get8BitValue(builder, Reg::R2, data);
        builder.mov(Reg::R3, Reg::R0); // TODO: cycle count

        builder.mov(Reg::R0, Reg::R8); // cpu pointer

        // get func ptr
        load32BitValue(builder, Reg::R4, reinterpret_cast<uintptr_t>(&DMGRecompiler::writeMem));

        builder.blx(Reg::R4); // do call

        // TODO: returns new cycle count

        builder.pop(1 << 4, false);

        cycleExecuted();
    };

    const auto add = [&instr, &builder](std::variant<RegInfo, uint8_t> b, bool withCarry = false)
    {
        auto a = reg(DMGReg::A);

        if(withCarry)
        {
            // we're going to need another reg
            builder.push(1, false); // R0
            builder.mov(Reg::R0, a.reg); // copy AF
            builder.mov(Reg::R1, DMGCPU::Flag_C);
            builder.and_(Reg::R0, Reg::R1);
        }

        bool bIsReg = std::holds_alternative<RegInfo>(b);

        if(instr.flags & Op_WriteFlags)
        {
            // clear flags
            builder.mov(Reg::R1, 0xFF);
            builder.bic(a.reg, Reg::R1);
        }

        if(instr.flags & DMGCPU::Flag_H)
        {
            // room for optimisation here, probably
            builder.mov(Reg::R1, 0xF);
            get8BitValue(builder, Reg::R2, a);
            get8BitValue(builder, Reg::R3, b);

            builder.and_(Reg::R2, Reg::R1);
            builder.and_(Reg::R3, Reg::R1);
            builder.add(Reg::R3, Reg::R2, Reg::R3);

            if(withCarry)
            {
                // add the carry
                builder.cmp(Reg::R0, 0);
                builder.b(Condition::EQ, 2);
                builder.add(Reg::R3, 1);
            }
        }

        // get first reg
        get8BitValue(builder, Reg::R1, a);

        // get second value and do add/adc
        if(bIsReg)
        {
            get8BitValue(builder, Reg::R2, b);
            if(withCarry)
            {
                builder.cmp(Reg::R0, 0xF); // set C
                builder.adc(Reg::R1, Reg::R2);
            }
            else
                builder.add(Reg::R1, Reg::R1, Reg::R2);
        }
        else if(withCarry)
        {
            // no adc imm
            builder.mov(Reg::R2, std::get<uint8_t>(b));
            builder.cmp(Reg::R0, 0xF); // set C
            builder.adc(Reg::R1, Reg::R2);
        }
        else
            builder.add(Reg::R1, std::get<uint8_t>(b));

        // flags
        // cant use carry from op here
        if(instr.flags & DMGCPU::Flag_C)
        {
            builder.cmp(Reg::R1, 0xFF);
            builder.b(Condition::LE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_C);
            builder.orr(a.reg, Reg::R2);
        }

        builder.uxtb(Reg::R1, Reg::R1); // mask

        write8BitReg(builder, a, Reg::R1);

        if(instr.flags & DMGCPU::Flag_H)
        {
            // half res > 0xF
            builder.cmp(Reg::R3, 0xF);
            builder.b(Condition::LE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_H);
            builder.orr(a.reg, Reg::R2);
        }

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.cmp(Reg::R1, 0);
            builder.b(Condition::NE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_Z);
            builder.orr(a.reg, Reg::R2);
        }

        if(withCarry)
            builder.pop(1, false);
    };

    const auto addWithCarry = [&add](std::variant<RegInfo, uint8_t> b)
    {
        return add(b, true);
    };

    // SUB, SBC and CP
    const auto doSub = [&instr, &builder](std::variant<RegInfo, uint8_t> b, bool storeResult, bool withCarry)
    {
        auto a = reg(DMGReg::A);

        if(withCarry)
        {
            // we're going to need another reg
            builder.push(1, false); // R0
            builder.mov(Reg::R0, a.reg); // copy AF
            builder.mov(Reg::R1, DMGCPU::Flag_C);
            builder.mvn(Reg::R0, Reg::R0); // invert
            builder.and_(Reg::R0, Reg::R1);
        }

        bool bIsReg = std::holds_alternative<RegInfo>(b);

        if(instr.flags & Op_WriteFlags)
        {
            // clear flags
            builder.mov(Reg::R1, 0xFF);
            builder.bic(a.reg, Reg::R1);
        }

        if(instr.flags & DMGCPU::Flag_H)
        {
            // room for optimisation here, probably
            builder.mov(Reg::R1, 0xF);
            get8BitValue(builder, Reg::R2, a);
            get8BitValue(builder, Reg::R3, b);

            builder.and_(Reg::R2, Reg::R1);
            builder.and_(Reg::R3, Reg::R1);
            builder.sub(Reg::R3, Reg::R2, Reg::R3);

            if(withCarry)
            {
                // sub the carry
                builder.cmp(Reg::R0, 0);
                builder.b(Condition::NE, 2); // we inverted it
                builder.sub(Reg::R3, 1);
            }
        }

        // set the N flag
        if(instr.flags & DMGCPU::Flag_N)
        {
            builder.mov(Reg::R1, DMGCPU::Flag_N);
            builder.orr(a.reg, Reg::R1);
        }

        // get first reg
        get8BitValue(builder, Reg::R1, a);

        // get second value and do sub/sbc
        if(bIsReg)
        {
            get8BitValue(builder, Reg::R2, b);
            if(withCarry)
            {
                builder.cmp(Reg::R0, 0xF); // set C
                builder.sbc(Reg::R1, Reg::R2);
            }
            else
                builder.sub(Reg::R1, Reg::R1, Reg::R2);
        }
        else if(withCarry)
        {
            // no sbc imm
            builder.mov(Reg::R2, std::get<uint8_t>(b));
            builder.cmp(Reg::R0, 0xF); // set C
            builder.sbc(Reg::R1, Reg::R2);
        }
        else
            builder.sub(Reg::R1, std::get<uint8_t>(b));

        builder.uxtb(Reg::R1, Reg::R1);

        // flags
        if(instr.flags & DMGCPU::Flag_C)
        {
            // carry is reversed for sub 
            builder.b(Condition::CS, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_C);
            builder.orr(a.reg, Reg::R2);
        }

        if(storeResult)
            write8BitReg(builder, a, Reg::R1);

        if(instr.flags & DMGCPU::Flag_H)
        {
            // half res < 0
            builder.cmp(Reg::R3, 0);
            builder.b(Condition::GE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_H);
            builder.orr(a.reg, Reg::R2);
        }

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.cmp(Reg::R1, 0);
            builder.b(Condition::NE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_Z);
            builder.orr(a.reg, Reg::R2);
        }

        if(withCarry)
            builder.pop(1, false);
    };

    const auto sub = [&doSub](std::variant<RegInfo, uint8_t> b)
    {
        doSub(b, true, false);
    };

    const auto subWithCarry = [&doSub](std::variant<RegInfo, uint8_t> b)
    {
        doSub(b, true, true);
    };

    // op helpers
    const auto bitAnd = [&instr, &builder](std::variant<RegInfo, uint8_t> b)
    {
        auto regA = reg(WReg::AF).reg;
        auto regB = Reg::R1;

        get8BitValue(builder, regB, b);

        builder.lsr(regA, regA, 8); // shift A down (clears the flags for us)
        builder.and_(regA, regB);
        builder.lsl(regA, regA, 8); // shift it back up (still have a Z flag we can use)

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.b(Condition::NE, 4);
            builder.mov(Reg::R1, DMGCPU::Flag_Z);
            builder.orr(regA, Reg::R1);
        }

        if(instr.flags & DMGCPU::Flag_H)
        {
            builder.mov(Reg::R1, DMGCPU::Flag_H);
            builder.orr(regA, Reg::R1);
        }
    };

    const auto bitOr = [&instr, &builder](std::variant<RegInfo, uint8_t> b)
    {
        auto regA = reg(WReg::AF).reg;
        auto regB = Reg::R1;

        get8BitValue(builder, regB, b);

        builder.lsr(regA, regA, 8); // shift A down (clears the flags for us)
        builder.orr(regA, regB);
        builder.lsl(regA, regA, 8); // shift it back up (still have a Z flag we can use)

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.b(Condition::NE, 4);
            builder.mov(Reg::R1, DMGCPU::Flag_Z);
            builder.orr(regA, Reg::R1);
        }
    };

    const auto bitXor = [&instr, &builder](std::variant<RegInfo, uint8_t> b)
    {
        auto regA = reg(WReg::AF).reg;
        auto regB = Reg::R1;

        get8BitValue(builder, regB, b);

        builder.lsr(regA, regA, 8); // shift A down (clears the flags for us)
        builder.eor(regA, regB);
        builder.lsl(regA, regA, 8); // shift it back up (still have a Z flag we can use)

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.b(Condition::NE, 4);
            builder.mov(Reg::R1, DMGCPU::Flag_Z);
            builder.orr(regA, Reg::R1);
        }
    };

    const auto cmp = [&doSub](std::variant<RegInfo, uint8_t> b)
    {
        doSub(b, false, false);
    };

    pc += instr.len;

    auto oldPtr = builder.getPtr();
    cycleExecuted();

    switch(opcode)
    {
        case 0x00: // NOP
            break;

        case 0x01: // LD BC,nn
        case 0x11: // LD DE,nn
        case 0x21: // LD HL,nn
        //case 0x31: // LD SP,nn
        {
            cycleExecuted();
            cycleExecuted();

            auto dst = regMap16[opcode >> 4];
            load16BitValue(builder, dst.reg, instr.opcode[1] | instr.opcode[2] << 8);

            break;
        }

        case 0x02: // LD (BC),A
        case 0x12: // LD (DE),A
            writeMem(regMap16[opcode >> 4].reg, reg(DMGReg::A));
            break;

        case 0x03: // INC BC
        case 0x13: // INC DE
        case 0x23: // INC HL
        //case 0x33: // INC SP
        {
            auto r = regMap16[opcode >> 4].reg;
            builder.add(r, 1);
            builder.uxth(r, r);
            cycleExecuted();
            break;
        }

        case 0x0B: // DEC BC
        case 0x1B: // DEC DE
        case 0x2B: // DEC HL
        //case 0x3B: // DEC SP
        {
            auto r = regMap16[opcode >> 4].reg;
            builder.sub(r, 1);
            builder.uxth(r, r);
            cycleExecuted();
            break;
        }

        case 0x06: // LD B,n
        case 0x0E: // LD C,n
        case 0x16: // LD D,n
        case 0x1E: // LD E,n
        case 0x26: // LD H,n
        case 0x2E: // LD L,n
        case 0x3E: // LD A,n
        {
            cycleExecuted();

            builder.mov(Reg::R1, instr.opcode[1]);
            write8BitReg(builder, regMap8[opcode >> 3], Reg::R1);

            break;
        }

        case 0x0A: // LD A,(BC)
        case 0x1A: // LD A,(DE)
            readMem(regMap16[opcode >> 4].reg, reg(DMGReg::A));
            break;

        case 0x22: // LDI (HL),A
        {
            auto r = reg(WReg::HL).reg;
            writeMem(r, reg(DMGReg::A));
            builder.add(r, 1);
            builder.uxth(r, r);
            break;
        }

        case 0x2A: // LDI A, (HL)
        {
            auto r = reg(WReg::HL).reg;
            readMem(r, reg(DMGReg::A));
            builder.add(r, 1);
            builder.uxth(r, r);
            break;
        }

        case 0x32: // LDD (HL),A
        {
            auto r = reg(WReg::HL).reg;
            writeMem(r, reg(DMGReg::A));
            builder.sub(r, 1);
            builder.uxth(r, r);
            break;
        }

        case 0x36: // LD (HL),n
        {
            cycleExecuted();
            writeMem(reg(WReg::HL).reg, instr.opcode[1]);
            break;
        }

        case 0x3A: // LDD A, (HL)
        {
            auto r = reg(WReg::HL).reg;
            readMem(r, reg(DMGReg::A));
            builder.sub(r, 1);
            builder.uxth(r, r);
            break;
        }

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
            get8BitValue(builder, Reg::R1, regMap8[opcode & 7]);
            write8BitReg(builder, regMap8[(opcode >> 3) & 7], Reg::R1);
            break;
        case 0x46: // LD B,(HL)
        case 0x4E: // LD C,(HL)
        case 0x56: // LD D,(HL)
        case 0x5E: // LD E,(HL)
        case 0x66: // LD H,(HL)
        case 0x6E: // LD L,(HL)
        case 0x7E: // LD A,(HL)
            readMem(reg(WReg::HL).reg, regMap8[(opcode >> 3) & 7]);
            break;
        case 0x70: // LD (HL),B
        case 0x71: // LD (HL),C
        case 0x72: // LD (HL),D
        case 0x73: // LD (HL),E
        case 0x74: // LD (HL),H
        case 0x75: // LD (HL),L
        case 0x77: // LD (HL),A
            writeMem(reg(WReg::HL).reg, regMap8[opcode & 7]);
            break;

        case 0x80: // ADD A,B
        case 0x81: // ADD A,C
        case 0x82: // ADD A,D
        case 0x83: // ADD A,E
        case 0x84: // ADD A,H
        case 0x85: // ADD A,L
        case 0x87: // ADD A,A
            add(regMap8[opcode & 7]);
            break;

        case 0x88: // ADC A,B
        case 0x89: // ADC A,C
        case 0x8A: // ADC A,D
        case 0x8B: // ADC A,E
        case 0x8C: // ADC A,F
        case 0x8D: // ADC A,H
        case 0x8F: // ADC A,A
            addWithCarry(regMap8[opcode & 7]);
            break;

        case 0x90: // SUB B
        case 0x91: // SUB C
        case 0x92: // SUB D
        case 0x93: // SUB E
        case 0x94: // SUB H
        case 0x95: // SUB L
        case 0x97: // SUB A
            sub(regMap8[opcode & 7]);
            break;

        case 0x98: // SBC B
        case 0x99: // SBC C
        case 0x9A: // SBC D
        case 0x9B: // SBC E
        case 0x9C: // SBC H
        case 0x9D: // SBC L
        case 0x9F: // SBC A
            subWithCarry(regMap8[opcode & 7]);
            break;

        case 0xA0: // AND B
        case 0xA1: // AND C
        case 0xA2: // AND D
        case 0xA3: // AND E
        case 0xA4: // AND H
        case 0xA5: // AND L
        case 0xA7: // AND A
            bitAnd(regMap8[opcode & 7]);
            break;

        case 0xA8: // XOR B
        case 0xA9: // XOR C
        case 0xAA: // XOR D
        case 0xAB: // XOR E
        case 0xAC: // XOR H
        case 0xAD: // XOR L
            bitXor(regMap8[opcode & 7]);
            break;

        case 0xAF: // XOR A
            builder.mov(reg(WReg::AF).reg, DMGCPU::Flag_Z); // A = 0, F = Z
            break;

        case 0xB0: // OR B
        case 0xB1: // OR C
        case 0xB2: // OR D
        case 0xB3: // OR E
        case 0xB4: // OR H
        case 0xB5: // OR L
        case 0xB7: // OR A
            bitOr(regMap8[opcode & 7]);
            break;

        case 0xB8: // CP B
        case 0xB9: // CP C
        case 0xBA: // CP D
        case 0xBB: // CP E
        case 0xBC: // CP H
        case 0xBD: // CP L
        case 0xBF: // CP A
            cmp(regMap8[opcode & 7]);
            break;

        case 0xC6: // ADD n
        {
            cycleExecuted();
            add(instr.opcode[1]);
            break;
        }

        case 0xCE: // ADC n
        {
            cycleExecuted();
            addWithCarry(instr.opcode[1]);
            break;
        }

        case 0xD6: // SUB n
        {
            cycleExecuted();
            sub(instr.opcode[1]);
            break;
        }

        case 0xDE: // SBC n
        {
            cycleExecuted();
            subWithCarry(instr.opcode[1]);
            break;
        }

        case 0xE0: // LDH (n),A
        {
            uint16_t addr = 0xFF00 | instr.opcode[1];
            cycleExecuted();
            writeMem(addr, reg(DMGReg::A));
            break;
        }

        case 0xE2: // LDH (C),A
        {
            get8BitValue(builder, Reg::R1, reg(DMGReg::C));
            // C | 0xFF00
            builder.mov(Reg::R2, 0xFF);
            builder.lsl(Reg::R2, Reg::R2, 8);
            builder.orr(Reg::R1, Reg::R2);

            writeMem(Reg::R1, reg(DMGReg::A));
            break;
        }

        case 0xE6: // AND n
        {
            cycleExecuted();
            bitAnd(instr.opcode[1]);
            break;
        }

        case 0xEA: // LD (nn),A
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            writeMem(addr, reg(DMGReg::A));
            break;
        }

        case 0xEE: // XOR n
        {
            cycleExecuted();
            bitXor(instr.opcode[1]);
            break;
        }

        case 0xF0: // LDH A,(n)
        {
            uint16_t addr = 0xFF00 | instr.opcode[1];
            cycleExecuted();
            readMem(addr, reg(DMGReg::A));
            break;
        }

        case 0xF2: // LDH A,(C)
        {
            get8BitValue(builder, Reg::R1, reg(DMGReg::C));
            // C | 0xFF00
            builder.mov(Reg::R2, 0xFF);
            builder.lsl(Reg::R2, Reg::R2, 8);
            builder.orr(Reg::R1, Reg::R2);

            readMem(Reg::R1, reg(DMGReg::A));
            break;
        }

        case 0xF6: // OR n
        {
            cycleExecuted();
            bitOr(instr.opcode[1]);
            break;
        }

        case 0xFA: // LD A,(nn)
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            readMem(addr, reg(DMGReg::A));
            break;
        }

        case 0xFE: // CP n
        {
            cycleExecuted();
            cmp(instr.opcode[1]);
            break;
        }

        default:
            printf("unhandled op in recompile %02X\n", opcode);
            builder.resetPtr(oldPtr);
            load16BitValue(builder, Reg::R1, pc - instr.len);
            builder.bl(getOff(exitPtr));
            return false;
    }

    syncCyclesExecuted();

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
    builder.push(0b11111100, true); // R2-7, LR
    
    // set the low bit so we stay in thumb mode
    builder.mov(Reg::R2, 1);
    builder.orr(Reg::R1, Reg::R2);

    // load cpu pointer
    builder.ldr(Reg::R2, 40);
    builder.mov(Reg::R8, Reg::R2);

    // load emu regs
    int regsOff = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtr;
    builder.add(Reg::R2, regsOff); // add to cpu ptr
    builder.ldrh(Reg::R4, Reg::R2, 0);
    builder.ldrh(Reg::R5, Reg::R2, 2);
    builder.ldrh(Reg::R6, Reg::R2, 4);
    builder.ldrh(Reg::R7, Reg::R2, 6);

    builder.bx(Reg::R1);

    exitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    // exit
    // store PC
    int pcOff = reinterpret_cast<uintptr_t>(&cpu.pc) - cpuPtr;
    assert(pcOff <= 0xFF);
    builder.mov(Reg::R2, Reg::R8); // cpu ptr
    builder.mov(Reg::R0, pcOff);
    builder.strh(Reg::R1, Reg::R2, Reg::R0);

    // save emu regs
    builder.add(Reg::R2, regsOff); // add to cpu ptr
    builder.strh(Reg::R4, Reg::R2, 0);
    builder.strh(Reg::R5, Reg::R2, 2);
    builder.strh(Reg::R6, Reg::R2, 4);
    builder.strh(Reg::R7, Reg::R2, 6);

    // restore regs and return
    builder.pop(0b11111100, false); // R2-7
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
