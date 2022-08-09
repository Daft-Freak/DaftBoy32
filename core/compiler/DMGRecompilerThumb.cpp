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

auto spReg = Reg::R9;

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
        else if(dst != reg.reg) // assume it's already masked if it's already there
            builder.uxtb(dst, reg.reg); // clear the high half
    }
    else
        builder.mov(dst, std::get<uint8_t>(b));
}

// store to 8-bit reg, modifies src
static void  write8BitReg(ThumbBuilder &builder, RegInfo dst, Reg src, Reg tmp = Reg::R2)
{
    // do nothing if already in the right place
    if(dst.reg == src)
    {
        assert(dst.part == RegPart::Low);
        return;
    }

    // don't preserve the other half if the dest is not an emulated register
    if(static_cast<int>(dst.reg) < 4)
    {
        if(dst.part == RegPart::Low)
            builder.mov(dst.reg, src);
        else
            builder.lsl(dst.reg, src, 8);

        return;
    }

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

    bool checkInterrupts = false;

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
        {spReg, RegPart::Both}, // unless it's push/pop, then it's AF
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

        builder.ldr(Reg::R3, Reg::R1, cycleCountOff);
        builder.add(Reg::R3, u8Cycles);
        builder.str(Reg::R3, Reg::R1, cycleCountOff);

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

            syncCyclesExecuted(); // uses R1/3

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

            load16BitValue(builder, Reg::R1, immAddr, Reg::R3);
        }
    };

    auto readMem = [&builder, &cycleExecuted, &syncCyclesExecuted, &setupMemAddr](std::variant<Reg, uint16_t> addr, RegInfo dst, bool postInc = false, int extraSave = 0)
    {
        int pushMask = 1 << 0 | extraSave; // R0

        // save addr if we're post-incrementing it (assume it's in R1-3)
        if(postInc && std::holds_alternative<Reg>(addr))
            pushMask |= 1 << static_cast<int>(std::get<Reg>(addr));

        builder.push(pushMask, false);

        setupMemAddr(addr);

        builder.mov(Reg::R0, Reg::R8); // cpu pointer

        // get func ptr
        load32BitValue(builder, Reg::R2, reinterpret_cast<uintptr_t>(&DMGRecompiler::readMem));

        builder.blx(Reg::R2); // do call

        // move to dest
        write8BitReg(builder, dst, Reg::R0);


        builder.pop(pushMask, false);

        if(postInc && std::holds_alternative<Reg>(addr))
        {
            auto addrReg = std::get<Reg>(addr);
            builder.add(addrReg, 1);
            builder.uxth(addrReg, addrReg);
        }

        cycleExecuted();
    };

    auto writeMem = [&builder, &cycleExecuted, &syncCyclesExecuted, &setupMemAddr](std::variant<Reg, uint16_t> addr, std::variant<RegInfo, uint8_t> data, bool preDec = false)
    {
        int pushMask = 1 << 4; // R4
        if(preDec && std::holds_alternative<Reg>(addr))
        {
            auto addrReg = std::get<Reg>(addr);
            builder.sub(addrReg, 1);
            builder.uxth(addrReg, addrReg);

            // used for push, we don't want to lose the reg
            pushMask |= 1 << static_cast<int>(addrReg);
        }

        builder.push(pushMask, false);

        setupMemAddr(addr);
        get8BitValue(builder, Reg::R2, data);
        builder.mov(Reg::R3, Reg::R0); // cycle count

        builder.mov(Reg::R0, Reg::R8); // cpu pointer

        // get func ptr
        load32BitValue(builder, Reg::R4, reinterpret_cast<uintptr_t>(&DMGRecompiler::writeMem));

        builder.blx(Reg::R4); // do call

        // new cycle count is already in R0

        builder.pop(pushMask, false);

        cycleExecuted();
    };

    const auto push = [&builder, &cycleExecuted, &writeMem](Reg r)
    {
        cycleExecuted(); // delay
        builder.mov(Reg::R2, spReg);
        writeMem(Reg::R2, RegInfo{r, RegPart::High}, true);
        writeMem(Reg::R2, RegInfo{r, RegPart::Low}, true);
        builder.mov(spReg, Reg::R2);
    };

    const auto pop = [&builder, &readMem](Reg r)
    {
        builder.mov(Reg::R2, spReg);
        readMem(Reg::R2, RegInfo{r, RegPart::Low}, true);
        readMem(Reg::R2, RegInfo{r, RegPart::High}, true);
        builder.mov(spReg, Reg::R2);
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
            if(bIsReg && std::get<RegInfo>(b).reg == Reg::R2) // (HL), save the value
                builder.push(1 << 2, false);
    
            // room for optimisation here, probably
            builder.mov(Reg::R1, 0xF);
            get8BitValue(builder, Reg::R3, b);
            get8BitValue(builder, Reg::R2, a);

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

            if(bIsReg && std::get<RegInfo>(b).reg == Reg::R2)
                builder.pop(1 << 2, false);
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
            if(bIsReg && std::get<RegInfo>(b).reg == Reg::R2) // (HL), save the value
                builder.push(1 << 2, false);
    
            // room for optimisation here, probably
            builder.mov(Reg::R1, 0xF);
            get8BitValue(builder, Reg::R3, b);
            get8BitValue(builder, Reg::R2, a);

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

            if(bIsReg && std::get<RegInfo>(b).reg == Reg::R2)
                builder.pop(1 << 2, false);
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

    const auto inc = [&instr, &builder](RegInfo r)
    {
        auto regA = reg(WReg::AF).reg;

        if(instr.flags & Op_WriteFlags)
        {
            builder.mov(Reg::R1, DMGCPU::Flag_H | DMGCPU::Flag_N | DMGCPU::Flag_Z);
            builder.bic(regA, Reg::R1); // preserve C (and all of A), clear others
        }

        get8BitValue(builder, Reg::R1, r);

        if(instr.flags & DMGCPU::Flag_H)
        {
            // & 0xF == 0xF
            builder.mov(Reg::R2, 0xF);
            builder.and_(Reg::R2, Reg::R1);
            builder.cmp(Reg::R2, 0xF);

            builder.b(Condition::NE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_H);
            builder.orr(regA, Reg::R2);
        }

        // do inc
        builder.add(Reg::R1, 1);
        builder.uxtb(Reg::R1, Reg::R1);
    
        write8BitReg(builder, r, Reg::R1);

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.cmp(Reg::R1, 0);
            builder.b(Condition::NE, 4);
            builder.mov(Reg::R1, DMGCPU::Flag_Z);
            builder.orr(regA, Reg::R1);
        }
    };

    const auto dec = [&instr, &builder](RegInfo r)
    {
        auto regA = reg(WReg::AF).reg;

        if(instr.flags & Op_WriteFlags)
        {
            builder.mov(Reg::R1, DMGCPU::Flag_H | DMGCPU::Flag_N | DMGCPU::Flag_Z);
            builder.bic(regA, Reg::R1); // preserve C (and all of A), clear others
        }

        if(instr.flags & DMGCPU::Flag_N)
        {
            builder.mov(Reg::R1, DMGCPU::Flag_N);
            builder.orr(regA, Reg::R1);
        }

        get8BitValue(builder, Reg::R1, r);

        if(instr.flags & DMGCPU::Flag_H)
        {
            // & 0xF == 0
            builder.mov(Reg::R2, 0xF);
            builder.and_(Reg::R2, Reg::R1);

            builder.b(Condition::NE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_H);
            builder.orr(regA, Reg::R2);
        }

        // do dec
        builder.sub(Reg::R1, 1);
        builder.uxtb(Reg::R1, Reg::R1);
    
        write8BitReg(builder, r, Reg::R1);

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.cmp(Reg::R1, 0);
            builder.b(Condition::NE, 4);
            builder.mov(Reg::R1, DMGCPU::Flag_Z);
            builder.orr(regA, Reg::R1);
        }
    };

    const auto add16 = [&instr, &builder, &cycleExecuted](Reg b)
    {
        auto a = reg(WReg::HL);
        auto f = reg(WReg::AF);

        if(instr.flags & Op_WriteFlags)
        {
            // clear flags
            builder.mov(Reg::R1, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_N);
            builder.bic(f.reg, Reg::R1); // preserve Z (and all of A), clear others
        }

        if(instr.flags & DMGCPU::Flag_H)
        {
            if(b == Reg::R2) // SP, save the value
                builder.push(1 << 2, false);
    
            // room for optimisation here, probably
            // & 0xFFF == & ~(0xF << 12)
            builder.mov(Reg::R1, 0xF);
            builder.lsl(Reg::R1, Reg::R1, 12);
            builder.mov(Reg::R3, b);
            builder.mov(Reg::R2, a.reg);

            builder.bic(Reg::R2, Reg::R1);
            builder.bic(Reg::R3, Reg::R1);
            builder.add(Reg::R3, Reg::R2, Reg::R3);

            builder.lsr(Reg::R3, Reg::R3, 8); // ignore the bottom 8 bits

            if(b == Reg::R2)
                builder.pop(1 << 2, false);
        }

        // do add
        builder.add(a.reg, a.reg, b);

        // flags
        // cant use carry from op here
        if(instr.flags & DMGCPU::Flag_C)
        {
            // > 0xFFFF == >> 8 > 0xFF
            builder.lsr(Reg::R1, a.reg, 8); // shift down for comparison
            builder.cmp(Reg::R1, 0xFF);
            builder.b(Condition::LE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_C);
            builder.orr(f.reg, Reg::R2);
        }

        builder.uxth(a.reg, a.reg); // mask

        if(instr.flags & DMGCPU::Flag_H)
        {
            // half res > 0xF (shifted down 8)
            builder.cmp(Reg::R3, 0xF);
            builder.b(Condition::LE, 4);
            builder.mov(Reg::R2, DMGCPU::Flag_H);
            builder.orr(f.reg, Reg::R2);
        }

        cycleExecuted();
    };

    const auto doJump = [this, &instr, &pc, &builder, &getOff, &cycleExecuted, &syncCyclesExecuted, &cyclesThisInstr](uint16_t addr, int flag = 0, bool set = true)
    {
        auto it = branchTargets.find(addr);

        // condition
        uint16_t *branchPtr = nullptr;
        if(flag)
        {
            syncCyclesExecuted();

            builder.mov(Reg::R2, flag);
            builder.and_(Reg::R2, reg(DMGReg::F).reg); // tst?

            branchPtr = builder.getPtr();
            builder.b(set ? Condition::EQ : Condition::NE, 0);
        }

        cycleExecuted();
        syncCyclesExecuted();

        load16BitValue(builder, Reg::R1, addr); // might not be able to patch branch (if it's too far)

        // sub cycles early (we jump past the usual code that does this)
        if(instr.flags & Op_Branch)
            builder.sub(Reg::R0, cyclesThisInstr);
        //else // or set PC if we're just going to exit

        // don't update twice for unconditional branches (if it doesn't have the branch flag it's an exit anyway)
        if(!flag)
            cyclesThisInstr = 0;

        if(it != branchTargets.end())
        {
            int off = getOff(it->second);
            if(off >= -2044)
            {
                builder.b(getOff(it->second));
                builder.nop(); // B is shorter than BL
            }
            else
                builder.bl(getOff(exitPtr)); // branch is too far, give up
        }
        else
        {
            if(instr.flags & Op_Branch)
                forwardBranchesToPatch.emplace(addr, reinterpret_cast<uint8_t *>(builder.getPtr()));

            // will be patched later if possible
            builder.bl(getOff(exitPtr));
        }

        if(branchPtr)
        {
            // update branch
            int off = (builder.getPtr() - (branchPtr + 1));
            *branchPtr = (*branchPtr & 0xFF00) | off;
        }

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
        {
            load16BitValue(builder, Reg::R1, pc);
            builder.bl(getOff(exitPtr));
        }
    };

    const auto jump = [&instr, &cycleExecuted, &doJump](int flag = 0, bool set = true)
    {
        uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
        cycleExecuted();
        cycleExecuted();

        doJump(addr, flag, set);
    };

    const auto jumpRel = [&instr, &pc, &cycleExecuted, &doJump](int flag = 0, bool set = true)
    {
        int8_t off = instr.opcode[1];
        cycleExecuted();

        doJump(pc + off, flag, set);
    };

    const auto call = [this, &instr, &pc, &builder, &getOff, &cycleExecuted, &syncCyclesExecuted, &writeMem](int flag = 0, bool set = true)
    {
        uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
        cycleExecuted();
        cycleExecuted();

        // condition
        uint16_t *branchPtr = nullptr;
        if(flag)
        {
            syncCyclesExecuted();

            builder.mov(Reg::R2, flag);
            builder.and_(Reg::R2, reg(DMGReg::F).reg); // tst?

            branchPtr = builder.getPtr();
            builder.b(set ? Condition::EQ : Condition::NE, 0);
        }

        cycleExecuted(); // delay

        // push PC
        builder.mov(Reg::R2, spReg);
        writeMem(Reg::R2, static_cast<uint8_t>(pc >> 8), true);
        writeMem(Reg::R2, static_cast<uint8_t>(pc), true);
        builder.mov(spReg, Reg::R2);

        syncCyclesExecuted();

        load16BitValue(builder, Reg::R1, addr);

        // exit but flag as a call so we can save the return addr
        builder.bl(getOff(exitForCallPtr));

        if(branchPtr)
        {
            // update branch
            int off = (builder.getPtr() - (branchPtr + 1));
            *branchPtr = (*branchPtr & 0xFF00) | off;
        }

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
        {
            load16BitValue(builder, Reg::R1, pc);
            builder.bl(getOff(exitPtr));
        }
    };

    const auto reset = [this, &pc, &builder, &getOff, &cycleExecuted, &syncCyclesExecuted, &writeMem](int addr)
    {
        cycleExecuted(); // delay

        // push PC
        builder.mov(Reg::R2, spReg);
        writeMem(Reg::R2, static_cast<uint8_t>(pc >> 8), true);
        writeMem(Reg::R2, static_cast<uint8_t>(pc), true);
        builder.mov(spReg, Reg::R2);

        // exit
        syncCyclesExecuted();
        load16BitValue(builder, Reg::R1, addr);
        builder.bl(getOff(exitPtr));
    };

    const auto ret = [this, &instr, &pc, &builder, &getOff, &cycleExecuted, &syncCyclesExecuted, &readMem](int flag = 0, bool set = true)
    {
        // condition
        uint16_t *branchPtr = nullptr;
        if(flag)
        {
            cycleExecuted(); // delay
            syncCyclesExecuted();

            builder.mov(Reg::R2, flag);
            builder.and_(Reg::R2, reg(DMGReg::F).reg); // tst?

            branchPtr = builder.getPtr();
            builder.b(set ? Condition::EQ : Condition::NE, 0);
        }

        // pop PC
        builder.mov(Reg::R1, spReg);
        readMem(Reg::R1, {Reg::R2, RegPart::Low}, true);
        readMem(Reg::R1, {Reg::R3, RegPart::High}, true, 1 << 2/*save R2*/);
        builder.orr(Reg::R2, Reg::R3);
        builder.mov(spReg, Reg::R1);

        cycleExecuted();
        syncCyclesExecuted(); // uses R1/3

        builder.mov(Reg::R1, Reg::R2); // move popped PC value
        builder.bl(getOff(exitPtr));

        if(branchPtr)
        {
            // update branch
            int off = (builder.getPtr() - (branchPtr + 1));
            *branchPtr = (*branchPtr & 0xFF00) | off;
        }

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
        {
            load16BitValue(builder, Reg::R1, pc);
            builder.bl(getOff(exitPtr));
        }
    };

    // handle branch targets
    if(instr.flags & Op_BranchTarget)
    {
        // store for backwards jumps
        if(lastInstrCycleCheck)
            branchTargets.emplace(pc, lastInstrCycleCheck); // after adjusting cycle count but before the jump
        else
        {
            // this is the first instruction, so make a cycle check for the branch to go to
            builder.b(14);
            lastInstrCycleCheck = reinterpret_cast<uint8_t *>(builder.getPtr());

            // if <= 0 exit
            builder.b(Condition::GT, 12);
            load16BitValue(builder, Reg::R1, pc); // currently always 4 instructions
            builder.bl(getOff(saveAndExitPtr));

            branchTargets.emplace(pc, lastInstrCycleCheck);
        }

        // patch forwards jumps
        // can't hit this for the first instruction, so the lastInstrCycleCheck will be valid
        auto jumps = forwardBranchesToPatch.equal_range(pc);
        for(auto it = jumps.first; it != jumps.second; ++it)
        {
            auto off = lastInstrCycleCheck - (it->second + 2);

            auto ptr = reinterpret_cast<uint16_t *>(it->second);

            // replace BL with B+NOP
            // TODO: need better branch patching
            if(off <= 2048)
            {
                *ptr++ = 0xE000 | (((off - 2) >> 1) & 0x7FF);
                *ptr++ = 0xBF00;
            }
        }
        forwardBranchesToPatch.erase(jumps.first, jumps.second);
    }

    pc += instr.len;

    auto oldPtr = builder.getPtr();
    cycleExecuted();

    // previous op was EI
    if(mem.read(pc - (instr.len + 1)) == 0xFB /*EI*/)
    {
        // enable interrupts for EI
        auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

        auto enableInterruptsNextCycleOff = reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle) - cpuPtr;
        auto masterInterruptEnableOff = reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtr;
        assert(enableInterruptsNextCycleOff < 32);
        assert(masterInterruptEnableOff < 32);        

        // if(enableInterruptsNextCycle)
        // probably don't need this check... might get a false positive in some extreme case though
        builder.mov(Reg::R2, Reg::R8); // cpu ptr
        builder.ldrb(Reg::R1, Reg::R2, enableInterruptsNextCycleOff);

        builder.cmp(Reg::R1, 0);
        builder.b(Condition::EQ, 6);

        // masterInterruptEnable = true
        builder.strb(Reg::R1, Reg::R2, masterInterruptEnableOff); // R1 == 1 here

        // enableInterruptsNextCycle = false
        builder.mov(Reg::R1, 0);
        builder.strb(Reg::R1, Reg::R2, enableInterruptsNextCycleOff);

        checkInterrupts = true;
    }

    switch(opcode)
    {
        case 0x00: // NOP
            break;

        case 0x01: // LD BC,nn
        case 0x11: // LD DE,nn
        case 0x21: // LD HL,nn
        {
            cycleExecuted();
            cycleExecuted();

            auto dst = regMap16[opcode >> 4];
            load16BitValue(builder, dst.reg, instr.opcode[1] | instr.opcode[2] << 8);

            break;
        }
        case 0x31: // LD SP,nn
        {
            cycleExecuted();
            cycleExecuted();

            load16BitValue(builder, Reg::R1, instr.opcode[1] | instr.opcode[2] << 8);
            builder.mov(spReg, Reg::R1);

            break;
        }

        case 0x02: // LD (BC),A
        case 0x12: // LD (DE),A
            writeMem(regMap16[opcode >> 4].reg, reg(DMGReg::A));
            break;

        case 0x03: // INC BC
        case 0x13: // INC DE
        case 0x23: // INC HL
        {
            auto r = regMap16[opcode >> 4].reg;
            builder.add(r, 1);
            builder.uxth(r, r);
            cycleExecuted();
            break;
        }
        case 0x33: // INC SP
        {
            auto r = Reg::R1;
            builder.mov(r, spReg);
            builder.add(r, 1);
            builder.uxth(r, r);
            builder.mov(spReg, r);
            cycleExecuted();
            break;
        }

        case 0x04: // INC B
        case 0x0C: // INC C
        case 0x14: // INC D
        case 0x1C: // INC E
        case 0x24: // INC H
        case 0x2C: // INC L
        case 0x3C: // INC A
            inc(regMap8[opcode >> 3]);
            break;

        case 0x05: // DEC B
        case 0x0D: // DEC C
        case 0x15: // DEC D
        case 0x1D: // DEC E
        case 0x25: // DEC H
        case 0x2D: // DEC L
        case 0x3D: // DEC A
            dec(regMap8[opcode >> 3]);
            break;

        case 0x0B: // DEC BC
        case 0x1B: // DEC DE
        case 0x2B: // DEC HL
        {
            auto r = regMap16[opcode >> 4].reg;
            builder.sub(r, 1);
            builder.uxth(r, r);
            cycleExecuted();
            break;
        }
        case 0x3B: // DEC SP
        {
            auto r = Reg::R1;
            builder.mov(r, spReg);
            builder.sub(r, 1);
            builder.uxth(r, r);
            builder.mov(spReg, r);
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

        case 0x07: // RLCA
        {
            auto a = reg(DMGReg::A).reg;
            builder.lsr(Reg::R1, a, 8); // shift out flags

            // = << 1 | >> 7
            builder.lsr(Reg::R2, Reg::R1, 7);
            builder.lsl(Reg::R1, Reg::R1, 1);
            builder.orr(Reg::R1, Reg::R2);

            if(instr.flags & Op_WriteFlags)
            {
                builder.mov(a, 0);
                builder.lsl(Reg::R2, Reg::R1, 24); // shift it again to set carry
                builder.b(Condition::CC, 2);
                builder.mov(a, DMGCPU::Flag_C);
            }

            builder.uxtb(Reg::R1, Reg::R1);

            if(instr.flags & Op_WriteFlags)
            {
                builder.lsl(Reg::R1, Reg::R1, 8);
                builder.orr(a, Reg::R1); // keep the flag
            }
            else
                builder.lsl(a, Reg::R1, 8);

            break;
        }

        case 0x08: // LD (nn),SP
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            builder.mov(Reg::R2, spReg);
            writeMem(addr++, RegInfo{Reg::R2, RegPart::Low});
            builder.mov(Reg::R2, spReg);
            writeMem(addr, RegInfo{Reg::R2, RegPart::High});
            break;
        }

        case 0x09: // ADD HL,BC
        case 0x19: // ADD HL,DE
        case 0x29: // ADD HL,HL
            add16(regMap16[opcode >> 4].reg);
            break;
        case 0x39: // ADD HL,SP
            builder.mov(Reg::R2, spReg);
            add16(Reg::R2);
            break;

        case 0x0A: // LD A,(BC)
        case 0x1A: // LD A,(DE)
            readMem(regMap16[opcode >> 4].reg, reg(DMGReg::A));
            break;

        case 0x0F: // RRCA
        {
            auto a = reg(DMGReg::A).reg;
            builder.lsr(Reg::R1, a, 8); // shift out flags

            // = << 7 | >> 1
            builder.lsl(Reg::R2, Reg::R1, 7);
            builder.lsr(Reg::R1, Reg::R1, 1);
            builder.orr(Reg::R1, Reg::R2);

            if(instr.flags & Op_WriteFlags)
            {
                builder.mov(a, 0);
                builder.lsl(Reg::R2, Reg::R1, 25); // shift it again to set carry
                builder.b(Condition::CC, 2);
                builder.mov(a, DMGCPU::Flag_C);
            }

            builder.uxtb(Reg::R1, Reg::R1);

            if(instr.flags & Op_WriteFlags)
            {
                builder.lsl(Reg::R1, Reg::R1, 8);
                builder.orr(a, Reg::R1); // keep the flag
            }
            else
                builder.lsl(a, Reg::R1, 8);

            break;
        }

        case 0x17: // RLA
        {
            auto a = reg(DMGReg::A).reg;
            builder.lsr(Reg::R1, a, 8); // shift out flags

            // <<= 1
            builder.lsl(Reg::R1, Reg::R1, 1);

            // copy carry
            builder.mov(Reg::R2, DMGCPU::Flag_C);
            builder.and_(Reg::R2, a);
            builder.lsr(Reg::R2, Reg::R2, 4); // C is bit 4
            builder.orr(Reg::R1, Reg::R2);

            if(instr.flags & Op_WriteFlags)
            {
                builder.mov(a, 0);
                builder.lsl(Reg::R2, Reg::R1, 24); // shift it again to set carry
                builder.b(Condition::CC, 2);
                builder.mov(a, DMGCPU::Flag_C);
            }

            builder.uxtb(Reg::R1, Reg::R1);

            if(instr.flags & Op_WriteFlags)
            {
                builder.lsl(Reg::R1, Reg::R1, 8);
                builder.orr(a, Reg::R1); // keep the flag
            }
            else
                builder.lsl(a, Reg::R1, 8);

            break;
        }

        case 0x18: // JR m
            jumpRel();
            break;

        case 0x1F: // RRA
        {
            auto a = reg(DMGReg::A).reg;

            // get the carry flag
            builder.mov(Reg::R2, DMGCPU::Flag_C);
            builder.and_(Reg::R2, a);

            builder.lsr(Reg::R1, a, 9); // shift out flags and left 1

            if(instr.flags & Op_WriteFlags)
            {
                // can actually use the carry flag here
                builder.mov(a, 0);
                builder.b(Condition::CC, 2);
                builder.mov(a, DMGCPU::Flag_C);
            }

            // copy carry in
            builder.lsl(Reg::R2, Reg::R2, 3); // C is bit 4
            builder.orr(Reg::R1, Reg::R2);

            if(instr.flags & Op_WriteFlags)
            {
                builder.lsl(Reg::R1, Reg::R1, 8);
                builder.orr(a, Reg::R1); // keep the flag
            }
            else
                builder.lsl(a, Reg::R1, 8);

            break;
        }

        case 0x20: // JR NZ,n
        case 0x28: // JR Z,n
        case 0x30: // JR NC,n
        case 0x38: // JR C,n
        {
            int flag = opcode & (1 << 4) ? DMGCPU::Flag_C : DMGCPU::Flag_Z;
            jumpRel(flag, opcode & (1 << 3));
            break;
        }

        case 0x22: // LDI (HL),A
        {
            auto r = reg(WReg::HL).reg;
            writeMem(r, reg(DMGReg::A));
            builder.add(r, 1);
            builder.uxth(r, r);
            break;
        }

        case 0x27: // DAA
        {
            auto f = reg(DMGReg::F).reg;
            auto a = Reg::R2;

            builder.lsr(a, f, 8); // get A

            // clear Z flag (will add back later)
            builder.mov(Reg::R1, DMGCPU::Flag_Z);
            builder.bic(f, Reg::R1);

            builder.mov(Reg::R1, DMGCPU::Flag_N);
            builder.and_(Reg::R1, f); // tst?
            builder.b(Condition::EQ, 18); // N not set

            // negative

            // if (Flag_C) A -= 0x60
            builder.mov(Reg::R1, DMGCPU::Flag_C);
            builder.and_(Reg::R1, f); // tst?
            builder.b(Condition::EQ, 2); // C not set
            builder.sub(a, 0x60);

            // if (Flag_H) A -= 0x06
            builder.mov(Reg::R1, DMGCPU::Flag_H);
            builder.and_(Reg::R1, f); // tst?
            builder.b(Condition::EQ, 2); // C not set
            builder.sub(a, 0x06);

            builder.b(32); // skip

            // positive

            // if (Flag_C ...
            builder.mov(Reg::R1, DMGCPU::Flag_C);
            builder.and_(Reg::R1, f); // tst?
            builder.b(Condition::NE, 4); // C set
            // ... || A > 0x99) ...
            builder.cmp(a, 0x99);
            builder.b(Condition::LE, 6);
            // ... A += 0x60
            builder.add(a, 0x60);
            builder.mov(Reg::R1, DMGCPU::Flag_C);
            builder.orr(f, Reg::R1); // set C flag

            // if (Flag_H ...
            builder.mov(Reg::R1, DMGCPU::Flag_H);
            builder.and_(Reg::R1, f); // tst?
            builder.b(Condition::NE, 8); // H set
            // ... || A & (0x0F) > 0x09) ...
            builder.mov(Reg::R1, 0xF);
            builder.and_(Reg::R1, a);
            builder.cmp(Reg::R1, 0x09);
            builder.b(Condition::LE, 2);
            // .. A += 0x06
            builder.add(a, 0x06);

            builder.uxtb(a, a);

            // Z flag
            builder.cmp(a, 0);
            builder.b(Condition::NE, 4);
            builder.mov(Reg::R1, DMGCPU::Flag_Z);
            builder.orr(f, Reg::R1);

            // clear H flag
            builder.mov(Reg::R1, DMGCPU::Flag_H);
            builder.bic(f, Reg::R1);

            // write A back
            write8BitReg(builder, reg(DMGReg::A), Reg::R2, Reg::R1);

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

        case 0x2F: // CPL
            // A is the high byte so AF ^ 0xFF00
            builder.mov(Reg::R1, 0xFF);
            builder.lsl(Reg::R1, Reg::R1, 8);
            builder.eor(reg(DMGReg::A).reg, Reg::R1);

            if(instr.flags & Op_WriteFlags)
            {
                // set H and N
                builder.mov(Reg::R1, DMGCPU::Flag_H | DMGCPU::Flag_N);
                builder.orr(reg(DMGReg::A).reg, Reg::R1);
            }
            break;

        case 0x32: // LDD (HL),A
        {
            auto r = reg(WReg::HL).reg;
            writeMem(r, reg(DMGReg::A));
            builder.sub(r, 1);
            builder.uxth(r, r);
            break;
        }

        case 0x34: // INC (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};

            readMem(reg(WReg::HL).reg, tmp);
            inc(tmp);
            writeMem(reg(WReg::HL).reg, tmp);

            break;
        }
        case 0x35: // DEC (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};

            readMem(reg(WReg::HL).reg, tmp);
            dec(tmp);
            writeMem(reg(WReg::HL).reg, tmp);

            break;
        }

        case 0x36: // LD (HL),n
        {
            cycleExecuted();
            writeMem(reg(WReg::HL).reg, instr.opcode[1]);
            break;
        }

        case 0x37: // SCF
            builder.mov(Reg::R1, DMGCPU::Flag_H | DMGCPU::Flag_N); // clear H/N
            builder.bic(reg(DMGReg::A).reg, Reg::R1);
            builder.mov(Reg::R1, DMGCPU::Flag_C);
            builder.orr(reg(DMGReg::A).reg, Reg::R1);
            break;

        case 0x3A: // LDD A, (HL)
        {
            auto r = reg(WReg::HL).reg;
            readMem(r, reg(DMGReg::A));
            builder.sub(r, 1);
            builder.uxth(r, r);
            break;
        }

        case 0x3F: // CCF
            builder.mov(Reg::R1, DMGCPU::Flag_H | DMGCPU::Flag_N); // clear H/N
            builder.bic(reg(DMGReg::A).reg, Reg::R1);
            builder.mov(Reg::R1, DMGCPU::Flag_C);
            builder.eor(reg(DMGReg::A).reg, Reg::R1);
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

        case 0x76: // HALT
        {
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
            auto haltedOff = reinterpret_cast<uintptr_t>(&cpu.halted) - cpuPtr;
            auto masterInterruptEnableOff = reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtr;
            auto serviceableInterruptsOff = reinterpret_cast<uintptr_t>(&cpu.serviceableInterrupts) - cpuPtr;
            auto haltBugOff = reinterpret_cast<uintptr_t>(&cpu.haltBug) - cpuPtr;
            assert(haltedOff < 32);
            assert(masterInterruptEnableOff < 32);
            assert(serviceableInterruptsOff < 32);
            assert(haltBugOff < 32);

            builder.mov(Reg::R2, Reg::R8); // cpu ptr

            // halted = true
            builder.mov(Reg::R1, 1);
            builder.strb(Reg::R1, Reg::R2, haltedOff);

            // if(!masterInterruptEnable && serviceableInterrupts)
            builder.ldrb(Reg::R1, Reg::R2, masterInterruptEnableOff);
            builder.cmp(Reg::R1, 0);
            builder.b(Condition::NE, 10);
            builder.ldrb(Reg::R1, Reg::R2, serviceableInterruptsOff);
            builder.cmp(Reg::R1, 0);
            builder.b(Condition::EQ, 4);
            // haltBug = true
            builder.mov(Reg::R1, 1);
            builder.strb(Reg::R1, Reg::R2, haltBugOff);

            // exit
            syncCyclesExecuted();
            load16BitValue(builder, Reg::R1, pc); // exits need to set PC themselves
            builder.bl(getOff(saveAndExitPtr));
            break;
        }

        case 0x80: // ADD A,B
        case 0x81: // ADD A,C
        case 0x82: // ADD A,D
        case 0x83: // ADD A,E
        case 0x84: // ADD A,H
        case 0x85: // ADD A,L
        case 0x87: // ADD A,A
            add(regMap8[opcode & 7]);
            break;
        case 0x86: // ADD (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};
            readMem(reg(WReg::HL).reg, tmp);
            add(tmp);
            break;
        }

        case 0x88: // ADC A,B
        case 0x89: // ADC A,C
        case 0x8A: // ADC A,D
        case 0x8B: // ADC A,E
        case 0x8C: // ADC A,F
        case 0x8D: // ADC A,H
        case 0x8F: // ADC A,A
            addWithCarry(regMap8[opcode & 7]);
            break;
        case 0x8E: // ADC (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};
            readMem(reg(WReg::HL).reg, tmp);
            addWithCarry(tmp);
            break;
        }

        case 0x90: // SUB B
        case 0x91: // SUB C
        case 0x92: // SUB D
        case 0x93: // SUB E
        case 0x94: // SUB H
        case 0x95: // SUB L
        case 0x97: // SUB A
            sub(regMap8[opcode & 7]);
            break;
        case 0x96: // SUB (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};
            readMem(reg(WReg::HL).reg, tmp);
            sub(tmp);
            break;
        }

        case 0x98: // SBC B
        case 0x99: // SBC C
        case 0x9A: // SBC D
        case 0x9B: // SBC E
        case 0x9C: // SBC H
        case 0x9D: // SBC L
        case 0x9F: // SBC A
            subWithCarry(regMap8[opcode & 7]);
            break;
        case 0x9E: // SBC (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};
            readMem(reg(WReg::HL).reg, tmp);
            subWithCarry(tmp);
            break;
        }

        case 0xA0: // AND B
        case 0xA1: // AND C
        case 0xA2: // AND D
        case 0xA3: // AND E
        case 0xA4: // AND H
        case 0xA5: // AND L
        case 0xA7: // AND A
            bitAnd(regMap8[opcode & 7]);
            break;
        case 0xA6: // AND (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};
            readMem(reg(WReg::HL).reg, tmp);
            bitAnd(tmp);
            break;
        }

        case 0xA8: // XOR B
        case 0xA9: // XOR C
        case 0xAA: // XOR D
        case 0xAB: // XOR E
        case 0xAC: // XOR H
        case 0xAD: // XOR L
            bitXor(regMap8[opcode & 7]);
            break;
        case 0xAE: // XOR (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};
            readMem(reg(WReg::HL).reg, tmp);
            bitXor(tmp);
            break;
        }
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
        case 0xB6: // OR (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};
            readMem(reg(WReg::HL).reg, tmp);
            bitOr(tmp);
            break;
        }

        case 0xB8: // CP B
        case 0xB9: // CP C
        case 0xBA: // CP D
        case 0xBB: // CP E
        case 0xBC: // CP H
        case 0xBD: // CP L
        case 0xBF: // CP A
            cmp(regMap8[opcode & 7]);
            break;
        case 0xBE: // CP (HL)
        {
            auto tmp = RegInfo{Reg::R2, RegPart::Low};
            readMem(reg(WReg::HL).reg, tmp);
            cmp(tmp);
            break;
        }

        case 0xC0: // RET NZ
        case 0xC8: // RET Z
        case 0xD0: // RET NC
        case 0xD8: // RET C
        {
            int flag = opcode & (1 << 4) ? DMGCPU::Flag_C : DMGCPU::Flag_Z;
            ret(flag, opcode & (1 << 3));
            break;
        }

        case 0xC1: // POP BC
        case 0xD1: // POP DE
        case 0xE1: // POP HL
            pop(regMap16[(opcode >> 4) & 3].reg);
            break;

        case 0xC2: // JP NZ,nn
        case 0xCA: // JP Z,nn
        case 0xDA: // JP C,nn
        case 0xD2: // JP NC,nn
        {
            int flag = opcode & (1 << 4) ? DMGCPU::Flag_C : DMGCPU::Flag_Z;
            jump(flag, opcode & (1 << 3));
            break;
        }
        case 0xC3: // JP nn
            jump();
            break;

        case 0xC4: // CALL NZ,nn
        case 0xCC: // CALL Z,nn
        case 0xD4: // CALL NC,nn
        case 0xDC: // CALL C,nn
        {
            int flag = opcode & (1 << 4) ? DMGCPU::Flag_C : DMGCPU::Flag_Z;
            call(flag, opcode & (1 << 3));
            break;
        }

        case 0xC5: // PUSH BC
        case 0xD5: // PUSH DE
        case 0xE5: // PUSH HL
            push(regMap16[(opcode >> 4) & 3].reg);
            break;

        case 0xC6: // ADD n
        {
            cycleExecuted();
            add(instr.opcode[1]);
            break;
        }

        case 0xC7: // RST 0
        case 0xCF: // RST 08
        case 0xD7: // RST 10
        case 0xDF: // RST 18
        case 0xE7: // RST 20
        case 0xEF: // RST 28
        case 0xF7: // RST 30
        case 0xFF: // RST 38
            reset(opcode & 0x38);
            break;

        case 0xC9: // RET
            ret();
            break;

        case 0xCB:
        {
            uint8_t exOpcode = instr.opcode[1];

            cycleExecuted();

            bool isMem = (exOpcode & 7) == 6; // (HL)
            auto tmp = RegInfo{Reg::R2, RegPart::Low};

            if(isMem)
                readMem(reg(WReg::HL).reg, tmp);

            recompileExInstruction(instr, builder);

            if(isMem && (exOpcode >= 0x80 || exOpcode < 0x40)) // BIT doesn't write back
                writeMem(reg(WReg::HL).reg, tmp);

            break;
        }

        case 0xCD: // CALL nn
            call();
            break;

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

        case 0xD9: // RETI
        {
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
            auto masterInterruptEnableOff = reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtr;
            assert(masterInterruptEnableOff < 32);

            // masterInterruptEnable = true
            builder.mov(Reg::R2, Reg::R8); // cpu ptr
            builder.mov(Reg::R1, 1);
            builder.strb(Reg::R1, Reg::R2, masterInterruptEnableOff);

            ret();
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

        case 0xE9: // JP (HL)
            syncCyclesExecuted();
            builder.mov(Reg::R1, reg(WReg::HL).reg);
            builder.bl(getOff(exitPtr));
            break;

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

        case 0xF1: // POP AF
            pop(reg(WReg::AF).reg);

            // low bits in F can never be set
            builder.mov(Reg::R1, 0xF);
            builder.bic(reg(WReg::AF).reg, Reg::R1);
            break;

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

        case 0xF3: // DI
        {
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
            auto masterInterruptEnableOff = reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtr;
            assert(masterInterruptEnableOff < 32);

            // masterInterruptEnable = false
            // TODO: after next instruction (DMGCPU also has this TODO)
            builder.mov(Reg::R2, Reg::R8); // cpu ptr
            builder.mov(Reg::R1, 0);
            builder.strb(Reg::R1, Reg::R2, masterInterruptEnableOff);
            break;
        }

        case 0xF5: // PUSH AF
            push(reg(WReg::AF).reg);
            break;

        case 0xF6: // OR n
        {
            cycleExecuted();
            bitOr(instr.opcode[1]);
            break;
        }

        case 0xF9: // LD SP,HL
            cycleExecuted();
            builder.mov(spReg, reg(WReg::HL).reg);
            break;

        case 0xFA: // LD A,(nn)
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            readMem(addr, reg(DMGReg::A));
            break;
        }

        case 0xFB: // EI
        {
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
            auto enableInterruptsNextCycleOff = reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle) - cpuPtr;
            assert(enableInterruptsNextCycleOff < 32);

            // enableInterruptsNextCycle = true
            builder.mov(Reg::R2, Reg::R8); // cpu ptr
            builder.mov(Reg::R1, 1);
            builder.strb(Reg::R1, Reg::R2, enableInterruptsNextCycleOff);
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

    if(!(instr.flags & Op_Last)) // TODO: also safe to omit if there's an unconditional exit
    {
        // cycles -= executed
        if(cyclesThisInstr) // 0 means we already did the sub
            builder.sub(Reg::R0, cyclesThisInstr);

        lastInstrCycleCheck = reinterpret_cast<uint8_t *>(builder.getPtr()); // save in case the next instr is a branch target

        // if <= 0 exit
        builder.b(Condition::GT, 12);
        load16BitValue(builder, Reg::R1, pc); // currently always 4 instructions
        builder.bl(getOff(saveAndExitPtr));

        // interrupt check after EI
        if(checkInterrupts)
        {
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
            auto serviceableInterruptsOff = reinterpret_cast<uintptr_t>(&cpu.serviceableInterrupts) - cpuPtr;
            assert(serviceableInterruptsOff < 32);

            // if servicableInterrupts != 0
            builder.mov(Reg::R2, Reg::R8); // cpu ptr
            builder.ldrb(Reg::R1, Reg::R2, serviceableInterruptsOff);
            builder.cmp(Reg::R1, 0);
            builder.b(Condition::EQ, 12);

            // exit
            load16BitValue(builder, Reg::R1, pc); 
            builder.bl(getOff(saveAndExitPtr));
        }
    }

    return true;
}

void DMGRecompilerThumb::recompileExInstruction(OpInfo &instr, ThumbBuilder &builder)
{
    uint8_t opcode = instr.opcode[1];

    using DMGReg = DMGCPU::Reg;

    static const RegInfo regMap8[]
    {
        reg(DMGReg::B),
        reg(DMGReg::C),
        reg(DMGReg::D),
        reg(DMGReg::E),
        reg(DMGReg::H),
        reg(DMGReg::L),
        {Reg::R2, RegPart::Low}, // where we load to for (HL)
        reg(DMGReg::A),
    };

    // flag helpers, all use R1
    auto clearFlags = [&instr, &builder](int preserve = 0)
    {
        if(instr.flags & Op_WriteFlags)
        {
            builder.mov(Reg::R1, ~preserve);
            builder.bic(reg(DMGReg::F).reg, Reg::R1); // clear flags
        }
    };

    // assumes value in R2 if needCompare is true
    auto updateC = [&instr, &builder](bool needCompare = false)
    {
        if(instr.flags & DMGCPU::Flag_C)
        {
            if(needCompare)
            {
                builder.cmp(Reg::R2, 0xFF);
                builder.b(Condition::LE, 4);
            }
            else
                builder.b(Condition::CC, 4);

            builder.mov(Reg::R1, DMGCPU::Flag_C);
            builder.orr(reg(DMGReg::F).reg, Reg::R1);
        }
    };

    // assumes value in R2 if needCompare is true
    auto updateZ = [&instr, &builder](bool needCompare = true)
    {
        if(instr.flags & DMGCPU::Flag_Z)
        {
            if(needCompare)
                builder.cmp(Reg::R2, 0);

            builder.b(Condition::NE, 4);
            builder.mov(Reg::R1, DMGCPU::Flag_Z);
            builder.orr(reg(DMGReg::F).reg, Reg::R1);
        }
    };

    auto r = regMap8[opcode & 7];

    if(opcode < 0x40) // shifts/rotates
    {
        switch(opcode & ~7)
        {
            case 0x00: // RLC
            {
                get8BitValue(builder, Reg::R2, r);

                clearFlags();

                // = << 1 | >> 7
                builder.lsr(Reg::R1, Reg::R2, 7);
                builder.lsl(Reg::R2, Reg::R2, 1);
                builder.orr(Reg::R2, Reg::R1);

                // flags
                updateC(true);

                builder.uxtb(Reg::R2, Reg::R2);
       
                updateZ();

                write8BitReg(builder, r, Reg::R2, Reg::R1);
                break;
            }

            case 0x08: // RRC
            {
                get8BitValue(builder, Reg::R2, r);

                clearFlags();

                // = << 7 | >> 1
                builder.lsl(Reg::R3, Reg::R2, 7);
                builder.lsr(Reg::R2, Reg::R2, 1);

                // flags
                // set C before the or
                updateC();

                builder.orr(Reg::R2, Reg::R3);
                builder.uxtb(Reg::R2, Reg::R2);
       
                updateZ();

                write8BitReg(builder, r, Reg::R2, Reg::R1);
                break;
            }

            case 0x10: // RL
            {
                auto f = reg(DMGReg::F).reg;
                get8BitValue(builder, Reg::R2, r);

                // get the carry flag
                builder.mov(Reg::R3, DMGCPU::Flag_C);
                builder.and_(Reg::R3, f);

                clearFlags();

                // <<= 1 
                builder.lsl(Reg::R2, Reg::R2, 1);

                // copy carry in
                builder.lsr(Reg::R3, Reg::R3, 4); // C is bit 4
                builder.orr(Reg::R2, Reg::R3);

                // flags
                updateC(true);

                builder.uxtb(Reg::R2, Reg::R2);
       
                updateZ();

                write8BitReg(builder, r, Reg::R2, Reg::R1);
                break;
            }

            case 0x18: // RR
            {
                auto f = reg(DMGReg::F).reg;
                get8BitValue(builder, Reg::R2, r);

                // get the carry flag
                builder.mov(Reg::R3, DMGCPU::Flag_C);
                builder.and_(Reg::R3, f);

                clearFlags();

                // >>= 1 
                builder.lsr(Reg::R2, Reg::R2, 1);

                // flags
                updateC();

                // copy carry in
                builder.lsl(Reg::R3, Reg::R3, 3); // C is bit 4
                builder.orr(Reg::R2, Reg::R3);

                updateZ();

                write8BitReg(builder, r, Reg::R2, Reg::R1);
                break;
            }

            case 0x20: // SLA
            {
                get8BitValue(builder, Reg::R2, r);

                clearFlags();

                builder.lsl(Reg::R2, Reg::R2, 1);

                // flags
                updateC(true);

                builder.uxtb(Reg::R2, Reg::R2);
       
                updateZ();

                write8BitReg(builder, r, Reg::R2, Reg::R1);
                break;
            }

            case 0x28: // SRA
            {
                get8BitValue(builder, Reg::R2, r);

                clearFlags();

                builder.sxtb(Reg::R2, Reg::R2); // TODO: can avoid this by replacing get8BitValue(lsr/uxtb) with asr/sxtb
                builder.asr(Reg::R2, Reg::R2, 1);

                // flags
                updateC();

                builder.uxtb(Reg::R2, Reg::R2);
       
                updateZ();

                write8BitReg(builder, r, Reg::R2, Reg::R1);
                break;
            }

            case 0x30: // SWAP
            {
                get8BitValue(builder, Reg::R2, r);

                clearFlags();

                builder.lsr(Reg::R1, Reg::R2, 4); // >> 4
                builder.lsl(Reg::R2, Reg::R2, 4); // << 4
                builder.orr(Reg::R2, Reg::R1);

                builder.uxtb(Reg::R2, Reg::R2);

                updateZ(false);

                write8BitReg(builder, r, Reg::R2, Reg::R1);
                break;
            }

            case 0x38: // SRL
            {
                get8BitValue(builder, Reg::R2, r);

                clearFlags();

                builder.lsr(Reg::R2, Reg::R2, 1);

                // flags
                updateC();
                updateZ();

                write8BitReg(builder, r, Reg::R2, Reg::R1);
                break;
            }
        }
    }
    else if(opcode < 0x80) // BIT
    {
        int bit = (opcode >> 3) & 7;
        auto f = reg(DMGReg::F).reg;

        clearFlags(DMGCPU::Flag_C);

        if(instr.flags & DMGCPU::Flag_H)
        {
            builder.mov(Reg::R1, DMGCPU::Flag_H);
            builder.orr(f, Reg::R1); // set H
        }

        get8BitValue(builder, Reg::R2, r);

        builder.mov(Reg::R1, 1 << bit);
        builder.and_(Reg::R1, Reg::R2); // could use TST

        updateZ(false);
    }
    else if(opcode < 0xC0) // RES
    {
        int bit = (opcode >> 3) & 7;
        get8BitValue(builder, Reg::R2, r);

        builder.mov(Reg::R1, 1 << bit);
        builder.bic(Reg::R2, Reg::R1);

        write8BitReg(builder, r, Reg::R2, Reg::R1);
    }
    else // SET
    {
        int bit = (opcode >> 3) & 7;
        get8BitValue(builder, Reg::R2, r);

        builder.mov(Reg::R1, 1 << bit);
        builder.orr(Reg::R2, Reg::R1);

        write8BitReg(builder, r, Reg::R2, Reg::R1);
    }
}

void DMGRecompilerThumb::compileEntry()
{
    auto codePtr16 = reinterpret_cast<uint16_t *>(codeBuf);
    ThumbBuilder builder(codePtr16, reinterpret_cast<uint16_t *>(codeBuf + codeBufSize));

    auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

    builder.mov(Reg::R2, Reg::R8);
    builder.mov(Reg::R3, Reg::R9);
    builder.push(0b11111100, true); // R2-7, LR
    
    // set the low bit so we stay in thumb mode
    builder.mov(Reg::R2, 1);
    builder.orr(Reg::R1, Reg::R2);

    // load cpu pointer
    builder.ldr(Reg::R2, 64);
    builder.mov(Reg::R8, Reg::R2);

    // load SP
    int spOff = reinterpret_cast<uintptr_t>(&cpu.sp) - cpuPtr;
    builder.mov(Reg::R3, spOff);
    builder.ldrh(Reg::R3, Reg::R2, Reg::R3);
    builder.mov(spReg, Reg::R3);

    // load emu regs
    int regsOff = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtr;
    builder.add(Reg::R2, regsOff); // add to cpu ptr
    builder.ldrh(Reg::R4, Reg::R2, 0);
    builder.ldrh(Reg::R5, Reg::R2, 2);
    builder.ldrh(Reg::R6, Reg::R2, 4);
    builder.ldrh(Reg::R7, Reg::R2, 6);

    builder.bx(Reg::R1);

    // exit setting the call flag ... and saving LR
    exitForCallPtr = reinterpret_cast<uint8_t *>(builder.getPtr());
    builder.mov(Reg::R0, 1);
    builder.ldr(Reg::R2, 44);
    builder.strb(Reg::R0, Reg::R2, 0);

    // exit saving LR
    saveAndExitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    builder.mov(Reg::R0, Reg::LR);
    builder.ldr(Reg::R2, 40);
    builder.str(Reg::R0, Reg::R2, 0);

    // exit
    exitPtr = reinterpret_cast<uint8_t *>(builder.getPtr());

    // store PC
    int pcOff = reinterpret_cast<uintptr_t>(&cpu.pc) - cpuPtr;
    assert(pcOff <= 0xFF);
    builder.mov(Reg::R2, Reg::R8); // cpu ptr
    builder.mov(Reg::R0, pcOff);
    builder.strh(Reg::R1, Reg::R2, Reg::R0);

    // store SP
    builder.mov(Reg::R0, spOff);
    builder.mov(Reg::R3, spReg);
    builder.strh(Reg::R3, Reg::R2, Reg::R0);

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

    // write addr of exitCallFlag
    auto addr = reinterpret_cast<uintptr_t>(&exitCallFlag);
    *ptr++ = addr;
    *ptr++ = addr >> 16;

    // write addr of tmpSavedPtr
    addr = reinterpret_cast<uintptr_t>(&tmpSavedPtr);
    *ptr++ = addr;
    *ptr++ = addr >> 16;

    curCodePtr = reinterpret_cast<uint8_t *>(ptr);

    //debug
    printf("generated %i halfwords for entry/exit\ncode:", len);

    for(auto p = codeBuf; p !=curCodePtr; p++)
        printf(" %02X", *p);

    printf("\n");
}
