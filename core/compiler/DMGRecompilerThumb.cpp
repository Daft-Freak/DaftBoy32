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

        case 0xE0: // LDH (n),A
        {
            uint16_t addr = 0xFF00 | instr.opcode[1];
            cycleExecuted();
            writeMem(addr, reg(DMGReg::A));
            break;
        }

        case 0xE6: // AND n
        {
            cycleExecuted();
            bitAnd(instr.opcode[1]);
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

        case 0xF6: // OR n
        {
            cycleExecuted();
            bitOr(instr.opcode[1]);
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
