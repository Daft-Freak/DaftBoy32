#include <cassert>
#include <cstdio>
#include <variant>

#include "DMGRecompilerX86.h"

#include "DMGCPU.h"
#include "X86Builder.h"

// size of the code to call these functions
static const int cycleExecutedCallSize = 23;
static const int readMemRegCallSize = 30;
static const int writeMemRegImmCallSize = 36; // this is only accurate for call (addr = SP)

static const int cycleExecutedInlineSize = 5;

// reg helpers
static const Reg32 pcReg32 = Reg32::R12D;
static const Reg16 pcReg16 = Reg16::R12W;

static const Reg32 spReg32 = Reg32::R13D;
static const Reg16 spReg16 = Reg16::R13W;
static const Reg8 spReg8 = Reg8::R13B; // mostly for the adds with the strange flags

inline constexpr Reg8 reg(DMGCPU::Reg r)
{
    const Reg8 regMap8[]
    {
        Reg8::AH, // A
        Reg8::AL, // F
        Reg8::CH, // B
        Reg8::CL, // C
        Reg8::DH, // D
        Reg8::DL, // E
        Reg8::BH, // H
        Reg8::BL  // L
    };
    return regMap8[static_cast<int>(r)];
}

inline constexpr Reg16 reg(DMGCPU::WReg r)
{
    const Reg16 regMap16[]
    {
        Reg16::AX, // AF
        Reg16::CX, // BC
        Reg16::DX, // DE
        Reg16::BX  // HL
    };
    return regMap16[static_cast<int>(r)];
}

// call helpers
static void callSave(X86Builder &builder)
{
    builder.push(Reg64::RAX);
    builder.push(Reg64::RCX);
    builder.push(Reg64::RDX);
    builder.push(Reg64::RDI);
    // builder.sub(Reg64::RSP, 8); // align stack
}

static void callSaveOrSkip(X86Builder &builder)
{
    auto tmpPtr = builder.getPtr() - 4;

    // skip pop/push if we just popped
    // only safe if values from RAX, RCX, RDX, RDI are not moved to args
    if(tmpPtr[0] == 0x5F/*pop rdi*/ && tmpPtr[1] == 0x5A/*pop rdx*/ && tmpPtr[2] == 0x59/*pop rcx*/ && tmpPtr[3] == 0x58/*pop rax*/)
        builder.resetPtr(tmpPtr);
    else if(tmpPtr[1] == 0x5A/*pop rdx*/ && tmpPtr[2] == 0x59/*pop rcx*/ && tmpPtr[3] == 0x58/*pop rax*/)
    {
        // skip 3/4 of the pops
        // (should be after a writeMem)
        builder.resetPtr(tmpPtr + 1);
        builder.push(Reg64::RDI);
    }
    else
        callSave(builder);
}

static void callRestore(X86Builder &builder)
{
    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);
    builder.pop(Reg64::RAX);
}

static void callRestore(X86Builder &builder, Reg32 dstReg)
{
    // mov ret val (this is never used for the emulated regs)
    assert(dstReg != Reg32::EAX && dstReg != Reg32::EDX && dstReg != Reg32::ECX);

    builder.mov(dstReg, Reg32::EAX);

    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(dstReg == Reg32::EDI ? Reg64::RSI : Reg64::RDI); // use RSI to pop the unneeded value
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);
    builder.pop(Reg64::RAX);
}

static void callRestore(X86Builder &builder, Reg8 dstReg)
{
    assert(dstReg != Reg8::DIL); // no

    // move before popping if possible
    bool isPoppedReg = dstReg == Reg8::AL || dstReg == Reg8::CL || dstReg == Reg8::DL || dstReg == Reg8::AH || dstReg == Reg8::CH || dstReg == Reg8::DH;

    if(!isPoppedReg)
        builder.mov(dstReg, Reg8::AL);

    // builder.add(Reg64::RSP, 8); // alignment
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RDX);
    builder.pop(Reg64::RCX);

    // mov ret val (if not going to RAX)
    if(isPoppedReg && dstReg != Reg8::AL && dstReg != Reg8::AH)
    {
        builder.mov(dstReg, Reg8::AL);
        builder.pop(Reg64::RAX);
    }
    else if(dstReg == Reg8::AH) // TODO: having a worse case for A is not great
    {
        builder.mov(Reg8::AH, Reg8::AL);
        builder.pop(Reg64::R10);
        builder.mov(Reg8::AL, Reg8::R10B); // restore low byte
    }
    else if(dstReg == Reg8::AL)// ... though this is the worst case... (AL == F, so unlikely)
    {
        // EAX = EAX + (R10D & 0xFF00)
        builder.pop(Reg64::R10);
        builder.and_(Reg32::R10D, 0xFF00);
        builder.movzx(Reg32::EAX, Reg8::AL);
        builder.add(Reg32::EAX, Reg32::R10D); // TODO: OR? (haven't added that to builder yet)
    }
    else // we already did it
        builder.pop(Reg64::RAX);
}

bool DMGRecompilerX86::compile(uint8_t *&codePtr, uint16_t pc, BlockInfo &blockInfo)
{
    X86Builder builder(codePtr, codeBuf + codeBufSize);

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
        printf("recompile @%04X failed due to error (out of space?)\n", startPC);
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
    int len = endPtr - codePtr;

    //debug
    printf("recompile @%04X generated %i bytes (%i instructions)\ncode:", startPC, len, numInstructions);

    for(auto p = codePtr; p != endPtr; p++)
        printf(" %02X", *p);

    printf("\n");
    printf("(addr %p->%p)\n", codePtr, endPtr);
#endif

    codePtr = endPtr;

    return true;
}

bool DMGRecompilerX86::recompileInstruction(uint16_t &pc, OpInfo &instr, X86Builder &builder)
{
    using Reg = DMGCPU::Reg;
    using WReg = DMGCPU::WReg;

    auto &mem = cpu.getMem();
    uint8_t opcode = instr.opcode[0];

    int cyclesThisInstr = 0;
    int delayedCyclesExecuted = 0;

    bool checkInterrupts = false;

    // AF = EAX
    // BC = ECX
    // DE = EDX
    // HL = EBX
    // PC = R12D
    // SP = R13D
    // cycles = EDI

    // mapping from opcodes
    static const Reg8 regMap8[]
    {
        reg(Reg::B),
        reg(Reg::C),
        reg(Reg::D),
        reg(Reg::E),
        reg(Reg::H),
        reg(Reg::L),
        Reg8::R10B, // where we load to for (HL)
        reg(Reg::A),
    };

    static const Reg16 regMap16[]
    {
        reg(WReg::BC),
        reg(WReg::DE),
        reg(WReg::HL),
        spReg16, // unless it's push/pop, then it's AF
    };

    bool inHRAM = pc >= 0xFF00;

    // TODO: shared trampolines?
    auto cycleExecuted = [this, &builder, inHRAM, &cyclesThisInstr, &delayedCyclesExecuted]()
    {
        cyclesThisInstr += 4;

        if(!inHRAM)
        {
            // since we refuse to compile when OAM DMA is active we can just inline cycleExecuted (-updateOAMDMA) when not running from HRAM
            // ...and we can also do it slightly later
            delayedCyclesExecuted += 4;
            return;
        }

        // safe to skip here as we take no args
        callSaveOrSkip(builder);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompilerX86::cycleExecuted)); // function ptr
        builder.mov(Reg64::RDI, Reg64::R14); // cpu/this ptr
        builder.call(Reg64::RAX); // do call

        callRestore(builder);
    };

    auto syncCyclesExecuted = [this, &builder, &delayedCyclesExecuted]()
    {
        if(!delayedCyclesExecuted)
            return;

        assert(delayedCyclesExecuted < 127);
        auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

        int8_t i8Cycles = delayedCyclesExecuted;

        // we don't update cyclesToRun here, do it after returning instead
        builder.addD(i8Cycles, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.cycleCount) - cpuPtr);

        delayedCyclesExecuted = 0;
    };

    auto setupMemAddr = [&builder, &syncCyclesExecuted](std::variant<Reg16, uint16_t> addr)
    {
        if(std::holds_alternative<Reg16>(addr))
        {
            syncCyclesExecuted();
            builder.movzx(Reg32::ESI, std::get<Reg16>(addr));
        }
        else
        {
            // accessing most ram shouldn't cause anything to be updated, so we don't need an accurate cycle count
            auto immAddr = std::get<uint16_t>(addr);
            if(immAddr >> 8 == 0xFF)
                syncCyclesExecuted();

            builder.mov(Reg32::ESI, immAddr);
        }
    };

    auto readMem = [&builder, &cycleExecuted, &syncCyclesExecuted, &setupMemAddr](std::variant<Reg16, uint16_t> addr, Reg8 dstReg, bool postInc = false)
    {
        // can skip push/pop if we don't need AX/CX/DX
        if(!std::holds_alternative<Reg16>(addr) || std::get<Reg16>(addr) == spReg16)
            callSaveOrSkip(builder);
        else
            callSave(builder);

        setupMemAddr(addr);

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompilerX86::readMem)); // function ptr
        builder.mov(Reg64::RDI, Reg64::R14); // cpu/this ptr

        builder.call(Reg64::RAX); // do call

        if(postInc && std::holds_alternative<Reg16>(addr))
            builder.inc(std::get<Reg16>(addr));

        callRestore(builder, dstReg);

        cycleExecuted();
    };

    auto writeMem = [&builder, &cycleExecuted, &syncCyclesExecuted, &setupMemAddr](std::variant<Reg16, uint16_t> addr, std::variant<Reg8, uint8_t> data, bool preDec = false)
    {
        // can skip push/pop if we don't need AX/CX/DX
        bool noPopAddr = !std::holds_alternative<Reg16>(addr) || std::get<Reg16>(addr) == spReg16;
        bool noPopData = !std::holds_alternative<Reg8>(data) || std::get<Reg8>(data) == Reg8::BL || std::get<Reg8>(data) == Reg8::BH;

        if(noPopAddr && noPopData)
            callSaveOrSkip(builder);
        else
            callSave(builder);

        if(preDec && std::holds_alternative<Reg16>(addr))
            builder.dec(std::get<Reg16>(addr));

        setupMemAddr(addr);

        if(std::holds_alternative<Reg8>(data))
            builder.movzx(Reg32::EDX, std::get<Reg8>(data));
        else
            builder.mov(Reg32::EDX, std::get<uint8_t>(data));

        builder.mov(Reg32::ECX, Reg32::EDI); // cycle count

        builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(&DMGRecompilerX86::writeMem)); // function ptr
        builder.mov(Reg64::RDI, Reg64::R14); // cpu/this ptr
        builder.call(Reg64::RAX); // do call

        callRestore(builder, Reg32::EDI);

        cycleExecuted();
    };

    // neededFlags is what we need to output
    // updateFlags is what we can output from the result
    // oneFlags is is what is alwyas set to 1
    // preservedFlags is what is... preserved
    const auto setFlags = [&builder](uint8_t &neededFlags, uint8_t updateFlags, uint8_t oneFlags = 0, uint8_t preservedFlags = 0)
    {
        auto f = reg(Reg::F);

        updateFlags &= neededFlags;  // mask out unneeded

        // preserved flags should have been preserved already
        // so F should have all other bits zero

        bool setF = preservedFlags != 0;
        bool haveOpFlags = true; // assuming nothing has destroyed flags before this

        if(haveOpFlags && (updateFlags & DMGCPU::Flag_C))
        {
            // copy + shift if not set
            if(!setF)
            {
                builder.setcc(Condition::B, f);
                builder.shl(f, 4); // C is bit 4
                setF = true;
            }
            else
            {
                builder.jcc(Condition::AE, 3); // if !carry
                builder.or_(f, DMGCPU::Flag_C); // set C
            }

            haveOpFlags = false;
            neededFlags &= ~DMGCPU::Flag_C;
        }

        if(haveOpFlags && (updateFlags & DMGCPU::Flag_Z))
        {
            if(!setF)
            {
                builder.setcc(Condition::E, f);
                builder.shl(f, 7); // Z is bit 7
                setF = true;
            }
            else
            {
                builder.jcc(Condition::NE, 3); // if != 0
                builder.or_(f, DMGCPU::Flag_Z); // set Z
            }

            haveOpFlags = false;
            neededFlags &= ~DMGCPU::Flag_Z;
        }

        if(neededFlags && !setF)// some flags left and no write, make sure constant flags are set
        {
            builder.mov(f, oneFlags);
            neededFlags &= ~oneFlags;
            setF = true;
        }

        // still haven't set always one flags
        if(neededFlags & oneFlags)
        {
            builder.or_(f, oneFlags);
            neededFlags &= ~oneFlags;
        }
    };

    // ADC/SBC, rotates
    const auto carryIn = [&builder]()
    {
        builder.test(reg(Reg::F), DMGCPU::Flag_C); // sets CF to 0
        builder.jcc(Condition::E, 1); // not set
        builder.stc(); // CF = 1
    };

    // used by the rotates
    const auto carryOut = [&instr, &builder]()
    {
        if(instr.flags & DMGCPU::Flag_C)
        {
            // copy carry out
            // (shortcut as this is the only flag)
            builder.setcc(Condition::B, reg(Reg::F));
            builder.shl(reg(Reg::F), 4); // C is bit 4
        }
    };

    const auto push = [&builder, &cycleExecuted, &writeMem](Reg16 r)
    {
        assert(r == Reg16::AX || r == Reg16::CX || r == Reg16::DX || r == Reg16::BX);
        auto lowReg = static_cast<Reg8>(r); // AX == AL, CX == CL, ...
        auto highReg = static_cast<Reg8>(static_cast<int>(lowReg) + 4); // AH == AL + 4

        cycleExecuted(); // delay

        writeMem(spReg16, highReg, true);
        writeMem(spReg16, lowReg, true);
    };

    const auto pop = [&builder, &readMem](Reg16 r)
    {
        assert(r == Reg16::AX || r == Reg16::CX || r == Reg16::DX || r == Reg16::BX);
        auto lowReg = static_cast<Reg8>(r); // AX == AL, CX == CL, ...
        auto highReg = static_cast<Reg8>(static_cast<int>(lowReg) + 4); // AH == AL + 4

        readMem(spReg16, lowReg, true);
        readMem(spReg16, highReg, true);

        // low bits in F can never be set
        if(r == reg(WReg::AF))
            builder.and_(reg(Reg::F), 0xF0);
    };

    const auto prepareForHCalc = [&builder](Reg8 a, std::variant<Reg8, uint8_t> b, bool saveF)
    {
        // shuffles around regs to get a,b in the F reg and R10B (masked to the low half)
        // ready to do whatever op needs to be done to calculate the H flag
        // also optionally saves F in R11B
        auto f = reg(Reg::F);

        if(saveF)
            builder.mov(Reg8::R11B, f);

        // copy src/dst (using flags reg as temp)
        if(std::holds_alternative<Reg8>(b))
        {
            auto rb = std::get<Reg8>(b);

            if(rb == Reg8::AH || rb == Reg8::CH || rb == Reg8::DH || rb == Reg8::BH)
            {
                // legacy regs, extra copy
                builder.mov(f, rb);
                builder.mov(Reg8::R10B, f);
            }
            else
                builder.mov(Reg8::R10B, rb);
        }
        else // yeah this case could be optimised further
            builder.mov(Reg8::R10B, std::get<uint8_t>(b));

        builder.mov(f, a);

        // mask and do half add
        builder.and_(f, 0xF);
        builder.and_(Reg8::R10B, 0xF);
    };

    const auto add = [&instr, &builder, &setFlags, &carryIn, &prepareForHCalc](std::variant<Reg8, uint8_t> b, bool withCarry = false)
    {
        auto a = reg(Reg::A);
        auto f = reg(Reg::F);

        bool bIsReg = std::holds_alternative<Reg8>(b);
        bool bIsF = bIsReg && std::get<Reg8>(b) == f;

        if(instr.flags & DMGCPU::Flag_H)
        {
            // preserve F if it was was used as tmp (affects ADD (HL))
            // also preserve F for ADC
            prepareForHCalc(a, b, bIsF || withCarry);

            // do half add
            builder.add(Reg8::R10B, f);

            // put tmp/flags reg back
            if(bIsF || withCarry)
                builder.mov(f, Reg8::R11B);

            if(withCarry)
            {
                // add carry to half add
                builder.test(f, DMGCPU::Flag_C); // sets CF to 0
                builder.jcc(Condition::E, 3); // not set
                builder.inc(Reg8::R10B);
            }
        }

        if(withCarry)
        {
            carryIn();

            if(bIsReg)
            {
                auto rb = std::get<Reg8>(b);

                if(rb == Reg8::SIL) // ADC (HL)
                {
                    // more reg shuffling...
                    builder.mov(f, rb);
                    builder.adc(a, f);
                }
                else
                    builder.adc(a, rb);
            }
            else
                builder.adc(a, std::get<uint8_t>(b));
        }
        else if(bIsReg)// do add
            builder.add(a, std::get<Reg8>(b));
        else
            builder.add(a, std::get<uint8_t>(b));

        // flags
        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_Z); // without any preserved flags, this will always set C (if needed)

        if(flags & DMGCPU::Flag_H)
        {
            // half carry flag
            // r10b > 0xF ? H : 0
            builder.cmp(Reg8::R10B, 0xF);
            builder.jcc(Condition::BE, 3); // <= 0xF
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        if(flags & DMGCPU::Flag_Z)
        {
            // zero flag
            builder.and_(a, a); // TODO: TEST?
            builder.jcc(Condition::NE, 3); // if != 0
            builder.or_(f, DMGCPU::Flag_Z); // set Z
        }

        return true;
    };

    const auto addWithCarry = [&add](std::variant<Reg8, uint8_t> b)
    {
        return add(b, true);
    };

    const auto sub = [&instr, &builder, &setFlags, &carryIn, &prepareForHCalc](std::variant<Reg8, uint8_t> b, bool withCarry = false)
    {
        auto a = reg(Reg::A);
        auto f = reg(Reg::F);

        bool bIsReg = std::holds_alternative<Reg8>(b);
        bool bIsF = bIsReg && std::get<Reg8>(b) == f;

        if(instr.flags & DMGCPU::Flag_H)
        {
            // preserve F if it was was used as tmp (affects SUB (HL))
            // also preserve F for SBC
            prepareForHCalc(a, b, bIsF || withCarry);

            // do half sub
            builder.sub(f, Reg8::R10B);
            builder.mov(Reg8::R10B, f);

            // put tmp/flags reg back
            if(bIsF || withCarry)
                builder.mov(f, Reg8::R11B);

            if(withCarry)
            {
                // sub carry from half sub
                builder.test(f, DMGCPU::Flag_C); // sets CF to 0
                builder.jcc(Condition::E, 3); // not set
                builder.dec(Reg8::R10B);
            }
        }

        if(withCarry)
        {
            carryIn();

            if(bIsReg)
            {
                auto rb = std::get<Reg8>(b);

                if(rb == Reg8::SIL) // SBC (HL)
                {
                    // more reg shuffling...
                    builder.mov(f, rb);
                    builder.sbb(a, f);
                }
                else
                    builder.sbb(a, rb);
            }
            else
                builder.sbb(a, std::get<uint8_t>(b));
        }
        else if(bIsReg) // do sub
            builder.sub(a, std::get<Reg8>(b));
        else
            builder.sub(a, std::get<uint8_t>(b));

        // flags
        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_Z, DMGCPU::Flag_N);

        if(flags & DMGCPU::Flag_H)
        {
            // half carry flag
            // r10b < 0 ? H : 0
            builder.cmp(Reg8::R10B, 0);
            builder.jcc(Condition::GE, 3); // >= 0
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        if(flags & DMGCPU::Flag_Z)
        {
            // zero flag
            builder.and_(a, a); // TODO: TEST?
            builder.jcc(Condition::NE, 3); // if != 0
            builder.or_(f, DMGCPU::Flag_Z); // set Z
        }
    };

    const auto subWithCarry = [&sub](std::variant<Reg8, uint8_t> b)
    {
        return sub(b, true);
    };

    const auto bitAnd = [&instr, &builder, &setFlags](std::variant<Reg8, uint8_t> b)
    {
        if(std::holds_alternative<Reg8>(b))
            builder.and_(reg(Reg::A), std::get<Reg8>(b));
        else
            builder.and_(reg(Reg::A), std::get<uint8_t>(b));

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z, DMGCPU::Flag_H);
    };

    const auto bitOr = [&instr, &builder, &setFlags](std::variant<Reg8, uint8_t> b)
    {
        if(std::holds_alternative<Reg8>(b))
            builder.or_(reg(Reg::A), std::get<Reg8>(b));
        else
            builder.or_(reg(Reg::A), std::get<uint8_t>(b));

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z);
    };

    const auto bitXor = [&instr, &builder, &setFlags, &pc/*!!*/](std::variant<Reg8, uint8_t> b)
    {
        if(std::holds_alternative<Reg8>(b))
            builder.xor_(reg(Reg::A), std::get<Reg8>(b));
        else
            builder.xor_(reg(Reg::A), std::get<uint8_t>(b));

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z);
    };

    const auto cmp = [&instr, &builder, &setFlags, &prepareForHCalc](std::variant<Reg8, uint8_t> b)
    {
        auto a = reg(Reg::A);
        auto f = reg(Reg::F);

        bool bIsReg = std::holds_alternative<Reg8>(b);
        bool bIsF = bIsReg && std::get<Reg8>(b) == f;

        if(instr.flags & DMGCPU::Flag_H)
        {
            // preserve F for (CP (HL))
            prepareForHCalc(a, b, bIsF);

            //  do half cmp
            builder.cmp(f, Reg8::R10B);
            builder.setcc(Condition::B, Reg8::R10B); // save carry

            // put tmp reg back
            if(bIsF)
                builder.mov(f, Reg8::R11B);
        }

        // do cmp
        if(bIsReg)
            builder.cmp(a, std::get<Reg8>(b));
        else
            builder.cmp(a, std::get<uint8_t>(b));

        if(instr.flags & DMGCPU::Flag_Z)
            builder.setcc(Condition::E, Reg8::R11B); // save zero

        // flags
        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_Z, DMGCPU::Flag_N);

        if(flags & DMGCPU::Flag_H)
        {
            // half carry flag
            builder.and_(Reg8::R10B, Reg8::R10B); // TODO: TEST?
            builder.jcc(Condition::E, 3); // carry not set
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        // zero flag
        if(flags & DMGCPU::Flag_Z)
        {
            builder.and_(Reg8::R11B, Reg8::R11B); // TODO: TEST?
            builder.jcc(Condition::E, 3); // if != 0
            builder.or_(f, DMGCPU::Flag_Z); // set Z
        }
    };

    const auto inc = [&instr, &builder, &setFlags](Reg8 r)
    {
        auto f = reg(Reg::F);

        if(instr.flags & Op_WriteFlags)
            builder.and_(f, DMGCPU::Flag_C); // preserve C

        // H flag
        if(instr.flags & DMGCPU::Flag_H)
        {
            builder.mov(Reg8::R10B, f); // save F

            builder.mov(f, r); // & 0xF == 0xF (use F as tmp)
            builder.and_(f, 0xF);
            builder.cmp(f, 0xF);
            builder.mov(f, Reg8::R10B); // restore F
            builder.jcc(Condition::NE, 3); // != 0xF
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        // do inc
        builder.inc(r);

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z, 0, DMGCPU::Flag_C); // only works because preserve C
    };

    const auto dec = [&instr, &builder, &setFlags](Reg8 r)
    {
        auto f = reg(Reg::F);

        if(instr.flags & Op_WriteFlags)
            builder.and_(f, DMGCPU::Flag_C); // preserve C

        // H flag
        if(instr.flags & DMGCPU::Flag_H)
        {
            builder.mov(Reg8::R10B, f); // save F

            builder.mov(f, r); // & 0xF == 0 (use F as tmp)
            builder.and_(f, 0xF);
            builder.mov(f, Reg8::R10B); // restore F
            builder.jcc(Condition::NE, 3); // != 0x0
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        // do dec
        builder.dec(r);

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_Z, DMGCPU::Flag_N, DMGCPU::Flag_C); // only works because preserve C
    };

    const auto add16 = [&instr, &builder, &setFlags, &cycleExecuted](Reg16 b)
    {
        auto a = reg(WReg::HL);
        auto f = reg(Reg::F);

        if(instr.flags & DMGCPU::Flag_H)
        {
            // copy src/dst
            builder.mov(Reg32::R10D, static_cast<Reg32>(a));
            builder.mov(Reg32::R11D, static_cast<Reg32>(b));

            // mask and do "half"(3/4?) add
            builder.and_(Reg32::R10D, 0xFFF);
            builder.and_(Reg32::R11D, 0xFFF);
            builder.add(Reg32::R10D, Reg32::R11D);
        }

        // preserve Z flag
        if(instr.flags & Op_WriteFlags)
            builder.and_(f, DMGCPU::Flag_Z);

        // do add
        builder.add(a, b);

        uint8_t flags = instr.flags & Op_WriteFlags;
        setFlags(flags, DMGCPU::Flag_C | DMGCPU::Flag_H | DMGCPU::Flag_N, 0, DMGCPU::Flag_Z);

        // half carry flag
        if(flags & Op_WriteH)
        {
            // r10d > 0xFFF ? H : 0
            builder.cmp(Reg32::R10D, 0xFFF);
            builder.jcc(Condition::BE, 3); // <= 0xFFF
            builder.or_(f, DMGCPU::Flag_H); // set H
        }

        cycleExecuted();
    };

    const auto doJump = [this, &instr, &pc, &builder, inHRAM, &cycleExecuted, &syncCyclesExecuted, &cyclesThisInstr](uint16_t addr, int flag = 0, bool set = true)
    {
        auto it = branchTargets.find(addr);

        // condition
        if(flag)
        {
            syncCyclesExecuted();
            if(!(instr.flags & Op_Branch)) // branch flag means the jump is in range, so we should be able to handle it
                builder.mov(pcReg32, pc); // otherwise we're going to exit, so save PC

            builder.test(reg(Reg::F), flag);
            int len = ((instr.flags & Op_Branch) ? 3/*sub*/ : 6/*mov*/) + 5/*jmp*/;

            if(inHRAM)
                len += cycleExecutedCallSize;
            else
                len += cycleExecutedInlineSize;

            builder.jcc(set ? Condition::E : Condition::NE, len);
        }

        cycleExecuted();
        syncCyclesExecuted();

        // sub cycles early (we jump past the usual code that does this)
        if(instr.flags & Op_Branch)
            builder.sub(Reg32::EDI, cyclesThisInstr);
        else // or set PC if we're just going to exit
            builder.mov(pcReg32, addr);

        // don't update twice for unconditional branches (if it doesn't have the branch flag it's an exit anyway)
        if(!flag)
            cyclesThisInstr = 0;

        if(it != branchTargets.end())
            builder.jmp(it->second - builder.getPtr(), flag != 0);
        else
        {
            if(instr.flags & Op_Branch)
                forwardBranchesToPatch.emplace(addr, builder.getPtr());

            // will be patched later if possible
            bool forceLong = flag != 0 && (instr.flags & Op_Branch);
            builder.jmp(exitPtr - builder.getPtr(), forceLong);
        }

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
            builder.jmp(exitPtr - builder.getPtr());
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

    const auto call = [this, &instr, &pc, &builder, inHRAM, &cycleExecuted, &syncCyclesExecuted, &writeMem](int flag = 0, bool set = true)
    {
        uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
        cycleExecuted();
        cycleExecuted();

        // condition
        if(flag)
        {
            syncCyclesExecuted();
            builder.mov(pcReg32, pc);
            builder.test(reg(Reg::F), flag);

            int len = 19 + writeMemRegImmCallSize * 2;
            if(inHRAM)
                len += cycleExecutedCallSize * 3 - (6 * 2 + 8 * 2) /*adjacent calls*/;
            else
                len += cycleExecutedInlineSize * 3 - 6;

            builder.jcc(set ? Condition::E : Condition::NE, len);
        }

        cycleExecuted(); // delay

        writeMem(spReg16, static_cast<uint8_t>(pc >> 8), true);
        writeMem(spReg16, static_cast<uint8_t>(pc), true);
        syncCyclesExecuted();

        builder.mov(pcReg32, addr);

        // exit but flag as a call so we can save the return addr
        builder.call(exitForCallPtr - builder.getPtr());

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
            builder.jmp(exitPtr - builder.getPtr());
    };

    const auto reset = [this, &pc, &builder, &cycleExecuted, &syncCyclesExecuted, &writeMem](int addr)
    {
        cycleExecuted(); // delay

        writeMem(spReg16, static_cast<uint8_t>(pc >> 8), true);
        writeMem(spReg16, static_cast<uint8_t>(pc), true);
        syncCyclesExecuted();

        builder.mov(pcReg32, addr);

        builder.jmp(exitPtr - builder.getPtr()); // exit
    };

    const auto ret = [this, &instr, &pc, &builder, inHRAM, &cycleExecuted, &syncCyclesExecuted, &readMem](int flag = 0, bool set = true)
    {
        // condition
        if(flag)
        {
            cycleExecuted(); // delay
            syncCyclesExecuted();

            builder.mov(pcReg32, pc);
            builder.test(reg(Reg::F), flag);

            int len = 27 + readMemRegCallSize * 2;
            if(inHRAM)
                len += cycleExecutedCallSize * 3 - 8 * 3/*adjacent calls*/;
            else
                len += cycleExecutedInlineSize * 2 - 8;

            builder.jcc(set ? Condition::E : Condition::NE, len);
        }

        auto pcReg8 = static_cast<Reg8>(pcReg16);

        builder.mov(pcReg32, 0);
        readMem(spReg16, pcReg8, true);
        readMem(spReg16, Reg8::R10B, true);

        builder.shl(Reg32::R10D, 8);
        builder.or_(pcReg16, Reg16::R10W);
        cycleExecuted();
        syncCyclesExecuted();

        assert(exitPtr - builder.getPtr() < -126);
        builder.jmp(exitPtr - builder.getPtr()); // it's > 128 bytes to here from the start of the op, so should always be 5 bytes

        // exit if branch not taken
        if(flag && (instr.flags & Op_Last))
            builder.jmp(exitPtr - builder.getPtr());
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
            builder.jmp(13);
            lastInstrCycleCheck = builder.getPtr();

            // if <= 0 exit
            builder.jcc(Condition::G, 11);
            builder.mov(pcReg32, pc);
            builder.call(saveAndExitPtr - builder.getPtr());

            // TODO: this is the first instruction
            branchTargets.emplace(pc, lastInstrCycleCheck);
        }

        // patch forwards jumps
        // can't hit this for the first instruction, so the lastInstrCycleCheck will be valid
        auto jumps = forwardBranchesToPatch.equal_range(pc);
        for(auto it = jumps.first; it != jumps.second; ++it)
        {
            // overrite 4 byte disp
            auto off = lastInstrCycleCheck - (it->second + 5);

            it->second[1] = off;
            it->second[2] = off >> 8;
            it->second[3] = off >> 16;
            it->second[4] = off >> 24;
        }
        forwardBranchesToPatch.erase(jumps.first, jumps.second);
    }

    pc += instr.len;

    // cycle for opcode read
    auto oldPtr = builder.getPtr();
    cycleExecuted();

    // previous op was EI
    if(mem.read(pc - (instr.len + 1)) == 0xFB /*EI*/)
    {
        // enable interrupts for EI
        auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

        // if(enableInterruptsNextCycle)
        // probably don't need this check... might get a false positive in some extreme case though
        builder.cmp(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle) - cpuPtr);
        builder.jcc(Condition::E, 10);

        // masterInterruptEnable = true
        builder.mov(1, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtr);

        // enableInterruptsNextCycle = false
        builder.mov(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle) - cpuPtr);

        checkInterrupts = true;
    }

    switch(opcode)
    {
        case 0x00: // NOP
            break;
        case 0x01: // LD BC,nn
        case 0x11: // LD DE,nn
        case 0x21: // LD HL,nn
        case 0x31: // LD SP,nn
        {
            uint16_t v = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            builder.mov(static_cast<Reg32>(regMap16[opcode >> 4]), v);
            break;
        }
        case 0x02: // LD (BC),A
        case 0x12: // LD (DE),A
            writeMem(regMap16[opcode >> 4], reg(Reg::A));
            break;

        case 0x03: // INC BC
        case 0x13: // INC DE
        case 0x23: // INC HL
        case 0x33: // INC SP
            builder.inc(regMap16[opcode >> 4]);
            cycleExecuted();
            break;

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
            dec(reg(Reg::B));
            break;
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
        case 0x3B: // DEC SP
            builder.dec(regMap16[opcode >> 4]);
            cycleExecuted();
            break;

        case 0x06: // LD B,n
        case 0x0E: // LD C,n
        case 0x16: // LD D,n
        case 0x1E: // LD E,n
        case 0x26: // LD H,n
        case 0x2E: // LD L,n
        case 0x3E: // LD A,n
            cycleExecuted();
            builder.mov(regMap8[opcode >> 3], instr.opcode[1]);
            break;

        case 0x07: // RLCA
            builder.rol(reg(Reg::A), 1);
            carryOut();
            break;
        case 0x08: // LD (nn),SP
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            writeMem(addr++, spReg8); // low byte

            // SP >> 8
            builder.mov(Reg32::R10D, spReg32);
            builder.shr(Reg32::R10D, 8);

            writeMem(addr, Reg8::R10B); // low byte
            break;
        }

        case 0x09: // ADD HL,BC
        case 0x19: // ADD HL,DE
        case 0x29: // ADD HL,HL
        case 0x39: // ADD HL,SP
            add16(regMap16[opcode >> 4]);
            break;

        case 0x0A: // LD A,(BC)
        case 0x1A: // LD A,(DE)
            readMem(regMap16[opcode >> 4], reg(Reg::A));
            break;

        case 0x0F: // RRCA
            builder.ror(reg(Reg::A), 1);
            carryOut();
            break;

        case 0x17: // RLA
            carryIn();
            builder.rcl(reg(Reg::A), 1);
            carryOut();
            break;

        case 0x18: // JR m
            jumpRel();
            break;

        case 0x1F: // RRA
            carryIn();
            builder.rcr(reg(Reg::A), 1);
            carryOut();
            break;

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
            writeMem(reg(WReg::HL), reg(Reg::A));
            builder.inc(reg(WReg::HL));
            break;

        case 0x27: // DAA
        {
            auto f = reg(Reg::F);
            auto a = reg(Reg::A);

            // clear Z flag (will add back later)
            builder.and_(f, DMGCPU::Flag_Z ^ 0xFF);

            builder.test(f, DMGCPU::Flag_N);
            builder.jcc(Condition::E, 18); // N not set

            // negative

            // if (Flag_C) A -= 0x60
            builder.test(f, DMGCPU::Flag_C);
            builder.jcc(Condition::E, 03); // C not set
            builder.sub(a, 0x60);

            // if (Flag_H) A -= 0x06
            builder.test(f, DMGCPU::Flag_H);
            builder.jcc(Condition::E, 3); // C not set
            builder.sub(a, 0x06);

            builder.jmp(43); // skip

            // positive

            // if (Flag_C ...
            builder.test(f, DMGCPU::Flag_C);
            builder.jcc(Condition::NE, 5); // C set
            // ... || A > 0x99) ...
            builder.cmp(a, 0x99);
            builder.jcc(Condition::BE, 6);
            // ... A += 0x60
            builder.add(a, 0x60);
            builder.or_(f, DMGCPU::Flag_C); // set C flag

            // if (Flag_H ...
            builder.test(f, DMGCPU::Flag_H);
            builder.jcc(Condition::NE, 19); // H set
            // ... || A & (0x0F) > 0x09) ...
            builder.mov(Reg32::R10D, static_cast<Reg32>(reg(WReg::AF))); // move the whole thing as we can't easily mov just the high byte
            builder.and_(Reg32::R10D, 0xF00);
            builder.cmp(Reg32::R10D, 0x0900);
            builder.jcc(Condition::BE, 3);
            // .. A += 0x06
            builder.add(a, 0x06);

            // Z flag
            builder.and_(a, a);
            builder.jcc(Condition::NE, 3);
            builder.or_(f, DMGCPU::Flag_Z);

            // clear H flag
            builder.and_(f, ~DMGCPU::Flag_H);

            break;
        }

        case 0x2A: // LDI A,(HL)
            readMem(reg(WReg::HL), reg(Reg::A));
            builder.inc(reg(WReg::HL));
            break;

        case 0x2F: // CPL
            builder.not_(reg(Reg::A));
            builder.or_(reg(Reg::F), DMGCPU::Flag_H | DMGCPU::Flag_N);
            break;

        case 0x32: // LDD (HL),A
            writeMem(reg(WReg::HL), reg(Reg::A));
            builder.dec(reg(WReg::HL));
            break;

        case 0x34: // INC (HL)
        {
            auto tmp = Reg8::R11B;

            readMem(reg(WReg::HL), tmp);
            inc(tmp);
            writeMem(reg(WReg::HL), tmp);

            break;
        }
        case 0x35: // DEC (HL)
        {
            auto tmp = Reg8::R11B;

            readMem(reg(WReg::HL), tmp);
            dec(tmp);
            writeMem(reg(WReg::HL), tmp);

            break;
        }
        case 0x36: // LD (HL),n
        {
            cycleExecuted();
            writeMem(reg(WReg::HL), instr.opcode[1]);
            break;
        }
        case 0x37: // SCF
        {
            builder.and_(reg(Reg::F), DMGCPU::Flag_Z); // H/N are cleared
            builder.or_(reg(Reg::F), DMGCPU::Flag_C);
            break;
        }

        case 0x3A: // LDD A,(HL)
            readMem(reg(WReg::HL), reg(Reg::A));
            builder.dec(reg(WReg::HL));
            break;

        case 0x3F: // CCF
        {
            builder.and_(reg(Reg::F), DMGCPU::Flag_C | DMGCPU::Flag_Z); // H/N are cleared
            builder.xor_(reg(Reg::F), DMGCPU::Flag_C);
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
            builder.mov(regMap8[(opcode >> 3) & 7], regMap8[opcode & 7]);
            break;
        case 0x46: // LD B,(HL)
        case 0x4E: // LD C,(HL)
        case 0x56: // LD D,(HL)
        case 0x5E: // LD E,(HL)
        case 0x66: // LD H,(HL)
        case 0x6E: // LD L,(HL)
        case 0x7E: // LD A,(HL)
            readMem(reg(WReg::HL), regMap8[(opcode >> 3) & 7]);
            break;
        case 0x70: // LD (HL),B
        case 0x71: // LD (HL),C
        case 0x72: // LD (HL),D
        case 0x73: // LD (HL),E
        case 0x74: // LD (HL),H
        case 0x75: // LD (HL),L
        case 0x77: // LD (HL),A
            writeMem(reg(WReg::HL), regMap8[opcode & 7]);
            break;

        case 0x76: // HALT
        {
            // halted = true
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
            builder.mov(1, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.halted) - cpuPtr);

            // if(!masterInterruptEnable && serviceableInterrupts)
            builder.cmp(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - cpuPtr);
            builder.jcc(Condition::NE, 12);
            builder.cmp(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu. serviceableInterrupts) - cpuPtr);
            builder.jcc(Condition::E, 5);
            // haltBug = true
            builder.mov(1, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.haltBug) - cpuPtr);

            // exit
            syncCyclesExecuted();
            builder.mov(pcReg32, pc); // exits need to set PC themselves
            builder.call(saveAndExitPtr - builder.getPtr());
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
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
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
            auto tmp = Reg8::SIL; // can't use F here so use something wierd
            readMem(reg(WReg::HL), tmp);
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
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
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
            auto tmp = Reg8::SIL; // can't use F here so use something wierd
            readMem(reg(WReg::HL), tmp);
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
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
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
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
            bitXor(tmp);
            break;
        }
        case 0xAF: // XOR A
            if(instr.flags & Op_WriteFlags)
                builder.mov(Reg32::EAX, DMGCPU::Flag_Z); // A = 0, F = Z
            else
                builder.mov(reg(Reg::A), 0);
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
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
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
            auto tmp = reg(Reg::F); // flags are set here, so we can use it as a temp
            readMem(reg(WReg::HL), tmp);
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
            pop(regMap16[(opcode >> 4) & 3]);
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
            push(regMap16[(opcode >> 4) & 3]);
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

            if(isMem)
                readMem(reg(WReg::HL), Reg8::R10B);

            recompileExInstruction(instr, builder);

            if(isMem && (exOpcode >= 0x80 || exOpcode < 0x40)) // BIT doesn't write back
                writeMem(reg(WReg::HL), Reg8::R10B);
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
            // masterInterruptEnable = true
            builder.mov(1, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - reinterpret_cast<uintptr_t>(&cpu));
            ret();
            break;

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
            writeMem(addr, reg(Reg::A));
            break;
        }

        case 0xE2: // LDH (C),A
        {
            builder.movzx(Reg32::R10D, reg(Reg::C));
            builder.or_(Reg32::R10D, 0xFF00);
            writeMem(Reg16::R10W, reg(Reg::A));
            break;
        }

        case 0xE6: // AND n
        {
            cycleExecuted();
            bitAnd(instr.opcode[1]);
            break;
        }

        case 0xE8: // ADD SP,n
        case 0xF8: // LDHL SP,n
        {
            auto f = reg(Reg::F);

            auto b = instr.opcode[1];
            cycleExecuted();

            // flags are set as if this is an 8 bit op
            builder.mov(f, 0);

            // 8 bit add
            builder.mov(Reg8::R10B, spReg8);
            builder.add(Reg8::R10B, b);

            // carry flag
            builder.jcc(Condition::AE, 3); // if !carry
            builder.or_(f, DMGCPU::Flag_C); // set C

            // half add
            builder.mov(Reg8::R10B, spReg8);
            builder.and_(Reg8::R10B, 0xF);
            builder.add(Reg8::R10B, b & 0xF);

            // half carry flag if > 0xF
            builder.cmp(Reg8::R10B, 0xF);
            builder.jcc(Condition::BE, 3); // <= 0xF
            builder.or_(f, DMGCPU::Flag_H); // set H

            // real add
            if(opcode == 0xE8) // ADD SP,n
            {
                builder.add(spReg16, static_cast<int8_t>(b));

                // 2x delay
                cycleExecuted();
            }
            else // LDHL SP,n
            {
                builder.mov(static_cast<Reg32>(reg(WReg::HL)), spReg32);
                builder.add(reg(WReg::HL), static_cast<int8_t>(b));
            }

            cycleExecuted();

            break;
        }
        case 0xE9: // JP (HL)
            syncCyclesExecuted();
            builder.movzx(pcReg32, reg(WReg::HL)); // PC = HL
            builder.jmp(exitPtr - builder.getPtr()); // exit
            break;
        case 0xEA: // LD (nn),A
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            writeMem(addr, reg(Reg::A));
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
            readMem(addr, reg(Reg::A));
            break;
        }
        case 0xF1: // POP AF
            pop(reg(WReg::AF));
            break;
        case 0xF2: // LDH A,(C)
        {
            builder.movzx(Reg32::R10D, reg(Reg::C));
            builder.or_(Reg32::R10D, 0xFF00);
            readMem(Reg16::R10W, reg(Reg::A));
            break;
        }
        case 0xF3: // DI
            // masterInterruptEnable = false
            // TODO: after next instruction (DMGCPU also has this TODO)
            builder.mov(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.masterInterruptEnable) - reinterpret_cast<uintptr_t>(&cpu));
            break;

        case 0xF5: // PUSH AF
            push(reg(WReg::AF));
            break;
        case 0xF6: // OR n
        {
            cycleExecuted();
            bitOr(instr.opcode[1]);
            break;
        }

        case 0xF9: // LD SP,HL
            cycleExecuted();
            builder.mov(spReg32, static_cast<Reg32>(reg(WReg::HL)));
            break;
        case 0xFA: // LD A,(nn)
        {
            uint16_t addr = instr.opcode[1] | instr.opcode[2] << 8;
            cycleExecuted();
            cycleExecuted();

            readMem(addr, reg(Reg::A));
            break;
        }
        case 0xFB: // EI
            // enableInterruptsNextCycle = true
            // TODO: if we store the CPU ptr then we can use a disp here
            builder.mov(Reg64::R10, reinterpret_cast<uintptr_t>(&cpu.enableInterruptsNextCycle));
            builder.mov(1, Reg64::R10);
            break;

        case 0xFE: // CP n
        {
            cycleExecuted();
            cmp(instr.opcode[1]);
            break;
        }

        default:
            printf("unhandled op in recompile %02X\n", opcode);
            builder.resetPtr(oldPtr);
            builder.mov(pcReg32, pc - 1);
            builder.jmp(exitPtr - builder.getPtr());
            return false;
    }

    syncCyclesExecuted();

    if(!(instr.flags & Op_Last)) // TODO: also safe to omit if there's an unconditional exit
    {
        // cycles -= executed
        if(cyclesThisInstr) // 0 means we already did the sub
            builder.sub(Reg32::EDI, cyclesThisInstr);

        lastInstrCycleCheck = builder.getPtr(); // save in case the next instr is a branch target

        // if <= 0 exit
        builder.jcc(Condition::G, 11);
        builder.mov(pcReg32, pc);
        builder.call(saveAndExitPtr - builder.getPtr());

        // interrupt check after EI
        if(checkInterrupts)
        {
            auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);

            // if servicableInterrupts != 0
            builder.cmp(0, Reg64::R14, reinterpret_cast<uintptr_t>(&cpu.serviceableInterrupts) - cpuPtr);
            builder.jcc(Condition::E, 11);
            builder.mov(pcReg32, pc);
            builder.call(saveAndExitPtr - builder.getPtr());
        }
    }

    return true;
}

void DMGRecompilerX86::recompileExInstruction(OpInfo &instr, X86Builder &builder)
{
    uint8_t opcode = instr.opcode[1];

    using Reg = DMGCPU::Reg;

    static const Reg8 regMap8[]
    {
        reg(Reg::B),
        reg(Reg::C),
        reg(Reg::D),
        reg(Reg::E),
        reg(Reg::H),
        reg(Reg::L),
        Reg8::R10B, // where we load to for (HL)
        reg(Reg::A),
    };

    // the flags here are more simple
    // all the shifts/rotates set C and Z
    // SWAP only sets Z
    const auto setFlags = [&instr, &builder](Reg8 r, bool isRotate = false, bool setC = true)
    {
        auto f = reg(Reg::F);

        if(setC && (instr.flags & DMGCPU::Flag_C))
        {
            builder.setcc(Condition::B, f);
            builder.shl(f, 4); // C is bit 4

            // also want the other one
            if(instr.flags & DMGCPU::Flag_Z)
            {
                builder.cmp(r, 0);
                builder.jcc(Condition::NE, 3); // if != 0
                builder.or_(f, DMGCPU::Flag_Z); // set Z
            }
        }
        else if(instr.flags & DMGCPU::Flag_Z)
        {
            if(isRotate)
                builder.cmp(r, 0); // rotates don't set ZF

            builder.setcc(Condition::E, f);
            builder.shl(f, 7); // Z is bit 7
        }
        else if(instr.flags & Op_WriteFlags)
            builder.mov(f, 0); // something relying on an unset flag
    };

    auto r = regMap8[opcode & 7];

    if(opcode < 0x40) // shifts/rotates
    {
        switch(opcode & ~7)
        {
            case 0x00: // RLC
                builder.rol(r, 1);
                setFlags(r, true);
                break;

            case 0x08: // RRC
                builder.ror(r, 1);
                setFlags(r, true);
                break;

            case 0x10: // RL
            {
                auto f = reg(Reg::F);

                // copy carry flag
                builder.test(f, DMGCPU::Flag_C); // sets CF to 0
                builder.jcc(Condition::E, 1); // not set
                builder.stc(); // CF = 1

                builder.rcl(r, 1);
                setFlags(r, true);
                break;
            }

            case 0x18: // RR
            {
                auto f = reg(Reg::F);

                // copy carry flag
                builder.test(f, DMGCPU::Flag_C); // sets CF to 0
                builder.jcc(Condition::E, 1); // not set
                builder.stc(); // CF = 1

                builder.rcr(r, 1);
                setFlags(r, true);
                break;
            }

            case 0x20: // SLA
                builder.shl(r, 1);
                setFlags(r);
                break;

            case 0x28: // SRA
                builder.sar(r, 1);
                setFlags(r);
                break;

            case 0x30: // SWAP
                builder.rol(r, 4); // v = (v >> 4) | (v << 4);
                setFlags(r, true, false);
                break;

            case 0x38: // SRL
                builder.shr(r, 1);
                setFlags(r);
                break;
        }
    }
    else if(opcode < 0x80) // BIT
    {
        int bit = (opcode >> 3) & 7;
        auto f = reg(Reg::F);

        if(instr.flags & Op_WriteFlags)
            builder.and_(f, DMGCPU::Flag_C); // preserve C

        if(instr.flags & DMGCPU::Flag_H)
            builder.or_(f, DMGCPU::Flag_H); // set H

        builder.test(r, 1 << bit);

        if(instr.flags & DMGCPU::Flag_Z)
        {
            builder.jcc(Condition::NE, 3); // if != 0
            builder.or_(f, DMGCPU::Flag_Z); // set Z
        }
    }
    else if(opcode < 0xC0) // RES
    {
        int bit = (opcode >> 3) & 7;
        builder.and_(r, ~(1 << bit));
    }
    else // SET
    {
        int bit = (opcode >> 3) & 7;
        builder.or_(r, 1 << bit);
    }
}


void DMGRecompilerX86::compileEntry()
{
    X86Builder builder(codeBuf, codeBuf + codeBufSize);

    // prologue
    builder.push(Reg64::RBP);
    builder.mov(Reg64::RBP, Reg64::RSP);

    // save
    builder.push(Reg64::R12);
    builder.push(Reg64::R13);
    builder.push(Reg64::R14);
    builder.push(Reg64::RBX);

    // store pointer to CPU
    auto cpuPtr = reinterpret_cast<uintptr_t>(&cpu);
    builder.mov(Reg64::R14, cpuPtr);

    // load emu sp
    auto spPtr = reinterpret_cast<uintptr_t>(&cpu.sp) - cpuPtr;
    builder.movzxW(spReg32, Reg64::R14, spPtr);

    // load emu regs
    auto regsPtr = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtr;
    builder.movzxW(Reg32::EAX, Reg64::R14, regsPtr);
    builder.movzxW(Reg32::ECX, Reg64::R14, regsPtr + 2);
    builder.movzxW(Reg32::EDX, Reg64::R14, regsPtr + 4);
    builder.movzxW(Reg32::EBX, Reg64::R14, regsPtr + 6);

    // jump to code
    builder.jmp(Reg64::RSI);

    // exit setting the call flag ... and saving ip
    exitForCallPtr = builder.getPtr();
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(&exitCallFlag));
    builder.mov(1, Reg64::R11);

    // exit saving ip
    saveAndExitPtr = builder.getPtr();
    builder.pop(Reg64::R10); // ret address (this is called)
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(&tmpSavedPtr));
    builder.mov(Reg64::R10, Reg64::R11, true);

    // just exit
    exitPtr = builder.getPtr();

    // save emu regs
    builder.mov(Reg16::AX, Reg64::R14, true, regsPtr);
    builder.mov(Reg16::CX, Reg64::R14, true, regsPtr + 2);
    builder.mov(Reg16::DX, Reg64::R14, true, regsPtr + 4);
    builder.mov(Reg16::BX, Reg64::R14, true, regsPtr + 6);

    // save emu pc/sp
    builder.mov(pcReg16, Reg64::R14, true, reinterpret_cast<uintptr_t>(&cpu.pc) - cpuPtr);
    builder.mov(spReg16, Reg64::R14, true, spPtr);

    // restore
    builder.pop(Reg64::RBX);
    builder.pop(Reg64::R14);
    builder.pop(Reg64::R13);
    builder.pop(Reg64::R12);

    // epilogue
    builder.pop(Reg64::RBP);
    builder.ret();

    entryFunc = reinterpret_cast<CompiledFunc>(codeBuf);

    curCodePtr = builder.getPtr();

#ifdef RECOMPILER_DEBUG
    int len = builder.getPtr() - codeBuf;

    //debug
    printf("generated %i bytes for entry/exit\ncode:", len);

    for(auto p = codeBuf; p !=curCodePtr; p++)
        printf(" %02X", *p);

    printf("\n");
#endif
}
