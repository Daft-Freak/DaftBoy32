#include <cstdio>
#include <cstdlib> //exit
#include <cstring>

#include "DMGCPU.h"
#include "DMGRegs.h"

// TODO: move memory handing out of CPU?

void DMGCPU::loadCartridge(const uint8_t *rom, uint32_t romLen)
{
    cartROM = rom;
    cartROMLen = romLen;

    switch(cartROM[0x147])
    {
        case 0:
            mbcType = MBCType::None;
            break;
        case 1:
        case 2: // + RAM
        case 3: // + RAM + Battery
            mbcType = MBCType::MBC1;
            break;

        default:
            printf("unhandled cartridge type %x\n", cartROM[0x147]);
            mbcType = MBCType::None;
    }

    mbcRAMEnabled = false;
    mbcROMBank = 1;
    mbcRAMBank = 0;
    mbcRAMBankMode = false;
}

void DMGCPU::reset()
{
    stopped = halted = false;
    masterInterruptEnable = false; //?

    // values after boot rom
    pc = 0x100;
    reg(WReg::AF) = 0x01B0;
    reg(WReg::BC) = 0x0013;
    reg(WReg::DE) = 0x00D8;
    reg(WReg::HL) = 0x01D4;
    sp = 0xFFFE;

    // io regs
    memset(iohram, 0xFF, 0x80);

    iohram[IO_TIMA] = 0x00; // TIMA
    iohram[IO_TMA] = 0x00; // TMA
    iohram[IO_TAC] = 0x00; // TAC
    iohram[0x10] = 0x80; // NR10
    iohram[0x11] = 0xBF; // NR11
    iohram[0x12] = 0xF3; // NR12
    iohram[0x14] = 0xBF; // NR14
    iohram[0x16] = 0x3F; // NR21
    iohram[0x17] = 0x00; // NR22
    iohram[0x19] = 0xBF; // NR24
    iohram[0x1A] = 0x7F; // NR30
    iohram[0x1B] = 0xFF; // NR31
    iohram[0x1C] = 0x9F; // NR32
    iohram[0x1E] = 0xBF; // NR33
    iohram[0x20] = 0xFF; // NR41
    iohram[0x21] = 0x00; // NR42
    iohram[0x22] = 0x00; // NR43
    iohram[0x23] = 0xBF; // NR30
    iohram[0x24] = 0x77; // NR50
    iohram[0x25] = 0xF3; // NR51
    iohram[0x26] = 0xF1; // NR52
    iohram[IO_LCDC] = 0x91; // LCDC
    iohram[IO_SCY] = 0x00; // SCY
    iohram[IO_SCX] = 0x00; // SCX
    iohram[IO_LYC] = 0x00; // LYC
    iohram[IO_BGP] = 0xFC; // BGP
    iohram[IO_OBP0] = 0xFF; // OBP0
    iohram[IO_OBP1] = 0xFF; // OBP1
    iohram[IO_WY] = 0x00; // WY
    iohram[IO_WX] = 0x00; // WX
    iohram[IO_IE] = 0x00; // IE
}

void DMGCPU::run(int ms)
{
    const int timerCycles[]
    {
        clockSpeed / 4096,
        clockSpeed / 262144,
        clockSpeed / 65536,
        clockSpeed / 16384
    };

    int cycles = (clockSpeed * ms) / 1000;

    while(!stopped && cycles > 0)
    {
        int exec = halted ? 1 : executeInstruction();
        if(cycleCallback)
            cycleCallback(exec, iohram);

        // divider
        divCounter += exec;
        if(divCounter >= 255)
        {
            divCounter -= 255;
            iohram[IO_DIV]++;
        }

        // timer
        if(iohram[IO_TAC] & TAC_Start)
        {
            timerCounter += exec;
            int timCylces = timerCycles[iohram[IO_TAC] & TAC_Clock];
            if(timerCounter > timCylces)
            {
                timerCounter -= timCylces;

                if(iohram[IO_TIMA] == 0xFF)
                {
                    //overflow
                    iohram[IO_TIMA] = iohram[IO_TMA];
                    flagInterrupt(Int_Timer);
                }
                else
                    iohram[IO_TIMA]++;
            }
        }

        // interrupts
        auto &flag = iohram[IO_IF];
        auto enabled = iohram[IO_IE];
        const uint16_t vectors[]{0x40, 0x48, 0x50, 0x58, 0x60};

        for(int i = 0; i < 5; i++)
        {
            int intBit = 1 << i;
            if((enabled & intBit) && (flag & intBit))
            {
                halted = false; // un-halt even if interrupts are disabled

                if(masterInterruptEnable)
                {
                    masterInterruptEnable = false;
                    flag &= ~intBit;

                    // call vector
                    sp -= 2;
                    writeMem16(sp, pc);
                    pc = vectors[i];
                }
            }
        }

        cycles -= exec;
    }
}

void DMGCPU::setCycleCallback(CycleCallback cycleCallback)
{
    this->cycleCallback = cycleCallback;
}

void DMGCPU::flagInterrupt(int interrupt)
{
    iohram[IO_IF] |= interrupt;
}

void DMGCPU::setInputs(uint8_t inputs)
{
    if(rawInputs == 0 && inputs != 0)
        flagInterrupt(Int_Joypad);

    rawInputs = inputs;
}

uint8_t DMGCPU::readMem(uint16_t addr) const
{
    if(addr < 0x8000)
    {
        if(addr > 0x4000) // handle banking
            addr += (mbcROMBank - 1) * 0x4000;

        if(addr < cartROMLen)
            return cartROM[addr];
    }
    else if(addr < 0xA000)
        return vram[addr - 0x8000];
    else if(addr < 0xC000)
    {} // cart ram
    else if(addr < 0xE000)
        return wram[addr - 0xC000];
    else if(addr < 0xFE00)
    {} // echo
    else if(addr < 0xFEA0)
        return oam[addr - 0xFE00];
    else if(addr < 0xFF00)
    {} //unusable
    else
    {
        if((addr & 0xFF) == IO_STAT)
            printf("r STAT @~%x\n", pc);

        // input
        if(addr == 0xFF00)
        {
            int ret = iohram[addr & 0xFF] & 0xF0;
            if(!(iohram[IO_JOYP] & JOYP_SelectDir))
                ret |= (~rawInputs) & 0xF;
            if(!(iohram[IO_JOYP] & JOYP_SelectButtons))
                ret |= (~rawInputs) >> 4;
            return ret;
        }

        return iohram[addr & 0xFF];
    }

    printf("read %x @%x\n", addr, pc);
    return 0;
}

void DMGCPU::writeMem(uint16_t addr, uint8_t data)
{
    if(addr < 0x8000)
    {
        writeMBC(addr, data); // cart rom
        return;
    }
    else if(addr < 0xA000)
    {
        vram[addr - 0x8000] = data;
        return;
    }
    else if(addr < 0xC000)
    {} // cart ram
    else if(addr < 0xE000)
    {
        wram[addr - 0xC000] = data;
        return;
    }
    else if(addr < 0xFE00)
    {} // echo
    else if(addr < 0xFEA0)
    {
        oam[addr - 0xFE00] = data;
        return;
    }
    else if(addr < 0xFF00)
    {
        //unusable
        return;
    }
    else
    {
        if((addr & 0xFF) == 0x46)
        {
            //TODO: put this not here
            //printf("DMA from %x @~%x\n", data << 8, pc);
            for(int i = 0; i < 0xA0; i++)
                oam[i] = readMem((data << 8) + i);
        }

        if((addr & 0xFF) == IO_LY || (addr & 0xFF) == IO_DIV)
            iohram[addr & 0xFF] = 0; // clear on write
        else
            iohram[addr & 0xFF] = data;
        return;
    }

    printf("write %x = %x @%x\n", addr, data, pc);
}

uint16_t DMGCPU::readMem16(uint16_t addr) const
{
    return readMem(addr) | (readMem(addr + 1) << 8);
}

void DMGCPU::writeMem16(uint16_t addr, uint16_t data)
{
    writeMem(addr, data);
    writeMem(addr + 1, data >> 8);
}

// returns cycle count
int DMGCPU::executeInstruction()
{
    // helpers
    const auto load8 = [this](Reg r)
    {
        reg(r) = readMem(pc++);
        return 8;
    };

    const auto copy8 = [this](Reg dst, Reg src)
    {
        reg(dst) = reg(src);
        return 4;
    };

    const auto load16 = [this](WReg r)
    {
        reg(r) = readMem16(pc);
        pc += 2;
        return 12;
    };

    const auto push = [this](WReg r)
    {
        sp -= 2;
        writeMem16(sp, reg(r));
        return 12;
    };

    const auto pop = [this](WReg r)
    {
        reg(r) = readMem16(sp);
        sp += 2;

        // low bits in F can never be set
        if(r == WReg::AF)
            reg(Reg::F) &= 0xF0;

        return 12;
    };

    const auto doAdd = [this](uint8_t a, uint8_t b, uint8_t c = 0)
    {
        uint16_t v = a + b + c;
        reg(Reg::A) = v;

        auto hV = (a & 0xF) + (b & 0xF) + c;

        reg(Reg::F) = (v > 0xFF ? Flag_C : 0) |
                      (hV > 0xF ? Flag_H : 0) |
                      ((v & 0xFF) == 0 ? Flag_Z : 0);
    };

    const auto add = [this, &doAdd](Reg r)
    {
        auto a = reg(Reg::A);
        auto b = reg(r);
        doAdd(a, b);

        return 4;
    };

    const auto addWithCarry = [this, &doAdd](Reg r)
    {
        auto a = reg(Reg::A);
        auto b = reg(r);
        doAdd(a, b, (reg(Reg::F) & Flag_C) ? 1 : 0);

        return 4;
    };

    const auto doSub = [this](uint8_t a, uint8_t b, uint8_t c = 0)
    {
        int v = a - (b + c);
        reg(Reg::A) = v & 0xFF;

        int hV = (a & 0xF) - (b & 0xF) - c;

        reg(Reg::F) = (v < 0 ? Flag_C : 0) |
                      (hV < 0 ? Flag_H : 0) |
                      Flag_N |
                      ((v & 0xFF) == 0 ? Flag_Z : 0);
    };

    const auto sub = [this, &doSub](Reg r)
    {
        auto a = reg(Reg::A);
        auto b = reg(r);
        doSub(a, b);

        return 4;
    };

    const auto subWithCarry = [this, &doSub](Reg r)
    {
        auto a = reg(Reg::A);
        auto b = reg(r);
        doSub(a, b, (reg(Reg::F) & Flag_C) ? 1 : 0);

        return 4;
    };

    const auto bitAnd = [this](Reg r)
    {
        auto res = reg(Reg::A) & reg(r);
        reg(Reg::A) = res;
        reg(Reg::F) = Flag_H | (res == 0 ? Flag_Z : 0);
        return 4;
    };

    const auto bitOr = [this](Reg r)
    {
        auto res = reg(Reg::A) | reg(r);
        reg(Reg::A) = res;
        reg(Reg::F) = res == 0 ? Flag_Z : 0;
        return 4;
    };

    const auto bitXor = [this](Reg r)
    {
        auto res = reg(Reg::A) ^ reg(r);
        reg(Reg::A) = res;
        reg(Reg::F) = res == 0 ? Flag_Z : 0;
        return 4;
    };

    const auto cmp = [this](Reg r)
    {
        auto a = reg(Reg::A);
        auto b = reg(r);
        //auto v = a - b;

        reg(Reg::F) = (a < b ? Flag_C : 0) |
                      ((a & 0xF) < (b & 0xF) ? Flag_H : 0) |
                      Flag_N |
                      (a == b ? Flag_Z : 0);

        return 4;
    };

    const auto inc = [this](Reg r)
    {
        auto v = reg(r)++;

        reg(Reg::F) = (reg(Reg::F) & Flag_C) |
                      ((v & 0xF) == 0xF ? Flag_H : 0) |
                      (v == 0xFF ? Flag_Z : 0);

        return 4;
    };

    const auto dec = [this](Reg r)
    {
        auto v = reg(r)--;

        reg(Reg::F) = (reg(Reg::F) & Flag_C) |
                      ((v & 0xF) == 0 ? Flag_H : 0) |
                      Flag_N |
                      (v == 1 ? Flag_Z : 0);

        return 4;
    };

    const auto doAdd16 = [this](uint16_t a, uint16_t b)
    {
        uint32_t v = a + b;
        reg(WReg::HL) = v;

        reg(Reg::F) = (v > 0xFFFF ? Flag_C : 0) |
                      ((a & 0xFFF) + (b & 0xFFF) > 0xFFF ? Flag_H : 0) | //?
                      (reg(Reg::F) & Flag_Z);
    };

    const auto add16 = [this, &doAdd16](WReg r)
    {
        auto a = reg(WReg::HL);
        auto b = reg(r);
        doAdd16(a, b);

        return 8;
    };

    const auto inc16 = [this](WReg r)
    {
        reg(r)++;
        return 8;
    };

    const auto dec16 = [this](WReg r)
    {
        reg(r)--;
        return 8;
    };

    const auto jump = [this](int flag = 0, bool set = true)
    {
        auto addr = readMem16(pc);
        pc += 2;
        if(flag == 0 || !!(reg(Reg::F) & flag) == set)
            pc = addr;

        return 12;
    };

    const auto jumpRel = [this](int flag = 0, bool set = true)
    {
        int8_t off = readMem(pc++);
        if(flag == 0 || !!(reg(Reg::F) & flag) == set)
            pc += off;

        return 8;
    };

    const auto call = [this](int flag = 0, bool set = true)
    {
        auto addr = readMem16(pc);
        pc += 2;

        if(flag == 0 || !!(reg(Reg::F) & flag) == set)
        {
            sp -= 2;
            writeMem16(sp, pc);
            pc = addr;
        }

        return 12;
    };

    const auto reset = [this](int addr)
    {
        sp -= 2;
        writeMem16(sp, pc);
        pc = addr;
        return 32;
    };

    const auto ret = [this](int flag = 0, bool set = true)
    {
        auto addr = readMem16(sp);

        if(flag == 0 || !!(reg(Reg::F) & flag) == set)
        {
            sp += 2;
            pc = addr;
        }

        return 8;
    };

    auto opcode = readMem(pc++);

    switch(opcode)
    {
        case 0x00: // NOP
            return 4;

        case 0x01: // LD BC,nn
            return load16(WReg::BC);

        case 0x02: // LD (BC),A
            writeMem(reg(WReg::BC), reg(Reg::A));
            return 8;

        case 0x03: // INC BC
            return inc16(WReg::BC);
        case 0x04: // INC B
            return inc(Reg::B);
        case 0x05: // DEC B
            return dec(Reg::B);

        case 0x06: // LD B,n
            return load8(Reg::B);

        case 0x07: // RLCA
        {
            auto v = reg(Reg::A);
            bool c = v & 0x80;
            v = (v << 1) | (v >> 7);
            reg(Reg::A) = v;

            reg(Reg::F) = (c ? Flag_C : 0);

            return 4;
        }

        case 0x08: // LD, (nn),SP
        {
            auto addr = readMem16(pc);
            pc += 2;
            writeMem16(addr, sp);
            return 20;
        }

        case 0x09: // ADD HL,BC
            return add16(WReg::BC);

        case 0x0A: // LDI A,BC
            reg(Reg::A) = readMem(reg(WReg::BC));
            return 8;

        case 0x0B: // DEC BC
            return dec16(WReg::BC);

        case 0x0C: // INC C
            return inc(Reg::C);

        case 0x0D: // DEC C
            return dec(Reg::C);

        case 0x0E: // LD C,n
            return load8(Reg::C);

        case 0x0F: // RRCA
        {
            auto v = reg(Reg::A);
            bool c = v & 0x01;
            v = (v >> 1) | (v << 7);
            reg(Reg::A) = v;

            reg(Reg::F) = (c ? Flag_C : 0);

            return 4;
        }

        case 0x10: // STOP
            stopped = true;
            break;

        case 0x11: // LD DE,nn
            return load16(WReg::DE);

        case 0x12: // LD (DE),A
            writeMem(reg(WReg::DE), reg(Reg::A));
            return 8;

        case 0x13: // INC DE
            return inc16(WReg::DE);

        case 0x14: // INC D
            return inc(Reg::D);

        case 0x15: // DEC D
            return dec(Reg::D);

        case 0x16: // LD D,n
            return load8(Reg::D);

        case 0x17: // RLA
        {
            bool c = reg(Reg::A) & 0x80;
            auto res = (reg(Reg::A) << 1) | ((reg(Reg::F) & Flag_C) ? 0x01 : 0);
            reg(Reg::A) = res;

            reg(Reg::F) = (c ? Flag_C : 0);

            return 4;
        }

        case 0x18: // JR m
            return jumpRel();

        case 0x19: // ADD HL,DE
            return add16(WReg::DE);

        case 0x1A: // LDI A,DE
            reg(Reg::A) = readMem(reg(WReg::DE));
            return 8;

        case 0x1B: // DEC DE
            return dec16(WReg::DE);

        case 0x1C: // INC E
            return inc(Reg::E);

        case 0x1D: // DEC E
            return dec(Reg::E);

        case 0x1E: // LD E,n
            return load8(Reg::E);

        case 0x1F: // RRA
        {
            bool c = reg(Reg::A) & 1;
            auto res = (reg(Reg::A) >> 1) | ((reg(Reg::F) & Flag_C) ? 0x80 : 0);
            reg(Reg::A) = res;

            reg(Reg::F) = (c ? Flag_C : 0);

            return 4;
        }

        case 0x20: // JR NZ,n
            return jumpRel(Flag_Z, false);

        case 0x21: // LD HL,nn
            return load16(WReg::HL);

        case 0x22: // LDI (HL),A
            writeMem(reg(WReg::HL)++, reg(Reg::A));
            return 8;

        case 0x23: // INC HL
            return inc16(WReg::HL);
        case 0x24: // INC H
            return inc(Reg::H);

        case 0x25: // DEC H
            return dec(Reg::H);

        case 0x26: // LD H,n
            return load8(Reg::H);

        case 0x27: // DAA
        {
            int flags = reg(Reg::F);
            uint8_t val = reg(Reg::A);
            int newFlags = flags & ~(Flag_H | Flag_Z);

            if(flags & Flag_N)
            {
                // sub
                if(flags & Flag_C)
                    val -= 0x60;
                if(flags & Flag_H)
                    val -= 0x06;
            }
            else
            {
                // add
                if((flags & Flag_C) || val > 0x99)
                {
                    val += 0x60;
                    newFlags |= Flag_C;
                }

                if((flags & Flag_H) || (val & 0x0F) > 0x09)
                    val += 0x06;
            }

            reg(Reg::A) = val;
            reg(Reg::F) = newFlags | (val == 0 ? Flag_Z : 0);
            
            return 4;
        }

        case 0x28: // JR Z,n
            return jumpRel(Flag_Z);

        case 0x29: // ADD HL,HL
            return add16(WReg::HL);

        case 0x2A: // LDI A,(HL)
            reg(Reg::A) = readMem(reg(WReg::HL)++);
            return 8;

        case 0x2B: // DEC HL
            return dec16(WReg::HL);

        case 0x2C: // INC L
            return inc(Reg::L);

        case 0x2D: // DEC L
            return dec(Reg::L);

        case 0x2E: // LD L,n
            return load8(Reg::L);

        case 0x2F: // CPL
            reg(Reg::A) = ~reg(Reg::A);
            reg(Reg::F) |= Flag_H | Flag_N;
            return 4;

        case 0x30: // JR NC,n
            return jumpRel(Flag_C, false);

        case 0x31: // LD SP,nn
            sp = readMem16(pc);
            pc += 2;
            return 12;

        case 0x32: // LDD (HL), A
            writeMem(reg(WReg::HL)--, reg(Reg::A));
            return 8;

        case 0x33: // INC SP
            sp++;
            return 8;

        case 0x34: // INC (HL)
        {
            auto v = readMem(reg(WReg::HL));
            writeMem(reg(WReg::HL), v + 1);

            reg(Reg::F) = (reg(Reg::F) & Flag_C) |
                          ((v & 0xF) == 0xF ? Flag_H : 0) |
                          (v == 0xFF ? Flag_Z : 0);

            return 12;
        }
        case 0x35: // DEC (HL)
        {
            auto v = readMem(reg(WReg::HL));
            writeMem(reg(WReg::HL), v - 1);

            reg(Reg::F) = (reg(Reg::F) & Flag_C) |
                          ((v & 0xF) == 0 ? Flag_H : 0) |
                          Flag_N |
                          (v == 1 ? Flag_Z : 0);

            return 12;
        }

        case 0x36: // LD (HL),n
            writeMem(reg(WReg::HL), readMem(pc++));
            return 12;

        case 0x37: // SCF
            reg(Reg::F) = Flag_C | (reg(Reg::F) & Flag_Z);
            return 4;

        case 0x38: // JR C,n
            return jumpRel(Flag_C);

        case 0x39: // ADD HL,SP
            doAdd16(reg(WReg::HL), sp);
            return 8;

        case 0x3A: // LDD A,(HL)
            reg(Reg::A) = readMem(reg(WReg::HL)--);
            return 8;

        case 0x3B: // DEC SP
            sp--;
            return 8;

        case 0x3C: // INC A
            return inc(Reg::A);

        case 0x3D: // DEC A
            return dec(Reg::A);

        case 0x3E: // LD A,#
            reg(Reg::A) = readMem(pc++);
            return 8;

        case 0x3F: // CCF
            reg(Reg::F) = (~reg(Reg::F) & Flag_C) | (reg(Reg::F) & Flag_Z);
            return 4;

        case 0x40: // LD B,B
            return copy8(Reg::B, Reg::B);
        case 0x41: // LD B,C
            return copy8(Reg::B, Reg::C);
        case 0x42: // LD B,D
            return copy8(Reg::B, Reg::D);
        case 0x43: // LD B,E
            return copy8(Reg::B, Reg::E);
        case 0x44: // LD B,H
            return copy8(Reg::B, Reg::H);
        case 0x45: // LD B,L
            return copy8(Reg::B, Reg::L);
        case 0x46: // LD B,(HL)
            reg(Reg::B) = readMem(reg(WReg::HL));
            return 8;
        case 0x47: // LD B,A
            return copy8(Reg::B, Reg::A);
        case 0x48: // LD C,B
            return copy8(Reg::C, Reg::B);
        case 0x49: // LD C,C
            return copy8(Reg::C, Reg::C);
        case 0x4A: // LD C,D
            return copy8(Reg::C, Reg::D);
        case 0x4B: // LD C,E
            return copy8(Reg::C, Reg::E);
        case 0x4C: // LD C,H
            return copy8(Reg::C, Reg::H);
        case 0x4D: // LD C,L
            return copy8(Reg::C, Reg::L);
        case 0x4E: // LD C,(HL)
            reg(Reg::C) = readMem(reg(WReg::HL));
            return 8;
        case 0x4F: // LD C,A
            return copy8(Reg::C, Reg::A);
        case 0x50: // LD D,B
            return copy8(Reg::D, Reg::B);
        case 0x51: // LD D,C
            return copy8(Reg::D, Reg::C);
        case 0x52: // LD D,D
            return copy8(Reg::D, Reg::D);
        case 0x53: // LD D,E
            return copy8(Reg::D, Reg::E);
        case 0x54: // LD D,H
            return copy8(Reg::D, Reg::H);
        case 0x55: // LD D,L
            return copy8(Reg::D, Reg::L);
        case 0x56: // LD D,(HL)
            reg(Reg::D) = readMem(reg(WReg::HL));
            return 8;
        case 0x57: // LD D,A
            return copy8(Reg::D, Reg::A);
        case 0x58: // LD E,B
            return copy8(Reg::E, Reg::B);
        case 0x59: // LD E,C
            return copy8(Reg::E, Reg::C);
        case 0x5A: // LD E,D
            return copy8(Reg::E, Reg::D);
        case 0x5B: // LD E,E
            return copy8(Reg::E, Reg::E);
        case 0x5C: // LD E,H
            return copy8(Reg::E, Reg::H);
        case 0x5D: // LD E,L
            return copy8(Reg::E, Reg::L);
        case 0x5E: // LD E,(HL)
            reg(Reg::E) = readMem(reg(WReg::HL));
            return 8;
        case 0x5F: // LD E,A
            return copy8(Reg::E, Reg::A);
        case 0x60: // LD H,B
            return copy8(Reg::H, Reg::B);
        case 0x61: // LD H,C
            return copy8(Reg::H, Reg::C);
        case 0x62: // LD H,D
            return copy8(Reg::H, Reg::D);
        case 0x63: // LD H,E
            return copy8(Reg::H, Reg::E);
        case 0x64: // LD H,H
            return copy8(Reg::H, Reg::H);
        case 0x65: // LD H,L
            return copy8(Reg::H, Reg::L);
        case 0x66: // LD H,(HL)
            reg(Reg::H) = readMem(reg(WReg::HL));
            return 8;
        case 0x67: // LD H,A
            return copy8(Reg::H, Reg::A);
        case 0x68: // LD L,B
            return copy8(Reg::L, Reg::B);
        case 0x69: // LD L,C
            return copy8(Reg::L, Reg::C);
        case 0x6A: // LD L,D
            return copy8(Reg::L, Reg::D);
        case 0x6B: // LD L,E
            return copy8(Reg::L, Reg::E);
        case 0x6C: // LD L,H
            return copy8(Reg::L, Reg::H);
        case 0x6D: // LD L,L
            return copy8(Reg::L, Reg::L);
        case 0x6E: // LD L,(HL)
            reg(Reg::L) = readMem(reg(WReg::HL));
            return 8;
        case 0x6F: // LD L,A
            return copy8(Reg::L, Reg::A);

        case 0x70: // LD (HL),B
            writeMem(reg(WReg::HL), reg(Reg::B));
            return 8;
        case 0x71: // LD (HL),C
            writeMem(reg(WReg::HL), reg(Reg::C));
            return 8;
        case 0x72: // LD (HL),D
            writeMem(reg(WReg::HL), reg(Reg::D));
            return 8;
        case 0x73: // LD (HL),E
            writeMem(reg(WReg::HL), reg(Reg::E));
            return 8;
        case 0x74: // LD (HL),H
            writeMem(reg(WReg::HL), reg(Reg::H));
            return 8;
        case 0x75: // LD (HL),L
            writeMem(reg(WReg::HL), reg(Reg::L));
            return 8;

        case 0x76: // HALT
            halted = true;
            return 4;

        case 0x77: // LD (HL),A
            writeMem(reg(WReg::HL), reg(Reg::A));
            return 8;

        case 0x78: // LD A,B
            return copy8(Reg::A, Reg::B);
        case 0x79: // LD A,C
            return copy8(Reg::A, Reg::C);
        case 0x7A: // LD A,D
            return copy8(Reg::A, Reg::D);
        case 0x7B: // LD A,E
            return copy8(Reg::A, Reg::E);
        case 0x7C: // LD A,H
            return copy8(Reg::A, Reg::H);
        case 0x7D: // LD A,L
            return copy8(Reg::A, Reg::L);
        case 0x7E: // LD A,(HL)
            reg(Reg::A) = readMem(reg(WReg::HL));
            return 8;
        case 0x7F: // LD A,A
            return copy8(Reg::A, Reg::A);

        case 0x80: // ADD A B
            return add(Reg::B);
        case 0x81: // ADD A C
            return add(Reg::C);
        case 0x82: // ADD A D
            return add(Reg::D);
        case 0x83: // ADD A E
            return add(Reg::E);
        case 0x84: // ADD A H
            return add(Reg::H);
        case 0x85: // ADD A L
            return add(Reg::L);
        case 0x86: // ADD A (HL)
            doAdd(reg(Reg::A), readMem(reg(WReg::HL)));
            return 8;
        case 0x87: // ADD A A
            return add(Reg::A);

        case 0x88: // ADC A,B
            return addWithCarry(Reg::B);
        case 0x89: // ADC A,C
            return addWithCarry(Reg::C);
        case 0x8A: // ADC A,D
            return addWithCarry(Reg::D);
        case 0x8B: // ADC A,E
            return addWithCarry(Reg::E);
        case 0x8C: // ADC A,F
            return addWithCarry(Reg::H);
        case 0x8D: // ADC A,H
            return addWithCarry(Reg::L);
        case 0x8E: // ADC A (HL)
            doAdd(reg(Reg::A), readMem(reg(WReg::HL)), (reg(Reg::F) & Flag_C) ? 1 : 0);
            return 8;
        case 0x8F: // ADC A,L
            return addWithCarry(Reg::A);

        case 0x90: // SUB B
            return sub(Reg::B);
        case 0x91: // SUB C
            return sub(Reg::C);
        case 0x92: // SUB D
            return sub(Reg::D);
        case 0x93: // SUB E
            return sub(Reg::E);
        case 0x94: // SUB H
            return sub(Reg::H);
        case 0x95: // SUB L
            return sub(Reg::L);
        case 0x96: // SUB (HL)
            doSub(reg(Reg::A), readMem(reg(WReg::HL)));
            return 8;
        case 0x97: // SUB A
            return sub(Reg::A);

        case 0x98: // SBC B
            return subWithCarry(Reg::B);
        case 0x99: // SBC C
            return subWithCarry(Reg::C);
        case 0x9A: // SBC D
            return subWithCarry(Reg::D);
        case 0x9B: // SBC E
            return subWithCarry(Reg::E);
        case 0x9C: // SBC H
            return subWithCarry(Reg::H);
        case 0x9D: // SBC L
            return subWithCarry(Reg::L);
        case 0x9E: // SBC (HL)
            doSub(reg(Reg::A), readMem(reg(WReg::HL)), (reg(Reg::F) & Flag_C) ? 1 : 0);
            return 8;
        case 0x9F: // SBC A
            return subWithCarry(Reg::A);

        case 0xA0: // AND B
            return bitAnd(Reg::B);
        case 0xA1: // AND C
            return bitAnd(Reg::C);
        case 0xA2: // AND D
            return bitAnd(Reg::D);
        case 0xA3: // AND E
            return bitAnd(Reg::E);
        case 0xA4: // AND H
            return bitAnd(Reg::H);
        case 0xA5: // AND L
            return bitAnd(Reg::L);
        case 0xA6: // AND (HL)
        {
            auto res = reg(Reg::A) & readMem(reg(WReg::HL));
            reg(Reg::A) = res;
            reg(Reg::F) = Flag_H | (res == 0 ? Flag_Z : 0);
            return 8;
        }
        case 0xA7: // AND A
            return bitAnd(Reg::A);

        case 0xA8: // XOR B
            return bitXor(Reg::B);
        case 0xA9: // XOR C
            return bitXor(Reg::C);
        case 0xAA: // XOR D
            return bitXor(Reg::D);
        case 0xAB: // XOR E
            return bitXor(Reg::E);
        case 0xAC: // XOR H
            return bitXor(Reg::H);
        case 0xAD: // XOR L
            return bitXor(Reg::L);
        case 0xAE: // XOR (HL)
        {
            auto res = reg(Reg::A) = reg(Reg::A) ^ readMem(reg(WReg::HL));
            reg(Reg::F) = res == 0 ? Flag_Z : 0;
            return 8;
        }
        case 0xAF: // XOR A
            reg(Reg::A) = 0; // A ^ A == 0
            reg(Reg::F) = Flag_Z;
            return 4;

        case 0xB0: // OR B
            return bitOr(Reg::B);
        case 0xB1: // OR C
            return bitOr(Reg::C);
        case 0xB2: // OR D
            return bitOr(Reg::D);
        case 0xB3: // OR E
            return bitOr(Reg::E);
        case 0xB4: // OR H
            return bitOr(Reg::H);
        case 0xB5: // OR L
            return bitOr(Reg::L);
        case 0xB6: // OR (HL)
        {
            auto res = reg(Reg::A) = reg(Reg::A) | readMem(reg(WReg::HL));
            reg(Reg::F) = res == 0 ? Flag_Z : 0;
            return 8;
        }
        case 0xB7: // OR A
            bitOr(Reg::A); // just a Z flag update...
            break;

        case 0xB8: // CP B
            return cmp(Reg::B);
        case 0xB9: // CP C
            return cmp(Reg::C);
        case 0xBA: // CP D
            return cmp(Reg::D);
        case 0xBB: // CP E
            return cmp(Reg::E);
        case 0xBC: // CP H
            return cmp(Reg::H);
        case 0xBD: // CP L
            return cmp(Reg::L);
        case 0xBE: // CP (HL)
        {
            auto a = reg(Reg::A);
            auto b = readMem(reg(WReg::HL));

            reg(Reg::F) = (a < b ? Flag_C : 0) |
                        ((a & 0xF) < (b & 0xF) ? Flag_H : 0) |
                        Flag_N |
                        (a == b ? Flag_Z : 0);

            return 8;
        }
        case 0xBF: // CP A
            return cmp(Reg::A);

        case 0xC0: // RET NZ
            return ret(Flag_Z, false);

        case 0xC1: // POP BC
            return pop(WReg::BC);

        case 0xC2: // JP NZ,nn
            return jump(Flag_Z, false);

        case 0xC3: // JP nn
            return jump();

        case 0xC4: // CALL NZ,nn
            return call(Flag_Z, false);

        case 0xC5: // PUSH BC
            return push(WReg::BC);

        case 0xC6: // ADD A,#
            doAdd(reg(Reg::A), readMem(pc++));
            return 8;

        case 0xC7: // RST 0
            return reset(0x00);

        case 0xC8: // RET Z
            return ret(Flag_Z);
        case 0xC9: // RET
            return ret();

        case 0xCA: // JP Z,nn
            return jump(Flag_Z);

        case 0xCB: // extended ops
            return executeExInstruction();

        case 0xCC: // CALL Z,nn
            return call(Flag_Z);

        case 0xCD: // CALL nn
            return call();

        case 0xCE: // ADC A,#
            doAdd(reg(Reg::A), readMem(pc++), (reg(Reg::F) & Flag_C) ? 1 : 0);
            return 8;

        case 0xCF: // RST 08
            return reset(0x08);

        case 0xD0: // RET NC
            return ret(Flag_C, false);

        case 0xD1: // POP DE
            return pop(WReg::DE);

        case 0xD2: // JP NC,nn
            return jump(Flag_C, false);
        case 0xD4: // CALL NC,nn
            return call(Flag_C, false);

        case 0xD5: // PUSH DE
            return push(WReg::DE);

        case 0xD6: // SUB #
            doSub(reg(Reg::A), readMem(pc++));
            return 8;

        case 0xD7: // RST 10
            return reset(0x10);

        case 0xD8: // RET C
            return ret(Flag_C);
        case 0xD9: // RETI
            masterInterruptEnable = true;
            return ret();

        case 0xDA: // JP C,nn
            return jump(Flag_C);

        case 0xDC: // CALL C,nn
            return call(Flag_C);

        case 0xDE: // SBC A,#
            doSub(reg(Reg::A), readMem(pc++), (reg(Reg::F) & Flag_C) ? 1 : 0);
            return 8;

        case 0xDF: // RST 18
            return reset(0x18);

        case 0xE0: // LDH (n),A
            writeMem(0xFF00 | readMem(pc++), reg(Reg::A));
            return 12;

        case 0xE1: // POP HL
            return pop(WReg::HL);

        case 0xE2: // LDH (C),A
            writeMem(0xFF00 | reg(Reg::C), reg(Reg::A));
            return 8;

        case 0xE5: // PUSH HL
            return push(WReg::HL);

        case 0xE6: // AND #
        {
            auto v = reg(Reg::A) & readMem(pc++);
            reg(Reg::A) = v;
            reg(Reg::F) = Flag_H | (v == 0 ? Flag_Z : 0);
            return 8;
        }

        case 0xE7: // RST 20
            return reset(0x20);

        case 0xE8: // ADD SP,n
        {
            // flags are set as if this is an 8 bit op
            auto a = sp & 0xFF;
            auto b = readMem(pc++);
            uint16_t v = a + b;
            sp = sp + (int8_t)b;

            auto hV = (a & 0xF) + (b & 0xF);

            reg(Reg::F) = (v > 0xFF ? Flag_C : 0) |
                        (hV > 0xF ? Flag_H : 0);

            return 16;
        }

        case 0xE9: // JP (HL)
            pc = reg(WReg::HL);
            return 4;

        case 0xEA: // LD (nn),A
            writeMem(readMem16(pc), reg(Reg::A));
            pc += 2;
            return 16;

        case 0xEE: // XOR #
        {
            auto v = reg(Reg::A) ^ readMem(pc++);
            reg(Reg::A) = v;
            reg(Reg::F) = (v == 0 ? Flag_Z : 0);
            return 8;
        }

        case 0xEF: // RST 28
            return reset(0x28);

        case 0xF0: // LDH A,(n)
            reg(Reg::A) = readMem(0xFF00 | readMem(pc++));
            return 12;

        case 0xF1: // POP AF
            return pop(WReg::AF);

        case 0xF2: // LDH A,(C)
            reg(Reg::A) = readMem(0xFF00 | reg(Reg::C));
            return 8;

        case 0xF3: // DI
            masterInterruptEnable = false; // TODO: after next instruction
            return 4;

        case 0xF5: // PUSH AF
            return push(WReg::AF);

        case 0xF6: // OR #
        {
            auto res = reg(Reg::A) | readMem(pc++);
            reg(Reg::A) = res;
            reg(Reg::F) = res == 0 ? Flag_Z : 0;
            return 8;
        }

        case 0xF7: // RST 30
            return reset(0x30);

        case 0xF8: // LDHL SP,n
        {
            // flags are set as if this is an 8 bit op
            auto a = sp & 0xFF;
            auto b = readMem(pc++);
            uint16_t v = a + b;
            reg(WReg::HL) = sp + (int8_t)b;

            auto hV = (a & 0xF) + (b & 0xF);

            reg(Reg::F) = (v > 0xFF ? Flag_C : 0) |
                        (hV > 0xF ? Flag_H : 0);
            return 12;
        }

        case 0xF9: // LD SP,HL
            sp = reg(WReg::HL);
            return 8;

        case 0xFA: // LD A,(nn)
            reg(Reg::A) = readMem(readMem16(pc));
            pc += 2;
            return 16;

        case 0xFB: // EI
            masterInterruptEnable = true; // TODO: after next instruction
            return 4;

        case 0xFE: // CP n
        {
            auto a = reg(Reg::A);
            auto b = readMem(pc++);
            //auto v = a - b;

            reg(Reg::F) = (a < b ? Flag_C : 0) |
                        ((a & 0xF) < (b & 0xF) ? Flag_H : 0) |
                        Flag_N |
                        (a == b ? Flag_Z : 0);

            return 8;
        }

        case 0xFF: // RST 38
            return reset(0x38);

        default:
            printf("op %x @%x\n", (int)opcode, pc - 1);
            exit(0);
            break;
    }
    return 0;
}

int DMGCPU::executeExInstruction()
{
    // helpers
    const auto swap = [this](Reg r)
    {
        auto v = reg(r);
        v = (v >> 4) | (v << 4);
        reg(r) = v;

        reg(Reg::F) = v == 0 ? Flag_Z : 0;

        return 8;
    };

    // RLC
    const auto rotLeftNoCarry = [this](Reg r)
    {
        bool c = reg(r) & 0x80;
        uint8_t res = (reg(r) << 1) | (reg(r) >> 7);
        reg(r) = res;

        reg(Reg::F) = (c ? Flag_C : 0) | (res == 0 ? Flag_Z : 0);

        return 8;
    };

    // RRC
    const auto rotRightNoCarry = [this](Reg r)
    {
        bool c = reg(r) & 1;
        uint8_t res = (reg(r) >> 1) | (reg(r) << 7);
        reg(r) = res;

        reg(Reg::F) = (c ? Flag_C : 0) | (res == 0 ? Flag_Z : 0);

        return 8;
    };

    // RL
    const auto rotLeft = [this](Reg r)
    {
        bool c = reg(r) & 0x80;
        uint8_t res = (reg(r) << 1) | ((reg(Reg::F) & Flag_C) ? 0x01 : 0);
        reg(r) = res;

        reg(Reg::F) = (c ? Flag_C : 0) | (res == 0 ? Flag_Z : 0);

        return 8;
    };

    // RR
    const auto rotRight = [this](Reg r)
    {
        bool c = reg(r) & 1;
        uint8_t res = (reg(r) >> 1) | ((reg(Reg::F) & Flag_C) ? 0x80 : 0);
        reg(r) = res;

        reg(Reg::F) = (c ? Flag_C : 0) | (res == 0 ? Flag_Z : 0);

        return 8;
    };

    // SLA
    const auto shiftLeft = [this](Reg r)
    {
        bool c = reg(r) & 0x80;
        uint8_t res = reg(r) << 1;
        reg(r) = res;

        reg(Reg::F) = (c ? Flag_C : 0) | (res == 0 ? Flag_Z : 0);

        return 8;
    };

    // SRA
    const auto shiftRightArith = [this](Reg r)
    {
        bool c = reg(r) & 0x1;
        uint8_t res = reg(r) >> 1 | (reg(r) & 0x80);
        reg(r) = res;

        reg(Reg::F) = (c ? Flag_C : 0) | (res == 0 ? Flag_Z : 0);

        return 8;
    };

    // SRL
    const auto shiftRight = [this](Reg r)
    {
        bool c = reg(r) & 0x1;
        uint8_t res = reg(r) >> 1;
        reg(r) = res;

        reg(Reg::F) = (c ? Flag_C : 0) | (res == 0 ? Flag_Z : 0);

        return 8;
    };

    const auto testBit = [this](Reg r, int bit)
    {
        bool set = !!(reg(r) & (1 << bit));

        reg(Reg::F) = (reg(Reg::F) & Flag_C) | Flag_H | (set ? 0 : Flag_Z);
        return 8;
    };

    const auto testBitHL = [this](int bit)
    {
        bool set = !!(readMem(reg(WReg::HL)) & (1 << bit));

        reg(Reg::F) = (reg(Reg::F) & Flag_C) | Flag_H | (set ? 0 : Flag_Z);
        return 12;
    };

    const auto set = [this](Reg r, int bit)
    {
        reg(r) |= (1 << bit);
        return 8;
    };

    const auto setHL = [this](int bit)
    {
        writeMem(reg(WReg::HL), readMem(reg(WReg::HL)) | (1 << bit));
        return 12;
    };

    const auto reset = [this](Reg r, int bit)
    {
        reg(r) &= ~(1 << bit);
        return 8;
    };

    const auto resetHL = [this](int bit)
    {
        writeMem(reg(WReg::HL), readMem(reg(WReg::HL)) & ~(1 << bit));
        return 12;
    };

    auto opcode = readMem(pc++);

    switch(opcode)
    {
        case 0x00: // RLC B
            return rotLeftNoCarry(Reg::B);
        case 0x01: // RLC C
            return rotLeftNoCarry(Reg::C);
        case 0x02: // RLC D
            return rotLeftNoCarry(Reg::D);
        case 0x03: // RLC E
            return rotLeftNoCarry(Reg::E);
        case 0x04: // RLC H
            return rotLeftNoCarry(Reg::H);
        case 0x05: // RLC L
            return rotLeftNoCarry(Reg::L);
        case 0x06: // RLC (HL)
        {
            auto v = readMem(reg(WReg::HL));
            v = (v << 1) | (v >> 7);
            writeMem(reg(WReg::HL), v);

            reg(Reg::F) = ((v & 0x1) ? Flag_C : 0) | (v == 0 ? Flag_Z : 0);
            return 8;
        }
        case 0x07: // RLC A
            return rotLeftNoCarry(Reg::A);

        case 0x08: // RRC B
            return rotRightNoCarry(Reg::B);
        case 0x09: // RRC C
            return rotRightNoCarry(Reg::C);
        case 0x0A: // RRC D
            return rotRightNoCarry(Reg::D);
        case 0x0B: // RRC E
            return rotRightNoCarry(Reg::E);
        case 0x0C: // RRC H
            return rotRightNoCarry(Reg::H);
        case 0x0D: // RRC L
            return rotRightNoCarry(Reg::L);
        case 0x0E: // RRC (HL)
        {
            auto v = readMem(reg(WReg::HL));
            v = (v >> 1) | (v << 7);
            writeMem(reg(WReg::HL), v);

            reg(Reg::F) = ((v & 0x80) ? Flag_C : 0) | (v == 0 ? Flag_Z : 0);
            return 8;
        }
        case 0x0F: // RRC A
            return rotRightNoCarry(Reg::A);

        case 0x10: // RL B
            return rotLeft(Reg::B);
        case 0x11: // RL C
            return rotLeft(Reg::C);
        case 0x12: // RL D
            return rotLeft(Reg::D);
        case 0x13: // RL E
            return rotLeft(Reg::E);
        case 0x14: // RL H
            return rotLeft(Reg::H);
        case 0x15: // RL L
            return rotLeft(Reg::L);
        case 0x16: // RL (HL)
        {
            auto v = readMem(reg(WReg::HL));
            bool c = v & 0x80;
            v = (v << 1) | ((reg(Reg::F) & Flag_C) ? 0x01 : 0);
            writeMem(reg(WReg::HL), v);

            reg(Reg::F) = (c ? Flag_C : 0) | (v == 0 ? Flag_Z : 0);
            return 16;
        }
        case 0x17: // RL A
            return rotLeft(Reg::A);

        case 0x18: // RR B
            return rotRight(Reg::B);
        case 0x19: // RR C
            return rotRight(Reg::C);
        case 0x1A: // RR D
            return rotRight(Reg::D);
        case 0x1B: // RR E
            return rotRight(Reg::E);
        case 0x1C: // RR H
            return rotRight(Reg::H);
        case 0x1D: // RR L
            return rotRight(Reg::L);
        case 0x1E: // RR (HL)
        {
            auto v = readMem(reg(WReg::HL));
            bool c = v & 0x01;
            v = (v >> 1) | ((reg(Reg::F) & Flag_C) ? 0x80 : 0);
            writeMem(reg(WReg::HL), v);

            reg(Reg::F) = (c ? Flag_C : 0) | (v == 0 ? Flag_Z : 0);
            return 16;
        }
        case 0x1F: // RR A
            return rotRight(Reg::A);

        case 0x20: // SLA B
            return shiftLeft(Reg::B);
        case 0x21: // SLA C
            return shiftLeft(Reg::C);
        case 0x22: // SLA D
            return shiftLeft(Reg::D);
        case 0x23: // SLA E
            return shiftLeft(Reg::E);
        case 0x24: // SLA H
            return shiftLeft(Reg::H);
        case 0x25: // SLA L
            return shiftLeft(Reg::L);
        case 0x26: // SLA (HL)
        {
            auto v = readMem(reg(WReg::HL));
            bool c = v & 0x80;
            v = v << 1;
            writeMem(reg(WReg::HL), v);

            reg(Reg::F) = (c ? Flag_C : 0) | (v == 0 ? Flag_Z : 0);

            return 16;
        }
        case 0x27: // SLA A
            return shiftLeft(Reg::A);

        case 0x28: // SRA B
            return shiftRightArith(Reg::B);
        case 0x29: // SRA C
            return shiftRightArith(Reg::C);
        case 0x2A: // SRA D
            return shiftRightArith(Reg::D);
        case 0x2B: // SRA E
            return shiftRightArith(Reg::E);
        case 0x2C: // SRA H
            return shiftRightArith(Reg::H);
        case 0x2D: // SRA L
            return shiftRightArith(Reg::L);
        case 0x2E: // SRA (HL)
        {
            auto v = readMem(reg(WReg::HL));
            bool c = v & 0x01;
            v = v >> 1 | (v & 0x80);
            writeMem(reg(WReg::HL), v);

            reg(Reg::F) = (c ? Flag_C : 0) | (v == 0 ? Flag_Z : 0);

            return 16;
        }
        case 0x2F: // SRA A
            return shiftRightArith(Reg::A);

        case 0x30: // SWAP B
            return swap(Reg::B);
        case 0x31: // SWAP C
            return swap(Reg::C);
        case 0x32: // SWAP D
            return swap(Reg::D);
        case 0x33: // SWAP E
            return swap(Reg::E);
        case 0x34: // SWAP H
            return swap(Reg::H);
        case 0x35: // SWAP L
            return swap(Reg::L);
        case 0x36: // SWAP (HL)
        {
            auto v = readMem(reg(WReg::HL));
            v = (v << 4) | (v >> 4);
            writeMem(reg(WReg::HL), v);

            reg(Reg::F) = (v == 0 ? Flag_Z : 0);

            return 16;
        }
        case 0x37: // SWAP A
            return swap(Reg::A);

        case 0x38: // SRL B
            return shiftRight(Reg::B);
        case 0x39: // SRL C
            return shiftRight(Reg::C);
        case 0x3A: // SRL D
            return shiftRight(Reg::D);
        case 0x3B: // SRL E
            return shiftRight(Reg::E);
        case 0x3C: // SRL H
            return shiftRight(Reg::H);
        case 0x3D: // SRL L
            return shiftRight(Reg::L);
        case 0x3E: // SRL (HL)
        {
            auto v = readMem(reg(WReg::HL));
            bool c = v & 0x01;
            v = v >> 1;
            writeMem(reg(WReg::HL), v);

            reg(Reg::F) = (c ? Flag_C : 0) | (v == 0 ? Flag_Z : 0);

            return 16;
        }
        case 0x3F: // SRL A
            return shiftRight(Reg::A);

        case 0x40: // BIT 0,B
            return testBit(Reg::B, 0);
        case 0x41: // BIT 0,C
            return testBit(Reg::C, 0);
        case 0x42: // BIT 0,D
            return testBit(Reg::D, 0);
        case 0x43: // BIT 0,E
            return testBit(Reg::E, 0);
        case 0x44: // BIT 0,H
            return testBit(Reg::H, 0);
        case 0x45: // BIT 0,L
            return testBit(Reg::L, 0);
        case 0x46: // BIT 0,(HL)
            return testBitHL(0);
        case 0x47: // BIT 0,A
            return testBit(Reg::A, 0);
        case 0x48: // BIT 1,B
            return testBit(Reg::B, 1);
        case 0x49: // BIT 1,C
            return testBit(Reg::C, 1);
        case 0x4A: // BIT 1,D
            return testBit(Reg::D, 1);
        case 0x4B: // BIT 1,E
            return testBit(Reg::E, 1);
        case 0x4C: // BIT 1,H
            return testBit(Reg::H, 1);
        case 0x4D: // BIT 1,L
            return testBit(Reg::L, 1);
        case 0x4E: // BIT 1,(HL)
            return testBitHL(1);
        case 0x4F: // BIT 1,A
            return testBit(Reg::A, 1);
        case 0x50: // BIT 2,B
            return testBit(Reg::B, 2);
        case 0x51: // BIT 2,C
            return testBit(Reg::C, 2);
        case 0x52: // BIT 2,D
            return testBit(Reg::D, 2);
        case 0x53: // BIT 2,E
            return testBit(Reg::E, 2);
        case 0x54: // BIT 2,H
            return testBit(Reg::H, 2);
        case 0x55: // BIT 2,L
            return testBit(Reg::L, 2);
        case 0x56: // BIT 2,(HL)
            return testBitHL(2);
        case 0x57: // BIT 2,A
            return testBit(Reg::A, 2);
        case 0x58: // BIT 3,B
            return testBit(Reg::B, 3);
        case 0x59: // BIT 3,C
            return testBit(Reg::C, 3);
        case 0x5A: // BIT 3,D
            return testBit(Reg::D, 3);
        case 0x5B: // BIT 3,E
            return testBit(Reg::E, 3);
        case 0x5C: // BIT 3,H
            return testBit(Reg::H, 3);
        case 0x5D: // BIT 3,L
            return testBit(Reg::L, 3);
        case 0x5E: // BIT 3,(HL)
            return testBitHL(3);
        case 0x5F: // BIT 3,A
            return testBit(Reg::A, 3);
        case 0x60: // BIT 4,B
            return testBit(Reg::B, 4);
        case 0x61: // BIT 4,C
            return testBit(Reg::C, 4);
        case 0x62: // BIT 4,D
            return testBit(Reg::D, 4);
        case 0x63: // BIT 4,E
            return testBit(Reg::E, 4);
        case 0x64: // BIT 4,H
            return testBit(Reg::H, 4);
        case 0x65: // BIT 4,L
            return testBit(Reg::L, 4);
        case 0x66: // BIT 4,(HL)
            return testBitHL(4);
        case 0x67: // BIT 4,A
            return testBit(Reg::A, 4);
        case 0x68: // BIT 5,B
            return testBit(Reg::B, 5);
        case 0x69: // BIT 5,C
            return testBit(Reg::C, 5);
        case 0x6A: // BIT 5,D
            return testBit(Reg::D, 5);
        case 0x6B: // BIT 5,E
            return testBit(Reg::E, 5);
        case 0x6C: // BIT 5,H
            return testBit(Reg::H, 5);
        case 0x6D: // BIT 5,L
            return testBit(Reg::L, 5);
        case 0x6E: // BIT 5,(HL)
            return testBitHL(5);
        case 0x6F: // BIT 5,A
            return testBit(Reg::A, 5);
        case 0x70: // BIT 6,B
            return testBit(Reg::B, 6);
        case 0x71: // BIT 6,C
            return testBit(Reg::C, 6);
        case 0x72: // BIT 6,D
            return testBit(Reg::D, 6);
        case 0x73: // BIT 6,E
            return testBit(Reg::E, 6);
        case 0x74: // BIT 6,H
            return testBit(Reg::H, 6);
        case 0x75: // BIT 6,L
            return testBit(Reg::L, 6);
        case 0x76: // BIT 6,(HL)
            return testBitHL(6);
        case 0x77: // BIT 6,A
            return testBit(Reg::A, 6);
        case 0x78: // BIT 7,B
            return testBit(Reg::B, 7);
        case 0x79: // BIT 7,C
            return testBit(Reg::C, 7);
        case 0x7A: // BIT 7,D
            return testBit(Reg::D, 7);
        case 0x7B: // BIT 7,E
            return testBit(Reg::E, 7);
        case 0x7C: // BIT 7,H
            return testBit(Reg::H, 7);
        case 0x7D: // BIT 7,L
            return testBit(Reg::L, 7);
        case 0x7E: // BIT 7,(HL)
            return testBitHL(7);
        case 0x7F: // BIT 7,A
            return testBit(Reg::A, 7);

        case 0x80: // RES 0,B
            return reset(Reg::B, 0);
        case 0x81: // RES 0,C
            return reset(Reg::C, 0);
        case 0x82: // RES 0,D
            return reset(Reg::D, 0);
        case 0x83: // RES 0,E
            return reset(Reg::E, 0);
        case 0x84: // RES 0,H
            return reset(Reg::H, 0);
        case 0x85: // RES 0,L
            return reset(Reg::L, 0);
        case 0x86: // RES 0,(HL)
            return resetHL(0);
        case 0x87: // RES 0,A
            return reset(Reg::A, 0);
        case 0x88: // RES 1,B
            return reset(Reg::B, 1);
        case 0x89: // RES 1,C
            return reset(Reg::C, 1);
        case 0x8A: // RES 1,D
            return reset(Reg::D, 1);
        case 0x8B: // RES 1,E
            return reset(Reg::E, 1);
        case 0x8C: // RES 1,H
            return reset(Reg::H, 1);
        case 0x8D: // RES 1,L
            return reset(Reg::L, 1);
        case 0x8E: // RES 1,(HL)
            return resetHL(1);
        case 0x8F: // RES 1,A
            return reset(Reg::A, 1);
        case 0x90: // RES 2,B
            return reset(Reg::B, 2);
        case 0x91: // RES 2,C
            return reset(Reg::C, 2);
        case 0x92: // RES 2,D
            return reset(Reg::D, 2);
        case 0x93: // RES 2,E
            return reset(Reg::E, 2);
        case 0x94: // RES 2,H
            return reset(Reg::H, 2);
        case 0x95: // RES 2,L
            return reset(Reg::L, 2);
        case 0x96: // RES 2,(HL)
            return resetHL(2);
        case 0x97: // RES 2,A
            return reset(Reg::A, 2);
        case 0x98: // RES 3,B
            return reset(Reg::B, 3);
        case 0x99: // RES 3,C
            return reset(Reg::C, 3);
        case 0x9A: // RES 3,D
            return reset(Reg::D, 3);
        case 0x9B: // RES 3,E
            return reset(Reg::E, 3);
        case 0x9C: // RES 3,H
            return reset(Reg::H, 3);
        case 0x9D: // RES 3,L
            return reset(Reg::L, 3);
        case 0x9E: // RES 3,(HL)
            return resetHL(3);
        case 0x9F: // RES 3,A
            return reset(Reg::A, 3);
        case 0xA0: // RES 4,B
            return reset(Reg::B, 4);
        case 0xA1: // RES 4,C
            return reset(Reg::C, 4);
        case 0xA2: // RES 4,D
            return reset(Reg::D, 4);
        case 0xA3: // RES 4,E
            return reset(Reg::E, 4);
        case 0xA4: // RES 4,H
            return reset(Reg::H, 4);
        case 0xA5: // RES 4,L
            return reset(Reg::L, 4);
        case 0xA6: // RES 4,(HL)
            return resetHL(4);
        case 0xA7: // RES 4,A
            return reset(Reg::A, 4);
        case 0xA8: // RES 5,B
            return reset(Reg::B, 5);
        case 0xA9: // RES 5,C
            return reset(Reg::C, 5);
        case 0xAA: // RES 5,D
            return reset(Reg::D, 5);
        case 0xAB: // RES 5,E
            return reset(Reg::E, 5);
        case 0xAC: // RES 5,H
            return reset(Reg::H, 5);
        case 0xAD: // RES 5,L
            return reset(Reg::L, 5);
        case 0xAE: // RES 5,(HL)
            return resetHL(5);
        case 0xAF: // RES 5,A
            return reset(Reg::A, 5);
        case 0xB0: // RES 6,B
            return reset(Reg::B, 6);
        case 0xB1: // RES 6,C
            return reset(Reg::C, 6);
        case 0xB2: // RES 6,D
            return reset(Reg::D, 6);
        case 0xB3: // RES 6,E
            return reset(Reg::E, 6);
        case 0xB4: // RES 6,H
            return reset(Reg::H, 6);
        case 0xB5: // RES 6,L
            return reset(Reg::L, 6);
        case 0xB6: // RES 6,(HL)
            return resetHL(6);
        case 0xB7: // RES 6,A
            return reset(Reg::A, 6);
        case 0xB8: // RES 7,B
            return reset(Reg::B, 7);
        case 0xB9: // RES 7,C
            return reset(Reg::C, 7);
        case 0xBA: // RES 7,D
            return reset(Reg::D, 7);
        case 0xBB: // RES 7,E
            return reset(Reg::E, 7);
        case 0xBC: // RES 7,H
            return reset(Reg::H, 7);
        case 0xBD: // RES 7,L
            return reset(Reg::L, 7);
        case 0xBE: // RES 7,(HL)
            return resetHL(7);
        case 0xBF: // RES 7,A
            return reset(Reg::A, 7);

        case 0xC0: // SET 0,B
            return set(Reg::B, 0);
        case 0xC1: // SET 0,C
            return set(Reg::C, 0);
        case 0xC2: // SET 0,D
            return set(Reg::D, 0);
        case 0xC3: // SET 0,E
            return set(Reg::E, 0);
        case 0xC4: // SET 0,H
            return set(Reg::H, 0);
        case 0xC5: // SET 0,L
            return set(Reg::L, 0);
        case 0xC6: // SET 0,(HL)
            return setHL(0);
        case 0xC7: // SET 0,A
            return set(Reg::A, 0);
        case 0xC8: // SET 1,B
            return set(Reg::B, 1);
        case 0xC9: // SET 1,C
            return set(Reg::C, 1);
        case 0xCA: // SET 1,D
            return set(Reg::D, 1);
        case 0xCB: // SET 1,E
            return set(Reg::E, 1);
        case 0xCC: // SET 1,H
            return set(Reg::H, 1);
        case 0xCD: // SET 1,L
            return set(Reg::L, 1);
        case 0xCE: // SET 1,(HL)
            return setHL(1);
        case 0xCF: // SET 1,A
            return set(Reg::A, 1);
        case 0xD0: // SET 2,B
            return set(Reg::B, 2);
        case 0xD1: // SET 2,C
            return set(Reg::C, 2);
        case 0xD2: // SET 2,D
            return set(Reg::D, 2);
        case 0xD3: // SET 2,E
            return set(Reg::E, 2);
        case 0xD4: // SET 2,H
            return set(Reg::H, 2);
        case 0xD5: // SET 2,L
            return set(Reg::L, 2);
        case 0xD6: // SET 2,(HL)
            return setHL(2);
        case 0xD7: // SET 2,A
            return set(Reg::A, 2);
        case 0xD8: // SET 3,B
            return set(Reg::B, 3);
        case 0xD9: // SET 3,C
            return set(Reg::C, 3);
        case 0xDA: // SET 3,D
            return set(Reg::D, 3);
        case 0xDB: // SET 3,E
            return set(Reg::E, 3);
        case 0xDC: // SET 3,H
            return set(Reg::H, 3);
        case 0xDD: // SET 3,L
            return set(Reg::L, 3);
        case 0xDE: // SET 3,(HL)
            return setHL(3);
        case 0xDF: // SET 3,A
            return set(Reg::A, 3);
        case 0xE0: // SET 4,B
            return set(Reg::B, 4);
        case 0xE1: // SET 4,C
            return set(Reg::C, 4);
        case 0xE2: // SET 4,D
            return set(Reg::D, 4);
        case 0xE3: // SET 4,E
            return set(Reg::E, 4);
        case 0xE4: // SET 4,H
            return set(Reg::H, 4);
        case 0xE5: // SET 4,L
            return set(Reg::L, 4);
        case 0xE6: // SET 4,(HL)
            return setHL(4);
        case 0xE7: // SET 4,A
            return set(Reg::A, 4);
        case 0xE8: // SET 5,B
            return set(Reg::B, 5);
        case 0xE9: // SET 5,C
            return set(Reg::C, 5);
        case 0xEA: // SET 5,D
            return set(Reg::D, 5);
        case 0xEB: // SET 5,E
            return set(Reg::E, 5);
        case 0xEC: // SET 5,H
            return set(Reg::H, 5);
        case 0xED: // SET 5,L
            return set(Reg::L, 5);
        case 0xEE: // SET 5,(HL)
            return setHL(5);
        case 0xEF: // SET 5,A
            return set(Reg::A, 5);
        case 0xF0: // SET 6,B
            return set(Reg::B, 6);
        case 0xF1: // SET 6,C
            return set(Reg::C, 6);
        case 0xF2: // SET 6,D
            return set(Reg::D, 6);
        case 0xF3: // SET 6,E
            return set(Reg::E, 6);
        case 0xF4: // SET 6,H
            return set(Reg::H, 6);
        case 0xF5: // SET 6,L
            return set(Reg::L, 6);
        case 0xF6: // SET 6,(HL)
            return setHL(6);
        case 0xF7: // SET 6,A
            return set(Reg::A, 6);
        case 0xF8: // SET 7,B
            return set(Reg::B, 7);
        case 0xF9: // SET 7,C
            return set(Reg::C, 7);
        case 0xFA: // SET 7,D
            return set(Reg::D, 7);
        case 0xFB: // SET 7,E
            return set(Reg::E, 7);
        case 0xFC: // SET 7,H
            return set(Reg::H, 7);
        case 0xFD: // SET 7,L
            return set(Reg::L, 7);
        case 0xFE: // SET 7,(HL)
            return setHL(7);
        case 0xFF: // SET 7,A
            return set(Reg::A, 7);
    }
    return 0;
}

void DMGCPU::writeMBC(uint16_t addr, uint8_t data)
{
    if(mbcType == MBCType::None)
        return;

    // MBC1

    if(addr < 0x2000)
        mbcRAMEnabled = (data & 0xF) == 0xA;
    else if(addr < 0x4000)
    {
        // low 5 bits of rom bank
        mbcROMBank = (mbcROMBank & 0xE0) | (data & 0x1F);

        if((mbcROMBank & 0x1F) == 0)
            mbcROMBank++; // bank 0 is handled as bank 1
    }
    else if(addr < 0x6000)
    {
        // high 2 bits of rom bank / ram bank
        if(mbcRAMBankMode)
            mbcRAMBank = data & 0x3;
        else
            mbcROMBank = (mbcROMBank & 0x1F) | (data & 0xE0);
    }
    else // < 0x8000
    {
        mbcRAMBankMode = data == 1;
        if(mbcRAMBankMode)
            mbcROMBank &= 0x1F;
        else
            mbcRAMBank = 0;
    }
}