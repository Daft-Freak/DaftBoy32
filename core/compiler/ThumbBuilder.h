#pragma once
#include <cassert>
#include <cstdint>

enum class Reg
{
    R0 = 0,
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
    R15,

    // aliases
    SP = R13,
    LR = R14,
    PC = R15,
};

// wrapper that asserts reg is in range
class LowReg final
{
public:
    constexpr LowReg(Reg r): val(r)
    {
        assert(static_cast<int>(r) < 8);
    }

    Reg val;
};

enum class Condition
{
    EQ = 0,
    NE,
    CS,
    CC,
    MI,
    PL,
    VS,
    VC,
    HI,
    LS,
    GE,
    LT,
    GT,
    LE,
    AL
};

class ThumbBuilder final
{
public:
    ThumbBuilder(uint16_t *ptr, uint16_t *endPtr) : ptr(ptr), endPtr(endPtr){}

    void adc(LowReg dn, LowReg m);

    void add(Reg dn, uint8_t imm);
    void add(Reg d, Reg n, uint32_t imm); // v7M if d >= 8 || r (n >= 8 and not SP) || imm > 0xFF
    void add(Reg d, Reg n, Reg m); // v7M if d >= 8 || n >= 8 || m >= 8
    void add(Reg dn, Reg m);

    void and_(LowReg dn, LowReg m);

    void asr(LowReg d, LowReg m, uint8_t imm);

    void b(Condition cond, int imm);
    void b(int imm);

    void bic(LowReg dn, LowReg m);

    void bl(int32_t off);

    void blx(Reg r);

    void bx(Reg r);

    void cmp(LowReg n, uint8_t imm);
    void cmp(Reg n, Reg m);

    void eor(LowReg dn, LowReg m);

    void ldm(uint16_t regList, Reg n, bool w = false); // v7M

    void ldr(Reg t, Reg n, uint16_t imm); // v7M if t/n >= 8 || imm > 124 || imm & 3
    void ldr(Reg t, int16_t imm); // v7M if t >= 8 || imm < 0 || imm > 1020 || imm & 3

    void ldrb(LowReg t, LowReg n, uint8_t imm);

    void ldrh(LowReg t, LowReg n, uint8_t imm);
    void ldrh(LowReg t, LowReg n, LowReg m);

    void lsl(LowReg d, LowReg m, uint8_t imm);

    void lsr(LowReg d, LowReg m, uint8_t imm);

    void mov(Reg d, uint32_t imm); // v7M if d >= 8 || imm > 0xFF
    void mov(Reg d, Reg m);

    void mrs(Reg d, uint8_t sysm);
    void msr(Reg n, uint8_t mask, uint8_t sysm);

    void mvn(LowReg d, LowReg m);

    void nop();

    void orr(LowReg d, LowReg m);
    void orr(Reg d, Reg n, Reg m, bool s = false); // v7M
    void orr(Reg d, Reg n, uint8_t imm, bool s = false); // v7M

    void pop(uint8_t regList, bool pc);
    void pop(uint16_t regList); // v7M

    void push(uint8_t regList, bool lr);
    void push(uint16_t regList); // v7M

    void stm(uint16_t regList, Reg n, bool w = false); // v7M

    void str(Reg t, Reg n, uint16_t imm); // v7M if t/n >= 8 || imm > 124 || imm & 3
    void str(LowReg t, uint16_t imm);

    void strb(LowReg t, LowReg n, uint8_t imm);

    void strh(LowReg t, LowReg n, uint8_t imm);
    void strh(LowReg t, LowReg n, LowReg m);

    void sbc(LowReg dn, LowReg m);

    void sub(Reg dn, uint8_t imm);
    void sub(LowReg d, LowReg n, LowReg m);

    void sxtb(LowReg d, LowReg m);

    void sxth(LowReg d, LowReg m);

    void tst(Reg n, uint32_t imm); // v7M

    void uxtb(LowReg d, LowReg m);

    void uxth(LowReg d, LowReg m);

    // not an instruction
    void data(uint16_t d);

    bool isValidModifiedImmediate(uint32_t val);

    uint16_t *getPtr() const {return ptr;}

    void resetPtr(uint16_t *oldPtr);

    bool getError() const {return error;}

private:
    void write(uint16_t hw);

    uint32_t encodeModifiedImmediate(uint32_t val);

    uint16_t *ptr, *endPtr;

    bool error = false; // super basic error handling
};
