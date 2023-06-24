#pragma once
#include <cstdint>

enum class Reg8
{
    AL = 0,
    CL,
    DL,
    BL,
    AH,
    CH,
    DH,
    BH,

    R8B,
    R9B,
    R10B,
    R11B,
    R12B,
    R13B,
    R14B,
    R15B,

    // these replace AH-DH with a REX prefix
    SPL = 4 | 0x10,
    BPL,
    SIL,
    DIL
};

enum class Reg16
{
    AX = 0,
    CX,
    DX,
    BX,
    SP,
    BP,
    SI,
    DI,

    R8W,
    R9W,
    R10W,
    R11W,
    R12W,
    R13W,
    R14W,
    R15W
};

enum class Reg32
{
    EAX = 0,
    ECX,
    EDX,
    EBX,
    ESP,
    EBP,
    ESI,
    EDI,

    R8D,
    R9D,
    R10D,
    R11D,
    R12D,
    R13D,
    R14D,
    R15D
};

enum class Reg64
{
    RAX = 0,
    RCX,
    RDX,
    RBX,
    RSP,
    RBP,
    RSI,
    RDI,

    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    R15
};

enum class Condition
{
    O = 0,
    NO,
    B,
    AE,
    E,
    NE,
    BE,
    A,
    S,
    NS,
    P,
    NP,
    L,
    GE,
    LE,
    G
};


class X86Builder
{
public:
    X86Builder(uint8_t *ptr, uint8_t *endPtr) : ptr(ptr), endPtr(endPtr){}

    void add(Reg32 dst, Reg32 src);
    void add(Reg16 dst, Reg16 src);
    void add(Reg8 dst, Reg8 src);
    void add(Reg32 dst, uint32_t imm);
    void add(Reg8 dst, uint8_t imm);
    void add(Reg64 dst, int8_t imm);
    void add(Reg32 dst, int8_t imm);
    void add(Reg16 dst, int8_t imm);
    void addD(int8_t imm, Reg64 base, int disp = 0);
    void addW(int8_t imm, Reg64 base, int disp = 0);

    void adc(Reg8 dst, Reg8 src);
    void adc(Reg8 dst, uint8_t imm);

    void and_(Reg32 dst, Reg32 src);
    void and_(Reg16 dst, Reg16 src);
    void and_(Reg8 dst, Reg8 src);
    void and_(Reg32 dst, uint32_t imm);
    void and_(Reg8 dst, uint8_t imm);

    void call(int disp);
    void call(Reg64 r);

    void cmp(Reg32 dst, Reg32 src);
    void cmp(Reg8 dst, Reg8 src);
    void cmp(Reg32 dst, uint32_t imm);
    void cmp(Reg8 dst, uint8_t imm);
    void cmp(Reg32 dst, int8_t imm);
    void cmp(uint8_t imm, Reg64 base, int disp = 0);

    void dec(Reg16 r);
    void dec(Reg8 r);

    void inc(Reg16 r);
    void inc(Reg8 r);

    void jcc(Condition cc, int disp);

    void jmp(int disp, bool forceLong = false);
    void jmp(Reg64 r);

    void lea(Reg32 r, Reg64 base, int disp = 0);

    void mov(Reg64 dst, Reg64 src);
    void mov(Reg32 dst, Reg32 src);
    void mov(Reg8 dst, Reg8 src);
    void mov(Reg64 r, Reg64 base, bool isStore, int disp = 0);
    void mov(Reg32 r, Reg64 base, bool isStore = false, int disp = 0);
    void mov(Reg16 r, Reg64 base, bool isStore = false, int disp = 0);
    void mov(Reg64 r, uint64_t imm);
    void mov(Reg32 r, uint32_t imm);
    void mov(Reg8 r, uint8_t imm);
    void mov(uint8_t imm, Reg64 base, int disp = 0);

    void movzx(Reg32 dst, Reg16 src);
    void movzx(Reg32 dst, Reg8 src);
    void movzxW(Reg32 r, Reg64 base, int disp = 0);

    void not_(Reg32 r);
    void not_(Reg8 r);

    void or_(Reg32 dst, Reg32 src);
    void or_(Reg16 dst, Reg16 src);
    void or_(Reg8 dst, Reg8 src);
    void or_(Reg32 dst, uint32_t imm);
    void or_(Reg8 dst, uint8_t imm);

    void pop(Reg64 r);

    void push(Reg64 r);

    void rclCL(Reg8 dst);
    void rcl(Reg8 r, uint8_t count);

    void rcrCL(Reg8 dst);
    void rcr(Reg8 r, uint8_t count);

    void ret();

    void rolCL(Reg8 dst);
    void rol(Reg8 r, uint8_t count);

    void rorCL(Reg8 dst);
    void ror(Reg8 r, uint8_t count);

    void sarCL(Reg32 dst);
    void sarCL(Reg8 dst);
    void sar(Reg32 r, uint8_t count);
    void sar(Reg8 r, uint8_t count);

    void sbb(Reg8 dst, Reg8 src);
    void sbb(Reg8 dst, uint8_t imm);

    void setcc(Condition cc, Reg8 dst);

    void shrCL(Reg32 dst);
    void shrCL(Reg8 dst);
    void shr(Reg32 r, uint8_t count);
    void shr(Reg8 r, uint8_t count);

    void shlCL(Reg32 dst);
    void shlCL(Reg8 dst);
    void shl(Reg32 r, uint8_t count);
    void shl(Reg8 r, uint8_t count);

    void stc();

    void sub(Reg32 dst, Reg32 src);
    void sub(Reg16 dst, Reg16 src);
    void sub(Reg8 dst, Reg8 src);
    void sub(Reg32 dst, uint32_t imm);
    void sub(Reg8 dst, uint8_t imm);
    void sub(Reg64 dst, int8_t imm);
    void sub(Reg32 dst, int8_t imm);

    void test(Reg8 dst, uint8_t imm);

    void xchg(Reg32 dst, Reg32 src);
    void xchg(Reg8 dst, Reg8 src);

    void xor_(Reg32 dst, Reg32 src);
    void xor_(Reg8 dst, Reg8 src);
    void xor_(Reg32 dst, uint32_t imm);
    void xor_(Reg8 dst, uint8_t imm);

    uint8_t *getPtr() const {return ptr;}

    void resetPtr(uint8_t *oldPtr);

    void patch(uint8_t *patchPtr, uint8_t *patchEndPtr);
    void endPatch();

    void removeRange(uint8_t *startPtr, uint8_t *endPtr);

    bool getError() const {return error;}

private:
    void write(uint8_t b);

    void encodeModRM(int reg, int baseReg, int disp); // mod 0-2
    void encodeModRM(int reg1, int reg2Op = 0); // mod 3
    void encodeModRMReg8(int reg1, int reg2); // mod 3 (extra validation fot 8bit regs)
    void encodeREX(bool w, int reg, int index, int base);

    uint8_t *ptr, *endPtr;
    uint8_t *savedPtr, *savedEndPtr;

    bool error = false; // super basic error handling
};
