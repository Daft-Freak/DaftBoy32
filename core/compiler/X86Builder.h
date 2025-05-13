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
    class RMOperand
    {
    public:
        explicit constexpr RMOperand(Reg64 reg) : base(reg), w(4) {} // register
        explicit constexpr RMOperand(Reg32 reg) : base(static_cast<Reg64>(reg)), w(3) {} // register
        explicit constexpr RMOperand(Reg16 reg) : base(static_cast<Reg64>(reg)), w(2) {} // register
        explicit constexpr RMOperand(Reg8 reg) : base(static_cast<Reg64>(reg)), w(1) {} // register
        constexpr RMOperand(Reg64 base, int disp) : base(base), disp(disp) {} // memory
        constexpr RMOperand(Reg64 base, Reg64 index, int disp = 0, int scale = 1) : base(base), index(index), scale(scale), disp(disp) // memory, with index/scale
        {
            assert(scale == 1 || scale == 2 || scale == 4 || scale == 8);
        }

        constexpr bool isMem() const {return w == 0;}

        constexpr Reg32 getReg32() const {return static_cast<Reg32>(base);}

        Reg64 base, index = Reg64::RSP;
        uint8_t w = 0; // 0 = indirect, 1 = 8, 2 = 16, ...
        uint8_t scale = 0;
        int disp = 0;
    };

    X86Builder(uint8_t *ptr, uint8_t *endPtr) : ptr(ptr), endPtr(endPtr){}

    void add(RMOperand dst, Reg32 src);
    void add(RMOperand dst, Reg16 src);
    void add(RMOperand dst, Reg8 src);
    void add(RMOperand dst, uint32_t imm);
    void add(RMOperand dst, uint8_t imm);
    void addQ(RMOperand dst, int8_t imm);
    void addD(RMOperand dst, int8_t imm);
    void addW(RMOperand dst, int8_t imm);
    void add(Reg32 dst, Reg32 src) {add(RMOperand{dst}, src);}
    void add(Reg16 dst, Reg16 src) {add(RMOperand{dst}, src);}
    void add(Reg8 dst, Reg8 src) {add(RMOperand{dst}, src);}
    void add(Reg32 dst, uint32_t imm) {add(RMOperand{dst}, imm);}
    void add(Reg8 dst, uint8_t imm) {add(RMOperand{dst}, imm);}
    void add(Reg64 dst, int8_t imm) {addQ(RMOperand{dst}, imm);}
    void add(Reg32 dst, int8_t imm) {addD(RMOperand{dst}, imm);}
    void add(Reg16 dst, int8_t imm) {addW(RMOperand{dst}, imm);}


    void adc(RMOperand dst, Reg32 src);
    void adc(RMOperand dst, Reg8 src);
    void adc(RMOperand dst, uint8_t imm);
    void adc(Reg32 dst, Reg32 src) {adc(RMOperand{dst}, src);}
    void adc(Reg8 dst, Reg8 src) {adc(RMOperand{dst}, src);}
    void adc(Reg8 dst, uint8_t imm) {adc(RMOperand{dst}, imm);}

    void and_(RMOperand dst, Reg32 src);
    void and_(RMOperand dst, Reg16 src);
    void and_(RMOperand dst, Reg8 src);
    void and_(RMOperand dst, uint32_t imm);
    void and_(RMOperand dst, uint8_t imm);
    void andD(RMOperand dst, int8_t imm);
    void and_(Reg32 dst, Reg32 src) {and_(RMOperand{dst}, src);}
    void and_(Reg16 dst, Reg16 src) {and_(RMOperand{dst}, src);}
    void and_(Reg8 dst, Reg8 src) {and_(RMOperand{dst}, src);}
    void and_(Reg32 dst, uint32_t imm) {and_(RMOperand{dst}, imm);}
    void and_(Reg8 dst, uint8_t imm) {and_(RMOperand{dst}, imm);}
    void and_(Reg32 dst, int8_t imm)  {andD(RMOperand{dst}, imm);}

    void bt(Reg32 base, uint8_t off);
    void btr(Reg32 base, uint8_t off);

    void call(int disp);
    void call(Reg64 r);

    void cdq();

    void cmc();

    void cmp(RMOperand dst, Reg32 src);
    void cmp(RMOperand dst, Reg8 src);
    void cmp(RMOperand dst, uint32_t imm);
    void cmp(RMOperand dst, uint8_t imm);
    void cmpD(RMOperand dst, int8_t imm);
    void cmp(Reg32 dst, Reg32 src) {cmp(RMOperand{dst}, src);}
    void cmp(Reg8 dst, Reg8 src) {cmp(RMOperand{dst}, src);}
    void cmp(Reg32 dst, uint32_t imm) {cmp(RMOperand{dst}, imm);}
    void cmp(Reg8 dst, uint8_t imm) {cmp(RMOperand{dst}, imm);}
    void cmp(Reg32 dst, int8_t imm)  {cmpD(RMOperand{dst}, imm);}

    void dec(Reg16 r);
    void dec(Reg8 r);

    void divD(RMOperand src);
    void div(Reg32 src) {divD(RMOperand{src});}

    void idivD(RMOperand src);
    void idiv(Reg32 src) {idivD(RMOperand{src});}

    void imul(Reg32 dst, RMOperand src);
    void imul(Reg32 dst, Reg32 src) {imul(dst, RMOperand{src});}

    void inc(Reg16 r);
    void inc(Reg8 r);

    void jcc(Condition cc, int disp);

    void jmp(int disp, bool forceLong = false);
    void jmp(Reg64 r);

    void lea(Reg64 dst, RMOperand m);
    void lea(Reg32 dst, RMOperand m);

    void mov(RMOperand dst, Reg64 src);
    void mov(RMOperand dst, Reg32 src);
    void mov(RMOperand dst, Reg16 src);
    void mov(RMOperand dst, Reg8 src);
    void mov(Reg64 dst, RMOperand src);
    void mov(Reg32 dst, RMOperand src);
    void mov(Reg16 dst, RMOperand src);
    void mov(Reg8 dst, RMOperand src);
    void mov(Reg64 r, uint64_t imm);
    void mov(Reg32 r, uint32_t imm);
    void mov(Reg8 r, uint8_t imm);
    void mov(RMOperand dst, uint32_t imm);
    void mov(RMOperand dst, uint8_t imm);
    void mov(Reg64 dst, Reg64 src) {mov(RMOperand{dst}, src);}
    void mov(Reg32 dst, Reg32 src) {mov(RMOperand{dst}, src);}
    void mov(Reg16 dst, Reg16 src) {mov(RMOperand{dst}, src);}
    void mov(Reg8  dst, Reg8  src) {mov(RMOperand{dst}, src);}

    void movsxW(Reg32 dst, RMOperand src);
    void movsxB(Reg32 dst, RMOperand src);
    void movsx(Reg32 dst, Reg16 src) {movsxW(dst, RMOperand{src});}
    void movsx(Reg32 dst, Reg8 src) {movsxB(dst, RMOperand{src});}

    void movzxW(Reg32 dst, RMOperand src);
    void movzxB(Reg32 dst, RMOperand src);
    void movzx(Reg32 dst, Reg16 src) {movzxW(dst, RMOperand{src});}
    void movzx(Reg32 dst, Reg8 src) {movzxB(dst, RMOperand{src});}

    void not_(Reg32 r);
    void not_(Reg8 r);

    void or_(RMOperand dst, Reg32 src);
    void or_(RMOperand dst, Reg16 src);
    void or_(RMOperand dst, Reg8 src);
    void or_(RMOperand dst, uint32_t imm);
    void or_(RMOperand dst, uint8_t imm);
    void or_(Reg32 dst, Reg32 src) {or_(RMOperand{dst}, src);}
    void or_(Reg16 dst, Reg16 src) {or_(RMOperand{dst}, src);}
    void or_(Reg8 dst, Reg8 src) {or_(RMOperand{dst}, src);}
    void or_(Reg32 dst, uint32_t imm) {or_(RMOperand{dst}, imm);}
    void or_(Reg8 dst, uint8_t imm) {or_(RMOperand{dst}, imm);}

    void pop(Reg64 r);

    void push(Reg64 r);

    void rclD(RMOperand r);
    void rclB(RMOperand r);
    void rclD(RMOperand r, uint8_t count);
    void rclB(RMOperand r, uint8_t count);
    void rcl(Reg32 dst) {rclD(RMOperand{dst});}
    void rcl(Reg8 dst) {rclB(RMOperand{dst});}
    void rcl(Reg32 r, uint8_t count) {rclD(RMOperand{r}, count);}
    void rcl(Reg8 r, uint8_t count) {rclB(RMOperand{r}, count);}

    void rcrD(RMOperand r);
    void rcrB(RMOperand r);
    void rcrD(RMOperand r, uint8_t count);
    void rcrB(RMOperand r, uint8_t count);
    void rcr(Reg32 dst) {rcrD(RMOperand{dst});}
    void rcr(Reg8 dst) {rcrB(RMOperand{dst});}
    void rcr(Reg32 r, uint8_t count) {rcrD(RMOperand{r}, count);}
    void rcr(Reg8 r, uint8_t count) {rcrB(RMOperand{r}, count);}

    void ret();

    void rolD(RMOperand r);
    void rolB(RMOperand r);
    void rolD(RMOperand r, uint8_t count);
    void rolB(RMOperand r, uint8_t count);
    void rol(Reg32 dst) {rolD(RMOperand{dst});}
    void rol(Reg8 dst) {rolB(RMOperand{dst});}
    void rol(Reg32 r, uint8_t count) {rolD(RMOperand{r}, count);}
    void rol(Reg8 r, uint8_t count) {rolB(RMOperand{r}, count);}

    void rorD(RMOperand r);
    void rorB(RMOperand r);
    void rorD(RMOperand r, uint8_t count);
    void rorB(RMOperand r, uint8_t count);
    void ror(Reg32 dst) {rorD(RMOperand{dst});}
    void ror(Reg8 dst) {rorB(RMOperand{dst});}
    void ror(Reg32 r, uint8_t count) {rorD(RMOperand{r}, count);}
    void ror(Reg8 r, uint8_t count) {rorB(RMOperand{r}, count);}

    void sarD(RMOperand r);
    void sarB(RMOperand r);
    void sarD(RMOperand r, uint8_t count);
    void sarB(RMOperand r, uint8_t count);
    void sar(Reg32 dst) {sarD(RMOperand{dst});}
    void sar(Reg8 dst) {sarB(RMOperand{dst});}
    void sar(Reg32 r, uint8_t count) {sarD(RMOperand{r}, count);}
    void sar(Reg8 r, uint8_t count) {sarB(RMOperand{r}, count);}

    void sbb(RMOperand dst, Reg32 src);
    void sbb(RMOperand dst, Reg8 src);
    void sbb(RMOperand dst, uint8_t imm);
    void sbb(Reg32 dst, Reg32 src) {sbb(RMOperand{dst}, src);}
    void sbb(Reg8 dst, Reg8 src) {sbb(RMOperand{dst}, src);}
    void sbb(Reg8 dst, uint8_t imm) {sbb(RMOperand{dst}, imm);}

    void setcc(Condition cc, Reg8 dst);

    void shrD(RMOperand r);
    void shrB(RMOperand r);
    void shrD(RMOperand r, uint8_t count);
    void shrB(RMOperand r, uint8_t count);
    void shr(Reg32 dst) {shrD(RMOperand{dst});}
    void shr(Reg8 dst) {shrB(RMOperand{dst});}
    void shr(Reg32 r, uint8_t count) {shrD(RMOperand{r}, count);}
    void shr(Reg8 r, uint8_t count) {shrB(RMOperand{r}, count);}

    void shlD(RMOperand r);
    void shlB(RMOperand r);
    void shlD(RMOperand r, uint8_t count);
    void shlB(RMOperand r, uint8_t count);
    void shl(Reg32 dst) {shlD(RMOperand{dst});}
    void shl(Reg8 dst) {shlB(RMOperand{dst});}
    void shl(Reg32 r, uint8_t count) {shlD(RMOperand{r}, count);}
    void shl(Reg8 r, uint8_t count) {shlB(RMOperand{r}, count);}

    void stc();

    void sub(RMOperand dst, Reg32 src);
    void sub(RMOperand dst, Reg16 src);
    void sub(RMOperand dst, Reg8 src);
    void sub(RMOperand dst, uint32_t imm);
    void sub(RMOperand dst, uint8_t imm);
    void subQ(RMOperand dst, int8_t imm);
    void subD(RMOperand dst, int8_t imm);
    void sub(Reg32 dst, Reg32 src) {sub(RMOperand{dst}, src);}
    void sub(Reg16 dst, Reg16 src) {sub(RMOperand{dst}, src);}
    void sub(Reg8 dst, Reg8 src) {sub(RMOperand{dst}, src);}
    void sub(Reg32 dst, uint32_t imm) {sub(RMOperand{dst}, imm);}
    void sub(Reg8 dst, uint8_t imm) {sub(RMOperand{dst}, imm);}
    void sub(Reg64 dst, int8_t imm) {subQ(RMOperand{dst}, imm);}
    void sub(Reg32 dst, int8_t imm) {subD(RMOperand{dst}, imm);}

    void test(Reg32 dst, uint32_t imm);
    void test(Reg8 dst, uint8_t imm);

    void xchg(RMOperand dst, Reg32 src);
    void xchg(RMOperand dst, Reg8 src);
    void xchg(Reg32 dst, Reg32 src) {xchg(RMOperand{dst}, src);}
    void xchg(Reg8 dst, Reg8 src) {xchg(RMOperand{dst}, src);}

    void xor_(RMOperand dst, Reg32 src);
    void xor_(RMOperand dst, Reg8 src);
    void xor_(RMOperand dst, uint32_t imm);
    void xor_(RMOperand dst, uint8_t imm);
    void xor_(Reg32 dst, Reg32 src) {xor_(RMOperand{dst}, src);}
    void xor_(Reg8 dst, Reg8 src) {xor_(RMOperand{dst}, src);}
    void xor_(Reg32 dst, uint32_t imm) {xor_(RMOperand{dst}, imm);}
    void xor_(Reg8 dst, uint8_t imm) {xor_(RMOperand{dst}, imm);}

    uint8_t *getPtr() const {return ptr;}

    void resetPtr(uint8_t *oldPtr);

    void patch(uint8_t *patchPtr, uint8_t *patchEndPtr);
    void endPatch();

    void removeRange(uint8_t *startPtr, uint8_t *endPtr);

    bool getError() const {return error;}

private:
    void write(uint8_t b);

    // generic/sub-opcodes
    void encode(uint8_t opcode, int regOp, RMOperand rm, int width, bool isReg = false);
    void encode0F(uint8_t opcode, int regOp, RMOperand rm, int width, bool isReg = false);

    // helpers for 2nd register operand
    void encode(uint8_t opcode, RMOperand rm, Reg64 r) {encode(opcode, static_cast<int>(r), rm, 64, true);}
    void encode(uint8_t opcode, RMOperand rm, Reg32 r) {encode(opcode, static_cast<int>(r), rm, 32, true);}
    void encode(uint8_t opcode, RMOperand rm, Reg16 r) {encode(opcode, static_cast<int>(r), rm, 16, true);}
    void encode(uint8_t opcode, RMOperand rm, Reg8 r)  {encode(opcode, static_cast<int>(r), rm,  8, true);}

    void encode0F(uint8_t opcode, RMOperand rm, Reg32 r) {encode0F(opcode, static_cast<int>(r), rm, 32, true);}

    // for immediate operands
    void encode(uint8_t opcode, int subOp, RMOperand rm, uint32_t imm);
    void encode(uint8_t opcode, int subOp, RMOperand rm, uint8_t imm);

    void encodeModRM(RMOperand rm, int reg2Op = 0, bool isReg = true);
    void encodeREX(bool w, int reg, int index, int base);
    void encodeREX(bool w, int reg, RMOperand rm);

    void encode(uint32_t v);
    void encode(uint64_t v);

    uint8_t *ptr, *endPtr;
    uint8_t *savedPtr, *savedEndPtr;

    bool error = false; // super basic error handling
};
