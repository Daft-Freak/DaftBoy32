#include <cstdio>

#include "RecompilerGeneric.h"

void printGenBlock(uint32_t pc, const GenBlockInfo &block, const SourceInfo &sourceInfo)
{
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
    

    uint32_t lastPC = 1;

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
        if(instr.flags & GenOp_Exit)
            printf(" ex");

        if(instr.flags & GenOp_BranchTarget)
            printf(" bt");

        printf("\n");


        pc += instr.len;
    }
}