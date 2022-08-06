#include <cassert>

#include "ThumbBuilder.h"

void ThumbBuilder::bx(Reg r)
{
    int reg = static_cast<int>(r);
    write(0x4700 | reg << 3);
}

void ThumbBuilder::resetPtr(uint16_t *oldPtr)
{
    assert(oldPtr < ptr);
    ptr = oldPtr;
    error = false;
}

void ThumbBuilder::write(uint16_t hw)
{
    if(ptr + 1 != endPtr)
        *ptr++ = hw;
    else
        error = true;
}