#pragma once
#include <cstdint>
#include <fstream>
#include <string_view>

class InputRecorder
{
public:
    bool update(uint32_t frame, uint16_t inputs);

    void setFile(std::string_view filename, bool replay);

    uint16_t getInputs() const {return lastInputs;}

    bool isReplay() const {return replay;}

private:
    bool record = false, replay = false;
    uint16_t lastInputs = 0;

    uint32_t nextFrame = 0;
    uint16_t nextInputs = 0;

    std::fstream file;
};