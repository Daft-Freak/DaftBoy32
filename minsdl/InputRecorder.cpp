#include "InputRecorder.h"

bool InputRecorder::update(uint32_t frame, uint16_t inputs)
{
    if(!record && !replay)
        return false;

    if(record)
    {
        if(inputs == lastInputs)
            return false;

        lastInputs = inputs;

        file << frame << " " << inputs << "\n";
    }
    else // replay
    {
        // check frame, set inputs
        if(frame == nextFrame)
        {
            lastInputs = nextInputs;

            file >> nextFrame;
            file >> nextInputs;

            return true;
        }
    }

    return false;
}

void InputRecorder::setFile(std::string_view filename, bool replay)
{
    file.open(std::string(filename), replay ? std::ios::in : std::ios::out);

    if(file)
    {
        record = !replay;
        this->replay = replay;

        if(replay)
        {
            file >> nextFrame;
            file >> nextInputs;
        }
    }
}
