#pragma once
#include <iostream>
#include "projector_base.h"
#include "serial_function.h"
#include "string.h"

class AinstecProjector: public BaseProjector
{
    private:
    std::string devName_;

    int serialHandle_;

    public:
    AinstecProjector();
    ~AinstecProjector();

    bool init();

    bool project();

    bool setProjectorCurrent(int current);

    bool setProjectorExposure(int exposure);

    bool getProjectorExposure(int& exposure);

    bool setProjectorTriggerDlay(int dlay);

    bool setProjectorWorkingMode(int mode);
};
