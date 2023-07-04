#pragma once
#include <iostream>

class BaseProjector
{
protected:
    int minProjectorExposure_;

    int projectorCurrent_;

    int projectorExposure_;

    int projectorTriggerDlay_;

    int projectorWorkingMode_;

public:
    BaseProjector();
    ~BaseProjector();

    virtual bool init() = 0;

    virtual bool project() = 0;

    virtual bool setProjectorCurrent(int current) = 0;

    virtual bool setProjectorExposure(int exposure) = 0;

    virtual bool setProjectorTriggerDlay(int dlay) = 0;

    virtual bool setProjectorWorkingMode(int mode) = 0;

};
