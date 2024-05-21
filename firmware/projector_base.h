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

    virtual bool setProjectorCurrent(int current) = 0; // current的值从0-1023，0表示最暗，1023表示最亮

    virtual bool setProjectorExposure(int exposure) = 0; // exposure的单位是us

    virtual bool setProjectorTriggerDlay(int dlay) = 0;

    virtual bool setProjectorWorkingMode(int mode) = 0;

    virtual bool getCorrectExposure(int& exposureTime) {return false;}

    virtual bool setProjectorMinTriggerIntervalTime(int minTriggerInterval) {return false;};

};
