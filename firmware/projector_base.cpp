#include "projector_base.h"

BaseProjector::BaseProjector()
{
    minProjectorExposure_ = 0;

    projectorCurrent_ = 1023;

    projectorExposure_ = 10000;

    projectorTriggerDlay_ = 0;

    projectorWorkingMode_ = 0;
}

BaseProjector::~BaseProjector()
{
}
