#pragma once

#include "application/foc/MotorFieldOrientedControllerInterface.hpp"
#include "numerical/controllers/Pid.hpp"

namespace application
{
    class FieldOrientedController
    {
    public:
        virtual std::tuple<Percent, Percent, Percent> Calculate(controllers::Pid<float>& dPid, controllers::Pid<float>& qPid, const std::tuple<MilliVolt, MilliVolt, MilliVolt>& voltagePhases, Degrees& position) = 0;
    };
}
