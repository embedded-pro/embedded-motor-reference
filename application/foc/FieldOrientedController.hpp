#pragma once

#include "application/foc/MotorFieldOrientedControllerInterface.hpp"
#include "numerical/controllers/Pid.hpp"

namespace application
{
    class FieldOrientedController
    {
    public:
        virtual std::tuple<hal::Percent, hal::Percent, hal::Percent> Calculate(controllers::Pid<float>& dPid, controllers::Pid<float>& qPid, const std::tuple<MilliAmpere, MilliAmpere, MilliAmpere>& currentPhases, Degrees& position) = 0;
    };
}
