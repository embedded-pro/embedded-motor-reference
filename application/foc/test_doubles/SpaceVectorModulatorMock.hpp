#pragma once

#include "application/foc/MotorFieldOrientedController.hpp"
#include <gmock/gmock.h>

namespace application
{
    class SpaceVectorModulatorMock
        : public SpaceVectorModulator
    {
    public:
        MOCK_METHOD((std::tuple<application::Percent, application::Percent, application::Percent>), Generate, (SpaceVectorModulator::TwoVoltagePhase & voltagePhase), (override));
    };
}
