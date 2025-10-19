#pragma once

#include "application/foc/MotorDriver.hpp"
#include "application/foc/MotorFieldOrientedController.hpp"
#include <gmock/gmock.h>

namespace application
{
    class MotorFieldOrientedControllerMock
        : public MotorFieldOrientedController
    {
    public:
        MOCK_METHOD(void, SetTunings, (application::Volts Vcd, MotorFieldOrientedController::IdAndIqTunings tunings), (override));
        MOCK_METHOD(void, SetPoint, (const IdAndIqPoint& point), (override));
        MOCK_METHOD(void, Enable, (), (override));
        MOCK_METHOD(void, Disable, (), (override));
        MOCK_METHOD(bool, IsRunning, (), (const, override));
    };
}
