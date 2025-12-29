#pragma once

#include "source/foc/interfaces/Controller.hpp"
#include <gmock/gmock.h>

namespace foc
{
    class TorqueControllerMock
        : public TorqueController
    {
    public:
        MOCK_METHOD(void, Enable, (), (override));
        MOCK_METHOD(void, Disable, (), (override));
        MOCK_METHOD(bool, IsRunning, (), (const, override));
        MOCK_METHOD(void, SetCurrentTunings, (Volts Vcd, IdAndIqTunings tunings), (override));
        MOCK_METHOD(void, SetPoint, (const IdAndIqPoint& point), (override));
    };

    class SpeedControllerMock
        : public SpeedController
    {
    public:
        MOCK_METHOD(void, Enable, (), (override));
        MOCK_METHOD(void, Disable, (), (override));
        MOCK_METHOD(bool, IsRunning, (), (const, override));
        MOCK_METHOD(void, SetSpeedTunings, (Volts Vcd, const SpeedTunings& speedTuning), (override));
        MOCK_METHOD(void, SetCurrentTunings, (Volts Vcd, IdAndIqTunings tunings), (override));
        MOCK_METHOD(void, SetPoint, (RadiansPerSecond point), (override));
    };
}
