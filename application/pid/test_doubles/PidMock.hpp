#pragma once

#include "application/pid/Pid.hpp"
#include <gmock/gmock.h>

namespace application
{
    MATCHER_P(TunningsEq, expected, "")
    {
        return arg.kp == expected.kp && arg.ki == expected.ki && arg.kd == expected.kd;
    }

    class PidMock
        : public application::Pid
    {
    public:
        MOCK_METHOD(void, SetTunnings, (application::Pid::Tunnings), (override));
        MOCK_METHOD(void, SetPoint, (float), (override));
        MOCK_METHOD(void, Enable, (), (override));
        MOCK_METHOD(void, Disable, (), (override));
        MOCK_METHOD(float, Process, (float), (override));
    };
}
