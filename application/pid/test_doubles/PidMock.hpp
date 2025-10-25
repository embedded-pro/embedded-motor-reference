#pragma once

#include "application/pid/PidInterface.hpp"
#include <gmock/gmock.h>

namespace application
{
    MATCHER_P(TuningsEq, expected, "")
    {
        return arg.kp == expected.kp && arg.ki == expected.ki && arg.kd == expected.kd;
    }

    class PidMock
        : public application::Pid
    {
    public:
        MOCK_METHOD(void, SetTunings, (application::Pid::Tunings), (override));
        MOCK_METHOD(void, SetPoint, (float), (override));
        MOCK_METHOD(void, Enable, (), (override));
        MOCK_METHOD(void, Disable, (), (override));
    };
}
