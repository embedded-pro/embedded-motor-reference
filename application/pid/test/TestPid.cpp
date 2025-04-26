#include "application/pid/PidImpl.hpp"
#include "application/pid/test_doubles/PidInterfaceMock.hpp"
#include "gmock/gmock.h"
#include <optional>
#include <utility>

namespace
{
    MATCHER_P3(TunningsEq, kp, ki, kd, "")
    {
        return std::abs(arg.kp - kp) < 1e-6f &&
               std::abs(arg.ki - ki) < 1e-6f &&
               std::abs(arg.kd - kd) < 1e-6f;
    }

    class TestPid
        : public ::testing::Test
    {
    public:
        ::testing::StrictMock<application::PidInterfaceMock> interfaceMock;
        std::optional<application::PidImpl> pid{ std::in_place, interfaceMock };
    };
}

TEST_F(TestPid, enableStartsTimerAndEnablesPid)
{
    const float inputValue = 25.0f;
    const float processedValue = 30.0f;

    EXPECT_CALL(interfaceMock, ControlAction(processedValue));
    pid->Enable();
    interfaceMock.TriggerCallback(inputValue);
}

TEST_F(TestPid, disableCancelsTimerAndDisablesOutputAndPid)
{
    pid->Enable();
    EXPECT_CALL(interfaceMock, Stop());
    pid->Disable();
}

TEST_F(TestPid, timerTriggersMultipleTimesAtCorrectIntervals)
{
    const float inputValue1 = 25.0f;
    const float processedValue1 = 30.0f;
    const float inputValue2 = 27.0f;
    const float processedValue2 = 32.0f;
    const float inputValue3 = 29.0f;
    const float processedValue3 = 34.0f;

    pid->Enable();

    EXPECT_CALL(interfaceMock, ControlAction(processedValue1));
    interfaceMock.TriggerCallback(inputValue1);

    EXPECT_CALL(interfaceMock, ControlAction(processedValue2));
    interfaceMock.TriggerCallback(inputValue2);

    EXPECT_CALL(interfaceMock, ControlAction(processedValue3));
    interfaceMock.TriggerCallback(inputValue3);
}

TEST_F(TestPid, timerRestartedAfterDisableAndEnable)
{
    const float inputValue1 = 25.0f;
    const float processedValue1 = 30.0f;
    const float inputValue2 = 27.0f;
    const float processedValue2 = 32.0f;

    pid->Enable();

    EXPECT_CALL(interfaceMock, ControlAction(processedValue1));
    interfaceMock.TriggerCallback(inputValue1);

    EXPECT_CALL(interfaceMock, Stop());
    pid->Disable();

    pid->Enable();

    EXPECT_CALL(interfaceMock, ControlAction(processedValue2));
    interfaceMock.TriggerCallback(inputValue2);
}
