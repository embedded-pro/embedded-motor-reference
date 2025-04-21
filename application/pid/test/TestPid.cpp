#include "application/pid/Pid.hpp"
#include "application/pid/test_doubles/PidInterfaceMock.hpp"
#include "application/pid/test_doubles/PidMock.hpp"
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
        ::testing::StrictMock<application::PidMock> pidMock;
        std::optional<application::PidAsync> pid{ std::in_place, interfaceMock, pidMock };
    };
}

TEST_F(TestPid, setTunningsCallsUnderlyingPid)
{
    EXPECT_CALL(pidMock, SetTunnings(TunningsEq(1.0f, 2.0f, 3.0f)));
    pid->SetTunnings(application::Pid::Tunnings{ 1.0f, 2.0f, 3.0f });
}

TEST_F(TestPid, setPointCallsUnderlyingPid)
{
    const float expectedSetPoint = 42.0f;

    EXPECT_CALL(pidMock, SetPoint(expectedSetPoint));
    pid->SetPoint(expectedSetPoint);
}

TEST_F(TestPid, enableStartsTimerAndEnablesPid)
{
    const float inputValue = 25.0f;
    const float processedValue = 30.0f;

    EXPECT_CALL(pidMock, Enable());
    pid->Enable();

    EXPECT_TRUE(pid->IsRunning());

    EXPECT_CALL(pidMock, Process(inputValue)).WillOnce(testing::Return(processedValue));
    EXPECT_CALL(interfaceMock, ControlAction(processedValue));
    interfaceMock.TriggerCallback(inputValue);
}

TEST_F(TestPid, disableCancelsTimerAndDisablesOutputAndPid)
{
    EXPECT_CALL(pidMock, Enable());
    pid->Enable();
    EXPECT_TRUE(pid->IsRunning());

    EXPECT_CALL(interfaceMock, Stop());
    EXPECT_CALL(pidMock, Disable());
    pid->Disable();

    EXPECT_FALSE(pid->IsRunning());
}

TEST_F(TestPid, timerTriggersMultipleTimesAtCorrectIntervals)
{
    const float inputValue1 = 25.0f;
    const float processedValue1 = 30.0f;
    const float inputValue2 = 27.0f;
    const float processedValue2 = 32.0f;
    const float inputValue3 = 29.0f;
    const float processedValue3 = 34.0f;

    EXPECT_CALL(pidMock, Enable());
    pid->Enable();

    EXPECT_CALL(pidMock, Process(inputValue1)).WillOnce(testing::Return(processedValue1));
    EXPECT_CALL(interfaceMock, ControlAction(processedValue1));
    interfaceMock.TriggerCallback(inputValue1);

    EXPECT_CALL(pidMock, Process(inputValue2)).WillOnce(testing::Return(processedValue2));
    EXPECT_CALL(interfaceMock, ControlAction(processedValue2));
    interfaceMock.TriggerCallback(inputValue2);

    EXPECT_CALL(pidMock, Process(inputValue3)).WillOnce(testing::Return(processedValue3));
    EXPECT_CALL(interfaceMock, ControlAction(processedValue3));
    interfaceMock.TriggerCallback(inputValue3);
}

TEST_F(TestPid, timerRestartedAfterDisableAndEnable)
{
    const float inputValue1 = 25.0f;
    const float processedValue1 = 30.0f;
    const float inputValue2 = 27.0f;
    const float processedValue2 = 32.0f;

    EXPECT_CALL(pidMock, Enable());
    pid->Enable();

    EXPECT_CALL(pidMock, Process(inputValue1)).WillOnce(testing::Return(processedValue1));
    EXPECT_CALL(interfaceMock, ControlAction(processedValue1));
    interfaceMock.TriggerCallback(inputValue1);

    EXPECT_CALL(interfaceMock, Stop());
    EXPECT_CALL(pidMock, Disable());
    pid->Disable();

    EXPECT_CALL(pidMock, Enable());
    pid->Enable();

    EXPECT_CALL(pidMock, Process(inputValue2)).WillOnce(testing::Return(processedValue2));
    EXPECT_CALL(interfaceMock, ControlAction(processedValue2));
    interfaceMock.TriggerCallback(inputValue2);
}

TEST_F(TestPid, isRunningReturnsFalseInitially)
{
    EXPECT_FALSE(pid->IsRunning());
}
