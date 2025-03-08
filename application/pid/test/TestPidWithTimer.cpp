#include "application/pid/PidWithTimer.hpp"
#include "infra/timer/test_helper/PerfectTimerService.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    class InputMock
        : public application::Input
    {
    public:
        MOCK_METHOD(float, Read, (), (override));
    };

    class OutputMock
        : public application::Output
    {
    public:
        MOCK_METHOD(void, Update, (float), (override));
        MOCK_METHOD(void, Disable, (), (override));
    };

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

    MATCHER_P3(TunningsEq, kp, ki, kd, "")
    {
        return std::abs(arg.kp - kp) < 1e-6f &&
               std::abs(arg.ki - ki) < 1e-6f &&
               std::abs(arg.kd - kd) < 1e-6f;
    }

    class TestPidWithTimer
        : public ::testing::Test
    {
    public:
        ::testing::StrictMock<InputMock> inputMock;
        ::testing::StrictMock<OutputMock> outputMock;
        ::testing::StrictMock<PidMock> pidMock;
        const uint32_t timerId = 1;
        const infra::Duration sampleTime = std::chrono::milliseconds(10);
        infra::PerfectTimerService timer{ timerId };
        application::PidWithTimer pid{ inputMock, outputMock, pidMock, sampleTime, timerId };
    };
}

TEST_F(TestPidWithTimer, setTunningsCallsUnderlyingPid)
{
    EXPECT_CALL(pidMock, SetTunnings(TunningsEq(1.0f, 2.0f, 3.0f)));
    pid.SetTunnings(application::Pid::Tunnings{ 1.0f, 2.0f, 3.0f });
}

TEST_F(TestPidWithTimer, setPointCallsUnderlyingPid)
{
    const float expectedSetPoint = 42.0f;

    EXPECT_CALL(pidMock, SetPoint(expectedSetPoint));
    pid.SetPoint(expectedSetPoint);
}

TEST_F(TestPidWithTimer, enableStartsTimerAndEnablesPid)
{
    const float inputValue = 25.0f;
    const float processedValue = 30.0f;

    EXPECT_CALL(pidMock, Enable());
    pid.Enable();

    EXPECT_TRUE(pid.IsRunning());

    EXPECT_CALL(inputMock, Read()).WillOnce(testing::Return(inputValue));
    EXPECT_CALL(pidMock, Process(inputValue)).WillOnce(testing::Return(processedValue));
    EXPECT_CALL(outputMock, Update(processedValue));

    timer.TimeProgressed(sampleTime);
}

TEST_F(TestPidWithTimer, disableCancelsTimerAndDisablesOutputAndPid)
{
    EXPECT_CALL(pidMock, Enable());
    pid.Enable();
    EXPECT_TRUE(pid.IsRunning());

    EXPECT_CALL(outputMock, Disable());
    EXPECT_CALL(pidMock, Disable());
    pid.Disable();

    EXPECT_FALSE(pid.IsRunning());
}

TEST_F(TestPidWithTimer, timerTriggersMultipleTimesAtCorrectIntervals)
{
    const float inputValue1 = 25.0f;
    const float processedValue1 = 30.0f;
    const float inputValue2 = 27.0f;
    const float processedValue2 = 32.0f;
    const float inputValue3 = 29.0f;
    const float processedValue3 = 34.0f;

    EXPECT_CALL(pidMock, Enable());
    pid.Enable();

    EXPECT_CALL(inputMock, Read()).WillOnce(testing::Return(inputValue1));
    EXPECT_CALL(pidMock, Process(inputValue1)).WillOnce(testing::Return(processedValue1));
    EXPECT_CALL(outputMock, Update(processedValue1));
    timer.TimeProgressed(sampleTime);

    EXPECT_CALL(inputMock, Read()).WillOnce(testing::Return(inputValue2));
    EXPECT_CALL(pidMock, Process(inputValue2)).WillOnce(testing::Return(processedValue2));
    EXPECT_CALL(outputMock, Update(processedValue2));
    timer.TimeProgressed(sampleTime);

    EXPECT_CALL(inputMock, Read()).WillOnce(testing::Return(inputValue3));
    EXPECT_CALL(pidMock, Process(inputValue3)).WillOnce(testing::Return(processedValue3));
    EXPECT_CALL(outputMock, Update(processedValue3));
    timer.TimeProgressed(sampleTime);
}

TEST_F(TestPidWithTimer, timerRestartedAfterDisableAndEnable)
{
    const float inputValue1 = 25.0f;
    const float processedValue1 = 30.0f;
    const float inputValue2 = 27.0f;
    const float processedValue2 = 32.0f;

    EXPECT_CALL(pidMock, Enable());
    pid.Enable();

    EXPECT_CALL(inputMock, Read()).WillOnce(testing::Return(inputValue1));
    EXPECT_CALL(pidMock, Process(inputValue1)).WillOnce(testing::Return(processedValue1));
    EXPECT_CALL(outputMock, Update(processedValue1));
    timer.TimeProgressed(sampleTime);

    EXPECT_CALL(outputMock, Disable());
    EXPECT_CALL(pidMock, Disable());
    pid.Disable();

    EXPECT_CALL(pidMock, Enable());
    pid.Enable();

    EXPECT_CALL(inputMock, Read()).WillOnce(testing::Return(inputValue2));
    EXPECT_CALL(pidMock, Process(inputValue2)).WillOnce(testing::Return(processedValue2));
    EXPECT_CALL(outputMock, Update(processedValue2));
    timer.TimeProgressed(sampleTime);
}

TEST_F(TestPidWithTimer, isRunningReturnsFalseInitially)
{
    EXPECT_FALSE(pid.IsRunning());
}

TEST_F(TestPidWithTimer, progressingTimeWithoutEnableDoesNotTriggerTimer)
{
    EXPECT_CALL(inputMock, Read()).Times(0);
    EXPECT_CALL(pidMock, Process(testing::_)).Times(0);
    EXPECT_CALL(outputMock, Update(testing::_)).Times(0);

    timer.TimeProgressed(sampleTime);
    timer.TimeProgressed(sampleTime);
}
