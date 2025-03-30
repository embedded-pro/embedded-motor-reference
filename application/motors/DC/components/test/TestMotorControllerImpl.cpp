#include "application/motors/DC/components/MotorController.hpp"
#include "application/motors/DC/components/MotorControllerImpl.hpp"
#include "application/pid/PidWithTimer.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include "infra/timer/test_helper/PerfectTimerService.hpp"
#include "infra/util/Function.hpp"
#include "gmock/gmock.h"
#include <chrono>

namespace
{
    MATCHER_P(PercentEq, expected, "is duty cycle equal")
    {
        return std::abs(arg.Value() - expected.Value()) < 1e-5f;
    }

    MATCHER_P3(TunningsEq, kp, ki, kd, "")
    {
        return std::abs(arg.kp - kp) < 1e-6f &&
               std::abs(arg.ki - ki) < 1e-6f &&
               std::abs(arg.kd - kd) < 1e-6f;
    }

    class QuadratureEncoderMock
        : public hal::SynchronousQuadratureEncoder
    {
    public:
        MOCK_METHOD(uint32_t, Position, (), (override));
        MOCK_METHOD(uint32_t, Resolution, (), (override));
        MOCK_METHOD(MotionDirection, Direction, (), (override));
        MOCK_METHOD(uint32_t, Speed, (), (override));
    };

    class PwmMock
        : public hal::SynchronousSingleChannelPwm
    {
    public:
        MOCK_METHOD(void, SetBaseFrequency, (hal::Hertz baseFrequency), (override));
        MOCK_METHOD(void, Start, (hal::Percent), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };

    class PidMock
        : public application::Pid
    {
    public:
        MOCK_METHOD(void, SetTunnings, (Tunnings tunnings), (override));
        MOCK_METHOD(void, SetPoint, (float setPoint), (override));
        MOCK_METHOD(void, Enable, (), (override));
        MOCK_METHOD(void, Disable, (), (override));
        MOCK_METHOD(float, Process, (float measuredProcessVariable), (override));
    };

    class TestMotorControllerImpl
        : public ::testing::Test
    {
    public:
        const uint32_t timerId = 1;
        infra::PerfectTimerService timer{ timerId };
        ::testing::StrictMock<QuadratureEncoderMock> inputMock;
        ::testing::StrictMock<PwmMock> outputMock;
        ::testing::StrictMock<PidMock> pidMock;
        application::MotorControllerImpl controller{ inputMock, outputMock, pidMock, timerId };
    };
}

TEST_F(TestMotorControllerImpl, auto_tune)
{
    controller.AutoTune(infra::emptyFunction);
}

TEST_F(TestMotorControllerImpl, set_pid_parameters)
{
    float kp = 0.1f;
    float ki = 0.2f;
    float kd = 0.3f;

    EXPECT_CALL(pidMock, SetTunnings(TunningsEq(kp, ki, kd)));
    controller.SetPidParameters(std::make_optional(kp), std::make_optional(ki), std::make_optional(kd));
}

TEST_F(TestMotorControllerImpl, set_speed)
{
    application::MotorController::RevPerMinute speed{ 1.234f };

    EXPECT_CALL(pidMock, SetPoint(speed.Value()));
    controller.SetSpeed(speed);
}

TEST_F(TestMotorControllerImpl, enable)
{
    EXPECT_CALL(pidMock, Enable());
    controller.Start();
}

TEST_F(TestMotorControllerImpl, disable)
{
    EXPECT_CALL(outputMock, Stop());
    EXPECT_CALL(pidMock, Disable());
    controller.Stop();
}

TEST_F(TestMotorControllerImpl, input_output)
{
    ::testing::InSequence _;

    EXPECT_CALL(pidMock, Enable());
    EXPECT_CALL(inputMock, Speed()).WillOnce(::testing::Return(256));
    EXPECT_CALL(pidMock, Process(256.0f)).WillOnce(::testing::Return(0.5f));
    EXPECT_CALL(outputMock, Start(PercentEq(hal::Percent(50.f))));

    controller.Start();
    timer.TimeProgressed(std::chrono::milliseconds(5));
}
