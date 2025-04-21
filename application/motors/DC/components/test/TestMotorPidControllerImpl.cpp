#include "application/motors/DC/components/MotorPidControllerImpl.hpp"
#include "application/pid/test_doubles/PidInterfaceMock.hpp"
#include "application/pid/test_doubles/PidMock.hpp"
#include "infra/util/Function.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <optional>

namespace
{
    MATCHER_P(PercentEq, expected, "is duty cycle equal")
    {
        return std::abs(arg.Value() - expected.Value()) < 1e-5f;
    }

    class TestMotorControllerImpl
        : public ::testing::Test
    {
    public:
        ::testing::StrictMock<application::PidInterfaceMock> pidInterfaceMock;
        ::testing::StrictMock<application::PidMock> pidMock;
        application::MotorPidControllerImpl controller{ pidInterfaceMock, pidMock };
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

    EXPECT_CALL(pidMock, SetTunnings(application::TunningsEq(application::Pid::Tunnings{ kp, ki, kd })));
    controller.SetPidParameters(std::make_optional(kp), std::make_optional(ki), std::make_optional(kd));
}

TEST_F(TestMotorControllerImpl, set_pid_parameters_with_null)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    EXPECT_CALL(pidMock, SetTunnings(application::TunningsEq(application::Pid::Tunnings{ kp, ki, kd })));
    controller.SetPidParameters(std::nullopt, std::nullopt, std::nullopt);
}

TEST_F(TestMotorControllerImpl, set_speed)
{
    application::MotorPidController::RevPerMinute speed{ 1.234f };

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
    EXPECT_CALL(pidInterfaceMock, Stop());
    EXPECT_CALL(pidMock, Disable());
    controller.Stop();
}

// TEST_F(TestMotorControllerImpl, input_output)
// {
//     ::testing::InSequence _;

//     EXPECT_CALL(pidMock, Enable());
//     EXPECT_CALL(inputMock, Speed()).WillOnce(::testing::Return(256));
//     EXPECT_CALL(pidMock, Process(256.0f)).WillOnce(::testing::Return(0.5f));
//     EXPECT_CALL(outputMock, Start(PercentEq(hal::Percent(50.f))));

//     controller.Start();
//     timer.TimeProgressed(std::chrono::milliseconds(5));
// }
