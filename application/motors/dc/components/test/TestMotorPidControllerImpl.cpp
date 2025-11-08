#include "application/motors/dc/components/MotorPidControllerImpl.hpp"
#include "gmock/gmock.h"

namespace
{
    MATCHER_P(PercentEq, expected, "is duty cycle equal")
    {
        return std::abs(arg.Value() - expected.Value()) < 1e-5f;
    }

    MATCHER_P(TuningsEq, expected, "")
    {
        return arg.kp == expected.kp &&
               arg.ki == expected.ki &&
               arg.kd == expected.kd;
    }

    template<typename T>
    class MockAsynchronousPidController
        : public controllers::AsynchronousPidController<T>
    {
    public:
        MOCK_METHOD(void, SetTunings, (controllers::PidTunings<T>), (override));
        MOCK_METHOD(void, SetLimits, (controllers::PidLimits<T>), (override));
        MOCK_METHOD(void, SetPoint, (T), (override));
        MOCK_METHOD(void, Enable, (), (override));
        MOCK_METHOD(void, Disable, (), (override));
    };

    class TestMotorControllerImpl
        : public ::testing::Test
    {
    public:
        ::testing::StrictMock<MockAsynchronousPidController<float>> pidMock;
        application::MotorPidControllerImpl controller{ pidMock };
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

    EXPECT_CALL(pidMock, SetTunings(TuningsEq(controllers::PidTunings<float>{ kp, ki, kd })));
    controller.SetPidParameters(std::make_optional(kp), std::make_optional(ki), std::make_optional(kd));
}

TEST_F(TestMotorControllerImpl, set_pid_parameters_with_null)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    EXPECT_CALL(pidMock, SetTunings(TuningsEq(controllers::PidTunings<float>{ kp, ki, kd })));
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
    EXPECT_CALL(pidMock, Disable());
    controller.Stop();
}
