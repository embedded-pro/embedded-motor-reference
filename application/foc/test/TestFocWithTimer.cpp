#include "application/foc/FocWithTimer.hpp"
#include "infra/timer/test_helper/PerfectTimerService.hpp"
#include "gmock/gmock.h"
#include <chrono>

MATCHER_P(TunningsEq, expected, "")
{
    return arg.kp == expected.kp && arg.ki == expected.ki && arg.kd == expected.kd;
}

namespace
{
    class InputMock
        : public application::FocInput
    {
    public:
        MOCK_METHOD(FocInput::Input&, Read, (), (override));
    };

    class OutputMock
        : public application::FocOutput
    {
    public:
        MOCK_METHOD(void, Update, (FocOutput::Output&), (override));
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

    class TrigonometricFunctionsMock
        : public math::TrigonometricFunctions<float>
    {
    public:
        MOCK_METHOD(float, Cosine, (const float& angle), (const, override));
        MOCK_METHOD(float, Sine, (const float& angle), (const, override));
        MOCK_METHOD(float, Arctangent, (const float& value), (const, override));
        MOCK_METHOD(float, Phase, (const float& real, const float& imag), (const, override));
    };

    class SpaceVectorModulatorMock
        : public application::SpaceVectorModulator
    {
    public:
        MOCK_METHOD(application::FocOutput::Output, Generate, (TwoVoltagePhase & voltagePhase), (override));
    };

    class TestFocWithTimer
        : public ::testing::Test
    {
    public:
        ::testing::StrictMock<InputMock> inputMock;
        ::testing::StrictMock<OutputMock> outputMock;
        ::testing::StrictMock<TrigonometricFunctionsMock> trigonometricMock;
        ::testing::StrictMock<SpaceVectorModulatorMock> spaceVectorModulatorMock;
        ::testing::StrictMock<PidMock> dPidMock;
        ::testing::StrictMock<PidMock> qPidMock;
        const uint32_t timerId = 1;
        const infra::Duration sampleTime = std::chrono::microseconds(100);
        infra::PerfectTimerService timer{ timerId };
        application::FocWithTimer::Components components{ trigonometricMock, spaceVectorModulatorMock, dPidMock, qPidMock, sampleTime, timerId };
        application::FocWithTimer foc{ inputMock, outputMock, components };

        controllers::ThreePhase<float> threePhaseCurrents{ 1.0f, 2.0f, 3.0f };
        float electricalAngle = 0.5f;
        application::FocInput::Input inputData{ threePhaseCurrents, electricalAngle };
        controllers::TwoPhase<float> alphaAndBeta{ 2.0f, 1.5f };
        controllers::RotatingFrame<float> dqFrame{ 1.2f, 0.8f };
        controllers::RotatingFrame<float> dqVoltage{ 5.0f, 3.0f };
        controllers::TwoPhase<float> voltageAlphaBeta{ 4.0f, 2.5f };
        controllers::SpaceVectorModulation<float>::Output pwmOutput = { 0.4f, 0.5f, 0.6f };

        void SetUp() override
        {
            ON_CALL(inputMock, Read())
                .WillByDefault(::testing::ReturnRef(inputData));
        }
    };
}

TEST_F(TestFocWithTimer, construction_doesnt_start_timer)
{
    EXPECT_FALSE(foc.IsRunning());
}

TEST_F(TestFocWithTimer, set_tunnings_delegates_to_pid_controllers)
{
    controllers::Pid<float>::Tunnings dTunnings{ 1.0f, 2.0f, 3.0f };
    controllers::Pid<float>::Tunnings qTunnings{ 4.0f, 5.0f, 6.0f };

    EXPECT_CALL(dPidMock, SetTunnings(TunningsEq(dTunnings)));
    EXPECT_CALL(qPidMock, SetTunnings(TunningsEq(qTunnings)));

    foc.SetTunnings({ dTunnings, qTunnings });
}

TEST_F(TestFocWithTimer, set_point_delegates_to_pid_controllers)
{
    EXPECT_CALL(dPidMock, SetPoint(2.5f));
    EXPECT_CALL(qPidMock, SetPoint(3.5f));

    foc.SetPoint({ 2.5f, 3.5f });
}

TEST_F(TestFocWithTimer, enable_starts_pid_controllers_and_timer)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());

    foc.Enable();

    EXPECT_TRUE(foc.IsRunning());
}

TEST_F(TestFocWithTimer, disable_stops_timer_and_pid_controllers)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();

    EXPECT_CALL(outputMock, Disable());
    EXPECT_CALL(dPidMock, Disable());
    EXPECT_CALL(qPidMock, Disable());

    foc.Disable();

    EXPECT_FALSE(foc.IsRunning());
}

TEST_F(TestFocWithTimer, timer_callback_executes_complete_foc_pipeline)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();

    {
        ::testing::InSequence seq;

        EXPECT_CALL(inputMock, Read())
            .WillOnce(::testing::ReturnRef(inputData));

        EXPECT_CALL(trigonometricMock, Cosine(electricalAngle))
            .WillOnce(::testing::Return(0.8775f));
        EXPECT_CALL(trigonometricMock, Sine(electricalAngle))
            .WillOnce(::testing::Return(0.4794f));

        EXPECT_CALL(dPidMock, Process(::testing::_))
            .WillOnce(::testing::Return(dqVoltage.d));
        EXPECT_CALL(qPidMock, Process(::testing::_))
            .WillOnce(::testing::Return(dqVoltage.q));

        EXPECT_CALL(trigonometricMock, Cosine(electricalAngle))
            .WillOnce(::testing::Return(0.8775f));
        EXPECT_CALL(trigonometricMock, Sine(electricalAngle))
            .WillOnce(::testing::Return(0.4794f));

        EXPECT_CALL(spaceVectorModulatorMock, Generate(::testing::_))
            .WillOnce(::testing::Return(pwmOutput));

        EXPECT_CALL(outputMock, Update(::testing::_));
    }

    timer.TimeProgressed(sampleTime);
}

TEST_F(TestFocWithTimer, timer_fires_at_correct_interval)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();

    EXPECT_CALL(inputMock, Read());
    EXPECT_CALL(trigonometricMock, Cosine(::testing::_)).Times(2);
    EXPECT_CALL(trigonometricMock, Sine(::testing::_)).Times(2);
    EXPECT_CALL(dPidMock, Process(::testing::_));
    EXPECT_CALL(qPidMock, Process(::testing::_));
    EXPECT_CALL(spaceVectorModulatorMock, Generate(::testing::_));
    EXPECT_CALL(outputMock, Update(::testing::_));

    timer.TimeProgressed(sampleTime);

    ::testing::Mock::VerifyAndClearExpectations(&inputMock);
    ::testing::Mock::VerifyAndClearExpectations(&trigonometricMock);
    ::testing::Mock::VerifyAndClearExpectations(&dPidMock);
    ::testing::Mock::VerifyAndClearExpectations(&qPidMock);
    ::testing::Mock::VerifyAndClearExpectations(&spaceVectorModulatorMock);
    ::testing::Mock::VerifyAndClearExpectations(&outputMock);

    EXPECT_CALL(inputMock, Read());
    EXPECT_CALL(trigonometricMock, Cosine(::testing::_)).Times(2);
    EXPECT_CALL(trigonometricMock, Sine(::testing::_)).Times(2);
    EXPECT_CALL(dPidMock, Process(::testing::_));
    EXPECT_CALL(qPidMock, Process(::testing::_));
    EXPECT_CALL(spaceVectorModulatorMock, Generate(::testing::_));
    EXPECT_CALL(outputMock, Update(::testing::_));

    timer.TimeProgressed(sampleTime);
}

TEST_F(TestFocWithTimer, disable_can_be_called_multiple_times)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();

    EXPECT_CALL(outputMock, Disable());
    EXPECT_CALL(dPidMock, Disable());
    EXPECT_CALL(qPidMock, Disable());

    foc.Disable();
}

TEST_F(TestFocWithTimer, is_running_returns_correct_state)
{
    EXPECT_FALSE(foc.IsRunning());

    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();
    EXPECT_TRUE(foc.IsRunning());

    EXPECT_CALL(outputMock, Disable());
    EXPECT_CALL(dPidMock, Disable());
    EXPECT_CALL(qPidMock, Disable());
    foc.Disable();
    EXPECT_FALSE(foc.IsRunning());
}
