#include "application/motors/BLDC/components/MotorControllerImpl.hpp"
#include "infra/timer/test_helper/PerfectTimerService.hpp"
#include "infra/util/MemoryRange.hpp"
#include "numerical/controllers/Pid.hpp"
#include "numerical/controllers/SpaceVectorModulation.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace
{
    class SynchronousAdcMock
        : public hal::SynchronousAdc
    {
    public:
        MOCK_METHOD(hal::SynchronousAdc::Samples, Measure, (std::size_t numberOfSamples), (override));
    };

    class SynchronousQuadratureEncoderMock
        : public hal::SynchronousQuadratureEncoder
    {
    public:
        MOCK_METHOD(uint32_t, Position, (), (override));
        MOCK_METHOD(uint32_t, Resolution, (), (override));
        MOCK_METHOD(hal::SynchronousQuadratureEncoder::MotionDirection, Direction, (), (override));
        MOCK_METHOD(uint32_t, Speed, (), (override));
    };

    class SynchronousThreeChannelsPwmMock
        : public hal::SynchronousThreeChannelsPwm
    {
    public:
        MOCK_METHOD(void, Start, (hal::Percent dutyCycle1, hal::Percent dutyCycle2, hal::Percent dutyCycle3), (override));
        MOCK_METHOD(void, Stop, (), (override));
        MOCK_METHOD(void, SetBaseFrequency, (hal::Hertz baseFrequency), (override));
    };

    class SpaceVectorModulatorMock
        : public application::SpaceVectorModulator
    {
    public:
        MOCK_METHOD(application::FocOutput::Output, Generate, (TwoVoltagePhase & voltagePhase), (override));
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

    class TrigonometricFunctionsMock
        : public math::TrigonometricFunctions<float>
    {
    public:
        MOCK_METHOD(float, Cosine, (const float& angle), (const, override));
        MOCK_METHOD(float, Sine, (const float& angle), (const, override));
        MOCK_METHOD(float, Arctangent, (const float& value), (const, override));
        MOCK_METHOD(float, Phase, (const float& real, const float& imag), (const, override));
    };

    class FocControllerTest
        : public testing::Test
    {
    public:
        const uint32_t timerId = 1;
        infra::PerfectTimerService timer{ timerId };
        testing::StrictMock<SynchronousAdcMock> phaseA;
        testing::StrictMock<SynchronousAdcMock> phaseB;
        testing::StrictMock<SynchronousQuadratureEncoderMock> theta;
        testing::StrictMock<SynchronousThreeChannelsPwmMock> pwm;
        testing::StrictMock<TrigonometricFunctionsMock> trigFunctions;
        testing::StrictMock<SpaceVectorModulatorMock> spaceVectorModulator;
        testing::StrictMock<PidMock> dPid;
        testing::StrictMock<PidMock> qPid;

        application::FocWithTimer::Components components{ trigFunctions, spaceVectorModulator, dPid, qPid, std::chrono::milliseconds(1), timerId };
        application::FocControllerImpl::Input input{ phaseA, phaseB, theta };
        application::FocControllerImpl controller{ input, pwm, components };
    };
}

TEST_F(FocControllerTest, WhenStartingControllerThenFocIsEnabled)
{
    EXPECT_CALL(dPid, Enable());
    EXPECT_CALL(qPid, Enable());

    controller.Start();
}

TEST_F(FocControllerTest, WhenStoppingControllerThenFocIsDisabled)
{
    EXPECT_CALL(dPid, Disable());
    EXPECT_CALL(qPid, Disable());

    EXPECT_CALL(pwm, Stop());

    controller.Stop();
}

TEST_F(FocControllerTest, WhenSettingTorqueThenFocSetPointIsUpdated)
{
    const float expectedTorque = 0.75f;

    EXPECT_CALL(qPid, SetPoint(0));
    EXPECT_CALL(dPid, SetPoint(expectedTorque));

    controller.SetTorque(application::FocController::Torque(expectedTorque));
}

TEST_F(FocControllerTest, WhenSettingDQPidParametersThenPidTunningsAreUpdated)
{
    application::FocController::PidFocParameters dParameters;
    dParameters.kp = 10.0f;
    dParameters.ki = 5.0f;
    dParameters.kd = 0.1f;

    application::FocController::PidFocParameters qParameters;
    qParameters.kp = 15.0f;
    qParameters.ki = 7.5f;
    qParameters.kd = 0.15f;

    EXPECT_CALL(dPid, SetTunnings(testing::AllOf(
                          testing::Field(&controllers::Pid<float>::Tunnings::kp, 10.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::ki, 5.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::kd, 0.1f))));

    EXPECT_CALL(qPid, SetTunnings(testing::AllOf(
                          testing::Field(&controllers::Pid<float>::Tunnings::kp, 15.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::ki, 7.5f),
                          testing::Field(&controllers::Pid<float>::Tunnings::kd, 0.15f))));

    controller.SetDQPidParameters(std::make_pair(dParameters, qParameters));
}

TEST_F(FocControllerTest, WhenSettingPartialDQParametersThenOnlyThoseParametersAreUpdated)
{
    application::FocController::PidFocParameters dParameters;
    dParameters.kp = 10.0f;

    application::FocController::PidFocParameters qParameters;
    qParameters.ki = 7.5f;

    EXPECT_CALL(dPid, SetTunnings(testing::AllOf(
                          testing::Field(&controllers::Pid<float>::Tunnings::kp, 10.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::ki, 0.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::kd, 0.0f))));

    EXPECT_CALL(qPid, SetTunnings(testing::AllOf(
                          testing::Field(&controllers::Pid<float>::Tunnings::kp, 0.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::ki, 7.5f),
                          testing::Field(&controllers::Pid<float>::Tunnings::kd, 0.0f))));

    controller.SetDQPidParameters(std::make_pair(dParameters, qParameters));
}

TEST_F(FocControllerTest, WhenReadingInputThenPhaseCurrentsAndThetaAreProcessedCorrectly)
{
    static uint16_t phaseAValues[] = { 1638 };
    static uint16_t phaseBValues[] = { 819 };
    infra::MemoryRange<const uint16_t> phaseARange(phaseAValues, phaseAValues + 1);
    infra::MemoryRange<const uint16_t> phaseBRange(phaseBValues, phaseBValues + 1);

    EXPECT_CALL(phaseA, Measure(1))
        .WillOnce(testing::Return(phaseARange));
    EXPECT_CALL(phaseB, Measure(1))
        .WillOnce(testing::Return(phaseBRange));

    EXPECT_CALL(theta, Position())
        .WillOnce(testing::Return(1000));
    EXPECT_CALL(theta, Resolution())
        .WillOnce(testing::Return(4000));

    auto result = controller.Read();

    EXPECT_FLOAT_EQ(result.first.a, 1638.0f * 3300.0f);
    EXPECT_FLOAT_EQ(result.first.b, 819.0f * 3300.0f);
    EXPECT_FLOAT_EQ(result.first.c, -(1638.0f * 3300.0f + 819.0f * 3300.0f));
    EXPECT_FLOAT_EQ(result.second, 0.0f);
}

TEST_F(FocControllerTest, WhenUpdatingOutputThenPwmIsStartedWithCorrectDutyCycles)
{
    application::FocOutput::Output output;
    output.a = 0.75f;
    output.b = 0.5f;
    output.c = 0.25f;

    EXPECT_CALL(pwm, Start(
                         hal::Percent(75),
                         hal::Percent(50),
                         hal::Percent(25)));

    controller.Update(output);
}

TEST_F(FocControllerTest, WhenDisablingThenPwmIsStopped)
{
    EXPECT_CALL(pwm, Stop());

    controller.Disable();
}

TEST_F(FocControllerTest, AutoTuneMethodIsEmpty)
{
    bool callbackInvoked = false;
    infra::Function<void()> callback = [&callbackInvoked]()
    {
        callbackInvoked = true;
    };

    controller.AutoTune(callback);

    EXPECT_FALSE(callbackInvoked);
}
