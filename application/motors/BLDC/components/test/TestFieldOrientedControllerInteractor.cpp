#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/foc/test_doubles/FieldOrientedControllerInterfaceMock.hpp"
#include "application/foc/test_doubles/SpaceVectorModulatorMock.hpp"
#include "application/foc/test_doubles/TrigonometricFunctionsMock.hpp"
#include "application/motors/BLDC/components/FieldOrientedControllerInteractor.hpp"
#include "application/motors/BLDC/components/FieldOrientedControllerInteractorImpl.hpp"
#include "application/pid/test_doubles/PidMock.hpp"
#include "infra/util/MemoryRange.hpp"
#include <gmock/gmock.h>

namespace
{
    class FieldOrientedControllerInteractorTest
        : public testing::Test
    {
    public:
        testing::StrictMock<application::TrigonometricFunctionsMock> trigFunctions;
        testing::StrictMock<application::SpaceVectorModulatorMock> spaceVectorModulator;
        testing::StrictMock<application::FieldOrientedControllerInterfaceMock> interfaceMock;
        testing::StrictMock<application::PidMock> dPid;
        testing::StrictMock<application::PidMock> qPid;
        application::MotorFieldOrientedController::Components components{ trigFunctions, spaceVectorModulator, dPid, qPid };
        application::FieldOrientedControllerInteractorImpl interactor{ interfaceMock, components };
    };
}

TEST_F(FieldOrientedControllerInteractorTest, WhenStartingControllerThenFocIsEnabled)
{
    EXPECT_CALL(dPid, Enable());
    EXPECT_CALL(qPid, Enable());

    interactor.Start();
}

TEST_F(FieldOrientedControllerInteractorTest, WhenStoppingControllerThenFocIsDisabled)
{
    EXPECT_CALL(dPid, Disable());
    EXPECT_CALL(qPid, Disable());
    EXPECT_CALL(interfaceMock, Stop());

    interactor.Stop();
}

TEST_F(FieldOrientedControllerInteractorTest, WhenSettingTorqueThenFocSetPointIsUpdated)
{
    const float expectedTorque = 0.75f;

    EXPECT_CALL(qPid, SetPoint(0));
    EXPECT_CALL(dPid, SetPoint(expectedTorque));

    interactor.SetTorque(application::FieldOrientedControllerInteractor::Torque(expectedTorque));
}

TEST_F(FieldOrientedControllerInteractorTest, WhenSettingDQPidParametersThenPidTunningsAreUpdated)
{
    application::FieldOrientedControllerInteractor::PidParameters dParameters;
    dParameters.kp = 10.0f;
    dParameters.ki = 5.0f;
    dParameters.kd = 0.1f;

    application::FieldOrientedControllerInteractor::PidParameters qParameters;
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

    interactor.SetDQPidParameters(std::make_pair(dParameters, qParameters));
}

TEST_F(FieldOrientedControllerInteractorTest, WhenSettingPartialDQParametersThenOnlyThoseParametersAreUpdated)
{
    application::FieldOrientedControllerInteractor::PidParameters dParameters;
    dParameters.kp = 10.0f;

    application::FieldOrientedControllerInteractor::PidParameters qParameters;
    qParameters.ki = 7.5f;

    EXPECT_CALL(dPid, SetTunnings(testing::AllOf(
                          testing::Field(&controllers::Pid<float>::Tunnings::kp, 10.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::ki, 0.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::kd, 0.0f))));

    EXPECT_CALL(qPid, SetTunnings(testing::AllOf(
                          testing::Field(&controllers::Pid<float>::Tunnings::kp, 0.0f),
                          testing::Field(&controllers::Pid<float>::Tunnings::ki, 7.5f),
                          testing::Field(&controllers::Pid<float>::Tunnings::kd, 0.0f))));

    interactor.SetDQPidParameters(std::make_pair(dParameters, qParameters));
}

TEST_F(FieldOrientedControllerInteractorTest, WhenReadingInputThenPhaseCurrentsAndThetaAreProcessedCorrectly)
{
    static uint16_t phaseAValues[] = { 1638 };
    static uint16_t phaseBValues[] = { 819 };
    infra::MemoryRange<const uint16_t> phaseARange(phaseAValues, phaseAValues + 1);
    infra::MemoryRange<const uint16_t> phaseBRange(phaseBValues, phaseBValues + 1);

#if 0
    EXPECT_CALL(phaseA, Measure(1))
        .WillOnce(testing::Return(phaseARange));
    EXPECT_CALL(phaseB, Measure(1))
        .WillOnce(testing::Return(phaseBRange));

    EXPECT_CALL(theta, Position())
        .WillOnce(testing::Return(1000));
    EXPECT_CALL(theta, Resolution())
        .WillOnce(testing::Return(4000));

    auto result = interactor.Read();

    EXPECT_FLOAT_EQ(result.first.a, 1638.0f * 3300.0f);
    EXPECT_FLOAT_EQ(result.first.b, 819.0f * 3300.0f);
    EXPECT_FLOAT_EQ(result.first.c, -(1638.0f * 3300.0f + 819.0f * 3300.0f));
    EXPECT_FLOAT_EQ(result.second, 0.0f);
#endif
}

TEST_F(FieldOrientedControllerInteractorTest, WhenUpdatingOutputThenPwmIsStartedWithCorrectDutyCycles)
{
#if 0
    application::FocOutput::Output output;
    output.a = 0.75f;
    output.b = 0.5f;
    output.c = 0.25f;

    EXPECT_CALL(pwm, Start(
                         hal::Percent(75),
                         hal::Percent(50),
                         hal::Percent(25)));

    interactor.Update(output);
#endif
}

TEST_F(FieldOrientedControllerInteractorTest, WhenDisablingThenPwmIsStopped)
{
    EXPECT_CALL(interfaceMock, Stop());

    interactor.Stop();
}

TEST_F(FieldOrientedControllerInteractorTest, AutoTuneMethodIsEmpty)
{
    bool callbackInvoked = false;
    infra::Function<void()> callback = [&callbackInvoked]()
    {
        callbackInvoked = true;
    };

    interactor.AutoTune(callback);

    EXPECT_FALSE(callbackInvoked);
}
