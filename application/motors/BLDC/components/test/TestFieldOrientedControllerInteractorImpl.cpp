#include "application/foc/test_doubles/MotorFieldOrientedControllerMock.hpp"
#include "application/motors/BLDC/components/FieldOrientedControllerInteractorImpl.hpp"
#include <gmock/gmock.h>

namespace
{
    using namespace testing;

    MATCHER_P(IdAndIqTunningsEq, expected, "")
    {
        return arg.first.kp == expected.first.kp &&
               arg.first.ki == expected.first.ki &&
               arg.first.kd == expected.first.kd &&
               arg.second.kp == expected.second.kp &&
               arg.second.ki == expected.second.ki &&
               arg.second.kd == expected.second.kd;
    }

    MATCHER_P(IdAndIqPointEq, expected, "")
    {
        return arg.first == expected.first && arg.second == expected.second;
    }

    class FieldOrientedControllerInteractorTest
        : public testing::Test
    {
    public:
        testing::StrictMock<application::MotorFieldOrientedControllerMock> motorFocMock;
        application::FieldOrientedControllerInteractorImpl interactor{ motorFocMock };
    };
}

TEST_F(FieldOrientedControllerInteractorTest, StartEnablesMotorFoc)
{
    EXPECT_CALL(motorFocMock, Enable()).Times(1);

    interactor.Start();
}

TEST_F(FieldOrientedControllerInteractorTest, StopDisablesMotorFoc)
{
    EXPECT_CALL(motorFocMock, Disable()).Times(1);

    interactor.Stop();
}

TEST_F(FieldOrientedControllerInteractorTest, SetTorqueUpdatesSetPoint)
{
    const float torqueValue = 0.75f;
    application::MotorFieldOrientedController::IdAndIqPoint expectedPoint{ torqueValue, 0.0 };
    EXPECT_CALL(motorFocMock, SetPoint(IdAndIqPointEq(expectedPoint))).Times(1);

    interactor.SetTorque(application::FieldOrientedControllerInteractor::Torque(torqueValue));
}

TEST_F(FieldOrientedControllerInteractorTest, SetDQPidParametersWithAllValuesPresent)
{
    const float kpD = 1.0f;
    const float kiD = 2.0f;
    const float kdD = 3.0f;
    const float kpQ = 4.0f;
    const float kiQ = 5.0f;
    const float kdQ = 6.0f;

    application::MotorFieldOrientedController::IdAndIqTunnings expectedTunnings{
        { kpD, kiD, kdD },
        { kpQ, kiQ, kdQ }
    };

    EXPECT_CALL(motorFocMock, SetTunnings(IdAndIqTunningsEq(expectedTunnings))).Times(1);

    application::FieldOrientedControllerInteractor::PidParameters dParams;
    dParams.kp = kpD;
    dParams.ki = kiD;
    dParams.kd = kdD;

    application::FieldOrientedControllerInteractor::PidParameters qParams;
    qParams.kp = kpQ;
    qParams.ki = kiQ;
    qParams.kd = kdQ;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
}

TEST_F(FieldOrientedControllerInteractorTest, SetDQPidParametersWithPartialValues)
{
    const float kpD = 1.0f;
    const float kiQ = 5.0f;

    application::MotorFieldOrientedController::IdAndIqTunnings expectedTunnings{
        { kpD, 0.0f, 0.0f },
        { 0.0f, kiQ, 0.0f }
    };

    EXPECT_CALL(motorFocMock, SetTunnings(IdAndIqTunningsEq(expectedTunnings))).Times(1);

    application::FieldOrientedControllerInteractor::PidParameters dParams;
    dParams.kp = kpD;

    application::FieldOrientedControllerInteractor::PidParameters qParams;
    qParams.ki = kiQ;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
}

TEST_F(FieldOrientedControllerInteractorTest, ExecutionOrder_StartThenSetTorque)
{
    {
        InSequence seq;
        EXPECT_CALL(motorFocMock, Enable()).Times(1);
        EXPECT_CALL(motorFocMock, SetPoint(_)).Times(1);
    }

    interactor.Start();
    interactor.SetTorque(application::FieldOrientedControllerInteractor::Torque(0.5f));
}

TEST_F(FieldOrientedControllerInteractorTest, ExecutionOrder_ConfigureThenStartThenStop)
{
    {
        InSequence seq;
        EXPECT_CALL(motorFocMock, SetTunnings(_)).Times(1);
        EXPECT_CALL(motorFocMock, Enable()).Times(1);
        EXPECT_CALL(motorFocMock, Disable()).Times(1);
    }

    application::FieldOrientedControllerInteractor::PidParameters dParams;
    dParams.kp = 1.0f;
    application::FieldOrientedControllerInteractor::PidParameters qParams;
    qParams.kp = 2.0f;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
    interactor.Start();
    interactor.Stop();
}

TEST_F(FieldOrientedControllerInteractorTest, AutoTuneDoesNotCallMock)
{
    bool callbackCalled = false;
    interactor.AutoTune([&callbackCalled]()
        {
            callbackCalled = true;
        });

    EXPECT_FALSE(callbackCalled);
}
