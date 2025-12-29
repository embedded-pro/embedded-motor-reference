#include "source/foc/implementations/test_doubles/ControllerMock.hpp"
#include "source/services/cli/FocTorqueInteractorImpl.hpp"
#include <gmock/gmock.h>

namespace
{
    using namespace testing;

    MATCHER_P(IdAndIqTuningsEq, expected, "")
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

    class FocTorqueInteractorTest
        : public testing::Test
    {
    public:
        testing::StrictMock<foc::TorqueControllerMock> focMock;
        services::FocTorqueInteractorImpl interactor{ foc::Volts{ 24.0f }, focMock };
    };
}

TEST_F(FocTorqueInteractorTest, StartEnablesMotorFoc)
{
    EXPECT_CALL(focMock, Enable()).Times(1);

    interactor.Start();
}

TEST_F(FocTorqueInteractorTest, StopDisablesMotorFoc)
{
    EXPECT_CALL(focMock, Disable()).Times(1);

    interactor.Stop();
}

TEST_F(FocTorqueInteractorTest, SetTorqueUpdatesSetPoint)
{
    const float torqueValue = 0.75f;
    foc::IdAndIqPoint expectedPoint{ torqueValue, 0.0 };
    EXPECT_CALL(focMock, SetPoint(IdAndIqPointEq(expectedPoint))).Times(1);

    interactor.SetTorque(foc::Nm(torqueValue));
}

TEST_F(FocTorqueInteractorTest, SetDQPidParametersWithAllValuesPresent)
{
    const float kpD = 1.0f;
    const float kiD = 2.0f;
    const float kdD = 3.0f;
    const float kpQ = 4.0f;
    const float kiQ = 5.0f;
    const float kdQ = 6.0f;

    foc::IdAndIqTunings expectedTunings{
        { kpD, kiD, kdD },
        { kpQ, kiQ, kdQ }
    };

    EXPECT_CALL(focMock, SetCurrentTunings(::testing::_, IdAndIqTuningsEq(expectedTunings))).Times(1);

    services::PidParameters dParams;
    dParams.kp = kpD;
    dParams.ki = kiD;
    dParams.kd = kdD;

    services::PidParameters qParams;
    qParams.kp = kpQ;
    qParams.ki = kiQ;
    qParams.kd = kdQ;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
}

TEST_F(FocTorqueInteractorTest, SetDQPidParametersWithPartialValues)
{
    const float kpD = 1.0f;
    const float kiQ = 5.0f;

    foc::IdAndIqTunings expectedTunings{
        { kpD, 0.0f, 0.0f },
        { 0.0f, kiQ, 0.0f }
    };

    EXPECT_CALL(focMock, SetCurrentTunings(::testing::_, IdAndIqTuningsEq(expectedTunings))).Times(1);

    services::PidParameters dParams;
    dParams.kp = kpD;

    services::PidParameters qParams;
    qParams.ki = kiQ;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
}

TEST_F(FocTorqueInteractorTest, ExecutionOrder_StartThenSetTorque)
{
    {
        InSequence seq;
        EXPECT_CALL(focMock, Enable()).Times(1);
        EXPECT_CALL(focMock, SetPoint(_)).Times(1);
    }

    interactor.Start();
    interactor.SetTorque(foc::Nm(0.5f));
}

TEST_F(FocTorqueInteractorTest, ExecutionOrder_ConfigureThenStartThenStop)
{
    {
        InSequence seq;
        EXPECT_CALL(focMock, SetCurrentTunings(::testing::_, ::testing::_)).Times(1);
        EXPECT_CALL(focMock, Enable()).Times(1);
        EXPECT_CALL(focMock, Disable()).Times(1);
    }

    services::PidParameters dParams;
    dParams.kp = 1.0f;
    services::PidParameters qParams;
    qParams.kp = 2.0f;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
    interactor.Start();
    interactor.Stop();
}

TEST_F(FocTorqueInteractorTest, AutoTuneDoesNotCallMock)
{
    bool callbackCalled = false;
    interactor.AutoTune([&callbackCalled]()
        {
            callbackCalled = true;
        });

    EXPECT_TRUE(callbackCalled);
}
