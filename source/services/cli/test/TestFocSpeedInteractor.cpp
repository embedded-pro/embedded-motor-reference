#include "source/foc/implementations/test_doubles/ControllerMock.hpp"
#include "source/services/cli/FocSpeedInteractorImpl.hpp"
#include <gmock/gmock.h>

namespace
{
    using namespace testing;

    MATCHER_P(SpeedTuningsEq, expected, "")
    {
        return arg.first.kp == expected.first.kp &&
               arg.first.ki == expected.first.ki &&
               arg.first.kd == expected.first.kd;
    }

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

    class FocSpeedInteractorTest
        : public testing::Test
    {
    public:
        testing::StrictMock<foc::SpeedControllerMock> focMock;
        services::FocSpeedInteractorImpl interactor{ foc::Volts{ 24.0f }, focMock };
    };
}

TEST_F(FocSpeedInteractorTest, StartEnablesMotorFoc)
{
    EXPECT_CALL(focMock, Enable()).Times(1);

    interactor.Start();
}

TEST_F(FocSpeedInteractorTest, StopDisablesMotorFoc)
{
    EXPECT_CALL(focMock, Disable()).Times(1);

    interactor.Stop();
}

TEST_F(FocSpeedInteractorTest, SetSpeedUpdatesSetPoint)
{
    const float speedValue = 0.75f;
    foc::IdAndIqPoint expectedPoint{ speedValue, 0.0 };
    EXPECT_CALL(focMock, SetPoint(IdAndIqPointEq(expectedPoint))).Times(1);

    interactor.SetSpeed(foc::RadiansPerSecond(speedValue));
}

TEST_F(FocSpeedInteractorTest, SetDQPidParametersWithAllValuesPresent)
{
    const float kpSpeed = 1.0f;
    const float kiSpeed = 2.0f;
    const float kdSpeed = 3.0f;

    const float kpD = 1.0f;
    const float kiD = 2.0f;
    const float kdD = 3.0f;
    const float kpQ = 4.0f;
    const float kiQ = 5.0f;
    const float kdQ = 6.0f;

    foc::SpeedTunings expectedSpeedTunings{ kpSpeed, kiSpeed, kdSpeed };

    foc::IdAndIqTunings expectedTunings{
        { kpD, kiD, kdD },
        { kpQ, kiQ, kdQ }
    };

    EXPECT_CALL(focMock, SetTunings(::testing::_, SpeedTuningsEq(expectedSpeedTunings), IdAndIqTuningsEq(expectedTunings))).Times(1);

    services::FocInteractor::PidParameters dParams;
    dParams.kp = kpD;
    dParams.ki = kiD;
    dParams.kd = kdD;

    services::FocInteractor::PidParameters qParams;
    qParams.kp = kpQ;
    qParams.ki = kiQ;
    qParams.kd = kdQ;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
}

TEST_F(FocSpeedInteractorTest, SetDQPidParametersWithPartialValues)
{
    const float kpD = 1.0f;
    const float kiQ = 5.0f;

    foc::IdAndIqTunings expectedTunings{
        { kpD, 0.0f, 0.0f },
        { 0.0f, kiQ, 0.0f }
    };

    EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, IdAndIqTuningsEq(expectedTunings))).Times(1);

    services::FocInteractor::PidParameters dParams;
    dParams.kp = kpD;

    services::FocInteractor::PidParameters qParams;
    qParams.ki = kiQ;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
}

TEST_F(FocSpeedInteractorTest, ExecutionOrder_StartThenSetSpeed)
{
    {
        InSequence seq;
        EXPECT_CALL(focMock, Enable()).Times(1);
        EXPECT_CALL(focMock, SetPoint(_)).Times(1);
    }

    interactor.Start();
    interactor.SetSpeed(foc::RadiansPerSecond(0.5f));
}

TEST_F(FocSpeedInteractorTest, ExecutionOrder_ConfigureThenStartThenStop)
{
    {
        InSequence seq;
        EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, ::testing::_)).Times(1);
        EXPECT_CALL(focMock, Enable()).Times(1);
        EXPECT_CALL(focMock, Disable()).Times(1);
    }

    services::FocInteractor::PidParameters dParams;
    dParams.kp = 1.0f;
    services::FocInteractor::PidParameters qParams;
    qParams.kp = 2.0f;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));
    interactor.Start();
    interactor.Stop();
}

TEST_F(FocSpeedInteractorTest, AutoTuneDoesNotCallMock)
{
    bool callbackCalled = false;
    interactor.AutoTune([&callbackCalled]()
        {
            callbackCalled = true;
        });

    EXPECT_FALSE(callbackCalled);
}
