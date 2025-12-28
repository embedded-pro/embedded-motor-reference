#include "source/foc/implementations/test_doubles/ControllerMock.hpp"
#include "source/services/cli/FocSpeedInteractorImpl.hpp"
#include <cmath>
#include <gmock/gmock.h>

namespace
{
    using namespace testing;

    MATCHER_P(SpeedTuningsEq, expected, "")
    {
        return arg.kp == expected.kp &&
               arg.ki == expected.ki &&
               arg.kd == expected.kd;
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

    MATCHER_P(SpeedEq, expected, "")
    {
        return std::abs(arg.Value() - expected.Value()) < 1e-5f;
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

TEST_F(FocSpeedInteractorTest, SetSpeedInRadiansPerSecondUpdatesSetPoint)
{
    const float speedValue = 0.75f;
    foc::RadiansPerSecond expectedSpeed{ speedValue };
    EXPECT_CALL(focMock, SetPoint(SpeedEq(expectedSpeed))).Times(1);

    interactor.SetSpeed(foc::RadiansPerSecond(speedValue));
}

TEST_F(FocSpeedInteractorTest, SetSpeedInRevPerMinuteUpdatesSetPoint)
{
    const float rpmValue = 100.0f;
    const float expectedRadPerSec = rpmValue * (M_PI / 30.0f);
    foc::RadiansPerSecond expectedSpeed{ expectedRadPerSec };
    EXPECT_CALL(focMock, SetPoint(SpeedEq(expectedSpeed))).Times(1);

    interactor.SetSpeed(foc::RevPerMinute(rpmValue));
}

TEST_F(FocSpeedInteractorTest, SetDQPidParametersWithAllValuesPresent)
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

    EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, IdAndIqTuningsEq(expectedTunings))).Times(1);

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

TEST_F(FocSpeedInteractorTest, SetDQPidParametersPreservesExistingValues)
{
    {
        services::FocInteractor::PidParameters dParams1;
        dParams1.kp = 1.0f;
        services::FocInteractor::PidParameters qParams1;
        qParams1.ki = 2.0f;
        EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, ::testing::_)).Times(1);
        interactor.SetDQPidParameters(std::make_pair(dParams1, qParams1));
    }

    {
        foc::IdAndIqTunings expectedTunings{
            { 1.0f, 3.0f, 0.0f },
            { 4.0f, 2.0f, 0.0f }
        };
        EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, IdAndIqTuningsEq(expectedTunings))).Times(1);

        services::FocInteractor::PidParameters dParams2;
        dParams2.ki = 3.0f;
        services::FocInteractor::PidParameters qParams2;
        qParams2.kp = 4.0f;
        interactor.SetDQPidParameters(std::make_pair(dParams2, qParams2));
    }
}

TEST_F(FocSpeedInteractorTest, SetDQPidParametersAllSixParametersIndividually)
{
    services::FocInteractor::PidParameters dParams;
    dParams.kp = 1.0f;
    services::FocInteractor::PidParameters qParams;
    EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, ::testing::_));
    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    dParams = {};
    dParams.ki = 2.0f;
    EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, ::testing::_));
    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    dParams = {};
    dParams.kd = 3.0f;
    EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, ::testing::_));
    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    qParams.kp = 4.0f;
    dParams = {};
    EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, ::testing::_));
    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    qParams = {};
    qParams.ki = 5.0f;
    EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, ::testing::_));
    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    qParams = {};
    qParams.kd = 6.0f;
    foc::IdAndIqTunings expectedFinal{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f }
    };
    EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, IdAndIqTuningsEq(expectedFinal)));
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

TEST_F(FocSpeedInteractorTest, SetSpeedPidParametersWithAllValues)
{
    const float kp = 1.5f;
    const float ki = 0.5f;
    const float kd = 0.1f;

    foc::SpeedTunings expectedTunings{ kp, ki, kd };

    EXPECT_CALL(focMock, SetTunings(::testing::_, SpeedTuningsEq(expectedTunings), ::testing::_)).Times(1);

    services::FocInteractor::PidParameters params;
    params.kp = kp;
    params.ki = ki;
    params.kd = kd;

    interactor.SetSpeedPidParameters(params);
}

TEST_F(FocSpeedInteractorTest, SetSpeedPidParametersWithPartialValues)
{
    const float kp = 2.0f;

    foc::SpeedTunings expectedTunings{ kp, 0.0f, 0.0f };

    EXPECT_CALL(focMock, SetTunings(::testing::_, SpeedTuningsEq(expectedTunings), ::testing::_)).Times(1);

    services::FocInteractor::PidParameters params;
    params.kp = kp;

    interactor.SetSpeedPidParameters(params);
}

TEST_F(FocSpeedInteractorTest, SetSpeedPidParametersPreservesExistingValues)
{
    {
        services::FocInteractor::PidParameters params1;
        params1.kp = 1.0f;
        EXPECT_CALL(focMock, SetTunings(::testing::_, ::testing::_, ::testing::_)).Times(1);
        interactor.SetSpeedPidParameters(params1);
    }

    {
        foc::SpeedTunings expectedTunings{ 1.0f, 0.5f, 0.0f };
        EXPECT_CALL(focMock, SetTunings(::testing::_, SpeedTuningsEq(expectedTunings), ::testing::_)).Times(1);

        services::FocInteractor::PidParameters params2;
        params2.ki = 0.5f;
        interactor.SetSpeedPidParameters(params2);
    }
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
