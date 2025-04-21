#include "application/foc/FieldOrientedControllerInterface.hpp"
#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/foc/test_doubles/FieldOrientedControllerInterfaceMock.hpp"
#include "application/foc/test_doubles/SpaceVectorModulatorMock.hpp"
#include "application/foc/test_doubles/TrigonometricFunctionsMock.hpp"
#include "application/pid/test_doubles/PidMock.hpp"
#include "gmock/gmock.h"

namespace
{
    class TestMotorFieldOrientedController
        : public ::testing::Test
    {
    public:
        ::testing::StrictMock<application::FieldOrientedControllerInterfaceMock> interfaceMock;
        ::testing::StrictMock<application::TrigonometricFunctionsMock> trigonometricMock;
        ::testing::StrictMock<application::SpaceVectorModulatorMock> spaceVectorModulatorMock;
        ::testing::StrictMock<application::PidMock> dPidMock;
        ::testing::StrictMock<application::PidMock> qPidMock;
        application::MotorFieldOrientedController::Components components{ trigonometricMock, spaceVectorModulatorMock, dPidMock, qPidMock };
        application::MotorFieldOrientedController foc{ interfaceMock, components };

        controllers::ThreePhase<float> threePhaseCurrents{ 1.0f, 2.0f, 3.0f };
        float electricalAngle = 0.5f;
        controllers::TwoPhase<float> alphaAndBeta{ 2.0f, 1.5f };
        controllers::RotatingFrame<float> dqFrame{ 1.2f, 0.8f };
        controllers::RotatingFrame<float> dqVoltage{ 5.0f, 3.0f };

        // application::TwoPhase<float> voltageAlphaBeta{ 4.0f, 2.5f };
        // application::SpaceVectorModulator::Output pwmOutput = { 0.4f, 0.5f, 0.6f };

        // void SetUp() override
        // {
        //     ON_CALL(inputMock, Read())
        //         .WillByDefault(::testing::Return(inputData));
        // }
    };
}

TEST_F(TestMotorFieldOrientedController, construction_doesnt_start_timer)
{
    EXPECT_FALSE(foc.IsRunning());
}

TEST_F(TestMotorFieldOrientedController, set_tunnings_delegates_to_pid_controllers)
{
    controllers::Pid<float>::Tunnings dTunnings{ 1.0f, 2.0f, 3.0f };
    controllers::Pid<float>::Tunnings qTunnings{ 4.0f, 5.0f, 6.0f };

    EXPECT_CALL(dPidMock, SetTunnings(application::TunningsEq(dTunnings)));
    EXPECT_CALL(qPidMock, SetTunnings(application::TunningsEq(qTunnings)));

    foc.SetTunnings({ dTunnings, qTunnings });
}

TEST_F(TestMotorFieldOrientedController, set_point_delegates_to_pid_controllers)
{
    EXPECT_CALL(dPidMock, SetPoint(2.5f));
    EXPECT_CALL(qPidMock, SetPoint(3.5f));

    foc.SetPoint({ 2.5f, 3.5f });
}

TEST_F(TestMotorFieldOrientedController, enable_starts_pid_controllers_and_timer)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());

    foc.Enable();

    EXPECT_TRUE(foc.IsRunning());
}

TEST_F(TestMotorFieldOrientedController, disable_stops_timer_and_pid_controllers)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();

    EXPECT_CALL(interfaceMock, Stop());
    EXPECT_CALL(dPidMock, Disable());
    EXPECT_CALL(qPidMock, Disable());

    foc.Disable();

    EXPECT_FALSE(foc.IsRunning());
}

TEST_F(TestMotorFieldOrientedController, timer_callback_executes_complete_foc_pipeline)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();

    {
        ::testing::InSequence seq;

        // EXPECT_CALL(inputMock, Read())
        //     .WillOnce(::testing::Return(inputData));

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

        // EXPECT_CALL(spaceVectorModulatorMock, Generate(::testing::_))
        //     .WillOnce(::testing::Return(pwmOutput));

        // EXPECT_CALL(outputMock, Update(::testing::_));
    }
}

TEST_F(TestMotorFieldOrientedController, timer_fires_at_correct_interval)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();

    // EXPECT_CALL(inputMock, Read());
    EXPECT_CALL(trigonometricMock, Cosine(::testing::_)).Times(2);
    EXPECT_CALL(trigonometricMock, Sine(::testing::_)).Times(2);
    EXPECT_CALL(dPidMock, Process(::testing::_));
    EXPECT_CALL(qPidMock, Process(::testing::_));
    EXPECT_CALL(spaceVectorModulatorMock, Generate(::testing::_));
    // EXPECT_CALL(outputMock, Update(::testing::_));

    // ::testing::Mock::VerifyAndClearExpectations(&inputMock);
    ::testing::Mock::VerifyAndClearExpectations(&trigonometricMock);
    ::testing::Mock::VerifyAndClearExpectations(&dPidMock);
    ::testing::Mock::VerifyAndClearExpectations(&qPidMock);
    ::testing::Mock::VerifyAndClearExpectations(&spaceVectorModulatorMock);
    // ::testing::Mock::VerifyAndClearExpectations(&outputMock);

    // EXPECT_CALL(inputMock, Read());
    EXPECT_CALL(trigonometricMock, Cosine(::testing::_)).Times(2);
    EXPECT_CALL(trigonometricMock, Sine(::testing::_)).Times(2);
    EXPECT_CALL(dPidMock, Process(::testing::_));
    EXPECT_CALL(qPidMock, Process(::testing::_));
    EXPECT_CALL(spaceVectorModulatorMock, Generate(::testing::_));
    // EXPECT_CALL(outputMock, Update(::testing::_));
}

TEST_F(TestMotorFieldOrientedController, disable_can_be_called_multiple_times)
{
    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();

    EXPECT_CALL(interfaceMock, Stop());
    EXPECT_CALL(dPidMock, Disable());
    EXPECT_CALL(qPidMock, Disable());

    foc.Disable();
}

TEST_F(TestMotorFieldOrientedController, is_running_returns_correct_state)
{
    EXPECT_FALSE(foc.IsRunning());

    EXPECT_CALL(dPidMock, Enable());
    EXPECT_CALL(qPidMock, Enable());
    foc.Enable();
    EXPECT_TRUE(foc.IsRunning());

    EXPECT_CALL(interfaceMock, Stop());
    EXPECT_CALL(dPidMock, Disable());
    EXPECT_CALL(qPidMock, Disable());
    foc.Disable();
    EXPECT_FALSE(foc.IsRunning());
}
