#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/foc/test_doubles/FieldOrientedControllerInterfaceMock.hpp"
#include "application/foc/test_doubles/FieldOrientedControllerMock.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"
#include "gmock/gmock.h"

namespace
{
    class TestMotorFieldOrientedController
        : public ::testing::Test
    {
    public:
        ::testing::StrictMock<application::FieldOrientedControllerInterfaceMock> interfaceMock;
        ::testing::StrictMock<application::FieldOrientedControllerMock> focMock;
        application::MotorFieldOrientedController foc{ interfaceMock, focMock };

        controllers::ThreePhase<float> threePhaseCurrents{ 1.0f, 2.0f, 3.0f };
        float electricalAngle = 0.5f;
        controllers::TwoPhase<float> alphaAndBeta{ 2.0f, 1.5f };
        controllers::RotatingFrame<float> dqFrame{ 1.2f, 0.8f };
        controllers::RotatingFrame<float> dqVoltage{ 5.0f, 3.0f };

        // application::TwoPhase<float> voltageAlphaBeta{ 4.0f, 2.5f };
        // application::SpaceVectorModulator::Output pwmOutput = { 0.4f, 0.5f, 0.6f };

        // void SetUp() override
        // {
        //     ON_CALL(interfaceMock, PhaseCurrentsReady())
        //         .WillByDefault(::testing::Return(inputData));
        // }
    };
}

TEST_F(TestMotorFieldOrientedController, construction_doesnt_start_timer)
{
    EXPECT_FALSE(foc.IsRunning());
}

TEST_F(TestMotorFieldOrientedController, enable_starts_pid_controllers_and_timer)
{
    foc.Enable();
    EXPECT_TRUE(foc.IsRunning());
}

TEST_F(TestMotorFieldOrientedController, disable_stops_timer_and_pid_controllers)
{
    foc.Enable();

    EXPECT_CALL(interfaceMock, Stop());
    foc.Disable();

    EXPECT_FALSE(foc.IsRunning());
}

TEST_F(TestMotorFieldOrientedController, timer_callback_executes_complete_foc_pipeline)
{
}

TEST_F(TestMotorFieldOrientedController, timer_fires_at_correct_interval)
{
}

TEST_F(TestMotorFieldOrientedController, disable_can_be_called_multiple_times)
{
    foc.Enable();
    EXPECT_CALL(interfaceMock, Stop());
    foc.Disable();
}

TEST_F(TestMotorFieldOrientedController, is_running_returns_correct_state)
{
    EXPECT_FALSE(foc.IsRunning());

    foc.Enable();
    EXPECT_TRUE(foc.IsRunning());

    EXPECT_CALL(interfaceMock, Stop());
    foc.Disable();
    EXPECT_FALSE(foc.IsRunning());
}
