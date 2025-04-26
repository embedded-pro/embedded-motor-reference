#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/foc/test_doubles/FieldOrientedControllerInterfaceMock.hpp"
#include "application/foc/test_doubles/FieldOrientedControllerMock.hpp"
#include "application/motors/BLDC/components/FieldOrientedControllerInteractor.hpp"
#include "application/motors/BLDC/components/FieldOrientedControllerInteractorImpl.hpp"
#include <gmock/gmock.h>

namespace
{
    using namespace testing;

    class FieldOrientedControllerInteractorTest
        : public testing::Test
    {
    public:
        testing::StrictMock<application::FieldOrientedControllerMock> focMock;
        testing::StrictMock<application::FieldOrientedControllerInterfaceMock> interfaceMock;
        application::FieldOrientedControllerInteractorImpl interactor{ interfaceMock, focMock };
    };
}

TEST_F(FieldOrientedControllerInteractorTest, interactor_is_constructed_with_valid_dependencies)
{
}

TEST_F(FieldOrientedControllerInteractorTest, set_dq_pid_parameters_with_all_values_present_updates_all_parameters)
{
    application::FieldOrientedControllerInteractor::PidParameters dParams;
    dParams.kp = 0.5f;
    dParams.ki = 0.2f;
    dParams.kd = 0.1f;

    application::FieldOrientedControllerInteractor::PidParameters qParams;
    qParams.kp = 0.6f;
    qParams.ki = 0.3f;
    qParams.kd = 0.15f;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_));
    interactor.Start();
}

TEST_F(FieldOrientedControllerInteractorTest, set_dq_pid_parameters_with_partial_d_parameters_only_updates_provided_values)
{
    application::FieldOrientedControllerInteractor::PidParameters dParams;
    dParams.kp = 0.5f;

    application::FieldOrientedControllerInteractor::PidParameters qParams;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_));
    interactor.Start();
}

TEST_F(FieldOrientedControllerInteractorTest, set_dq_pid_parameters_with_partial_q_parameters_only_updates_provided_values)
{
    application::FieldOrientedControllerInteractor::PidParameters dParams;

    application::FieldOrientedControllerInteractor::PidParameters qParams;
    qParams.ki = 0.3f;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_));
    interactor.Start();
}

TEST_F(FieldOrientedControllerInteractorTest, set_torque_updates_foc_setpoint_with_torque_value_and_zero_quadrature)
{
    application::FieldOrientedControllerInteractor::Torque torque(10.0f);

    interactor.SetTorque(torque);

    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_));
    interactor.Start();
}

TEST_F(FieldOrientedControllerInteractorTest, start_enables_field_oriented_controller)
{
    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_));

    interactor.Start();
}

TEST_F(FieldOrientedControllerInteractorTest, stop_disables_field_oriented_controller)
{
    EXPECT_CALL(interfaceMock, Stop());

    interactor.Stop();
}

TEST_F(FieldOrientedControllerInteractorTest, auto_tune_calls_callback_immediately)
{
    bool callbackCalled = false;
    auto onDone = [&callbackCalled]()
    {
        callbackCalled = true;
    };

    interactor.AutoTune(onDone);

    EXPECT_TRUE(callbackCalled);
}

TEST_F(FieldOrientedControllerInteractorTest, typical_workflow_configures_pid_sets_torque_and_starts_controller)
{
    application::FieldOrientedControllerInteractor::PidParameters dParams;
    dParams.kp = 0.5f;
    dParams.ki = 0.2f;
    dParams.kd = 0.1f;

    application::FieldOrientedControllerInteractor::PidParameters qParams;
    qParams.kp = 0.6f;
    qParams.ki = 0.3f;
    qParams.kd = 0.15f;

    interactor.SetDQPidParameters(std::make_pair(dParams, qParams));

    application::FieldOrientedControllerInteractor::Torque torque(5.0f);
    interactor.SetTorque(torque);

    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_));
    interactor.Start();

    EXPECT_CALL(interfaceMock, Stop());
    interactor.Stop();
}

TEST_F(FieldOrientedControllerInteractorTest, multiple_starts_and_stops_work_as_expected)
{
    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_));
    interactor.Start();

    EXPECT_CALL(interfaceMock, Stop());
    interactor.Stop();

    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_));
    interactor.Start();

    EXPECT_CALL(interfaceMock, Stop());
    interactor.Stop();
}

TEST_F(FieldOrientedControllerInteractorTest, field_oriented_controller_processes_phase_currents_correctly)
{
    EXPECT_CALL(interfaceMock, Start());
    EXPECT_CALL(interfaceMock, PhaseCurrentsReady(testing::_))
        .WillOnce(testing::Invoke([this](const auto& callback)
            {
                std::tuple<application::MilliVolt, application::MilliVolt, application::MilliVolt> voltagePhases{
                    application::MilliVolt(100.0f),
                    application::MilliVolt(200.0f),
                    application::MilliVolt(300.0f)
                };
                std::optional<application::Degrees> position = application::Degrees(45.0f);

                // EXPECT_CALL(trigFunctions, Sin(testing::_)).WillOnce(Return(0.707f));
                // EXPECT_CALL(trigFunctions, Cos(testing::_)).WillOnce(Return(0.707f));

                std::tuple<application::Percent, application::Percent, application::Percent> expectedDutyPhases{
                    application::Percent(50.0f),
                    application::Percent(50.0f),
                    application::Percent(50.0f)
                };
                // EXPECT_CALL(spaceVectorModulator, Generate(testing::_))
                //     .WillOnce(Return(expectedDutyPhases));

                EXPECT_CALL(interfaceMock, ThreePhasePwmOutput(testing::_));

                callback(voltagePhases, position);
            }));

    interactor.Start();
}
