#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/foc/MotorFieldOrientedControllerInterface.hpp"
#include "application/foc/test_doubles/FieldOrientedControllerInterfaceMock.hpp"
#include "application/foc/test_doubles/FieldOrientedControllerMock.hpp"
#include "gmock/gmock.h"

namespace
{
    class TestMotorFieldOrientedController
        : public ::testing::Test
    {
    public:
        TestMotorFieldOrientedController()
        {
            EXPECT_CALL(interfaceMock, PhaseCurrentsReady(::testing::_))
                .WillOnce(::testing::Invoke(&interfaceMock, &application::FieldOrientedControllerInterfaceMock::StorePhaseCurrentsCallback));

            foc.emplace(interfaceMock, encoderMock, focMock);
        }

        ::testing::StrictMock<application::FieldOrientedControllerInterfaceMock> interfaceMock;
        ::testing::StrictMock<application::EncoderMock> encoderMock;
        ::testing::StrictMock<application::FieldOrientedControllerMock> focMock;
        std::optional<application::MotorFieldOrientedControllerImpl> foc;
    };
}

TEST_F(TestMotorFieldOrientedController, enable_starts_interface_and_enables_pid_controllers)
{
    EXPECT_CALL(interfaceMock, Start()).Times(1);
    foc->Enable();

    EXPECT_TRUE(foc->IsRunning());
}

TEST_F(TestMotorFieldOrientedController, disable_stops_interface_and_disables_pid_controllers)
{
    EXPECT_CALL(interfaceMock, Start()).Times(1);
    foc->Enable();

    EXPECT_CALL(interfaceMock, Stop()).Times(1);
    foc->Disable();

    EXPECT_FALSE(foc->IsRunning());
}

TEST_F(TestMotorFieldOrientedController, set_tunnings_updates_pid_controllers)
{
    controllers::Pid<float>::Tunnings dTunnings{ 1.0f, 0.5f, 0.1f };
    controllers::Pid<float>::Tunnings qTunnings{ 2.0f, 1.0f, 0.2f };

    foc->SetTunnings({ dTunnings, qTunnings });
}

TEST_F(TestMotorFieldOrientedController, set_point_updates_pid_controllers)
{
    float dSetpoint = 0.5f;
    float qSetpoint = 0.75f;

    foc->SetPoint({ dSetpoint, qSetpoint });
}

TEST_F(TestMotorFieldOrientedController, phase_currents_callback_triggers_foc_calculation_and_output)
{
    std::tuple<application::MilliVolt, application::MilliVolt, application::MilliVolt> voltagePhases{ 100, 200, 300 };
    std::tuple<hal::Percent, hal::Percent, hal::Percent> pwmOutput{ 0.25, 0.5, 0.75 };

    EXPECT_CALL(encoderMock, Read())
        .WillOnce(::testing::Return(application::Degrees{ 0.0f }));
    EXPECT_CALL(focMock, Calculate(::testing::_, ::testing::_, voltagePhases, ::testing::_))
        .WillOnce(::testing::Return(pwmOutput));
    EXPECT_CALL(interfaceMock, ThreePhasePwmOutput(pwmOutput)).Times(1);

    interfaceMock.TriggerPhaseCurrentsCallback(voltagePhases);
}

TEST_F(TestMotorFieldOrientedController, disabled_pid_controllers_are_reenabled_after_enable)
{
    EXPECT_CALL(interfaceMock, Stop()).Times(1);
    foc->Disable();
    EXPECT_FALSE(foc->IsRunning());

    EXPECT_CALL(interfaceMock, Start()).Times(1);
    foc->Enable();
    EXPECT_TRUE(foc->IsRunning());

    std::tuple<application::MilliVolt, application::MilliVolt, application::MilliVolt> voltagePhases{ 100, 200, 300 };
    std::tuple<hal::Percent, hal::Percent, hal::Percent> pwmOutput{ 0.3, 0.6, 0.9 };

    EXPECT_CALL(encoderMock, Read())
        .WillOnce(::testing::Return(application::Degrees{ 0.0f }));
    EXPECT_CALL(focMock, Calculate(::testing::_, ::testing::_, voltagePhases, ::testing::_))
        .WillOnce(::testing::Return(pwmOutput));
    EXPECT_CALL(interfaceMock, ThreePhasePwmOutput(pwmOutput)).Times(1);

    interfaceMock.TriggerPhaseCurrentsCallback(voltagePhases);
}

TEST_F(TestMotorFieldOrientedController, phase_currents_with_modified_pid_values)
{
    controllers::Pid<float>::Tunnings dTunnings{ 1.0f, 0.5f, 0.1f };
    controllers::Pid<float>::Tunnings qTunnings{ 2.0f, 1.0f, 0.2f };
    foc->SetTunnings({ dTunnings, qTunnings });

    float dSetpoint = 0.5f;
    float qSetpoint = 0.75f;
    foc->SetPoint({ dSetpoint, qSetpoint });

    std::tuple<application::MilliVolt, application::MilliVolt, application::MilliVolt> voltagePhases{ 150, 250, 350 };
    std::tuple<hal::Percent, hal::Percent, hal::Percent> pwmOutput{ 0.4, 0.6, 0.8 };

    EXPECT_CALL(encoderMock, Read())
        .WillOnce(::testing::Return(application::Degrees{ 0.0f }));
    EXPECT_CALL(focMock, Calculate(::testing::_, ::testing::_, voltagePhases, ::testing::_))
        .WillOnce(::testing::Return(pwmOutput));
    EXPECT_CALL(interfaceMock, ThreePhasePwmOutput(pwmOutput)).Times(1);

    interfaceMock.TriggerPhaseCurrentsCallback(voltagePhases);
}
