#include "application/pid/PidImpl.hpp"
#include "application/pid/test_doubles/PidInterfaceMock.hpp"
#include "gmock/gmock.h"
#include <optional>
#include <utility>

namespace
{
    MATCHER_P3(TuningsEq, kp, ki, kd, "")
    {
        return std::abs(arg.kp - kp) < 1e-6f &&
               std::abs(arg.ki - ki) < 1e-6f &&
               std::abs(arg.kd - kd) < 1e-6f;
    }

    class TestPid
        : public ::testing::Test
    {
    public:
        TestPid()
        {
            EXPECT_CALL(interfaceMock, Read(::testing::_))
                .WillOnce(::testing::Invoke(&interfaceMock, &application::PidInterfaceMock::StoreReadCallback));
            pid.emplace(interfaceMock);
        }

        ::testing::StrictMock<application::PidInterfaceMock> interfaceMock;
        std::optional<application::PidImpl> pid;
    };
}

TEST_F(TestPid, test_process_input_triggers_control_action)
{
    EXPECT_CALL(interfaceMock, ControlAction(::testing::_)).Times(1);
    interfaceMock.TriggerCallback(10);
}

TEST_F(TestPid, test_set_point_does_not_interact_with_interface)
{
    const float newSetPoint = 25.5f;
    pid->SetPoint(newSetPoint);
}

TEST_F(TestPid, test_set_tunings_does_not_interact_with_interface)
{
    application::Pid::Tunings newTunings{ 0.5f, 0.3f, 0.1f };
    pid->SetTunings(newTunings);
}

TEST_F(TestPid, test_enable_starts_interface)
{
    EXPECT_CALL(interfaceMock, Start(::testing::_)).Times(1);
    pid->Enable();
}

TEST_F(TestPid, test_disable_stops_interface)
{
    EXPECT_CALL(interfaceMock, Stop()).Times(1);
    pid->Disable();
}

TEST_F(TestPid, test_order_of_operations_in_enable_disable_sequence)
{
    ::testing::InSequence seq;
    EXPECT_CALL(interfaceMock, Start(::testing::_)).Times(1);
    EXPECT_CALL(interfaceMock, Stop()).Times(1);
    pid->Enable();
    pid->Disable();
}

TEST_F(TestPid, test_processing_input_produces_control_action_after_enable)
{
    ::testing::InSequence seq;
    EXPECT_CALL(interfaceMock, Start(::testing::_)).Times(1);
    EXPECT_CALL(interfaceMock, ControlAction(::testing::_)).Times(1);
    pid->Enable();
    interfaceMock.TriggerCallback(15);
}

TEST_F(TestPid, test_multiple_control_actions_from_multiple_inputs)
{
    EXPECT_CALL(interfaceMock, ControlAction(::testing::_)).Times(3);
    interfaceMock.TriggerCallback(10);
    interfaceMock.TriggerCallback(15);
    interfaceMock.TriggerCallback(20);
}

TEST_F(TestPid, test_enable_disable_enable_sequence)
{
    ::testing::InSequence seq;
    EXPECT_CALL(interfaceMock, Start(::testing::_)).Times(1);
    EXPECT_CALL(interfaceMock, Stop()).Times(1);
    EXPECT_CALL(interfaceMock, Start(::testing::_)).Times(1);
    pid->Enable();
    pid->Disable();
    pid->Enable();
}
