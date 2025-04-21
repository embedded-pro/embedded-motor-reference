#pragma once

#include "application/foc/FieldOrientedControllerInterface.hpp"
#include <gmock/gmock.h>

namespace application
{
    class FieldOrientedControllerInterfaceMock
        : public FieldOrientedControllerInterface
    {
    public:
        MOCK_METHOD(void, PhaseCurrentsReady, (const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)>& onDone), (override));
        MOCK_METHOD(void, HallSensorInterrupt, (const infra::Function<void(HallState state, Direction direction)>& onDone), (override));
        MOCK_METHOD(void, ThreePhasePwmOutput, ((const std::tuple<Percent, Percent, Percent>&)), (override));
        MOCK_METHOD(void, Start, (), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };
}
