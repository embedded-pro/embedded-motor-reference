#pragma once

#include "application/foc/MotorFieldOrientedControllerInterface.hpp"
#include <gmock/gmock.h>

namespace application
{
    class FieldOrientedControllerInterfaceMock
        : public MotorFieldOrientedControllerInterface
    {
    public:
        MOCK_METHOD(void, PhaseCurrentsReady, (const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)>& onDone), (override));
        MOCK_METHOD(void, HallSensorInterrupt, (const infra::Function<void(HallState state, Direction direction)>& onDone), (override));
        MOCK_METHOD(void, ThreePhasePwmOutput, ((const std::tuple<Percent, Percent, Percent>&)), (override));
        MOCK_METHOD(void, Start, (), (override));
        MOCK_METHOD(void, Stop, (), (override));

        void StorePhaseCurrentsCallback(const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)>& onDone)
        {
            phaseCurrentsCallback = onDone;
        }

        void TriggerPhaseCurrentsCallback(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)
        {
            if (phaseCurrentsCallback)
                phaseCurrentsCallback(voltagePhases, position);
        }

    private:
        infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)> phaseCurrentsCallback;
    };
}
