#pragma once

#include "application/foc/MotorFieldOrientedControllerInterface.hpp"
#include <gmock/gmock.h>

namespace application
{
    class EncoderMock
        : public Encoder
    {
    public:
        MOCK_METHOD(Degrees, Read, (), (override));
        MOCK_METHOD(void, Set, (Degrees value), (override));
        MOCK_METHOD(void, SetZero, (), (override));
    };

    class HallSensorMock
        : public HallSensor
    {
    public:
        MOCK_METHOD((std::pair<HallState, Direction>), Read, (), (const, override));
    };

    class FieldOrientedControllerInterfaceMock
        : public MotorFieldOrientedControllerInterface
    {
    public:
        MOCK_METHOD(void, PhaseCurrentsReady, (const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)>& onDone), (override));
        MOCK_METHOD(void, ThreePhasePwmOutput, ((const std::tuple<Percent, Percent, Percent>&)), (override));
        MOCK_METHOD(void, Start, (), (override));
        MOCK_METHOD(void, Stop, (), (override));

        void StorePhaseCurrentsCallback(const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)>& onDone)
        {
            phaseCurrentsCallback = onDone;
        }

        void TriggerPhaseCurrentsCallback(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)
        {
            if (phaseCurrentsCallback)
                phaseCurrentsCallback(voltagePhases);
        }

    private:
        infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)> phaseCurrentsCallback;
    };
}
