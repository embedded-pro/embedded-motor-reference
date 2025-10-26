#pragma once

#include "application/foc/interfaces/Driver.hpp"
#include <gmock/gmock.h>

namespace foc
{
    class EncoderMock
        : public Encoder
    {
    public:
        MOCK_METHOD(Radians, Read, (), (override));
        MOCK_METHOD(void, Set, (Radians value), (override));
        MOCK_METHOD(void, SetZero, (), (override));
    };

    class HallSensorMock
        : public HallSensor
    {
    public:
        MOCK_METHOD((std::pair<HallState, Direction>), Read, (), (const, override));
    };

    class FieldOrientedControllerInterfaceMock
        : public MotorDriver
    {
    public:
        MOCK_METHOD(void, PhaseCurrentsReady, (hal::Hertz baseFrequency, const infra::Function<void(std::tuple<Ampere, Ampere, Ampere> voltagePhases)>& onDone), (override));
        MOCK_METHOD(void, ThreePhasePwmOutput, ((const std::tuple<hal::Percent, hal::Percent, hal::Percent>&)), (override));
        MOCK_METHOD(void, Start, (), (override));
        MOCK_METHOD(void, Stop, (), (override));

        void StorePhaseCurrentsCallback(const infra::Function<void(std::tuple<Ampere, Ampere, Ampere> voltagePhases)>& onDone)
        {
            phaseCurrentsCallback = onDone;
        }

        void TriggerPhaseCurrentsCallback(std::tuple<Ampere, Ampere, Ampere> voltagePhases)
        {
            if (phaseCurrentsCallback)
                phaseCurrentsCallback(voltagePhases);
        }

    private:
        infra::Function<void(std::tuple<Ampere, Ampere, Ampere> voltagePhases)> phaseCurrentsCallback;
    };
}
