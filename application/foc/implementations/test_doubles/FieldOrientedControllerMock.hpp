#pragma once

#include "application/foc/interfaces/FieldOrientedController.hpp"
#include "gmock/gmock.h"
#include <gmock/gmock.h>

namespace foc
{
    class FieldOrientedControllerMock
        : public FieldOrientedControllerTorqueControl
    {
    public:
        MOCK_METHOD(void, SetPolePairs, (std::size_t polePairs), (override));
        MOCK_METHOD(void, Reset, (), (override));
        MOCK_METHOD(void, SetPoint, (IdAndIqPoint), (override));
        MOCK_METHOD(void, SetTunings, (Volts Vdc, const IdAndIqTunings& tunings), (override));
        MOCK_METHOD(PhasePwmDutyCycles, Calculate, (const PhaseCurrents& currentPhases, Radians& position), (override));
    };

    class FieldOrientedControllerSpeedControlMock
        : public FieldOrientedControllerSpeedControl
    {
    public:
        MOCK_METHOD(void, SetPolePairs, (std::size_t polePairs), (override));
        MOCK_METHOD(void, Reset, (), (override));
        MOCK_METHOD(void, SetPoint, (RadiansPerSecond), (override));
        MOCK_METHOD(void, SetTunings, (Volts Vdc, const SpeedTunings& speedTuning, const IdAndIqTunings& torqueTunings), (override));
        MOCK_METHOD(PhasePwmDutyCycles, Calculate, (const PhaseCurrents& currentPhases, Radians& position), (override));
    };
}
