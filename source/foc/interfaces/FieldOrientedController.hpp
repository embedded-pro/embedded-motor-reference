#pragma once

#include "numerical/controllers/interfaces/PidController.hpp"
#include "source/foc/interfaces/Driver.hpp"

namespace foc
{
    using IdAndIqPoint = std::pair<Ampere, Ampere>;
    using IdAndIqTunings = std::pair<controllers::PidTunings<float>, controllers::PidTunings<float>>;
    using SpeedTunings = controllers::PidTunings<float>;
    using PositionTunings = controllers::PidTunings<float>;

    class FieldOrientedControllerBase
    {
    public:
        virtual void SetPolePairs(std::size_t polePairs) = 0;
        virtual void Reset() = 0;
        virtual PhasePwmDutyCycles Calculate(const PhaseCurrents& currentPhases, Radians& position) = 0;
    };

    class FieldOrientedControllerTorqueControl
        : public FieldOrientedControllerBase
    {
    public:
        virtual void SetPoint(IdAndIqPoint setPoint) = 0;
        virtual void SetTunings(Volts Vdc, const IdAndIqTunings& tunings) = 0;
    };

    class FieldOrientedControllerSpeedControl
        : public FieldOrientedControllerBase
    {
    public:
        virtual void SetPoint(RadiansPerSecond setPoint) = 0;
        virtual void SetTunings(Volts Vdc, const SpeedTunings& speedTuning, const IdAndIqTunings& torqueTunings) = 0;
    };
}
