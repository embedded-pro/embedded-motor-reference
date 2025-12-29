#pragma once

#include "source/foc/interfaces/FieldOrientedController.hpp"

namespace foc
{
    class ControllerBase
    {
    public:
        virtual void Enable() = 0;
        virtual void Disable() = 0;
        virtual bool IsRunning() const = 0;
        virtual void SetCurrentTunings(Volts Vcd, IdAndIqTunings tunings) = 0;
    };

    class TorqueController
        : public ControllerBase
    {
    public:
        virtual void SetPoint(const IdAndIqPoint& point) = 0;
    };

    class SpeedController
        : public ControllerBase
    {
    public:
        virtual void SetSpeedTunings(Volts Vcd, const SpeedTunings& speedTuning) = 0;
        virtual void SetPoint(RadiansPerSecond point) = 0;
    };

    class PositionController
        : public ControllerBase
    {
    public:
        virtual void SetSpeedTunings(Volts Vcd, const SpeedTunings& speedTuning) = 0;
        virtual void SetPositionTunings(Volts Vcd, const PositionTunings& positionTuning) = 0;
        virtual void SetPoint(Radians point) = 0;
    };
}
