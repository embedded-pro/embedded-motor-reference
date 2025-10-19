#pragma once

#include "application/foc/interfaces/FieldOrientedController.hpp"

namespace foc
{
    class ControllerBase
    {
    public:
        virtual void Enable() = 0;
        virtual void Disable() = 0;
        virtual bool IsRunning() const = 0;
    };

    class TorqueController
        : public ControllerBase
    {
    public:
        virtual void SetTunings(Volts Vcd, IdAndIqTunings tunings) = 0;
        virtual void SetPoint(const IdAndIqPoint& point) = 0;
    };

    class FocMotorSpeedController
        : public ControllerBase
    {
    public:
        virtual void SetTunings(Volts Vcd, const SpeedTunings& speedTuning, const IdAndIqTunings& torqueTunings) = 0;
        virtual void SetPoint(RadiansPerSecond point) = 0;
    };
}
