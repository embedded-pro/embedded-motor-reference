#include "source/foc/implementations/SpeedControllerImpl.hpp"

namespace foc
{
    SpeedControllerImpl::SpeedControllerImpl(MotorDriver& interface, Encoder& position, FieldOrientedControllerSpeedControl& foc)
        : ControllerBaseImpl{ interface, position, foc }
        , foc{ foc }
    {}

    void SpeedControllerImpl::SetTunings(Volts Vcd, const SpeedTunings& speedTuning, const IdAndIqTunings& torqueTunings)
    {
        foc.SetTunings(Vcd, speedTuning, torqueTunings);
    }

    void SpeedControllerImpl::SetPoint(RadiansPerSecond point)
    {
        foc.SetPoint(point);
    }

    void SpeedControllerImpl::Enable()
    {
        ControllerBaseImpl::Enable();
    }

    void SpeedControllerImpl::Disable()
    {
        ControllerBaseImpl::Disable();
    }

    bool SpeedControllerImpl::IsRunning() const
    {
        return ControllerBaseImpl::IsRunning();
    }
}
