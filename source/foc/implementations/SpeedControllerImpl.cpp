#include "source/foc/implementations/SpeedControllerImpl.hpp"

namespace foc
{
    SpeedControllerImpl::SpeedControllerImpl(MotorDriver& interface, Encoder& position, FieldOrientedControllerSpeedControl& foc)
        : ControllerBaseImpl{ interface, position, foc }
        , foc{ foc }
    {}

    void SpeedControllerImpl::SetCurrentTunings(Volts Vcd, IdAndIqTunings tunings)
    {
        foc.SetCurrentTunings(Vcd, tunings);
    }

    void SpeedControllerImpl::SetSpeedTunings(Volts Vcd, const SpeedTunings& speedTuning)
    {
        foc.SetSpeedTunings(Vcd, speedTuning);
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

    hal::Hertz SpeedControllerImpl::BaseFrequency() const
    {
        return ControllerBaseImpl::BaseFrequency();
    }
}
