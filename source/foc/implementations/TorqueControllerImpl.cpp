#include "source/foc/implementations/TorqueControllerImpl.hpp"

namespace foc
{
    TorqueControllerImpl::TorqueControllerImpl(MotorDriver& interface, Encoder& position, FieldOrientedControllerTorqueControl& foc)
        : ControllerBaseImpl{ interface, position, foc }
        , foc{ foc }
    {
    }

    void TorqueControllerImpl::SetTunings(Volts Vcd, IdAndIqTunings tunings)
    {
        foc.SetTunings(Vcd, tunings);
    }

    void TorqueControllerImpl::SetPoint(const IdAndIqPoint& point)
    {
        foc.SetPoint(point);
    }

    void TorqueControllerImpl::Enable()
    {
        ControllerBaseImpl::Enable();
    }

    void TorqueControllerImpl::Disable()
    {
        ControllerBaseImpl::Disable();
    }

    bool TorqueControllerImpl::IsRunning() const
    {
        return ControllerBaseImpl::IsRunning();
    }
}
