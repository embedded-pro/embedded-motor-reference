#include "application/foc/MotorFieldOrientedController.hpp"

namespace application
{
    MotorFieldOrientedControllerImpl::MotorFieldOrientedControllerImpl(MotorFieldOrientedControllerInterface& interface, FieldOrientedController& foc)
        : interface{ interface }
        , foc{ foc }
    {
        interface.PhaseCurrentsReady([this](std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)
            {
                this->interface.ThreePhasePwmOutput(this->foc.Calculate(dPid, qPid, voltagePhases, position));
            });
    }

    void MotorFieldOrientedControllerImpl::SetTunnings(IdAndIqTunnings tunnings)
    {
        dPid.SetTunnings(tunnings.first);
        qPid.SetTunnings(tunnings.second);
    }

    void MotorFieldOrientedControllerImpl::SetPoint(const IdAndIqPoint& point)
    {
        dPid.SetPoint(point.first);
        qPid.SetPoint(point.second);
    }

    void MotorFieldOrientedControllerImpl::Enable()
    {
        enabled = true;
        dPid.Enable();
        qPid.Enable();
        interface.Start();
    }

    void MotorFieldOrientedControllerImpl::Disable()
    {
        enabled = false;
        interface.Stop();
        dPid.Disable();
        qPid.Disable();
    }

    bool MotorFieldOrientedControllerImpl::IsRunning() const
    {
        return enabled;
    }
}
