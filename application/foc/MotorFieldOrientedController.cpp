#include "application/foc/MotorFieldOrientedController.hpp"

namespace application
{
    MotorFieldOrientedController::MotorFieldOrientedController(MotorFieldOrientedControllerInterface& interface, FieldOrientedController& foc)
        : interface{ interface }
        , foc{ foc }
        , dPid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f }, true }
        , qPid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f }, true }
    {
        interface.PhaseCurrentsReady([this](std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)
            {
                this->interface.ThreePhasePwmOutput(this->foc.Calculate(dPid, qPid, voltagePhases, position));
            });
    }

    void MotorFieldOrientedController::SetTunnings(IdAndIqTunnings tunnings)
    {
        dPid.SetTunnings(tunnings.first);
        qPid.SetTunnings(tunnings.second);
    }

    void MotorFieldOrientedController::SetPoint(const IdAndIqPoint& point)
    {
        dPid.SetPoint(point.first);
        qPid.SetPoint(point.second);
    }

    void MotorFieldOrientedController::Enable()
    {
        enabled = true;
        dPid.Enable();
        qPid.Enable();
        interface.Start();
    }

    void MotorFieldOrientedController::Disable()
    {
        enabled = false;
        interface.Stop();
        dPid.Disable();
        qPid.Disable();
    }

    bool MotorFieldOrientedController::IsRunning() const
    {
        return enabled;
    }
}
