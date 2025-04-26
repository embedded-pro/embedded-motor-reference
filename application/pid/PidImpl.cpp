#include "application/pid/PidImpl.hpp"

namespace application
{
    PidImpl::PidImpl(PidInterface& interface)
        : interface{ interface }
        , pid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f }, true }
    {
        interface.Read([this](float input)
            {
                this->interface.ControlAction(this->pid.Process(input));
            });
    }

    void PidImpl::SetPoint(float setPoint)
    {
        pid.SetPoint(setPoint);
    }

    void PidImpl::SetTunnings(Pid::Tunnings tunnings)
    {
        pid.SetTunnings(tunnings);
    }

    void PidImpl::Enable()
    {
        pid.Enable();
        interface.Start(sampleTime);
    }

    void PidImpl::Disable()
    {
        interface.Stop();
        pid.Disable();
    }
}
