#include "application/pid/PidImpl.hpp"

namespace application
{
    PidImpl::PidImpl(PidInterface& interface)
        : interface{ interface }
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

    void PidImpl::SetTunings(Pid::Tunings tunings)
    {
        pid.SetTunings(tunings);
    }

    void PidImpl::Enable()
    {
        pid.Reset();
        interface.Start(sampleTime);
    }

    void PidImpl::Disable()
    {
        interface.Stop();
    }
}
