#include "application/pid/instantiations/PidImpl.hpp"

namespace application
{
    PidImpl::PidImpl()
        : pid(Tunnings{ 0.0f, 0.0f, 0.0f }, controllers::Pid<float>::Limits{ 0.0f, 0.9999f })
    {}

    void PidImpl::SetTunnings(Tunnings tunnings)
    {
        pid.SetTunnings(tunnings);
    }

    void PidImpl::SetPoint(float setPoint)
    {
        pid.SetPoint(setPoint);
    }

    void PidImpl::Enable()
    {
        pid.Enable();
    }

    void PidImpl::Disable()
    {
        pid.Disable();
    }

    float PidImpl::Process(float measuredProcessVariable)
    {
        return pid.Process(measuredProcessVariable);
    }
}
