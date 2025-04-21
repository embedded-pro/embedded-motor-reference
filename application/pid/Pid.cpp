#include "application/pid/Pid.hpp"

namespace application
{
    PidAsync::PidAsync(PidInterface& interface, Pid& pid)
        : interface(interface)
        , pid(pid)
    {
        interface.Read([this](float input)
            {
                this->interface.ControlAction(this->pid.Process(input));
            });
    }

    void PidAsync::SetPoint(float setPoint)
    {
        pid.SetPoint(setPoint);
    }

    void PidAsync::SetTunnings(Pid::Tunnings tunnings)
    {
        pid.SetTunnings(tunnings);
    }

    void PidAsync::Enable()
    {
        running = true;
        pid.Enable();
        interface.Start(sampleTime);
    }

    void PidAsync::Disable()
    {
        running = false;
        interface.Stop();
        pid.Disable();
    }

    bool PidAsync::IsRunning() const
    {
        return running;
    }
}
