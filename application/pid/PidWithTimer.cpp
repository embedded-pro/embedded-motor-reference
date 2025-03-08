#include "application/pid/PidWithTimer.hpp"

namespace application
{
    PidWithTimer::PidWithTimer(Input& input, Output& output, Pid& pid, infra::Duration sampleTime, const uint32_t& timerId)
        : input(input)
        , output(output)
        , pid(pid)
        , timer(timerId)
        , sampleTime(sampleTime)
    {
        really_assert(timerId != infra::systemTimerServiceId);
    }

    void PidWithTimer::SetPoint(float setPoint)
    {
        pid.SetPoint(setPoint);
    }

    void PidWithTimer::SetTunnings(Pid::Tunnings tunnings)
    {
        pid.SetTunnings(tunnings);
    }

    void PidWithTimer::Enable()
    {
        pid.Enable();
        timer.Start(sampleTime, [this]()
            {
                output.Update(this->pid.Process(input.Read()));
            });
    }

    void PidWithTimer::Disable()
    {
        timer.Cancel();
        output.Disable();
        pid.Disable();
    }

    bool PidWithTimer::IsRunning() const
    {
        return timer.Armed();
    }
}
