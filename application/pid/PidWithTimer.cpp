#include "application/pid/PidWithTimer.hpp"

namespace application
{
    PidWithTimer::PidWithTimer(Input& input, Output& output, infra::Duration sampleTime, const uint32_t& timerId)
        : controllers::Pid<float>(Tunnings{ 0.0f, 0.0f, 0.0f }, controllers::Pid<float>::Limits{ 0.0f, 0.9999f })
        , input(input)
        , output(output)
        , timer(timerId)
        , sampleTime(sampleTime)
    {
        really_assert(timerId != infra::systemTimerServiceId);
    }

    void PidWithTimer::SetPoint(float setPoint)
    {
        this->SetPoint(setPoint);
    }

    void PidWithTimer::SetTunnings(Tunnings tunnings)
    {
        this->SetTunnings(tunnings);
    }

    void PidWithTimer::Enable()
    {
        this->Enable();
        timer.Start(sampleTime, [this]()
            {
                output.Update(Process(input.Read()));
            });
    }

    void PidWithTimer::Disable()
    {
        timer.Cancel();
        output.Disable();
        this->Disable();
    }

    bool PidWithTimer::IsRunning() const
    {
        return timer.Armed();
    }
}
