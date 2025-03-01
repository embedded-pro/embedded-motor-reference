#include "application/dc/logic/MotorControllerImpl.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "infra/timer/Timer.hpp"
#include <chrono>

namespace application
{
    MotorControllerImpl::MotorControllerImpl(hal::SynchronousQuadratureEncoder& encoder, hal::SynchronousPwm& outputPwm, const uint32_t& timerId)
        : pid(encoder, outputPwm, std::chrono::milliseconds(5), timerId)
    {}

    void MotorControllerImpl::AutoTune(const infra::Function<void()>& onDone)
    {
    }

    void MotorControllerImpl::SetPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd)
    {
        if (kp)
            tunnings.kp = *kp;

        if (ki)
            tunnings.ki = *ki;

        if (kd)
            tunnings.kd = *kd;

        pid.SetTunnings(tunnings);
    }

    void MotorControllerImpl::SetSpeed(const RevPerMinute& speed)
    {
        pid.SetPoint(speed.Value());
    }

    void MotorControllerImpl::Start()
    {
        pid.Enable();
    }

    void MotorControllerImpl::Stop()
    {
        pid.Disable();
    }

    MotorControllerImpl::PidWithTimer::PidWithTimer(hal::SynchronousQuadratureEncoder& input, hal::SynchronousPwm& output, infra::Duration sampleTime, const uint32_t& timerId)
        : controllers::Pid<float>(Tunnings{ 0.0f, 0.0f, 0.0f }, controllers::Pid<float>::Limits{ 0.0f, 0.9999f })
        , input(input)
        , output(output)
        , timer(timerId)
        , sampleTime(sampleTime)
    {
        really_assert(timerId != infra::systemTimerServiceId);
    }

    void MotorControllerImpl::PidWithTimer::SetPoint(float setPoint)
    {
        this->SetPoint(setPoint);
    }

    void MotorControllerImpl::PidWithTimer::SetTunnings(Tunnings tunnings)
    {
        this->SetTunnings(tunnings);
    }

    void MotorControllerImpl::PidWithTimer::Enable()
    {
        this->Enable();
        timer.Start(sampleTime, [this]()
            {
                output.Start(hal::Percent(Process(input.Speed()) * 100.0f));
            });
    }

    void MotorControllerImpl::PidWithTimer::Disable()
    {
        timer.Cancel();
        output.Stop();
        this->Disable();
    }

    bool MotorControllerImpl::PidWithTimer::IsRunning() const
    {
        return timer.Armed();
    }
}
