#include "application/motors/DC/logic/MotorControllerImpl.hpp"
#include <chrono>

namespace application
{
    MotorControllerImpl::MotorControllerImpl(hal::SynchronousQuadratureEncoder& input, hal::SynchronousPwm& output, Pid& pid, const uint32_t& timerId)
        : input(input)
        , output(output)
        , pidWithTimer(*this, *this, pid, std::chrono::milliseconds(5), timerId)
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

        pidWithTimer.SetTunnings(tunnings);
    }

    void MotorControllerImpl::SetSpeed(const RevPerMinute& speed)
    {
        pidWithTimer.SetPoint(speed.Value());
    }

    void MotorControllerImpl::Start()
    {
        pidWithTimer.Enable();
    }

    void MotorControllerImpl::Stop()
    {
        pidWithTimer.Disable();
    }

    float MotorControllerImpl::Read()
    {
        return static_cast<float>(input.Speed());
    }

    void MotorControllerImpl::Update(float action)
    {
        output.Start(hal::Percent(action * 100.0f));
    }

    void MotorControllerImpl::Disable()
    {
        output.Stop();
    }
}
