#include "application/motors/dc/components/MotorPidControllerImpl.hpp"

namespace application
{
    MotorPidControllerImpl::MotorPidControllerImpl(Pid& pid)
        : pid{ pid }
    {}

    void MotorPidControllerImpl::AutoTune(const infra::Function<void()>& onDone)
    {
    }

    void MotorPidControllerImpl::SetPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd)
    {
        if (kp)
            tunnings.kp = *kp;

        if (ki)
            tunnings.ki = *ki;

        if (kd)
            tunnings.kd = *kd;

        pid.SetTunnings(tunnings);
    }

    void MotorPidControllerImpl::SetSpeed(const RevPerMinute& speed)
    {
        pid.SetPoint(speed.Value());
    }

    void MotorPidControllerImpl::Start()
    {
        pid.Enable();
    }

    void MotorPidControllerImpl::Stop()
    {
        pid.Disable();
    }
}
