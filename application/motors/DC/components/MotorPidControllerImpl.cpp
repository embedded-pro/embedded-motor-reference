#include "application/motors/DC/components/MotorPidControllerImpl.hpp"

namespace application
{
    MotorPidControllerImpl::MotorPidControllerImpl(PidInterface& interface, Pid& pid)
        : pidAsync{ interface, pid }
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

        pidAsync.SetTunnings(tunnings);
    }

    void MotorPidControllerImpl::SetSpeed(const RevPerMinute& speed)
    {
        pidAsync.SetPoint(speed.Value());
    }

    void MotorPidControllerImpl::Start()
    {
        pidAsync.Enable();
    }

    void MotorPidControllerImpl::Stop()
    {
        pidAsync.Disable();
    }
}
