#include "application/motors/synchronous_foc_sensored/torque/components/FieldOrientedControllerInteractorImpl.hpp"

namespace application
{
    FieldOrientedControllerInteractorImpl::FieldOrientedControllerInteractorImpl(MotorFieldOrientedController& motorFoc)
        : motorFoc{ motorFoc }
    {}

    void FieldOrientedControllerInteractorImpl::AutoTune(const infra::Function<void()>& onDone)
    {
    }

    void FieldOrientedControllerInteractorImpl::SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters)
    {
        if (pidDAndQParameters.first.kp)
            idAndIqTunnings.first.kp = *pidDAndQParameters.first.kp;

        if (pidDAndQParameters.first.ki)
            idAndIqTunnings.first.ki = *pidDAndQParameters.first.ki;

        if (pidDAndQParameters.first.kd)
            idAndIqTunnings.first.kd = *pidDAndQParameters.first.kd;

        if (pidDAndQParameters.second.kp)
            idAndIqTunnings.second.kp = *pidDAndQParameters.second.kp;

        if (pidDAndQParameters.second.ki)
            idAndIqTunnings.second.ki = *pidDAndQParameters.second.ki;

        if (pidDAndQParameters.second.kd)
            idAndIqTunnings.second.kd = *pidDAndQParameters.second.kd;

        motorFoc.SetTunnings(idAndIqTunnings);
    }

    void FieldOrientedControllerInteractorImpl::SetTorque(const Torque& speed)
    {
        focSetPoint.first = speed.Value();
        focSetPoint.second = 0.0;

        motorFoc.SetPoint(focSetPoint);
    }

    void FieldOrientedControllerInteractorImpl::Start()
    {
        motorFoc.Enable();
    }

    void FieldOrientedControllerInteractorImpl::Stop()
    {
        motorFoc.Disable();
    }
}
