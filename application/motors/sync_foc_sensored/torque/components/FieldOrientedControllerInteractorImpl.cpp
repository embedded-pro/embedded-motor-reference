#include "application/motors/sync_foc_sensored/torque/components/FieldOrientedControllerInteractorImpl.hpp"

namespace application
{
    FieldOrientedControllerInteractorImpl::FieldOrientedControllerInteractorImpl(foc::Volts vdc, foc::TorqueController& focTorqueController)
        : vdc{ vdc }
        , focTorqueController{ focTorqueController }
    {}

    void FieldOrientedControllerInteractorImpl::AutoTune(const infra::Function<void()>& onDone)
    {
    }

    void FieldOrientedControllerInteractorImpl::SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters)
    {
        if (pidDAndQParameters.first.kp)
            IdAndIqTunings.first.kp = *pidDAndQParameters.first.kp;

        if (pidDAndQParameters.first.ki)
            IdAndIqTunings.first.ki = *pidDAndQParameters.first.ki;

        if (pidDAndQParameters.first.kd)
            IdAndIqTunings.first.kd = *pidDAndQParameters.first.kd;

        if (pidDAndQParameters.second.kp)
            IdAndIqTunings.second.kp = *pidDAndQParameters.second.kp;

        if (pidDAndQParameters.second.ki)
            IdAndIqTunings.second.ki = *pidDAndQParameters.second.ki;

        if (pidDAndQParameters.second.kd)
            IdAndIqTunings.second.kd = *pidDAndQParameters.second.kd;

        focTorqueController.SetTunings(vdc, IdAndIqTunings);
    }

    void FieldOrientedControllerInteractorImpl::SetTorque(const Torque& speed)
    {
        focSetPoint.first = foc::Ampere{ speed.Value() };
        focSetPoint.second = foc::Ampere{ 0.0f };

        focTorqueController.SetPoint(focSetPoint);
    }

    void FieldOrientedControllerInteractorImpl::Start()
    {
        focTorqueController.Enable();
    }

    void FieldOrientedControllerInteractorImpl::Stop()
    {
        focTorqueController.Disable();
    }
}
