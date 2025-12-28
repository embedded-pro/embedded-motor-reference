#include "source/services/cli/FocTorqueInteractorImpl.hpp"

namespace services
{
    FocTorqueInteractorImpl::FocTorqueInteractorImpl(foc::Volts vdc, foc::TorqueController& focTorqueController)
        : vdc{ vdc }
        , focTorqueController{ focTorqueController }
    {}

    void FocTorqueInteractorImpl::AutoTune(const infra::Function<void()>& onDone)
    {
    }

    void FocTorqueInteractorImpl::SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters)
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

    void FocTorqueInteractorImpl::SetTorque(const foc::Nm& torque)
    {
        focSetPoint.first = foc::Ampere{ torque.Value() };
        focSetPoint.second = foc::Ampere{ 0.0f };

        focTorqueController.SetPoint(focSetPoint);
    }

    void FocTorqueInteractorImpl::Start()
    {
        focTorqueController.Enable();
    }

    void FocTorqueInteractorImpl::Stop()
    {
        focTorqueController.Disable();
    }
}
