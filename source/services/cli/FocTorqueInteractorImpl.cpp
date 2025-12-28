#include "source/services/cli/FocTorqueInteractorImpl.hpp"

namespace services
{
    FocTorqueInteractorImpl::FocTorqueInteractorImpl(foc::Volts vdc, foc::TorqueController& foc)
        : vdc{ vdc }
        , foc{ foc }
    {}

    void FocTorqueInteractorImpl::AutoTune(const infra::Function<void()>& onDone)
    {
    }

    void FocTorqueInteractorImpl::SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters)
    {
        if (pidDAndQParameters.first.kp.has_value())
            IdAndIqTunings.first.kp = *pidDAndQParameters.first.kp;

        if (pidDAndQParameters.first.ki.has_value())
            IdAndIqTunings.first.ki = *pidDAndQParameters.first.ki;

        if (pidDAndQParameters.first.kd.has_value())
            IdAndIqTunings.first.kd = *pidDAndQParameters.first.kd;

        if (pidDAndQParameters.second.kp.has_value())
            IdAndIqTunings.second.kp = *pidDAndQParameters.second.kp;

        if (pidDAndQParameters.second.ki.has_value())
            IdAndIqTunings.second.ki = *pidDAndQParameters.second.ki;

        if (pidDAndQParameters.second.kd.has_value())
            IdAndIqTunings.second.kd = *pidDAndQParameters.second.kd;

        foc.SetTunings(vdc, IdAndIqTunings);
    }

    void FocTorqueInteractorImpl::SetTorque(const foc::Nm& torque)
    {
        setPoint.first = foc::Ampere{ torque.Value() };
        setPoint.second = foc::Ampere{ 0.0f };

        foc.SetPoint(setPoint);
    }

    void FocTorqueInteractorImpl::Start()
    {
        foc.Enable();
    }

    void FocTorqueInteractorImpl::Stop()
    {
        foc.Disable();
    }
}
