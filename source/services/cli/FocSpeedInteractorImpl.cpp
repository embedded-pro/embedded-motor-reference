#include "source/services/cli/FocSpeedInteractorImpl.hpp"

namespace
{
    float ToRadiansPerSecond(const foc::RevPerMinute& rpm)
    {
        return rpm.Value() * (M_PI / 30.0);
    }
}

namespace services
{
    FocSpeedInteractorImpl::FocSpeedInteractorImpl(foc::Volts vdc, foc::SpeedController& foc)
        : vdc(vdc)
        , foc(foc)
    {
    }

    void FocSpeedInteractorImpl::AutoTune(const infra::Function<void()>& onDone)
    {
    }

    void FocSpeedInteractorImpl::SetSpeed(const foc::RevPerMinute& speed)
    {
        foc.SetPoint(foc::RadiansPerSecond(ToRadiansPerSecond(speed)));
    }

    void FocSpeedInteractorImpl::SetSpeed(const foc::RadiansPerSecond& speed)
    {
        foc.SetPoint(speed);
    }

    void FocSpeedInteractorImpl::SetSpeedPidParameters(const PidParameters& pidParameters)
    {
        speedTunings.kd = pidParameters.kd.has_value() ? *pidParameters.kd : speedTunings.kd;
        speedTunings.ki = pidParameters.ki.has_value() ? *pidParameters.ki : speedTunings.ki;
        speedTunings.kp = pidParameters.kp.has_value() ? *pidParameters.kp : speedTunings.kp;

        foc.SetTunings(vdc, speedTunings, IdAndIqTunings);
    }

    void FocSpeedInteractorImpl::SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters)
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

        foc.SetTunings(vdc, speedTunings, IdAndIqTunings);
    }

    void FocSpeedInteractorImpl::Start()
    {
        foc.Enable();
    }

    void FocSpeedInteractorImpl::Stop()
    {
        foc.Disable();
    }
}
