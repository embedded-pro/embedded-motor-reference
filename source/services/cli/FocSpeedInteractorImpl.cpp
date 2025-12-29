#include "source/services/cli/FocSpeedInteractorImpl.hpp"

namespace
{
    float ToRadiansPerSecond(const foc::RevPerMinute& rpm)
    {
        return rpm.Value() * (M_PI / 30.0f);
    }
}

namespace services
{
    FocSpeedInteractorImpl::FocSpeedInteractorImpl(foc::Volts vdc, foc::SpeedController& foc)
        : FocInteractorImpl<foc::SpeedController>(vdc, foc)
    {
    }

    void FocSpeedInteractorImpl::SetSpeed(const foc::RevPerMinute& speed)
    {
        Foc().SetPoint(foc::RadiansPerSecond(ToRadiansPerSecond(speed)));
    }

    void FocSpeedInteractorImpl::SetSpeed(const foc::RadiansPerSecond& speed)
    {
        Foc().SetPoint(speed);
    }

    void FocSpeedInteractorImpl::SetSpeedPidParameters(const PidParameters& pidParameters)
    {
        speedTunings.kd = pidParameters.kd.has_value() ? *pidParameters.kd : speedTunings.kd;
        speedTunings.ki = pidParameters.ki.has_value() ? *pidParameters.ki : speedTunings.ki;
        speedTunings.kp = pidParameters.kp.has_value() ? *pidParameters.kp : speedTunings.kp;

        Foc().SetSpeedTunings(Vdc(), speedTunings);
    }
}
