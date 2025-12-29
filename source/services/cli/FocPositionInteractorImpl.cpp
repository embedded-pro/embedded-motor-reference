#include "source/services/cli/FocPositionInteractorImpl.hpp"

namespace services
{
    FocPositionInteractorImpl::FocPositionInteractorImpl(foc::Volts vdc, foc::PositionController& foc)
        : FocInteractorImpl<foc::PositionController>(vdc, foc)
    {
    }

    void FocPositionInteractorImpl::FocPositionInteractorImpl::SetPosition(const foc::Radians& position)
    {
        Foc().SetPoint(position);
    }

    void FocPositionInteractorImpl::SetSpeedPidParameters(const PidParameters& pidParameters)
    {
        foc::SpeedTunings speedTunings{};

        speedTunings.kd = pidParameters.kd.has_value() ? *pidParameters.kd : speedTunings.kd;
        speedTunings.ki = pidParameters.ki.has_value() ? *pidParameters.ki : speedTunings.ki;
        speedTunings.kp = pidParameters.kp.has_value() ? *pidParameters.kp : speedTunings.kp;

        Foc().SetSpeedTunings(Vdc(), speedTunings);
    }

    void FocPositionInteractorImpl::SetPositionPidParameters(const PidParameters& pidParameters)
    {
        foc::PositionTunings positionTunings{};

        positionTunings.kd = pidParameters.kd.has_value() ? *pidParameters.kd : positionTunings.kd;
        positionTunings.ki = pidParameters.ki.has_value() ? *pidParameters.ki : positionTunings.ki;
        positionTunings.kp = pidParameters.kp.has_value() ? *pidParameters.kp : positionTunings.kp;

        Foc().SetPositionTunings(Vdc(), positionTunings);
    }
}
