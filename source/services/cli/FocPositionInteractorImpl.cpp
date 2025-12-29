#include "source/services/cli/FocPositionInteractorImpl.hpp"

namespace services
{
    FocPositionInteractorImpl::FocPositionInteractorImpl(foc::Volts vdc, foc::PositionController& foc)
        : FocInteractorImpl<foc::PositionController>(vdc, foc)
    {
    }

    void FocPositionInteractorImpl::SetPosition(const foc::Radians& position)
    {
        Foc().SetPoint(position);
    }

    void FocPositionInteractorImpl::SetPositionPidParameters(const PidParameters& pidParameters)
    {
        positionTunings.kd = pidParameters.kd.has_value() ? *pidParameters.kd : positionTunings.kd;
        positionTunings.ki = pidParameters.ki.has_value() ? *pidParameters.ki : positionTunings.ki;
        positionTunings.kp = pidParameters.kp.has_value() ? *pidParameters.kp : positionTunings.kp;

        Foc().SetTunings(Vdc(), positionTunings, speedTunings, CurrentTunings());
    }
}
