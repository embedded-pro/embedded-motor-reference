#pragma once

#include "source/foc/interfaces/Controller.hpp"
#include "source/services/cli/FocInteractor.hpp"
#include "source/services/cli/FocInteractorImpl.hpp"

namespace services
{
    class FocSpeedInteractorImpl
        : public FocSpeedInteractor
        , public FocInteractorImpl<foc::SpeedController>
    {
    public:
        explicit FocSpeedInteractorImpl(foc::Volts vdc, foc::SpeedController& focSpeedController);

        // Implementation of FocSpeedInteractor
        void SetSpeed(const foc::RevPerMinute& speed) override;
        void SetSpeed(const foc::RadiansPerSecond& speed) override;
        void SetSpeedPidParameters(const PidParameters& pidParameters) override;
    };
}
