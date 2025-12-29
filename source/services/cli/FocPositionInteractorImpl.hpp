#pragma once

#include "source/foc/interfaces/Controller.hpp"
#include "source/services/cli/FocInteractor.hpp"
#include "source/services/cli/FocInteractorImpl.hpp"

namespace services
{
    class FocPositionInteractorImpl
        : public FocPositionInteractor
        , public FocInteractorImpl<foc::PositionController>
    {
    public:
        explicit FocPositionInteractorImpl(foc::Volts vdc, foc::PositionController& focPositionController);

        // Implementation of FocPositionInteractor
        void SetPosition(const foc::Radians& position) override;
        void SetSpeedPidParameters(const PidParameters& pidParameters) override;
        void SetPositionPidParameters(const PidParameters& pidParameters) override;
    };
}
