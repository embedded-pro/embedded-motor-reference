#pragma once

#include "source/foc/interfaces/Controller.hpp"
#include "source/services/cli/FocInteractor.hpp"
#include "source/services/cli/FocInteractorImpl.hpp"

namespace services
{
    class FocTorqueInteractorImpl
        : public FocTorqueInteractor
        , public FocInteractorImpl<foc::TorqueController>
    {
    public:
        explicit FocTorqueInteractorImpl(foc::Volts vdc, foc::TorqueController& foc);

        // Implementation of FocTorqueInteractor
        void SetTorque(const foc::Nm& torque) override;

    private:
        foc::IdAndIqPoint setPoint{};
    };
}
