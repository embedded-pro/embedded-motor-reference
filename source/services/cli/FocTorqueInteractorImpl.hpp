#pragma once

#include "source/foc/interfaces/Controller.hpp"
#include "source/services/cli/FocInteractor.hpp"

namespace services
{
    class FocTorqueInteractorImpl
        : public FocTorqueInteractor
    {
    public:
        explicit FocTorqueInteractorImpl(foc::Volts vdc, foc::TorqueController& foc);

        // Implementation of FocTorqueInteractor
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters) override;
        void SetTorque(const foc::Nm& torque) override;
        void Start() override;
        void Stop() override;

    private:
        foc::Volts vdc;
        foc::TorqueController& foc;
        foc::IdAndIqPoint setPoint;
        foc::IdAndIqTunings IdAndIqTunings;
    };
}
