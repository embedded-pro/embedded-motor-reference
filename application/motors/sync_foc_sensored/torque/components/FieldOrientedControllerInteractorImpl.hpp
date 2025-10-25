#pragma once

#include "application/foc/interfaces/Controller.hpp"
#include "application/motors/sync_foc_sensored/torque/components/FieldOrientedControllerInteractor.hpp"

namespace application
{
    class FieldOrientedControllerInteractorImpl
        : public FieldOrientedControllerInteractor
    {
    public:
        explicit FieldOrientedControllerInteractorImpl(foc::Volts vdc, foc::TorqueController& focTorqueController);

        // Implementation of MotorController
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters) override;
        void SetTorque(const Torque& torque) override;
        void Start() override;
        void Stop() override;

    private:
        foc::Volts vdc;
        foc::TorqueController& focTorqueController;
        foc::IdAndIqPoint focSetPoint;
        foc::IdAndIqTunings IdAndIqTunings;
    };
}
