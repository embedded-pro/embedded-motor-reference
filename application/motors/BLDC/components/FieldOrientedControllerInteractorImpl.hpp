#pragma once

#include "application/foc/FieldOrientedControllerInterface.hpp"
#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/motors/BLDC/components/FieldOrientedControllerInteractor.hpp"

namespace application
{
    class FieldOrientedControllerInteractorImpl
        : public FieldOrientedControllerInteractor
    {
    public:
        FieldOrientedControllerInteractorImpl(FieldOrientedControllerInterface& interface, MotorFieldOrientedController::Components& components);

        // Implementation of MotorController
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters) override;
        void SetTorque(const Torque& torque) override;
        void Start() override;
        void Stop() override;

    private:
        MotorFieldOrientedController foc;
        MotorFieldOrientedController::IdAndIqPoint focSetPoint;
        MotorFieldOrientedController::IdAndIqTunnings idAndIqTunnings{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };
    };
}
