#pragma once

#include "application/foc/FieldOrientedController.hpp"
#include "application/foc/MotorFieldOrientedController.hpp"
#include "application/motors/BLDC/components/FieldOrientedControllerInteractor.hpp"

namespace application
{
    class FieldOrientedControllerInteractorImpl
        : public FieldOrientedControllerInteractor
    {
    public:
        FieldOrientedControllerInteractorImpl(MotorFieldOrientedControllerInterface& interface, FieldOrientedController& foc);

        // Implementation of MotorController
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters) override;
        void SetTorque(const Torque& torque) override;
        void Start() override;
        void Stop() override;

    private:
        MotorFieldOrientedController motorFoc;
        MotorFieldOrientedController::IdAndIqPoint focSetPoint;
        MotorFieldOrientedController::IdAndIqTunnings idAndIqTunnings{ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };
    };
}
