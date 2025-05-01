#pragma once

#include "application/motors/DC/components/MotorPidController.hpp"
#include "application/pid/PidImpl.hpp"

namespace application
{
    class MotorPidControllerImpl
        : public MotorPidController
    {
    public:
        explicit MotorPidControllerImpl(Pid& pid);

        // Implementation of MotorController
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd) override;
        void SetSpeed(const RevPerMinute& speed) override;
        void Start() override;
        void Stop() override;

    private:
        Pid::Tunnings tunnings{ 0.0f, 0.0f, 0.0f };
        Pid& pid;
    };
}
