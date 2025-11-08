#pragma once

#include "application/motors/dc/components/MotorPidController.hpp"
#include "numerical/controllers/interfaces/PidController.hpp"

namespace application
{
    class MotorPidControllerImpl
        : public MotorPidController
    {
    public:
        using Pid = controllers::AsynchronousPidController<float>;

        explicit MotorPidControllerImpl(Pid& pid);

        // Implementation of MotorController
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd) override;
        void SetSpeed(const RevPerMinute& speed) override;
        void Start() override;
        void Stop() override;

    private:
        controllers::PidTunings<float> tunings{ 0.0f, 0.0f, 0.0f };
        Pid& pid;
    };
}
