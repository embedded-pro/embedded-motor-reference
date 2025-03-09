#ifndef APPLICATION_DC_LOGIC_MOTOR_CONTROLLER_IMPL_HPP
#define APPLICATION_DC_LOGIC_MOTOR_CONTROLLER_IMPL_HPP

#include "application/motors/DC/logic/MotorController.hpp"
#include "application/pid/PidWithTimer.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"

namespace application
{
    class MotorControllerImpl
        : public MotorController
        , private Input
        , private Output
    {
    public:
        MotorControllerImpl(hal::SynchronousQuadratureEncoder& input, hal::SynchronousPwm& output, Pid& pid, const uint32_t& timerId);

        // Implementation of MotorController
        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd) override;
        void SetSpeed(const RevPerMinute& speed) override;
        void Start() override;
        void Stop() override;

        // Implementation of Input
        float Read() override;

        // Implementation of Output
        void Update(float) override;
        void Disable() override;

    private:
        hal::SynchronousQuadratureEncoder& input;
        hal::SynchronousPwm& output;
        PidWithTimer pidWithTimer;
        Pid::Tunnings tunnings{ 0.0f, 0.0f, 0.0f };
    };
}

#endif
