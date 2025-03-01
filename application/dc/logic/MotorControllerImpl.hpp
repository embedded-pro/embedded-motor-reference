#ifndef APPLICATION_DC_LOGIC_MOTOR_CONTROLLER_IMPL_HPP
#define APPLICATION_DC_LOGIC_MOTOR_CONTROLLER_IMPL_HPP

#include "application/dc/logic/MotorController.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include "infra/timer/Timer.hpp"
#include "numerical/controllers/Pid.hpp"

namespace application
{
    class MotorControllerImpl
        : public MotorController
    {
    public:
        MotorControllerImpl(hal::SynchronousQuadratureEncoder& encoder, hal::SynchronousPwm& outputPwm, const uint32_t& timerId);

        void AutoTune(const infra::Function<void()>& onDone) override;
        void SetPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd) override;
        void SetSpeed(const RevPerMinute& speed) override;
        void Start() override;
        void Stop() override;

    private:
        class PidWithTimer
            : private controllers::Pid<float>
        {
        public:
            using Tunnings = controllers::Pid<float>::Tunnings;

            PidWithTimer(hal::SynchronousQuadratureEncoder& encoder, hal::SynchronousPwm& outputPwm, infra::Duration sampleTime, const uint32_t& timerId);

            void SetTunnings(Tunnings tunnings);
            void SetPoint(float setPoint);
            void Enable();
            void Disable();
            bool IsRunning() const;

        private:
            hal::SynchronousQuadratureEncoder& input;
            hal::SynchronousPwm& output;
            infra::TimerRepeating timer;
            infra::Duration sampleTime;
        };

    private:
        PidWithTimer pid;
        PidWithTimer::Tunnings tunnings{ 0.0f, 0.0f, 0.0f };
    };
}

#endif
