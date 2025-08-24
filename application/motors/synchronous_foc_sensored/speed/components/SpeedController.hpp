#pragma once

#include "application/foc/MotorFieldOrientedController.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/Unit.hpp"
#include "numerical/controllers/TrajectoryGenerator.hpp"
#include <optional>

namespace application
{
    namespace unit
    {
        using Revolution = infra::BaseUnit<8>;
        using SpeedUnit = infra::BaseUnit<12>; // Use a different unit ID

        using RevPerSecond = Revolution::Div<infra::Second>::Inverse;
        using RevPerMinute = RevPerSecond::Scale<infra::StaticRational<60, 1>>;
        using SpeedPerSecond = SpeedUnit::Div<infra::Second>::Inverse;
    }

    class SpeedController
    {
    public:
        using Speed = infra::Quantity<unit::SpeedUnit, float>;
        using Acceleration = infra::Quantity<unit::SpeedPerSecond, float>;

        struct TrajectoryConstraints
        {
            Acceleration maxAcceleration;
            Acceleration maxDeceleration;
        };

        struct PidParameters
        {
            std::optional<float> kp;
            std::optional<float> ki;
            std::optional<float> kd;
        };

        virtual void SetTrajectoryConstraints(const TrajectoryConstraints& constraints) = 0;
        virtual void SetSpeedPidParameters(const PidParameters& pidParameters) = 0;
        virtual void SetSpeed(const Speed& targetSpeed) = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
        virtual bool IsTrajectoryComplete() const = 0;
        virtual Speed GetCurrentSpeed() const = 0;
    };

    class SpeedControllerImpl
        : public SpeedController
    {
    public:
        explicit SpeedControllerImpl(MotorFieldOrientedController& motorFoc,
            const infra::Function<SpeedController::Speed()>& speedFeedback,
            hal::Hertz controlFrequency);

        // Implementation of SpeedController
        void SetTrajectoryConstraints(const TrajectoryConstraints& constraints) override;
        void SetSpeedPidParameters(const PidParameters& pidParameters) override;
        void SetSpeed(const SpeedController::Speed& targetSpeed) override;
        void Start() override;
        void Stop() override;
        bool IsTrajectoryComplete() const override;
        SpeedController::Speed GetCurrentSpeed() const override;

    private:
        MotorFieldOrientedController& motorFoc_;
        infra::Function<SpeedController::Speed()> speedFeedback_;
        hal::Hertz controlFrequency_;
        float dt_;

        controllers::TrajectoryGenerator<float> trajectoryGenerator_;

        // PID controller for speed control
        struct PidState
        {
            float kp = 1.0f;
            float ki = 0.1f;
            float kd = 0.01f;
            float integral = 0.0f;
            float previousError = 0.0f;
            float output = 0.0f;
        };

        PidState speedPid_;

        // Current state
        SpeedController::Speed currentSpeed_;

        bool enabled_;
        bool trajectoryActive_;

        void UpdateControl();
        float ProcessPid(PidState& pid, float error, float dt);
        void ResetPid(PidState& pid);
    };
}
