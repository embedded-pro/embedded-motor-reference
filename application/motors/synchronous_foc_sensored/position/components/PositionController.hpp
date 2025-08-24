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
        using Torque = infra::BaseUnit<9>;
        using Position = infra::BaseUnit<11>; // Use a different unit ID to avoid conflicts

        using RevPerSecond = Revolution::Div<infra::Second>::Inverse;
        using RevPerMinute = RevPerSecond::Scale<infra::StaticRational<60, 1>>;
        using PositionPerSecond = Position::Div<infra::Second>::Inverse;
    }

    class PositionController
    {
    public:
        using Position = infra::Quantity<unit::Position, float>;
        using Velocity = infra::Quantity<unit::PositionPerSecond, float>;
        using Acceleration = infra::Quantity<unit::PositionPerSecond::Div<infra::Second>::Inverse, float>;

        struct TrajectoryConstraints
        {
            Velocity maxVelocity;
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
        virtual void SetPositionPidParameters(const PidParameters& pidParameters) = 0;
        virtual void SetVelocityPidParameters(const PidParameters& pidParameters) = 0;
        virtual void SetPosition(const Position& targetPosition) = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
        virtual bool IsTrajectoryComplete() const = 0;
        virtual Position GetCurrentPosition() const = 0;
        virtual Velocity GetCurrentVelocity() const = 0;
    };

    class PositionControllerImpl
        : public PositionController
    {
    public:
        explicit PositionControllerImpl(MotorFieldOrientedController& motorFoc,
            const infra::Function<PositionController::Position()>& positionFeedback,
            hal::Hertz controlFrequency);

        // Implementation of PositionController
        void SetTrajectoryConstraints(const TrajectoryConstraints& constraints) override;
        void SetPositionPidParameters(const PidParameters& pidParameters) override;
        void SetVelocityPidParameters(const PidParameters& pidParameters) override;
        void SetPosition(const PositionController::Position& targetPosition) override;
        void Start() override;
        void Stop() override;
        bool IsTrajectoryComplete() const override;
        PositionController::Position GetCurrentPosition() const override;
        PositionController::Velocity GetCurrentVelocity() const override;

    private:
        MotorFieldOrientedController& motorFoc_;
        infra::Function<PositionController::Position()> positionFeedback_;
        hal::Hertz controlFrequency_;
        float dt_;

        controllers::TrajectoryGenerator<float> trajectoryGenerator_;

        // PID controllers for cascaded control
        struct PidState
        {
            float kp = 1.0f;
            float ki = 0.0f;
            float kd = 0.0f;
            float integral = 0.0f;
            float previousError = 0.0f;
            float output = 0.0f;
        };

        PidState positionPid_;
        PidState velocityPid_;

        // Current state
        PositionController::Position currentPosition_;
        PositionController::Velocity currentVelocity_;
        PositionController::Position previousPosition_;

        bool enabled_;
        bool trajectoryActive_;

        void UpdateControl();
        float ProcessPid(PidState& pid, float error, float dt);
        void ResetPid(PidState& pid);
    };
}
