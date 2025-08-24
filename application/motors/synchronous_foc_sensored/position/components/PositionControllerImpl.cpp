#include "application/motors/synchronous_foc_sensored/position/components/PositionController.hpp"
#include <algorithm>

namespace application
{
    PositionControllerImpl::PositionControllerImpl(MotorFieldOrientedController& motorFoc,
        const infra::Function<PositionController::Position()>& positionFeedback,
        hal::Hertz controlFrequency)
        : motorFoc_(motorFoc)
        , positionFeedback_(positionFeedback)
        , controlFrequency_(controlFrequency)
        , dt_(1.0f / controlFrequency.Value())
        , trajectoryGenerator_({ 10.0f, 5.0f, 5.0f }) // Default constraints
        , currentPosition_(PositionController::Position{ 0.0f })
        , currentVelocity_(PositionController::Velocity{ 0.0f })
        , previousPosition_(PositionController::Position{ 0.0f })
        , enabled_(false)
        , trajectoryActive_(false)
    {
        // Initialize PID controllers with reasonable defaults
        positionPid_.kp = 1.0f;
        positionPid_.ki = 0.1f;
        positionPid_.kd = 0.05f;

        velocityPid_.kp = 0.5f;
        velocityPid_.ki = 0.1f;
        velocityPid_.kd = 0.01f;
    }

    void PositionControllerImpl::SetTrajectoryConstraints(const TrajectoryConstraints& constraints)
    {
        controllers::TrajectoryGenerator<float>::Constraints genConstraints;
        genConstraints.maxVelocity = constraints.maxVelocity.Value();
        genConstraints.maxAcceleration = constraints.maxAcceleration.Value();
        genConstraints.maxDeceleration = constraints.maxDeceleration.Value();

        trajectoryGenerator_ = controllers::TrajectoryGenerator<float>(genConstraints);
    }

    void PositionControllerImpl::SetPositionPidParameters(const PidParameters& pidParameters)
    {
        if (pidParameters.kp)
            positionPid_.kp = *pidParameters.kp;
        if (pidParameters.ki)
            positionPid_.ki = *pidParameters.ki;
        if (pidParameters.kd)
            positionPid_.kd = *pidParameters.kd;
    }

    void PositionControllerImpl::SetVelocityPidParameters(const PidParameters& pidParameters)
    {
        if (pidParameters.kp)
            velocityPid_.kp = *pidParameters.kp;
        if (pidParameters.ki)
            velocityPid_.ki = *pidParameters.ki;
        if (pidParameters.kd)
            velocityPid_.kd = *pidParameters.kd;
    }

    void PositionControllerImpl::SetPosition(const PositionController::Position& targetPosition)
    {
        if (!enabled_)
            return;

        // Update current position from feedback
        currentPosition_ = positionFeedback_();

        // Set initial conditions for trajectory generator
        trajectoryGenerator_.SetInitialConditions(currentPosition_.Value(), currentVelocity_.Value());

        // Generate new trajectory
        trajectoryGenerator_.SetTarget(targetPosition.Value());
        trajectoryActive_ = true;

        // Reset PID controllers
        ResetPid(positionPid_);
        ResetPid(velocityPid_);
    }

    void PositionControllerImpl::Start()
    {
        if (enabled_)
            return;

        enabled_ = true;
        currentPosition_ = positionFeedback_();
        previousPosition_ = currentPosition_;
        currentVelocity_ = PositionController::Velocity{ 0.0f };

        // Initialize trajectory generator at current position
        trajectoryGenerator_.Reset(currentPosition_.Value());
        trajectoryActive_ = false;

        // Reset PID controllers
        ResetPid(positionPid_);
        ResetPid(velocityPid_);

        // Start the motor controller
        motorFoc_.Enable();

        // Start periodic control update (this would typically be done via timer interrupt)
        // For now, we'll assume UpdateControl() is called periodically by the application
    }

    void PositionControllerImpl::Stop()
    {
        enabled_ = false;
        trajectoryActive_ = false;

        // Stop the motor controller
        motorFoc_.Disable();

        // Reset PID controllers
        ResetPid(positionPid_);
        ResetPid(velocityPid_);
    }

    bool PositionControllerImpl::IsTrajectoryComplete() const
    {
        return !trajectoryActive_ || trajectoryGenerator_.IsComplete();
    }

    PositionController::Position PositionControllerImpl::GetCurrentPosition() const
    {
        return currentPosition_;
    }

    PositionController::Velocity PositionControllerImpl::GetCurrentVelocity() const
    {
        return currentVelocity_;
    }

    void PositionControllerImpl::UpdateControl()
    {
        if (!enabled_)
            return;

        // Update current position and calculate velocity
        previousPosition_ = currentPosition_;
        currentPosition_ = positionFeedback_();

        float positionDiff = currentPosition_.Value() - previousPosition_.Value();
        currentVelocity_ = PositionController::Velocity{ positionDiff / dt_ };

        if (trajectoryActive_ && !trajectoryGenerator_.IsComplete())
        {
            // Get trajectory setpoints
            auto motionProfile = trajectoryGenerator_.Update(dt_);

            // Position control loop (outer loop)
            float positionError = motionProfile.position - currentPosition_.Value();
            float velocitySetpoint = ProcessPid(positionPid_, positionError, dt_);

            // Add feedforward velocity from trajectory
            velocitySetpoint += motionProfile.velocity;

            // Velocity control loop (inner loop)
            float velocityError = velocitySetpoint - currentVelocity_.Value();
            float torqueSetpoint = ProcessPid(velocityPid_, velocityError, dt_);

            // Convert to motor torque command (simplified)
            // In a real implementation, you would convert from mechanical torque to electrical torque
            MotorFieldOrientedController::IdAndIqPoint currentSetpoint;
            currentSetpoint.first = 0.0f;                                       // d-axis current (flux control)
            currentSetpoint.second = std::clamp(torqueSetpoint, -10.0f, 10.0f); // q-axis current (torque)

            motorFoc_.SetPoint(currentSetpoint);
        }
        else
        {
            // No active trajectory, hold current position
            if (trajectoryActive_)
            {
                trajectoryActive_ = false; // Mark trajectory as complete
            }

            // Simple position hold
            float positionError = currentPosition_.Value() - currentPosition_.Value(); // Hold current position
            float velocitySetpoint = ProcessPid(positionPid_, positionError, dt_);

            float velocityError = velocitySetpoint - currentVelocity_.Value();
            float torqueSetpoint = ProcessPid(velocityPid_, velocityError, dt_);

            MotorFieldOrientedController::IdAndIqPoint holdSetpoint;
            holdSetpoint.first = 0.0f;
            holdSetpoint.second = std::clamp(torqueSetpoint * 0.1f, -1.0f, 1.0f); // Reduced torque for holding

            motorFoc_.SetPoint(holdSetpoint);
        }
    }

    float PositionControllerImpl::ProcessPid(PidState& pid, float error, float dt)
    {
        // Proportional term
        float proportional = pid.kp * error;

        // Integral term with anti-windup
        pid.integral += error * dt;
        pid.integral = std::clamp(pid.integral, -100.0f, 100.0f); // Anti-windup limit
        float integral = pid.ki * pid.integral;

        // Derivative term
        float derivative = pid.kd * (error - pid.previousError) / dt;
        pid.previousError = error;

        // Calculate output
        pid.output = proportional + integral + derivative;

        return pid.output;
    }

    void PositionControllerImpl::ResetPid(PidState& pid)
    {
        pid.integral = 0.0f;
        pid.previousError = 0.0f;
        pid.output = 0.0f;
    }
}
