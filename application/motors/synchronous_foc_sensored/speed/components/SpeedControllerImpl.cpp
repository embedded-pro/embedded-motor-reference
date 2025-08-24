#include "application/motors/synchronous_foc_sensored/speed/components/SpeedController.hpp"
#include <algorithm>

namespace application
{
    SpeedControllerImpl::SpeedControllerImpl(MotorFieldOrientedController& motorFoc,
        const infra::Function<SpeedController::Speed()>& speedFeedback,
        hal::Hertz controlFrequency)
        : motorFoc_(motorFoc)
        , speedFeedback_(speedFeedback)
        , controlFrequency_(controlFrequency)
        , dt_(1.0f / controlFrequency.Value())
        , trajectoryGenerator_({ 0.0f, 100.0f, 100.0f }) // No max velocity limit for speed, only acceleration
        , currentSpeed_(SpeedController::Speed{ 0.0f })
        , enabled_(false)
        , trajectoryActive_(false)
    {
        // Initialize PID controller with reasonable defaults for speed control
        speedPid_.kp = 1.0f;
        speedPid_.ki = 0.2f;
        speedPid_.kd = 0.05f;
    }

    void SpeedControllerImpl::SetTrajectoryConstraints(const TrajectoryConstraints& constraints)
    {
        controllers::TrajectoryGenerator<float>::Constraints genConstraints;
        genConstraints.maxVelocity = 1000.0f; // Large value - no velocity limit for speed control
        genConstraints.maxAcceleration = constraints.maxAcceleration.Value();
        genConstraints.maxDeceleration = constraints.maxDeceleration.Value();

        trajectoryGenerator_ = controllers::TrajectoryGenerator<float>(genConstraints);
    }

    void SpeedControllerImpl::SetSpeedPidParameters(const PidParameters& pidParameters)
    {
        if (pidParameters.kp)
            speedPid_.kp = *pidParameters.kp;
        if (pidParameters.ki)
            speedPid_.ki = *pidParameters.ki;
        if (pidParameters.kd)
            speedPid_.kd = *pidParameters.kd;
    }

    void SpeedControllerImpl::SetSpeed(const SpeedController::Speed& targetSpeed)
    {
        if (!enabled_)
            return;

        // Update current speed from feedback
        currentSpeed_ = speedFeedback_();

        // Set initial conditions for trajectory generator
        trajectoryGenerator_.SetInitialConditions(currentSpeed_.Value(), 0.0f);

        // Generate new trajectory
        trajectoryGenerator_.SetTarget(targetSpeed.Value());
        trajectoryActive_ = true;

        // Reset PID controller
        ResetPid(speedPid_);
    }

    void SpeedControllerImpl::Start()
    {
        if (enabled_)
            return;

        enabled_ = true;
        currentSpeed_ = speedFeedback_();

        // Initialize trajectory generator at current speed
        trajectoryGenerator_.Reset(currentSpeed_.Value());
        trajectoryActive_ = false;

        // Reset PID controller
        ResetPid(speedPid_);

        // Start the motor controller
        motorFoc_.Enable();
    }

    void SpeedControllerImpl::Stop()
    {
        enabled_ = false;
        trajectoryActive_ = false;

        // Stop the motor controller
        motorFoc_.Disable();

        // Reset PID controller
        ResetPid(speedPid_);
    }

    bool SpeedControllerImpl::IsTrajectoryComplete() const
    {
        return !trajectoryActive_ || trajectoryGenerator_.IsComplete();
    }

    SpeedController::Speed SpeedControllerImpl::GetCurrentSpeed() const
    {
        return currentSpeed_;
    }

    void SpeedControllerImpl::UpdateControl()
    {
        if (!enabled_)
            return;

        // Update current speed from feedback
        currentSpeed_ = speedFeedback_();

        if (trajectoryActive_ && !trajectoryGenerator_.IsComplete())
        {
            // Get trajectory setpoints
            auto motionProfile = trajectoryGenerator_.Update(dt_);

            // Speed control loop
            float speedError = motionProfile.position - currentSpeed_.Value();
            float torqueSetpoint = ProcessPid(speedPid_, speedError, dt_);

            // Add feedforward acceleration from trajectory
            torqueSetpoint += motionProfile.acceleration * 0.1f; // Feedforward gain

            // Convert to motor torque command
            MotorFieldOrientedController::IdAndIqPoint currentSetpoint;
            currentSetpoint.first = 0.0f;                                       // d-axis current (flux control)
            currentSetpoint.second = std::clamp(torqueSetpoint, -20.0f, 20.0f); // q-axis current (torque)

            motorFoc_.SetPoint(currentSetpoint);
        }
        else
        {
            // No active trajectory, maintain current speed
            if (trajectoryActive_)
            {
                trajectoryActive_ = false; // Mark trajectory as complete
            }

            // Simple speed hold (maintain current speed)
            float speedError = 0.0f; // No error, maintain current speed
            float torqueSetpoint = ProcessPid(speedPid_, speedError, dt_);

            MotorFieldOrientedController::IdAndIqPoint holdSetpoint;
            holdSetpoint.first = 0.0f;
            holdSetpoint.second = std::clamp(torqueSetpoint * 0.5f, -5.0f, 5.0f); // Reduced torque for holding

            motorFoc_.SetPoint(holdSetpoint);
        }
    }

    float SpeedControllerImpl::ProcessPid(PidState& pid, float error, float dt)
    {
        // Proportional term
        float proportional = pid.kp * error;

        // Integral term with anti-windup
        pid.integral += error * dt;
        pid.integral = std::clamp(pid.integral, -50.0f, 50.0f); // Anti-windup limit
        float integral = pid.ki * pid.integral;

        // Derivative term
        float derivative = pid.kd * (error - pid.previousError) / dt;
        pid.previousError = error;

        // Calculate output
        pid.output = proportional + integral + derivative;

        return pid.output;
    }

    void SpeedControllerImpl::ResetPid(PidState& pid)
    {
        pid.integral = 0.0f;
        pid.previousError = 0.0f;
        pid.output = 0.0f;
    }
}
