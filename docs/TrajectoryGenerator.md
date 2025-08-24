# Trajectory Generator for Motor Control

This document describes the trajectory generator implementation for smooth motor control in PMSM and BLDC motor applications.

## Overview

The trajectory generator creates smooth motion profiles with controlled acceleration and deceleration phases, preventing abrupt setpoint changes that could cause mechanical stress, vibrations, or control instability.

## Features

### Core Trajectory Generator (`TrajectoryGenerator.hpp`)

- **Trapezoidal Profile**: For long distances requiring constant velocity phase
- **Triangular Profile**: For short distances where maximum velocity is not reached
- **Configurable Constraints**: Maximum velocity, acceleration, and deceleration limits
- **Template Design**: Supports both floating-point and fixed-point types
- **Real-time Execution**: Efficient computation suitable for embedded systems

### Motion Controllers

#### Position Controller (`PositionController.hpp`)
- **Cascaded Control**: Position → Velocity → Torque control loops
- **Trajectory Integration**: Uses trajectory generator for smooth position transitions
- **Configurable PID**: Separate tuning for position and velocity loops
- **Safety Features**: Output limiting and anti-windup protection

#### Speed Controller (`SpeedController.hpp`)
- **Direct Speed Control**: Single-loop speed control with trajectory planning
- **Acceleration Limiting**: Smooth speed transitions without velocity limits
- **Feedforward Control**: Includes acceleration feedforward for improved tracking
- **Torque Output**: Direct torque commands to field-oriented controller

## Usage Examples

### Basic Trajectory Generation

```cpp
#include "numerical/controllers/TrajectoryGenerator.hpp"

// Define constraints
controllers::TrajectoryGenerator<float>::Constraints constraints;
constraints.maxVelocity = 10.0f;      // 10 units/second
constraints.maxAcceleration = 5.0f;   // 5 units/second²
constraints.maxDeceleration = 5.0f;   // 5 units/second²

// Create trajectory generator
controllers::TrajectoryGenerator<float> generator(constraints);

// Set initial conditions
generator.SetInitialConditions(0.0f, 0.0f); // position=0, velocity=0

// Generate trajectory to target
generator.SetTarget(25.0f);

// Execute trajectory
float dt = 0.01f; // 10ms time step
while (!generator.IsComplete()) {
    auto profile = generator.Update(dt);
    
    // Use profile.position, profile.velocity, profile.acceleration
    // for your control loops
    
    std::cout << "Position: " << profile.position 
              << ", Velocity: " << profile.velocity 
              << ", Acceleration: " << profile.acceleration << std::endl;
}
```

### Position Control Integration

```cpp
#include "application/motors/synchronous_foc_sensored/position/components/PositionController.hpp"

// Setup position controller
PositionControllerImpl positionController(motorFOC, positionFeedback, hal::Hertz{1000});

// Configure trajectory constraints
PositionController::TrajectoryConstraints constraints;
constraints.maxVelocity = application::PositionController::Velocity{360.0f}; // 360°/s
constraints.maxAcceleration = application::PositionController::Acceleration{180.0f}; // 180°/s²
constraints.maxDeceleration = application::PositionController::Acceleration{180.0f}; // 180°/s²

positionController.SetTrajectoryConstraints(constraints);

// Configure PID parameters
PositionController::PidParameters posPid{1.0f, 0.1f, 0.05f}; // kp, ki, kd
PositionController::PidParameters velPid{0.5f, 0.1f, 0.01f};

positionController.SetPositionPidParameters(posPid);
positionController.SetVelocityPidParameters(velPid);

// Start controller and move to target
positionController.Start();
positionController.SetPosition(application::PositionController::Position{90.0f}); // Move to 90°

// Check completion
while (!positionController.IsTrajectoryComplete()) {
    // Controller runs in background
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
```

### Speed Control Integration

```cpp
#include "application/motors/synchronous_foc_sensored/speed/components/SpeedController.hpp"

// Setup speed controller
SpeedControllerImpl speedController(motorFOC, speedFeedback, hal::Hertz{1000});

// Configure trajectory constraints
SpeedController::TrajectoryConstraints constraints;
constraints.maxAcceleration = application::SpeedController::Acceleration{100.0f}; // 100 units/s²
constraints.maxDeceleration = application::SpeedController::Acceleration{150.0f}; // 150 units/s²

speedController.SetTrajectoryConstraints(constraints);

// Configure PID parameters
SpeedController::PidParameters speedPid{1.0f, 0.2f, 0.05f}; // kp, ki, kd
speedController.SetSpeedPidParameters(speedPid);

// Start controller and set target speed
speedController.Start();
speedController.SetSpeed(application::SpeedController::Speed{1000.0f}); // Target speed

// Monitor trajectory completion
while (!speedController.IsTrajectoryComplete()) {
    auto currentSpeed = speedController.GetCurrentSpeed();
    std::cout << "Current speed: " << currentSpeed.Value() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
```

## Integration with Motor Control System

### Architecture

```
Application Layer:
├── Position Controller (PositionControllerImpl)
│   ├── Trajectory Generator (position setpoints)
│   ├── Position PID Controller (position → velocity)
│   └── Velocity PID Controller (velocity → torque)
│
├── Speed Controller (SpeedControllerImpl)
│   ├── Trajectory Generator (speed setpoints)
│   └── Speed PID Controller (speed → torque)
│
└── Motor FOC Layer (MotorFieldOrientedController)
    ├── d-axis Current Controller (flux control)
    ├── q-axis Current Controller (torque control)
    └── Field Oriented Control (FOC algorithm)
```

### Control Flow

1. **Setpoint Generation**: Trajectory generator creates smooth setpoint profiles
2. **Control Loops**: PID controllers process setpoints and feedback
3. **Torque Commands**: Controllers output torque commands (q-axis current)
4. **FOC Execution**: Motor FOC converts torque to three-phase PWM outputs

## Benefits

### Smooth Motion
- Eliminates step changes in setpoints
- Reduces mechanical stress and vibrations
- Improves system stability and performance

### Configurable Profiles
- Adjustable acceleration and velocity limits
- Automatic profile selection (triangular vs trapezoidal)
- Suitable for different application requirements

### Real-time Performance
- Efficient algorithms for embedded systems
- Template-based implementation for compile-time optimization
- Minimal memory footprint

## Configuration Guidelines

### Trajectory Constraints

**Position Control:**
- `maxVelocity`: Should be within mechanical limits of the system
- `maxAcceleration`: Balance between speed and smoothness
- `maxDeceleration`: Often higher than acceleration for safety

**Speed Control:**
- `maxAcceleration`: Determines how quickly speed changes occur
- `maxDeceleration`: Important for emergency stops

### PID Tuning

**Position Controller:**
- Position PID: Higher proportional gain for stiffness
- Velocity PID: Lower gains to avoid oscillations

**Speed Controller:**
- Single PID loop with moderate gains
- Integral term important for steady-state accuracy

## Safety Considerations

1. **Output Limiting**: All controllers include torque output limits
2. **Anti-windup**: Integral terms are limited to prevent windup
3. **Graceful Degradation**: Controllers handle sensor failures appropriately
4. **Emergency Stops**: Immediate trajectory cancellation capability

## Testing

Comprehensive test suites are provided:

- `TestTrajectoryGenerator.cpp`: Core trajectory generation tests
- Unit tests for position and speed controllers
- Integration tests with motor FOC system

Run tests with:
```bash
cd build
ctest -R trajectory
```

## Future Enhancements

1. **Advanced Profiles**: S-curve (jerk-limited) trajectories
2. **Multi-axis Coordination**: Synchronized multi-motor control
3. **Adaptive Control**: Auto-tuning based on system response
4. **Path Planning**: Complex path following capabilities
