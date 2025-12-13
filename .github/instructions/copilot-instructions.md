# GitHub Copilot Instructions for e-foc

## Project Overview

This is an embedded motor control application with strict performance and memory constraints. The project is organized into modular components with clear separation of concerns.

## Repository Structure

- **embedded-infra-lib/**: Submodule containing infrastructure foundations and utilities for embedded systems
- **hal/**: Hardware Abstraction Layer for different microcontroller families (ST, TI)
- **application/**: Application-specific code for motor control, FOC algorithms, and hardware adapters
- **numerical-toolbox/**: Generic algorithms (filters, PID controllers, etc.) that can be reused across projects
- **simulator/**: C++ simulator that plots curves for evaluating motors based on mathematical models

## Critical Constraints

### Memory Management

- **NO HEAP ALLOCATION**: Avoid `new`, `delete`, `malloc`, `free`, and `std::make_unique/std::make_shared` at all costs
- **NO DYNAMIC CONTAINERS**: Replace standard library containers that use dynamic allocation:
  - Use `infra::BoundedVector` instead of `std::vector`
  - Use `infra::BoundedString` instead of `std::string`
  - Use `infra::BoundedDeque` instead of `std::deque`
  - Use `infra::BoundedList` instead of `std::list`
- **STATIC ALLOCATION**: All memory must be allocated at compile-time or on the stack
- **AVOID RECURSION**: Stack usage must be predictable and minimal

### Performance Requirements

- **REAL-TIME CONSTRAINTS**: Code must execute deterministically within strict timing requirements
- **MINIMIZE BRANCHING**: Reduce conditional statements in critical paths (e.g., FOC control loops)
- **AVOID VIRTUAL CALLS IN ISR**: Virtual function calls add overhead; avoid in interrupt service routines
- **INLINE CRITICAL CODE**: Use `inline` for small, frequently-called functions
- **CONST CORRECTNESS**: Mark all non-mutating methods as `const` for compiler optimization
- **PREFER CONSTEXPR**: Use `constexpr` for compile-time calculations when possible

### Memory Consumption

- **MINIMIZE FOOTPRINT**: Every byte counts in embedded systems
- **USE FIXED-SIZE TYPES**: Prefer `uint8_t`, `int32_t`, etc., over `int` for predictable sizing
- **PACK STRUCTURES**: Use `#pragma pack` or compiler attributes when appropriate
- **AVOID COPYING**: Use references and move semantics to prevent unnecessary copies
- **MEASURE SIZE**: Be aware of sizeof() for all data structures

## Code Organization Guidelines

### Application Code (`application/`)

Place code here when it is:
- Specific to motor control application logic
- Hardware integration and adapters (e.g., `MotorFieldOrientedControllerAdapter`)
- FOC implementations and instantiations
- Application-level state machines and coordination

### Numerical Toolbox (`numerical-toolbox/`)

Place code here when it is:
- Generic mathematical algorithms (PID, filters, integrators, etc.)
- Reusable across multiple embedded projects
- Not specific to motor control
- Hardware-agnostic

### Hardware Abstraction Layer (`hal/`)

- Vendor-specific implementations (ST, TI)
- Low-level peripheral drivers
- Should be kept minimal; prefer using embedded-infra-lib HAL when possible

## Coding Style and Patterns

### Dependency Injection

- Use constructor injection for dependencies
- Pass interfaces, not concrete implementations
- Example:
  ```cpp
  class MotorController {
  public:
      MotorController(IPwmController& pwm, ICurrentSensor& sensor)
          : pwm(pwm), sensor(sensor) {}
  private:
      IPwmController& pwm;
      ICurrentSensor& sensor;
  };
  ```

### Error Handling

- Use `infra::Optional` for functions that may not return a value
- Return error codes or status enums, not exceptions
- Assert preconditions in debug builds with `assert()` or `really_assert()`

### Testing

- Write unit tests using GoogleTest
- Mock hardware interfaces for testability
- Aim for high code coverage, especially in control algorithms

## Common Patterns

### Instead of this (BAD):
```cpp
std::vector<float> samples;
samples.push_back(value);

std::string message = "Error: " + errorCode;

auto ptr = std::make_unique<Motor>();
```

### Do this (GOOD):
```cpp
infra::BoundedVector<float>::WithMaxSize<100> samples;
samples.push_back(value);

infra::BoundedString::WithStorage<64> message;
message = "Error: ";

Motor motor; // Stack allocation
```

## Additional Guidelines

- **RAII**: Use Resource Acquisition Is Initialization for resource management
- **INTERFACES**: Define interfaces (pure virtual classes) for testability and flexibility
- **NAMESPACE**: Use appropriate namespaces (e.g., `motor::foc`, `motor::hardware`)
- **SELF-DOCUMENTING CODE**: Avoid comments; write clear, self-explanatory code with descriptive names
- **UNITS**: Be explicit about units (radians, Hz, A, V) in variable names or comments
- **FIXED-POINT**: Consider fixed-point arithmetic for performance-critical math on microcontrollers without FPU
- **CODE FORMATTING**: Strictly adhere to the rules defined in `.clang-format` for consistent code style

## Build System

- CMake-based build with presets for different targets
- Support for host (simulation) and embedded targets (EK-TM4C1294XL, STM32F407G-DISC1)
- Separate build configurations for Debug, Release, RelWithDebInfo

## Version Control

- Keep commits atomic and focused
- Write clear commit messages
- Update CHANGELOG.md according to release-please conventions
