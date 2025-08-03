#pragma once

#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/Unit.hpp"

namespace application
{
    namespace unit
    {
        using Angle = infra::BaseUnit<10>;
        using Degrees = Angle::Scale<infra::StaticRational<360, 0>>;
    }

    using MilliVolt = infra::Quantity<infra::MilliVolt, float>;
    using Degrees = infra::Quantity<unit::Degrees, float>;
    using HallState = uint8_t;

    enum class Direction : uint8_t
    {
        forward,
        reverse,
    };

    class Encoder
    {
    public:
        virtual Degrees Read() = 0;
        virtual void Set(Degrees value) = 0;
        virtual void SetZero() = 0;
    };

    class HallSensor
    {
    public:
        virtual std::pair<HallState, Direction> Read() const = 0;
    };

    class MotorFieldOrientedControllerInterface
    {
    public:
        virtual void PhaseCurrentsReady(hal::Hertz baseFrequency, const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)>& onDone) = 0;
        virtual void ThreePhasePwmOutput(const std::tuple<hal::Percent, hal::Percent, hal::Percent>& dutyPhases) = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
    };
}
