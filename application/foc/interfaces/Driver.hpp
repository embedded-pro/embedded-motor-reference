#pragma once

#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/Unit.hpp"

namespace foc
{
    namespace unit
    {
        using Angle = infra::BaseUnit<10>;
        using Radians = Angle::Scale<infra::StaticRational<10, 0>>;
    }

    using Ampere = infra::Quantity<infra::Ampere, float>;
    using Volts = infra::Quantity<infra::Volt, float>;
    using Radians = infra::Quantity<unit::Radians, float>;
    using RadiansPerSecond = infra::Quantity<unit::Radians::Div<infra::Second>, float>;
    using HallState = uint8_t;

    struct PhaseCurrents
    {
        Ampere a;
        Ampere b;
        Ampere c;
    };

    struct PhasePwmDutyCycles
    {
        hal::Percent a;
        hal::Percent b;
        hal::Percent c;
    };

    enum class Direction : uint8_t
    {
        forward,
        reverse,
    };

    class Encoder
    {
    public:
        virtual Radians Read() = 0;
        virtual void Set(Radians value) = 0;
        virtual void SetZero() = 0;
    };

    class HallSensor
    {
    public:
        virtual std::pair<HallState, Direction> Read() const = 0;
    };

    class MotorDriver
    {
    public:
        virtual void PhaseCurrentsReady(hal::Hertz baseFrequency, const infra::Function<void(PhaseCurrents currentPhases)>& onDone) = 0;
        virtual void ThreePhasePwmOutput(const PhasePwmDutyCycles& dutyPhases) = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
    };
}
