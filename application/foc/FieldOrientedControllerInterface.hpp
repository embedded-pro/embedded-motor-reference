#pragma once

#include "infra/util/Function.hpp"
#include "infra/util/Unit.hpp"
#include <optional>

namespace application
{
    namespace unit
    {
        using Angle = infra::BaseUnit<10>;
        using Degrees = Angle::Scale<infra::StaticRational<360, 0>>;
    }

    using MilliVolt = infra::Quantity<infra::MilliVolt, float>;
    using Percent = infra::Quantity<infra::Percent, float>;
    using Degrees = infra::Quantity<unit::Degrees, float>;
    using HallState = uint8_t;

    enum class Direction : uint8_t
    {
        forward,
        reverse,
    };

    class FieldOrientedControllerInterface
    {
    public:
        virtual void PhaseCurrentsReady(const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)>& onDone) = 0;
        virtual void HallSensorInterrupt(const infra::Function<void(HallState state, Direction direction)>& onDone) = 0;
        virtual void ThreePhasePwmOutput(const std::tuple<Percent, Percent, Percent>& dutyPhases) = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
    };
}
