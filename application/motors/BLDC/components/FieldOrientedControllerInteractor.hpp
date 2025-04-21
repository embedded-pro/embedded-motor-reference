#pragma once

#include "infra/util/Function.hpp"
#include "infra/util/Unit.hpp"
#include <optional>

namespace application
{
    namespace unit
    {
        using Revolution = infra::BaseUnit<8>;
        using Torque = infra::BaseUnit<9>;

        using RevPerSecond = Revolution::Div<infra::Second>::Inverse;
        using RevPerMinute = RevPerSecond::Scale<infra::StaticRational<60, 1>>;
    }

    class FieldOrientedControllerInteractor
    {
    public:
        using RevPerMinute = infra::Quantity<unit::RevPerMinute, float>;
        using Torque = infra::Quantity<unit::Torque, float>;

        struct PidParameters
        {
            std::optional<float> kp;
            std::optional<float> ki;
            std::optional<float> kd;
        };

        virtual void AutoTune(const infra::Function<void()>& onDone) = 0;
        virtual void SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters) = 0;
        virtual void SetTorque(const Torque& torque) = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
    };
}
