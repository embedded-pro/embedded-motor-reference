#ifndef APPLICATION_DC_LOGIC_MOTOR_CONTROLLER_HPP
#define APPLICATION_DC_LOGIC_MOTOR_CONTROLLER_HPP

#include "application/hardware/HardwareFactory.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/Unit.hpp"
#include <optional>

namespace application
{
    namespace unit
    {
        using Revolution = infra::BaseUnit<8>;
        using RevPerSecond = Revolution::Div<infra::Second>::Inverse;
        using RevPerMinute = RevPerSecond::Scale<infra::StaticRational<60, 1>>;
    }

    class MotorController
    {
    public:
        using RevPerMinute = infra::Quantity<unit::RevPerMinute, float>;

        virtual void AutoTune(const infra::Function<void()>& onDone) = 0;
        virtual void SetPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd) = 0;
        virtual void SetSpeed(const RevPerMinute& speed) = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
    };
}

#endif
