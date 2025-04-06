#ifndef APPLICATION_BLDC_LOGIC_MOTOR_CONTROLLER_HPP
#define APPLICATION_BLDC_LOGIC_MOTOR_CONTROLLER_HPP

#include "application/hardware/HardwareFactory.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/Unit.hpp"
#include <optional>

namespace application
{
    namespace unit
    {
        using Revolution = infra::BaseUnit<8>;
        using Torque = infra::BaseUnit<9>;
        using Angle = infra::BaseUnit<10>;

        using RevPerSecond = Revolution::Div<infra::Second>::Inverse;
        using RevPerMinute = RevPerSecond::Scale<infra::StaticRational<60, 1>>;
        using Degrees = Angle::Scale<infra::StaticRational<360, 1>>;
    }

    class FocController
    {
    public:
        using RevPerMinute = infra::Quantity<unit::RevPerMinute, float>;
        using Torque = infra::Quantity<unit::Torque, float>;
        using Degrees = infra::Quantity<unit::Degrees, float>;

        struct PidFocParameters
        {
            std::optional<float> kp;
            std::optional<float> ki;
            std::optional<float> kd;
        };

        virtual void AutoTune(const infra::Function<void()>& onDone) = 0;

        virtual void SetDQPidParameters(const std::pair<PidFocParameters, PidFocParameters>& pidDAndQParameters) = 0;
        virtual void SetSpeedPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd) = 0;
        virtual void SetPositionPidParameters(std::optional<float> kp, std::optional<float> ki, std::optional<float> kd) = 0;

        virtual void SetTorque(const Torque& torque) = 0;
        virtual void SetSpeed(const RevPerMinute& speed) = 0;
        virtual void SetPosition(const Degrees& position) = 0;

        virtual void Start() = 0;
        virtual void Stop() = 0;

        virtual void EnableTorqueControl() = 0;
        virtual void EnableSpeedControl() = 0;
        virtual void EnablePositionControl() = 0;
    };
}

#endif
