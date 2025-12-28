#pragma once

#include "infra/util/Function.hpp"
#include "source/foc/interfaces/Units.hpp"
#include <optional>

namespace services
{
    class FocInteractor
    {
    public:
        struct PidParameters
        {
            std::optional<float> kp;
            std::optional<float> ki;
            std::optional<float> kd;
        };

        virtual void AutoTune(const infra::Function<void()>& onDone) = 0;
        virtual void SetDQPidParameters(const std::pair<PidParameters, PidParameters>& pidDAndQParameters) = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
    };

    class FocTorqueInteractor
        : public FocInteractor
    {
    public:
        virtual void SetTorque(const foc::Nm& torque) = 0;
    };

    class FocSpeedInteractor
        : public FocInteractor
    {
    public:
        virtual void SetSpeed(const foc::RevPerMinute& speed) = 0;
        virtual void SetSpeed(const foc::RadiansPerSecond& speed) = 0;
        virtual void SetSpeedPidParameters(const PidParameters& pidParameters) = 0;
    };

    class FocPositionInteractor
        : public FocInteractor
    {
    public:
        virtual void SetPosition(const foc::Radians& position) = 0;
        virtual void SetPositionPidParameters(const PidParameters& pidParameters) = 0;
    };
}
