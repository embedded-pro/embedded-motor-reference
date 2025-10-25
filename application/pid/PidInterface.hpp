#pragma once

#include "infra/timer/Timer.hpp"
#include "infra/util/Function.hpp"
#include "numerical/controllers/Pid.hpp"

namespace application
{
    class PidInterface
    {
    public:
        virtual void Read(const infra::Function<void(float)>& onDone) = 0;
        virtual void ControlAction(float) = 0;
        virtual void Start(infra::Duration sampleTime) = 0;
        virtual void Stop() = 0;
    };

    class Pid
    {
    public:
        using Tunings = controllers::Pid<float>::Tunings;

        virtual void SetTunings(Tunings tunings) = 0;
        virtual void SetPoint(float setPoint) = 0;
        virtual void Enable() = 0;
        virtual void Disable() = 0;
    };
}
