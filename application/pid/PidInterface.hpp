#pragma once

#include "infra/timer/Timer.hpp"
#include "infra/util/Function.hpp"

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
}
