#pragma once

#include "application/pid/PidInterface.hpp"
#include "infra/timer/Timer.hpp"
#include "numerical/controllers/Pid.hpp"

namespace application
{
    class Pid
    {
    public:
        using Tunnings = controllers::Pid<float>::Tunnings;

        virtual void SetTunnings(Tunnings tunnings) = 0;
        virtual void SetPoint(float setPoint) = 0;
        virtual void Enable() = 0;
        virtual void Disable() = 0;
        virtual float Process(float measuredProcessVariable) = 0;
    };

    class PidAsync
    {
    public:
        PidAsync(PidInterface& interface, Pid& pid);

        void SetTunnings(Pid::Tunnings tunnings);
        void SetPoint(float setPoint);
        void Enable();
        void Disable();
        bool IsRunning() const;

    private:
        PidInterface& interface;
        Pid& pid;
        infra::Duration sampleTime;
        bool running = false;
    };
}
