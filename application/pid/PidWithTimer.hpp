#ifndef APPLICATION_PID_WITH_TIMER_HPP
#define APPLICATION_PID_WITH_TIMER_HPP

#include "infra/timer/Timer.hpp"
#include "numerical/controllers/Pid.hpp"

namespace application
{
    class Input
    {
    public:
        virtual float Read() = 0;
    };

    class Output
    {
    public:
        virtual void Update(float) = 0;
        virtual void Disable() = 0;
    };

    class PidWithTimer
        : private controllers::Pid<float>
    {
    public:
        using Tunnings = controllers::Pid<float>::Tunnings;

        PidWithTimer(Input& input, Output& output, infra::Duration sampleTime, const uint32_t& timerId);

        void SetTunnings(Tunnings tunnings);
        void SetPoint(float setPoint);
        void Enable();
        void Disable();
        bool IsRunning() const;

    private:
        Input& input;
        Output& output;
        infra::TimerRepeating timer;
        infra::Duration sampleTime;
    };
}

#endif
