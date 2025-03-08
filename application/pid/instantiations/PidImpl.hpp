#ifndef APPLICATION_PID_INSTANTIATIONS_HPP
#define APPLICATION_PID_INSTANTIATIONS_HPP

#include "application/pid/PidWithTimer.hpp"

namespace application
{
    class PidImpl
        : public Pid
    {
    public:
        PidImpl();

        void SetTunnings(Tunnings tunnings) override;
        void SetPoint(float setPoint) override;
        void Enable() override;
        void Disable() override;
        float Process(float measuredProcessVariable) override;

    private:
        controllers::Pid<float> pid;
    };
}

#endif
