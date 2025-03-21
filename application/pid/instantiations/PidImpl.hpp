#ifndef APPLICATION_PID_INSTANTIATIONS_HPP
#define APPLICATION_PID_INSTANTIATIONS_HPP

#include "application/pid/PidWithTimer.hpp"

namespace application
{
    class PidImpl
        : public Pid
    {
    public:
        PidImpl() = default;

        void SetTunnings(Tunnings tunnings) override;
        void SetPoint(float setPoint) override;
        void Enable() override;
        void Disable() override;
        float Process(float measuredProcessVariable) override;

    private:
        controllers::Pid<float> pid{ Tunnings{ 0.0f, 0.0f, 0.0f }, controllers::Pid<float>::Limits{ 0.0f, 0.9999f } };
    };
}

#endif
