#pragma once

#include "application/pid/PidInterface.hpp"
#include "numerical/controllers/Pid.hpp"

namespace application
{
    class PidImpl
        : public Pid
    {
    public:
        PidImpl(PidInterface& interface);

        void SetTunnings(Tunnings tunnings) override;
        void SetPoint(float setPoint) override;
        void Enable() override;
        void Disable() override;

    private:
        PidInterface& interface;
        infra::Duration sampleTime;
        controllers::Pid<float> pid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f }, true };
    };
}
