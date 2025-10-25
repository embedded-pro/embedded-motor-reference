#pragma once

#include "application/pid/PidInterface.hpp"
#include "numerical/controllers/Pid.hpp"

namespace application
{
    class PidImpl
        : public Pid
    {
    public:
        explicit PidImpl(PidInterface& interface);

        void SetTunings(Tunings tunings) override;
        void SetPoint(float setPoint) override;
        void Enable() override;
        void Disable() override;

    private:
        PidInterface& interface;
        infra::Duration sampleTime;
        controllers::Pid<float> pid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f } };
    };
}
