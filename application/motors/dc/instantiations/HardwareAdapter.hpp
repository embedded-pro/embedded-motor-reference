#pragma once

#include "application/hardware/HardwareFactory.hpp"
#include "application/pid/PidInterface.hpp"

namespace application
{
    class HardwareAdapter
        : public PidInterface
    {
    public:
        explicit HardwareAdapter(application::HardwareFactory& hardware);

        // Implementation of PidInterface
        void Read(const infra::Function<void(float)>& onDone) override;
        void ControlAction(float) override;
        void Start(infra::Duration sampleTime) override;
        void Stop() override;
    };
}
