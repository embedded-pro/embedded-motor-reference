#pragma once

#include "application/hardware/HardwareFactory.hpp"
#include "numerical/controllers/interfaces/PidDriver.hpp"

namespace application
{
    class PidDriverImpl
        : public controllers::PidDriver<float>
    {
    public:
        explicit PidDriverImpl(application::HardwareFactory& hardware);

        // Implementation of PidDriver
        void Read(const infra::Function<void(float)>& onDone) override;
        void ControlAction(float) override;
        void Start(std::chrono::system_clock::duration sampleTime) override;
        void Stop() override;

        std::chrono::system_clock::duration SampleTime() const;
    };
}
