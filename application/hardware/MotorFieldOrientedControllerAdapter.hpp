#pragma once

#include "application/foc/interfaces/Driver.hpp"
#include "application/hardware/HardwareFactory.hpp"

namespace application
{
    class HardwareAdapter
        : public foc::MotorDriver
        , public foc::Encoder
    {
    public:
        explicit HardwareAdapter(HardwareFactory& hardware);

        // Implementation of MotorDriver
        void PhaseCurrentsReady(hal::Hertz baseFrequency, const infra::Function<void(std::tuple<foc::Ampere, foc::Ampere, foc::Ampere> currentPhases)>& onDone) override;
        void ThreePhasePwmOutput(const std::tuple<hal::Percent, hal::Percent, hal::Percent>& dutyPhases) override;
        void Start() override;
        void Stop() override;

        // Implementation of Encoder
        foc::Radians Read() override;
        void Set(foc::Radians value) override;
        void SetZero() override;

    private:
        infra::ProxyCreator<hal::AdcMultiChannel, void(HardwareFactory::SampleAndHold)> adcMultiChannelCreator;
        infra::ProxyCreator<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)> synchronousThreeChannelsPwmCreator;
        infra::ProxyCreator<hal::SynchronousQuadratureEncoder, void()> synchronousQuadratureEncoderCreator;
    };
}
