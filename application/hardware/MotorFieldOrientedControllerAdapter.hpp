#pragma once

#include "application/foc/MotorFieldOrientedControllerInterface.hpp"
#include "application/hardware/HardwareFactory.hpp"

namespace application
{
    class HardwareAdapter
        : public MotorFieldOrientedControllerInterface
        , public Encoder
    {
    public:
        explicit HardwareAdapter(application::HardwareFactory& hardware);

        // Implementation of MotorFieldOrientedControllerInterface
        void PhaseCurrentsReady(hal::Hertz baseFrequency, const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)>& onDone) override;
        void ThreePhasePwmOutput(const std::tuple<hal::Percent, hal::Percent, hal::Percent>& dutyPhases) override;
        void Start() override;
        void Stop() override;

        // Implementation of Encoder
        Degrees Read() override;
        void Set(Degrees value) override;
        void SetZero() override;

    private:
        infra::ProxyCreator<hal::AdcMultiChannel, void(HardwareFactory::SampleAndHold)> adcMultiChannelCreator;
        infra::ProxyCreator<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)> synchronousThreeChannelsPwmCreator;
        infra::ProxyCreator<hal::SynchronousQuadratureEncoder, void()> synchronousQuadratureEncoderCreator;
    };
}
