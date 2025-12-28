#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "source/foc/interfaces/Driver.hpp"
#include "source/hardware/HardwareFactory.hpp"

namespace application
{
    class HardwareAdapter
        : public foc::MotorDriver
        , public foc::Encoder
    {
    public:
        explicit HardwareAdapter(HardwareFactory& hardware);

        // Implementation of MotorDriver
        OPTIMIZE_FOR_SPEED
        void PhaseCurrentsReady(hal::Hertz baseFrequency, const infra::Function<void(foc::PhaseCurrents currentPhases)>& onDone) override;
        OPTIMIZE_FOR_SPEED
        void ThreePhasePwmOutput(const foc::PhasePwmDutyCycles& dutyPhases) override;
        void Start() override;
        void Stop() override;

        // Implementation of Encoder
        OPTIMIZE_FOR_SPEED
        foc::Radians Read() override;
        void Set(foc::Radians value) override;
        void SetZero() override;

    private:
        infra::ProxyCreator<AdcMultiChannelDecorator, void(HardwareFactory::SampleAndHold)> adcMultiChannelCreator;
        infra::ProxyCreator<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)> synchronousThreeChannelsPwmCreator;
        infra::ProxyCreator<QuadratureEncoderDecorator, void()> synchronousQuadratureEncoderCreator;
        infra::Function<void(foc::PhaseCurrents currentPhases)> onPhaseCurrentsReady;
        foc::Radians encoderOffset{ 0.0f };
    };
}
