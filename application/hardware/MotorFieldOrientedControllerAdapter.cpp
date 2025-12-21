#include "application/hardware/MotorFieldOrientedControllerAdapter.hpp"
#include <chrono>

namespace application
{
    HardwareAdapter::HardwareAdapter(HardwareFactory& hardware)
        : adcMultiChannelCreator{ hardware.AdcMultiChannelCreator(), HardwareFactory::SampleAndHold::shorter }
        , synchronousThreeChannelsPwmCreator{ hardware.SynchronousThreeChannelsPwmCreator(), std::chrono::nanoseconds{ 500 }, hal::Hertz{ 10000 } }
        , synchronousQuadratureEncoderCreator(hardware.SynchronousQuadratureEncoderCreator())
    {
    }

    void HardwareAdapter::PhaseCurrentsReady(hal::Hertz baseFrequency, const infra::Function<void(foc::PhaseCurrents currentPhases)>& onDone)
    {
        onPhaseCurrentsReady = onDone;
        synchronousThreeChannelsPwmCreator->SetBaseFrequency(baseFrequency);
        adcMultiChannelCreator->Measure([this](auto phaseA, auto phaseB, auto phaseC)
            {
                onPhaseCurrentsReady(foc::PhaseCurrents{ phaseA, phaseB, phaseC });
            });
    }

    void HardwareAdapter::ThreePhasePwmOutput(const foc::PhasePwmDutyCycles& dutyPhases)
    {
        synchronousThreeChannelsPwmCreator->Start(dutyPhases.a, dutyPhases.b, dutyPhases.c);
    }

    void HardwareAdapter::Start()
    {
        synchronousThreeChannelsPwmCreator->Start(hal::Percent{ 1 }, hal::Percent{ 1 }, hal::Percent{ 1 });
    }

    void HardwareAdapter::Stop()
    {
        synchronousThreeChannelsPwmCreator->Stop();
    }

    foc::Radians HardwareAdapter::Read()
    {
        return synchronousQuadratureEncoderCreator->Read() - encoderOffset;
    }

    void HardwareAdapter::Set(foc::Radians value)
    {
        encoderOffset = synchronousQuadratureEncoderCreator->Read() - value;
    }

    void HardwareAdapter::SetZero()
    {
        encoderOffset = synchronousQuadratureEncoderCreator->Read();
    }
}
