#include "application/hardware/MotorFieldOrientedControllerAdapter.hpp"
#include <chrono>

namespace application
{
    HardwareAdapter::HardwareAdapter(HardwareFactory& hardware)
        : adcMultiChannelCreator{ hardware.AdcMultiChannelCreator(), HardwareFactory::SampleAndHold::shorter }
        , synchronousThreeChannelsPwmCreator{ hardware.SynchronousThreeChannelsPwmCreator(), std::chrono::nanoseconds{ 1000 }, hal::Hertz{ 10000 } }
        , synchronousQuadratureEncoderCreator(hardware.SynchronousQuadratureEncoderCreator())
    {
    }

    void HardwareAdapter::PhaseCurrentsReady(hal::Hertz baseFrequency, const infra::Function<void(foc::PhaseCurrents currentPhases)>& onDone)
    {
        synchronousThreeChannelsPwmCreator->SetBaseFrequency(baseFrequency);
    }

    void HardwareAdapter::ThreePhasePwmOutput(const foc::PhasePwmDutyCycles& dutyPhases)
    {
        // Implementation for handling three-phase PWM output
    }

    void HardwareAdapter::Start()
    {
    }

    void HardwareAdapter::Stop()
    {
        // Implementation for stopping the motor
    }

    foc::Radians HardwareAdapter::Read()
    {
        // Implementation for reading the encoder value
        return foc::Radians{ 0 };
    }

    void HardwareAdapter::Set(foc::Radians value)
    {
        // Implementation for setting the encoder value
    }

    void HardwareAdapter::SetZero()
    {
        // Implementation for setting the encoder value to zero
    }
}
