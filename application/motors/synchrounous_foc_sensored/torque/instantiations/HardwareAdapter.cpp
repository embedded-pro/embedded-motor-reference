#include "application/motors/synchrounous_foc_sensored/torque/instantiations/HardwareAdapter.hpp"
#include <chrono>

namespace application
{
    HardwareAdapter::HardwareAdapter(application::HardwareFactory& hardware)
        : adcMultiChannelCreator{ hardware.AdcMultiChannelCreator(), application::HardwareFactory::SampleAndHold::shorter }
        , synchronousThreeChannelsPwmCreator{ hardware.SynchronousThreeChannelsPwmCreator(), std::chrono::nanoseconds{ 1000 }, hal::Hertz{ 10000 } }
        , synchronousQuadratureEncoderCreator(hardware.SynchronousQuadratureEncoderCreator())
    {
    }

    void HardwareAdapter::PhaseCurrentsReady(hal::Hertz baseFrequency, const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)>& onDone)
    {
        synchronousThreeChannelsPwmCreator->SetBaseFrequency(baseFrequency);
    }

    void HardwareAdapter::ThreePhasePwmOutput(const std::tuple<hal::Percent, hal::Percent, hal::Percent>& dutyPhases)
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

    Degrees HardwareAdapter::Read()
    {
        // Implementation for reading the encoder value
        return Degrees{ 0 };
    }

    void HardwareAdapter::Set(Degrees value)
    {
        // Implementation for setting the encoder value
    }

    void HardwareAdapter::SetZero()
    {
        // Implementation for setting the encoder value to zero
    }
}
