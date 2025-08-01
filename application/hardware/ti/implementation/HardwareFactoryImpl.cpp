#include "application/hardware/ti/implementation/HardwareFactoryImpl.hpp"
#include "infra/util/MemoryRange.hpp"

namespace application
{
    HardwareFactoryImpl::HardwareFactoryImpl(const infra::Function<void()>& onInitialized)
        : onInitialized(onInitialized)
    {
        application::Clocks::Initialize();
        peripherals.emplace();
        this->onInitialized();
    }

    void HardwareFactoryImpl::Run()
    {
        peripherals->cortex.eventDispatcher.Run();
    }

    services::Tracer& HardwareFactoryImpl::Tracer()
    {
        return peripherals->terminalAndTracer.tracer;
    }

    services::TerminalWithCommands& HardwareFactoryImpl::Terminal()
    {
        return peripherals->terminalAndTracer.terminal;
    }

    infra::MemoryRange<hal::GpioPin> HardwareFactoryImpl::Leds()
    {
        return infra::MakeRangeFromSingleObject(application::Pins::led1);
    }

    infra::CreatorBase<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)>& HardwareFactoryImpl::SynchronousThreeChannelsPwmCreator()
    {
        return peripherals->motorFieldOrientedController.pwmBrushless;
    }

    infra::CreatorBase<hal::AdcMultiChannel, void(HardwareFactory::SampleAndHold)>& HardwareFactoryImpl::AdcMultiChannelCreator()
    {
        return peripherals->motorFieldOrientedController.adcCurrentPhases;
    }

    PidInterface& HardwareFactoryImpl::MotorPid()
    {
        return peripherals->motorPid;
    }

    MotorFieldOrientedControllerInterface& HardwareFactoryImpl::MotorFieldOrientedController()
    {
        return peripherals->motorFieldOrientedController;
    }

    Encoder& HardwareFactoryImpl::MotorPosition()
    {
        return peripherals->encoderImpl;
    }

    void HardwareFactoryImpl::PidInterfaceImpl::Read(const infra::Function<void(float)>& onDone)
    {
    }

    void HardwareFactoryImpl::PidInterfaceImpl::ControlAction(float)
    {
    }

    void HardwareFactoryImpl::PidInterfaceImpl::Start(infra::Duration sampleTime)
    {
    }

    void HardwareFactoryImpl::PidInterfaceImpl::Stop()
    {
    }

    void HardwareFactoryImpl::MotorFieldOrientedControllerInterfaceImpl::PhaseCurrentsReady(const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt>)>& onDone)
    {
        phaseCurrentsReady = onDone;
        pwmBrushless->SetBaseFrequency(hal::Hertz(10000.0f));
        adcCurrentPhases->Measure([this](auto samples)
            {
                auto voltagePhases = std::make_tuple(MilliVolt{ static_cast<float>(samples[0]) }, MilliVolt{ static_cast<float>(samples[1]) }, MilliVolt{ static_cast<float>(samples[2]) });
                this->phaseCurrentsReady(voltagePhases);
            });
    }

    void HardwareFactoryImpl::MotorFieldOrientedControllerInterfaceImpl::ThreePhasePwmOutput(const std::tuple<hal::Percent, hal::Percent, hal::Percent>& dutyPhases)
    {
        pwmBrushless->Start(std::get<0>(dutyPhases), std::get<1>(dutyPhases), std::get<2>(dutyPhases));
    }

    void HardwareFactoryImpl::MotorFieldOrientedControllerInterfaceImpl::Start()
    {
    }

    void HardwareFactoryImpl::MotorFieldOrientedControllerInterfaceImpl::Stop()
    {
        pwmBrushless->Stop();
    }

    Degrees HardwareFactoryImpl::EncoderImpl::Read()
    {
        auto resolution = static_cast<float>(this->encoder.Resolution());
        auto position = static_cast<float>(this->encoder.Position());
        return Degrees((position / resolution) * 360.0f);
    }

    void HardwareFactoryImpl::EncoderImpl::Set(Degrees)
    {
    }

    void HardwareFactoryImpl::EncoderImpl::SetZero()
    {
    }
}
