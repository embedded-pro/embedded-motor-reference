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

    infra::CreatorBase<hal::SynchronousQuadratureEncoder, void()>& HardwareFactoryImpl::SynchronousQuadratureEncoderCreator()
    {
        return peripherals->encoderImpl.synchronousQuadratureEncoderCreator;
    }
}
