#include "application/hardware/st/implementation/HardwareFactoryImpl.hpp"
#include "infra/util/MemoryRange.hpp"
#include DEVICE_HEADER

unsigned int hse_value = 8'000'000;

namespace application
{
    HardwareFactoryImpl::HardwareFactoryImpl(const infra::Function<void()>& onInitialized)
        : onInitialized(onInitialized)
    {
        HAL_Init();
    }

    void HardwareFactoryImpl::Run()
    {
    }

    services::Tracer& HardwareFactoryImpl::Tracer()
    {
        return terminalAndTracer.tracer;
    }

    services::TerminalWithCommands& HardwareFactoryImpl::Terminal()
    {
        return terminalAndTracer.terminal;
    }

    infra::MemoryRange<hal::GpioPin> HardwareFactoryImpl::Leds()
    {
        return infra::MakeRangeFromSingleObject(pin);
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
