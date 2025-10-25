#include "application/hardware/ti/implementation/HardwareFactoryImpl.hpp"
#include "infra/util/MemoryRange.hpp"

namespace application
{
    extern "C" uint32_t SystemCoreClock;

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

    hal::PerformanceTracker& HardwareFactoryImpl::PerformanceTimer()
    {
        return *this;
    }

    hal::Hertz HardwareFactoryImpl::BaseFrequency() const
    {
        return hal::Hertz(SystemCoreClock);
    }

    foc::Volts HardwareFactoryImpl::PowerSupplyVoltage()
    {
        return foc::Volts(24.0f);
    }

    void HardwareFactoryImpl::Start()
    {
        return peripherals->cortex.dataWatchPointAndTrace.Start();
    }

    uint32_t HardwareFactoryImpl::ElapsedCycles()
    {
        return peripherals->cortex.dataWatchPointAndTrace.Stop();
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
