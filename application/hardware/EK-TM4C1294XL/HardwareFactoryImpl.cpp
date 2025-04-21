#include "application/hardware/EK-TM4C1294XL/HardwareFactoryImpl.hpp"
#include "application/hardware/HardwareFactory.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include "infra/util/MemoryRange.hpp"

namespace application
{
    HardwareFactoryImpl::HardwareFactoryImpl(const infra::Function<void()>& onInitialized)
        : onInitialized(onInitialized)
    {}

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
        return infra::MakeRangeFromSingleObject(application::Pins::led1);
    }

    hal::SynchronousQuadratureEncoder& HardwareFactoryImpl::QuadratureEncoder()
    {
        return encoder;
    }

    hal::SynchronousSingleChannelPwm& HardwareFactoryImpl::PwmSinglePhaseOutput()
    {
        return pwm;
    }

    hal::SynchronousThreeChannelsPwm& HardwareFactoryImpl::PwmThreePhaseOutput()
    {
        return pwm;
    }

    uint32_t HardwareFactoryImpl::ControlTimerId() const
    {
        return timerId;
    }

    void HardwareFactoryImpl::PhaseCurrentsReady(const infra::Function<void(MilliVolt phaseA, MilliVolt phaseB, MilliVolt phaseC)>& onDone)
    {
        phaseCurrentsReady = onDone;
    }

    void HardwareFactoryImpl::HallSensorInterrupt(const infra::Function<void(HallState state, Direction direction)>& onDone)
    {
        hallSensorInterrupt = onDone;
    }
}
