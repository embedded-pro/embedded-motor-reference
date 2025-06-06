#include "application/hardware/Host/implementation/HardwareFactoryImpl.hpp"
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
        return infra::MakeRangeFromSingleObject(pin);
    }

    PidInterface& HardwareFactoryImpl::MotorPid()
    {
        return *this;
    }

    MotorFieldOrientedControllerInterface& HardwareFactoryImpl::MotorFieldOrientedController()
    {
        return *this;
    }

    Encoder& HardwareFactoryImpl::MotorPosition()
    {
        return *this;
    }

    void HardwareFactoryImpl::PhaseCurrentsReady(const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)>& onDone)
    {
    }

    void HardwareFactoryImpl::ThreePhasePwmOutput(const std::tuple<hal::Percent, hal::Percent, hal::Percent>& dutyPhases)
    {
    }

    Degrees HardwareFactoryImpl::Read()
    {
        return Degrees(0.0f);
    }

    void HardwareFactoryImpl::SetZero()
    {
    }

    void HardwareFactoryImpl::Set(Degrees position)
    {
    }

    void HardwareFactoryImpl::Start()
    {
    }

    void HardwareFactoryImpl::Stop()
    {
    }

    void HardwareFactoryImpl::Read(const infra::Function<void(float)>& onDone)
    {
    }

    void HardwareFactoryImpl::ControlAction(float)
    {
    }

    void HardwareFactoryImpl::Start(infra::Duration sampleTime)
    {
    }

    void HardwareFactoryImpl::SerialCommunicationStub::SendData(infra::ConstByteRange data, infra::Function<void()> actionOnCompletion)
    {}

    void HardwareFactoryImpl::SerialCommunicationStub::ReceiveData(infra::Function<void(infra::ConstByteRange data)> dataReceived)
    {}

    bool HardwareFactoryImpl::GpioPinStub::Get() const
    {
        return false;
    }

    void HardwareFactoryImpl::GpioPinStub::Set(bool value)
    {}

    bool HardwareFactoryImpl::GpioPinStub::GetOutputLatch() const
    {
        return false;
    }

    void HardwareFactoryImpl::GpioPinStub::SetAsInput()
    {}

    bool HardwareFactoryImpl::GpioPinStub::IsInput() const
    {
        return false;
    }

    void HardwareFactoryImpl::GpioPinStub::Config(hal::PinConfigType config)
    {}

    void HardwareFactoryImpl::GpioPinStub::Config(hal::PinConfigType config, bool startOutputState)
    {}

    void HardwareFactoryImpl::GpioPinStub::ResetConfig()
    {}

    void HardwareFactoryImpl::GpioPinStub::EnableInterrupt(const infra::Function<void()>& action, hal::InterruptTrigger trigger, hal::InterruptType type)
    {}

    void HardwareFactoryImpl::GpioPinStub::DisableInterrupt()
    {}
}
