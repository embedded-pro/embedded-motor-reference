#include "application/hardware/Host/HardwareFactoryImpl.hpp"
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
        return infra::MakeRangeFromSingleObject(pin);
    }

    hal::SynchronousAdc& HardwareFactoryImpl::PhaseA()
    {
        return phaseA;
    }

    hal::SynchronousAdc& HardwareFactoryImpl::PhaseB()
    {
        return phaseB;
    }

    hal::SynchronousQuadratureEncoder& HardwareFactoryImpl::QuadratureEncoder()
    {
        return encoder;
    }

    hal::SynchronousPwm& HardwareFactoryImpl::PwmOutput()
    {
        return pwm;
    }

    uint32_t HardwareFactoryImpl::ControlTimerId() const
    {
        return timerId;
    }

    hal::HallSensor& HardwareFactoryImpl::HallSensor()
    {
        return *this;
    }

    hal::HallSensor::State HardwareFactoryImpl::Read()
    {
        return 0;
    }

    void HardwareFactoryImpl::SynchronousPwmStub::SetBaseFrequency(hal::Hertz baseFrequency)
    {
    }

    void HardwareFactoryImpl::SynchronousPwmStub::Start(hal::Percent globalDutyCycle)
    {
    }

    void HardwareFactoryImpl::SynchronousPwmStub::Stop()
    {
    }

    uint32_t HardwareFactoryImpl::SynchronousQuadratureEncoderStub::Position()
    {
        return 0;
    }

    uint32_t HardwareFactoryImpl::SynchronousQuadratureEncoderStub::Resolution()
    {
        return 0;
    }

    hal::SynchronousQuadratureEncoder::MotionDirection HardwareFactoryImpl::SynchronousQuadratureEncoderStub::Direction()
    {
        return hal::SynchronousQuadratureEncoder::MotionDirection::forward;
    }

    uint32_t HardwareFactoryImpl::SynchronousQuadratureEncoderStub::Speed()
    {
        return 0;
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
