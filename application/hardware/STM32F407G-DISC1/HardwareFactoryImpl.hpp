#ifndef HARDWARE_FACTORY_IMPL_HPP
#define HARDWARE_FACTORY_IMPL_HPP

#include "application/hardware/HardwareFactory.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/interfaces/SerialCommunication.hpp"
#include "services/tracer/StreamWriterOnSerialCommunication.hpp"
#include "services/tracer/TracerWithDateTime.hpp"

namespace application
{
    class HardwareFactoryImpl
        : public HardwareFactory
        , private hal::HallSensor
    {
    public:
        explicit HardwareFactoryImpl(const infra::Function<void()>& onInitialized);

        // Implementation of HardwareFactory
        void Run() override;
        services::Tracer& Tracer() override;
        services::TerminalWithCommands& Terminal() override;
        infra::MemoryRange<hal::GpioPin> Leds() override;
        hal::SynchronousAdc& PhaseA() override;
        hal::SynchronousAdc& PhaseB() override;
        hal::SynchronousQuadratureEncoder& QuadratureEncoder() override;
        hal::SynchronousPwm& PwmOutput() override;
        uint32_t ControlTimerId() const override;
        hal::HallSensor& HallSensor() override;

        // Implementation of hal::HallSensor
        State Read() override;

    private:
        class SynchronousAdcStub
            : public hal::SynchronousAdc
        {
        public:
            Samples Measure(std::size_t numberOfSamples) override
            {
                return Samples();
            }
        };

        class SynchronousPwmStub
            : public hal::SynchronousPwm
        {
        public:
            void SetBaseFrequency(hal::Hertz baseFrequency) override;
            void Start(hal::Percent globalDutyCycle) override;
            void Stop() override;
        };

        class SynchronousQuadratureEncoderStub
            : public hal::SynchronousQuadratureEncoder
        {
            uint32_t Position() override;
            uint32_t Resolution() override;
            MotionDirection Direction() override;
            uint32_t Speed() override;
        };

        class SerialCommunicationStub
            : public hal::SerialCommunication
        {
        public:
            void SendData(infra::ConstByteRange data, infra::Function<void()> actionOnCompletion) override;
            void ReceiveData(infra::Function<void(infra::ConstByteRange data)> dataReceived) override;
        };

        class GpioPinStub
            : public hal::GpioPin
        {
        public:
            bool Get() const override;
            void Set(bool value) override;
            bool GetOutputLatch() const override;
            void SetAsInput() override;
            bool IsInput() const override;
            void Config(hal::PinConfigType config) override;
            void Config(hal::PinConfigType config, bool startOutputState) override;
            void ResetConfig() override;
            void EnableInterrupt(const infra::Function<void()>& action, hal::InterruptTrigger trigger, hal::InterruptType type) override;
            void DisableInterrupt() override;
        };

        struct TerminalAndTracer
        {
            explicit TerminalAndTracer(hal::SerialCommunication& com)
                : streamWriterOnSerialCommunication(com)
                , tracerStream(streamWriterOnSerialCommunication)
                , tracer(tracerStream)
                , terminal(com, tracer)
            {}

            services::StreamWriterOnSerialCommunication::WithStorage<8192> streamWriterOnSerialCommunication;
            infra::TextOutputStream::WithErrorPolicy tracerStream;
            services::TracerWithDateTime tracer;
            services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<> terminal;
        };

    private:
        infra::Function<void()> onInitialized;
        static constexpr uint32_t timerId = 1;
        SynchronousAdcStub phaseA;
        SynchronousAdcStub phaseB;
        SynchronousQuadratureEncoderStub encoder;
        SynchronousPwmStub pwm;
        GpioPinStub pin;
        SerialCommunicationStub serial;
        TerminalAndTracer terminalAndTracer{ serial };
    };
}

#endif
