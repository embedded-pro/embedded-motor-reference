#pragma once

#include "application/hardware/HardwareFactory.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/interfaces/SerialCommunication.hpp"
#include "hal/synchronous_interfaces/SynchronousAdc.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "services/tracer/StreamWriterOnSerialCommunication.hpp"
#include "services/tracer/TracerWithDateTime.hpp"

namespace application
{
    class HardwareFactoryImpl
        : public HardwareFactory
    {
    public:
        explicit HardwareFactoryImpl(const infra::Function<void()>& onInitialized);

        // Implementation of HardwareFactory
        void Run() override;
        services::Tracer& Tracer() override;
        services::TerminalWithCommands& Terminal() override;
        infra::MemoryRange<hal::GpioPin> Leds() override;

        infra::CreatorBase<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)>& SynchronousThreeChannelsPwmCreator() override;
        infra::CreatorBase<hal::AdcMultiChannel, void(SampleAndHold)>& AdcMultiChannelCreator() override;
        infra::CreatorBase<hal::SynchronousQuadratureEncoder, void()>& SynchronousQuadratureEncoderCreator() override;

    private:
        class AdcMultiChannelStub
            : public hal::AdcMultiChannel
        {
        public:
            // Implementation of hal::AdcMultiChannel
            void Measure(const infra::Function<void(Samples)>& onDone) override;
        };

        class SynchronousThreeChannelsPwmStub
            : public hal::SynchronousThreeChannelsPwm
        {
        public:
            // Implementation of hal::SynchronousThreeChannelsPwm
            void SetBaseFrequency(hal::Hertz baseFrequency) override;
            void Stop() override;
            void Start(hal::Percent dutyCycle1, hal::Percent dutyCycle2, hal::Percent dutyCycle3) override;
        };

        class SynchronousQuadratureEncoderStub
            : public hal::SynchronousQuadratureEncoder
        {
        public:
            uint32_t Position() override;
            uint32_t Resolution() override;
            MotionDirection Direction() override;
            uint32_t Speed() override;
        };

        class SynchronousAdcStub
            : public hal::SynchronousAdc
        {
        public:
            Samples Measure(std::size_t numberOfSamples) override
            {
                return Samples();
            }
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
        GpioPinStub pin;
        SerialCommunicationStub serial;
        TerminalAndTracer terminalAndTracer{ serial };
        infra::Creator<hal::AdcMultiChannel, AdcMultiChannelStub, void(SampleAndHold)> adcCurrentPhases{ [this](auto& object, auto sampleAndHold)
            {
                object.Emplace();
            } };
        infra::Creator<hal::SynchronousThreeChannelsPwm, SynchronousThreeChannelsPwmStub, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)> pwmBrushless{ [this](auto& object, auto deadTime, auto frequency)
            {
                object.Emplace();
            } };
        infra::Creator<hal::SynchronousQuadratureEncoder, SynchronousQuadratureEncoderStub, void()> synchronousQuadratureEncoderCreator{ [this](auto& object)
            {
                object.Emplace();
            } };
    };
}
