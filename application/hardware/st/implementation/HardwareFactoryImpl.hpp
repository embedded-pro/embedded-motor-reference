#pragma once

#include HARDWARE_PINS_AND_PERIPHERALS_HEADER
#include "application/hardware/HardwareFactory.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/interfaces/SerialCommunication.hpp"
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

        PidInterface& MotorPid() override;
        MotorFieldOrientedControllerInterface& MotorFieldOrientedController() override;
        Encoder& MotorPosition() override;

    private:
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

        struct PidInterfaceImpl
            : public PidInterface
        {
            void Read(const infra::Function<void(float)>& onDone) override
            {
            }

            void ControlAction(float) override
            {
            }

            void Start(infra::Duration sampleTime) override
            {
            }

            void Stop() override
            {
            }
        };

        struct MotorFieldOrientedControllerInterfaceImpl
            : public MotorFieldOrientedControllerInterface
        {
            void PhaseCurrentsReady(const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)>& onDone) override
            {
            }

            void ThreePhasePwmOutput(const std::tuple<Percent, Percent, Percent>& dutyPhases) override
            {
            }

            void Start() override
            {
            }

            void Stop() override
            {
            }
        };

        struct EncoderImpl
            : public Encoder
        {
            Degrees Read() override
            {
                return Degrees(0.0f);
            }

            void Set(Degrees value) override
            {
            }

            void SetZero() override
            {
            }
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
        PidInterfaceImpl motorPid;
        MotorFieldOrientedControllerInterfaceImpl motorFieldOrientedController;
        EncoderImpl encoderImpl;
        GpioPinStub pin;
        SerialCommunicationStub serial;
        TerminalAndTracer terminalAndTracer{ serial };
    };
}
