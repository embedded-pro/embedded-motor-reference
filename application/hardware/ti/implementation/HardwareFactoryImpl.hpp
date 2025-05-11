#pragma once

#include HARDWARE_PINS_AND_PERIPHERALS_HEADER
#include "application/hardware/HardwareFactory.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/interfaces/SerialCommunication.hpp"
#include "hal/ti/hal_tiva/synchronous_tiva/SynchronousPwm.hpp"
#include "hal/ti/hal_tiva/synchronous_tiva/SynchronousQuadratureEncoder.hpp"
#include "hal/ti/hal_tiva/tiva/Adc.hpp"
#include "hal/ti/hal_tiva/tiva/Uart.hpp"
#include "hal_tiva/tiva/Gpio.hpp"
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

    private:
        infra::Function<void()> onInitialized;
        infra::Function<void(MilliVolt phaseA, MilliVolt phaseB, MilliVolt phaseC)> phaseCurrentsReady;
        infra::Function<void(HallState state, Direction direction)> hallSensorInterrupt;
        static constexpr uint32_t timerId = 1;

        hal::tiva::QuadratureEncoder::Config qeiConfig;
        hal::tiva::QuadratureEncoder encoder{ Peripheral::QeiIndex, Pins::encoderA, Pins::encoderB, Pins::encoderZ, qeiConfig };
        hal::tiva::Adc::Config adcConfig{ false, 0, hal::tiva::Adc::Trigger::pwmGenerator0, hal::tiva::Adc::SampleAndHold::sampleAndHold4 };
        std::array<hal::tiva::AnalogPin, 2> currentPhaseAnalogPins{ { hal::tiva::AnalogPin{ Pins::currentPhaseA }, hal::tiva::AnalogPin{ Pins::currentPhaseB } } };
        hal::tiva::Adc adcCurrentPhases{ Peripheral::AdcIndex, Peripheral::AdcSequencerIndex, currentPhaseAnalogPins, adcConfig };
        hal::tiva::SynchronousPwm::Config::Control controlConfig{ hal::tiva::SynchronousPwm::Config::Control::Mode::centerAligned, hal::tiva::SynchronousPwm::Config::Control::UpdateMode::globally, true };
        hal::tiva::SynchronousPwm::Config::DeadTime deadTimeConfig{ 0, 0 };
        hal::tiva::SynchronousPwm::Config::Trigger triggerConfig{ hal::tiva::SynchronousPwm::Config::Trigger::countLoad };
        hal::tiva::SynchronousPwm::Config pwmConfig{ false, false, controlConfig, hal::tiva::SynchronousPwm::Config::ClockDivisor::divisor1, std::make_optional(deadTimeConfig), std::make_optional(triggerConfig) };
        hal::tiva::SynchronousPwm::PinChannel pwmPhase1{ Pins::pwmPhase1a, Pins::pwmPhase1b, true, true };
        hal::tiva::SynchronousPwm::PinChannel pwmPhase2{ Pins::pwmPhase2a, Pins::pwmPhase2b, true, true };
        hal::tiva::SynchronousPwm::PinChannel pwmPhase3{ Pins::pwmPhase3a, Pins::pwmPhase3b, true, true };
        hal::tiva::SynchronousPwm pwmBrushless{ Peripheral::PwmIndex, pwmPhase1, pwmPhase2, pwmPhase3, pwmConfig };
        hal::tiva::Uart::Config uartConfig{ true, true, hal::tiva::Uart::Baudrate::_921000_bps, hal::tiva::Uart::FlowControl::none, hal::tiva::Uart::Parity::none, hal::tiva::Uart::StopBits::one, hal::tiva::Uart::NumberOfBytes::_8_bytes, infra::none };
        hal::tiva::Uart uart{ Peripheral::UartIndex, Pins::uartTx, Pins::uartRx, uartConfig };
        hal::SynchronousPwmImpl pwm;
        TerminalAndTracer terminalAndTracer{ uart };
        EncoderImpl encoderImpl;

        PidInterfaceImpl motorPid;
        MotorFieldOrientedControllerInterfaceImpl motorFieldOrientedController;
    };
}
