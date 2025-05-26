#pragma once

#include HARDWARE_PINS_AND_PERIPHERALS_HEADER
#include "application/hardware/HardwareFactory.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/ti/hal_tiva/cortex/SystemTickTimerService.hpp"
#include "hal/ti/hal_tiva/synchronous_tiva/SynchronousPwm.hpp"
#include "hal/ti/hal_tiva/synchronous_tiva/SynchronousQuadratureEncoder.hpp"
#include "hal/ti/hal_tiva/tiva/Adc.hpp"
#include "hal/ti/hal_tiva/tiva/Uart.hpp"
#include "hal_tiva/tiva/Gpio.hpp"
#include "infra/event/EventDispatcherWithWeakPtr.hpp"
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
        struct Cortex
        {
            hal::InterruptTable::WithStorage<128> interruptTable;
            hal::tiva::Gpio gpio{ hal::tiva::pinoutTableDefault, hal::tiva::analogTableDefault };
            hal::cortex::SystemTickTimerService systemTick{ std::chrono::milliseconds(1) };
            infra::EventDispatcherWithWeakPtr::WithSize<50> eventDispatcher;
        };

        struct TerminalAndTracer
        {
            hal::tiva::Uart::Config uartConfig{ true, true, hal::tiva::Uart::Baudrate::_921000_bps, hal::tiva::Uart::FlowControl::none, hal::tiva::Uart::Parity::none, hal::tiva::Uart::StopBits::one, hal::tiva::Uart::NumberOfBytes::_8_bytes, infra::none };
            hal::tiva::Uart uart{ Peripheral::UartIndex, Pins::uartTx, Pins::uartRx, uartConfig };
            services::StreamWriterOnSerialCommunication::WithStorage<2048> streamWriterOnSerialCommunication{ uart };
            infra::TextOutputStream::WithErrorPolicy tracerStream{ streamWriterOnSerialCommunication };
            services::TracerWithDateTime tracer{ tracerStream };
            services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<256, 2> terminal{ uart, tracer };
        };

        struct PidInterfaceImpl
            : public PidInterface
        {
            void Read(const infra::Function<void(float)>& onDone) override;
            void ControlAction(float) override;
            void Start(infra::Duration sampleTime) override;
            void Stop() override;

            hal::SynchronousPwmImpl pwm;
        };

        struct MotorFieldOrientedControllerInterfaceImpl
            : public MotorFieldOrientedControllerInterface
        {
            void PhaseCurrentsReady(const infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)>& onDone) override;
            void ThreePhasePwmOutput(const std::tuple<hal::Percent, hal::Percent, hal::Percent>& dutyPhases) override;
            void Start() override;
            void Stop() override;

            hal::tiva::Adc::Config adcConfig{ false, 0, hal::tiva::Adc::Trigger::pwmGenerator0, hal::tiva::Adc::SampleAndHold::sampleAndHold4 };
            std::array<hal::tiva::AnalogPin, 3> currentPhaseAnalogPins{ { hal::tiva::AnalogPin{ Pins::currentPhaseA }, hal::tiva::AnalogPin{ Pins::currentPhaseB }, hal::tiva::AnalogPin{ Pins::powerSupplyVoltage } } };
            hal::tiva::Adc adcCurrentPhases{ Peripheral::AdcIndex, Peripheral::AdcSequencerIndex, currentPhaseAnalogPins, adcConfig };
            hal::tiva::SynchronousPwm::Config::Control controlConfig{ hal::tiva::SynchronousPwm::Config::Control::Mode::centerAligned, hal::tiva::SynchronousPwm::Config::Control::UpdateMode::globally, true };
            hal::tiva::SynchronousPwm::Config::DeadTime deadTimeConfig{ 0, 0 };
            hal::tiva::SynchronousPwm::Config::Trigger triggerConfig{ hal::tiva::SynchronousPwm::Config::Trigger::countLoad };
            hal::tiva::SynchronousPwm::Config pwmConfig{ false, false, controlConfig, hal::tiva::SynchronousPwm::Config::ClockDivisor::divisor1, std::make_optional(deadTimeConfig), std::make_optional(triggerConfig) };
            hal::tiva::SynchronousPwm pwmBrushless{ Peripheral::PwmIndex, Peripheral::pwmPhases, pwmConfig };
            infra::Function<void(std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases)> phaseCurrentsReady;
        };

        struct EncoderImpl
            : public Encoder
        {
            Degrees Read() override;
            void Set(Degrees value) override;
            void SetZero() override;

            hal::tiva::QuadratureEncoder::Config qeiConfig;
            hal::tiva::QuadratureEncoder encoder{ Peripheral::QeiIndex, Pins::encoderA, Pins::encoderB, Pins::encoderZ, qeiConfig };
        };

        struct Peripherals
        {
            Peripherals(){};

            Cortex cortex;
            TerminalAndTracer terminalAndTracer;
            EncoderImpl encoderImpl;
            PidInterfaceImpl motorPid;
            MotorFieldOrientedControllerInterfaceImpl motorFieldOrientedController;
        };

    private:
        infra::Function<void()> onInitialized;
        std::optional<Peripherals> peripherals;
    };
}
