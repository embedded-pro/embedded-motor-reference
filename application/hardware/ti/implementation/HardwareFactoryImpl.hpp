#pragma once

#include "application/foc/interfaces/Driver.hpp"
#include "hal/interfaces/AdcMultiChannel.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include HARDWARE_PINS_AND_PERIPHERALS_HEADER
#include "application/hardware/HardwareFactory.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/ti/hal_tiva/cortex/DataWatchpointAndTrace.hpp"
#include "hal/ti/hal_tiva/cortex/SystemTickTimerService.hpp"
#include "hal/ti/hal_tiva/synchronous_tiva/SynchronousPwm.hpp"
#include "hal/ti/hal_tiva/synchronous_tiva/SynchronousQuadratureEncoder.hpp"
#include "hal/ti/hal_tiva/tiva/Adc.hpp"
#include "hal/ti/hal_tiva/tiva/Uart.hpp"
#include "hal_tiva/tiva/Gpio.hpp"
#include "infra/event/EventDispatcherWithWeakPtr.hpp"
#include "services/tracer/SerialCommunicationOnSeggerRtt.hpp"
#include "services/tracer/StreamWriterOnSerialCommunication.hpp"
#include "services/tracer/TracerWithDateTime.hpp"

namespace application
{
    using namespace std::chrono_literals;

    class HardwareFactoryImpl
        : public HardwareFactory
        , public hal::PerformanceTracker
    {
    public:
        explicit HardwareFactoryImpl(const infra::Function<void()>& onInitialized);

        // Implementation of HardwareFactory
        void Run() override;
        services::Tracer& Tracer() override;
        services::TerminalWithCommands& Terminal() override;
        infra::MemoryRange<hal::GpioPin> Leds() override;
        hal::PerformanceTracker& PerformanceTimer() override;
        hal::Hertz BaseFrequency() const override;
        foc::Volts PowerSupplyVoltage() override;
        foc::Ampere MaxCurrentSupported() override;
        infra::CreatorBase<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)>& SynchronousThreeChannelsPwmCreator() override;
        infra::CreatorBase<hal::AdcMultiChannel, void(SampleAndHold)>& AdcMultiChannelCreator() override;
        infra::CreatorBase<hal::SynchronousQuadratureEncoder, void()>& SynchronousQuadratureEncoderCreator() override;

        // Implementation of hal::PerformanceTracker
        void Start() override;
        uint32_t ElapsedCycles() override;

    private:
        struct Cortex
        {
            infra::EventDispatcherWithWeakPtr::WithSize<50> eventDispatcher;
            hal::DataWatchPointAndTrace dataWatchPointAndTrace;
            hal::cortex::SystemTickTimerService systemTick{ std::chrono::milliseconds(1) };
        };

        struct TerminalAndTracer
        {
            hal::tiva::Uart::Config uartConfig{ true, true, hal::tiva::Uart::Baudrate::_921000_bps, hal::tiva::Uart::FlowControl::none, hal::tiva::Uart::Parity::none, hal::tiva::Uart::StopBits::one, hal::tiva::Uart::NumberOfBytes::_8_bytes, infra::none };
            hal::tiva::Uart uart{ Peripheral::UartIndex, Pins::uartTx, Pins::uartRx, uartConfig };
            services::SerialCommunicationOnSeggerRtt serialCommunicationOnSeggerRtt;
            services::StreamWriterOnSerialCommunication::WithStorage<2048> streamWriterOnSerialCommunication{ serialCommunicationOnSeggerRtt };
            infra::TextOutputStream::WithErrorPolicy tracerStream{ streamWriterOnSerialCommunication };
            services::TracerWithDateTime tracer{ tracerStream };
            services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<256, 10> terminal{ serialCommunicationOnSeggerRtt, tracer };
        };

        struct MotorFieldOrientedControllerInterfaceImpl
        {
            const std::array<hal::tiva::Adc::SampleAndHold, 5> toSampleAndHold{ { hal::tiva::Adc::SampleAndHold::sampleAndHold4,
                hal::tiva::Adc::SampleAndHold::sampleAndHold16,
                hal::tiva::Adc::SampleAndHold::sampleAndHold32,
                hal::tiva::Adc::SampleAndHold::sampleAndHold64,
                hal::tiva::Adc::SampleAndHold::sampleAndHold256 } };
            hal::tiva::Adc::Config adcConfig{ false, 0, hal::tiva::Adc::Trigger::pwmGenerator0, hal::tiva::Adc::SampleAndHold::sampleAndHold4 };
            std::array<hal::tiva::AnalogPin, 3> currentPhaseAnalogPins{ { hal::tiva::AnalogPin{ Pins::currentPhaseA }, hal::tiva::AnalogPin{ Pins::currentPhaseB }, hal::tiva::AnalogPin{ Pins::powerSupplyVoltage } } };
            infra::Creator<hal::AdcMultiChannel, hal::tiva::Adc, void(SampleAndHold)> adcCurrentPhases{ [this](auto& object, auto sampleAndHold)
                {
                    adcConfig.sampleAndHold = toSampleAndHold.at(static_cast<std::size_t>(sampleAndHold));

                    object.Emplace(Peripheral::AdcIndex, Peripheral::AdcSequencerIndex, currentPhaseAnalogPins, adcConfig);
                } };
            hal::tiva::SynchronousPwm::Config::ClockDivisor clockDivisor{ hal::tiva::SynchronousPwm::Config::ClockDivisor::divisor8 };
            hal::tiva::SynchronousPwm::Config::Control controlConfig{ hal::tiva::SynchronousPwm::Config::Control::Mode::centerAligned, hal::tiva::SynchronousPwm::Config::Control::UpdateMode::globally, false };
            hal::tiva::SynchronousPwm::Config::DeadTime deadTimeConfig{ hal::tiva::SynchronousPwm::CalculateDeadTimeCycles(1000ns, clockDivisor), hal::tiva::SynchronousPwm::CalculateDeadTimeCycles(1000ns, clockDivisor) };
            hal::tiva::SynchronousPwm::Config::Trigger triggerConfig{ hal::tiva::SynchronousPwm::Config::Trigger::countLoad };
            hal::tiva::SynchronousPwm::Config pwmConfig{ false, true, controlConfig, clockDivisor, std::make_optional(deadTimeConfig), std::make_optional(triggerConfig) };
            infra::Creator<hal::SynchronousThreeChannelsPwm, hal::tiva::SynchronousPwm, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)> pwmBrushless{ [this](auto& object, auto deadTime, auto frequency)
                {
                    deadTimeConfig.fallInClockCycles = hal::tiva::SynchronousPwm::CalculateDeadTimeCycles(deadTime, clockDivisor);
                    deadTimeConfig.riseInClockCycles = hal::tiva::SynchronousPwm::CalculateDeadTimeCycles(deadTime, clockDivisor);

                    pwmConfig.deadTime = std::make_optional(deadTimeConfig);

                    object.Emplace(Peripheral::PwmIndex, Peripheral::pwmPhases, pwmConfig);
                    object->SetBaseFrequency(frequency);
                } };
            infra::Function<void(std::tuple<infra::Ampere, infra::Ampere, infra::Ampere> voltagePhases)>
                phaseCurrentsReady;
        };

        struct EncoderImpl
        {
            hal::tiva::QuadratureEncoder::Config qeiConfig;
            infra::Creator<hal::SynchronousQuadratureEncoder, hal::tiva::QuadratureEncoder, void()> synchronousQuadratureEncoderCreator{ [this](auto& object)
                {
                    object.Emplace(Peripheral::QeiIndex, Pins::encoderA, Pins::encoderB, Pins::encoderZ, qeiConfig);
                } };
        };

        struct Peripherals
        {
            Peripherals() {};

            Cortex cortex;
            TerminalAndTracer terminalAndTracer;
            EncoderImpl encoderImpl;
            MotorFieldOrientedControllerInterfaceImpl motorFieldOrientedController;
        };

    private:
        infra::Function<void()> onInitialized;
        std::optional<Peripherals> peripherals;
    };
}
