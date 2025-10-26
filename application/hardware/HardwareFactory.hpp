#pragma once

#include "application/foc/interfaces/Driver.hpp"
#include "hal/interfaces/AdcMultiChannel.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include "infra/util/MemoryRange.hpp"
#include "infra/util/ProxyCreator.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/Terminal.hpp"
#include <chrono>

namespace hal
{
    class PerformanceTracker
    {
    public:
        virtual void Start() = 0;
        virtual uint32_t ElapsedCycles() = 0;
    };
}

namespace application
{
    class HardwareFactory
    {
    public:
        enum class SampleAndHold
        {
            shortest,
            shorter,
            medium,
            longer,
            longest,
        };

        virtual void Run() = 0;
        virtual services::Tracer& Tracer() = 0;
        virtual services::TerminalWithCommands& Terminal() = 0;
        virtual infra::MemoryRange<hal::GpioPin> Leds() = 0;
        virtual hal::PerformanceTracker& PerformanceTimer() = 0;
        virtual hal::Hertz BaseFrequency() const = 0;
        virtual foc::Volts PowerSupplyVoltage() = 0;
        virtual infra::CreatorBase<hal::SynchronousThreeChannelsPwm, void(std::chrono::nanoseconds deadTime, hal::Hertz frequency)>& SynchronousThreeChannelsPwmCreator() = 0;
        virtual infra::CreatorBase<hal::AdcMultiChannel, void(SampleAndHold)>& AdcMultiChannelCreator() = 0;
        virtual infra::CreatorBase<hal::SynchronousQuadratureEncoder, void()>& SynchronousQuadratureEncoderCreator() = 0;
    };
}
