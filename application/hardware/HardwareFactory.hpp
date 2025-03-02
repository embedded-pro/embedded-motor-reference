#ifndef HARDWARE_FACTORY_HPP
#define HARDWARE_FACTORY_HPP

#include "hal/interfaces/Gpio.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include "infra/util/MemoryRange.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/Terminal.hpp"

namespace hal
{
    class HallSensor
    {
    public:
        using State = uint8_t;

        virtual State Read() = 0;
    };
}

namespace application
{
    class HardwareFactory
    {
    public:
        explicit HardwareFactory(const infra::Function<void()>& onInitialized)
            : onInitialized(onInitialized)
        {}

        virtual void Run() = 0;
        virtual services::Tracer& Tracer() = 0;
        virtual services::TerminalWithCommands& Terminal() = 0;
        virtual infra::MemoryRange<hal::GpioPin> Leds() = 0;
        virtual hal::SynchronousQuadratureEncoder& QuadratureEncoder() = 0;
        virtual hal::SynchronousPwm& PwmOutput() = 0;
        virtual uint32_t ControlTimerId() const = 0;
        virtual hal::HallSensor& HallSensor() = 0;

    protected:
        infra::Function<void()> onInitialized;
    };
}

#endif
