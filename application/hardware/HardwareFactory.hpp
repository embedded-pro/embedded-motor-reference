#pragma once

#include "application/foc/MotorFieldOrientedControllerInterface.hpp"
#include "application/pid/PidInterface.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include "infra/util/MemoryRange.hpp"
#include "infra/util/Unit.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/Terminal.hpp"

namespace hal
{
    class PerformanceTracker
    {
    public:
        virtual void Start() = 0;
        virtual uint32_t ElapsedCycles() = 0;
    };

    class SynchronousPwmImpl
        : public SynchronousSingleChannelPwm
        , public SynchronousTwoChannelsPwm
        , public SynchronousThreeChannelsPwm
        , public SynchronousFourChannelsPwm
    {
    public:
        void SetBaseFrequency(Hertz baseFrequency) override
        {
        }

        void Stop() override
        {
        }

        void Start(Percent dutyCycle1) override
        {
        }

        void Start(Percent dutyCycle1, Percent dutyCycle2) override
        {
        }

        void Start(Percent dutyCycle1, Percent dutyCycle2, Percent dutyCycle3) override
        {
        }

        void Start(Percent dutyCycle1, Percent dutyCycle2, Percent dutyCycle3, Percent dutyCycle4) override
        {
        }
    };
}

namespace application
{
    class HardwareFactory
    {
    public:
        virtual void Run() = 0;
        virtual services::Tracer& Tracer() = 0;
        virtual services::TerminalWithCommands& Terminal() = 0;
        virtual infra::MemoryRange<hal::GpioPin> Leds() = 0;

        virtual PidInterface& MotorPid() = 0;
        virtual MotorFieldOrientedControllerInterface& MotorFieldOrientedController() = 0;
        virtual Encoder& MotorPosition() = 0;
    };
}
