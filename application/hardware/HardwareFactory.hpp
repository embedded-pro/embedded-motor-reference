#ifndef HARDWARE_FACTORY_HPP
#define HARDWARE_FACTORY_HPP

#include "hal/interfaces/Gpio.hpp"
#include "hal/interfaces/SerialCommunication.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "hal/synchronous_interfaces/SynchronousQuadratureEncoder.hpp"
#include "infra/util/MemoryRange.hpp"
#include "infra/util/ProxyCreator.hpp"

namespace application
{
    class QuadratureEncoder
    {
    public:
    };

    class HardwareFactory
    {
    public:
        explicit HardwareFactory(const infra::Function<void()>& onInitialized)
            : onInitialized(onInitialized)
        {}

        virtual void Run() = 0;
        virtual hal::SerialCommunication& Serial() = 0;
        virtual infra::MemoryRange<hal::GpioPin> Leds() = 0;
        virtual infra::CreatorBase<hal::SynchronousQuadratureEncoder, void()>& QuadratureEncoderCreator() = 0;
        virtual infra::CreatorBase<hal::SynchronousPwm, void()>& PwmOutputCreator() = 0;

    protected:
        infra::Function<void()> onInitialized;
    };
}

#endif
