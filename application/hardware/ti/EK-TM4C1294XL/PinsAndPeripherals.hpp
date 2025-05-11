#pragma once

#include "hal/ti/hal_tiva/tiva/Gpio.hpp"

namespace application
{
    namespace Pins
    {
        static hal::tiva::GpioPin currentPhaseA{ hal::tiva::Port::E, 2 };
        static hal::tiva::GpioPin currentPhaseB{ hal::tiva::Port::E, 3 };

        static hal::tiva::GpioPin hallSensorA{ hal::tiva::Port::E, 4 };
        static hal::tiva::GpioPin hallSensorB{ hal::tiva::Port::E, 5 };
        static hal::tiva::GpioPin hallSensorC{ hal::tiva::Port::E, 6 };

        static hal::tiva::GpioPin encoderA{ hal::tiva::Port::B, 4 };
        static hal::tiva::GpioPin encoderB{ hal::tiva::Port::B, 5 };
        static hal::tiva::GpioPin encoderZ{ hal::tiva::Port::B, 6 };

        static hal::tiva::GpioPin pwmPhase1a{ hal::tiva::Port::B, 5 };
        static hal::tiva::GpioPin pwmPhase1b{ hal::tiva::Port::B, 6 };
        static hal::tiva::GpioPin pwmPhase2a{ hal::tiva::Port::B, 5 };
        static hal::tiva::GpioPin pwmPhase2b{ hal::tiva::Port::B, 6 };
        static hal::tiva::GpioPin pwmPhase3a{ hal::tiva::Port::B, 5 };
        static hal::tiva::GpioPin pwmPhase3b{ hal::tiva::Port::B, 6 };

        static hal::tiva::GpioPin led1{ hal::tiva::Port::F, 0 };

        static hal::tiva::GpioPin uartTx{ hal::tiva::Port::A, 0 };
        static hal::tiva::GpioPin uartRx{ hal::tiva::Port::A, 1 };
    }

    namespace Peripheral
    {
        constexpr static uint8_t QeiIndex = 0;
        constexpr static uint8_t AdcIndex = 0;
        constexpr static uint8_t AdcSequencerIndex = 0;
        constexpr static uint8_t UartIndex = 0;
        constexpr static uint8_t PwmIndex = 0;
    }
}
