#pragma once

#include "hal/ti/hal_tiva/tiva/Gpio.hpp"

namespace application
{
    namespace Pins
    {
        hal::tiva::GpioPin currentPhaseA{ hal::tiva::Port::E, 2 };
        hal::tiva::GpioPin currentPhaseB{ hal::tiva::Port::E, 3 };

        hal::tiva::GpioPin hallSensorA{ hal::tiva::Port::E, 4 };
        hal::tiva::GpioPin hallSensorB{ hal::tiva::Port::E, 5 };
        hal::tiva::GpioPin hallSensorC{ hal::tiva::Port::E, 6 };

        hal::tiva::GpioPin encoderA{ hal::tiva::Port::B, 4 };
        hal::tiva::GpioPin encoderB{ hal::tiva::Port::B, 5 };
        hal::tiva::GpioPin encoderZ{ hal::tiva::Port::B, 6 };

        hal::tiva::GpioPin pwmPhase1a{ hal::tiva::Port::B, 5 };
        hal::tiva::GpioPin pwmPhase1b{ hal::tiva::Port::B, 6 };
        hal::tiva::GpioPin pwmPhase2a{ hal::tiva::Port::B, 5 };
        hal::tiva::GpioPin pwmPhase2b{ hal::tiva::Port::B, 6 };
        hal::tiva::GpioPin pwmPhase3a{ hal::tiva::Port::B, 5 };
        hal::tiva::GpioPin pwmPhase3b{ hal::tiva::Port::B, 6 };

        hal::tiva::GpioPin led1{ hal::tiva::Port::F, 0 };

        hal::tiva::GpioPin uartTx{ hal::tiva::Port::A, 0 };
        hal::tiva::GpioPin uartRx{ hal::tiva::Port::A, 1 };
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
