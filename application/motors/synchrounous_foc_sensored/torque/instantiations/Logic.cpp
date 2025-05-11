#include "application/motors/synchrounous_foc_sensored/torque/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : motorFocImpl{ hardware.MotorFieldOrientedController(), hardware.MotorPosition(), focImpl }
        , focInteractor{ motorFocImpl }
        , terminalWithStorage{ hardware.Terminal(), hardware.Tracer() }
        , terminal{ terminalWithStorage, focInteractor }
        , debugLed{ hardware.Leds().front(), std::chrono::milliseconds(50), std::chrono::milliseconds(1950) }
    {}
}
