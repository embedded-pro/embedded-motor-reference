#include "application/motors/synchronous_foc_sensored/torque/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : hardwareAdapter{ hardware }
        , motorFocImpl{ hardwareAdapter, hardwareAdapter, focImpl }
        , focInteractor{ motorFocImpl }
        , terminalWithStorage{ hardware.Terminal(), hardware.Tracer() }
        , terminal{ terminalWithStorage, focInteractor }
        , debugLed{ hardware.Leds().front(), std::chrono::milliseconds(50), std::chrono::milliseconds(1950) }
    {}
}
