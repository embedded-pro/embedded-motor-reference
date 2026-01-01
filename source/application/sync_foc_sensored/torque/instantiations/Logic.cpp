#include "source/application/sync_foc_sensored/torque/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : hardwareAdapter{ hardware }
        , motorFocImpl{ hardwareAdapter, hardwareAdapter, focImpl }
        , terminalWithStorage{ hardware.Terminal(), hardware.Tracer() }
        , terminal{ terminalWithStorage, hardware.PowerSupplyVoltage(), motorFocImpl, motorFocImpl }
        , debugLed{ hardware.Leds().front(), std::chrono::milliseconds(50), std::chrono::milliseconds(1950) }
    {}
}
