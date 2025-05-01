#include "application/motors/DC/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : pid{ hardware.MotorPid() }
        , motorController{ pid }
        , terminalWithStorage{ hardware.Terminal(), hardware.Tracer() }
        , terminal{ terminalWithStorage, motorController }
    {}
}
