#include "application/motors/BLDC/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : focInteractor{ hardware.MotorFieldOrientedController(), focImpl }
        , terminalWithStorage(hardware.Terminal(), hardware.Tracer())
        , terminal(terminalWithStorage, focInteractor)
    {}
}
