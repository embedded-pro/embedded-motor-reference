#include "application/motors/synchrounous_foc_sensored/torque/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : motorFocImpl{ hardware.MotorFieldOrientedController(), focImpl }
        , focInteractor{ motorFocImpl }
        , terminalWithStorage(hardware.Terminal(), hardware.Tracer())
        , terminal(terminalWithStorage, focInteractor)
    {}
}
