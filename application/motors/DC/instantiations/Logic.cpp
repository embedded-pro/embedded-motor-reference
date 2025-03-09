#include "application/motors/DC/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : motorController(hardware.QuadratureEncoder(), hardware.PwmOutput(), pid, hardware.ControlTimerId())
        , terminalWithStorage(hardware.Terminal(), hardware.Tracer())
        , terminal(terminalWithStorage, hardware.Tracer(), motorController)
    {}
}
