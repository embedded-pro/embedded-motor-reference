#include "application/motors/DC/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : motorController(hardware, pid)
        , terminalWithStorage(hardware.Terminal(), hardware.Tracer())
        , terminal(terminalWithStorage, motorController)
    {}
}
