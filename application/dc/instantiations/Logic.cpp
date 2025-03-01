#include "application/dc/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : hardware(hardware)
        , motorController(hardware.QuadratureEncoder(), hardware.PwmOutput(), hardware.ControlTimerId())
        , terminalWithStorage(hardware.Terminal(), hardware.Tracer())
        , terminal(terminalWithStorage, hardware.Tracer(), motorController)
    {}
}
