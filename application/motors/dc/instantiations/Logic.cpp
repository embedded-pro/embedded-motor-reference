#include "application/motors/dc/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : pidDriver{ hardware }
        , pid{ pidDriver, pidDriver.SampleTime(), controllers::PidTunings<float>{ 0.0f, 0.0f, 0.0f }, controllers::PidLimits<float>{ -1.0f, 1.0f } }
        , motorController{ pid }
        , terminalWithStorage{ hardware.Terminal(), hardware.Tracer() }
        , terminal{ terminalWithStorage, motorController }
    {}
}
