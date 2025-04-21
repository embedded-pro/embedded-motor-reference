#include "application/motors/BLDC/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : components{ trigonometricFunctions, spaceVectorModulation, dPid, qPid }
        , focInteractor{ hardware, components }
        , terminalWithStorage(hardware.Terminal(), hardware.Tracer())
        , terminal(terminalWithStorage, focInteractor)
    {}
}
