#include "application/motors/BLDC/instantiations/Logic.hpp"

namespace application
{
    Logic::Logic(application::HardwareFactory& hardware)
        : components{ trigonometricFunctions, spaceVectorModulation, dPid, qPid, std::chrono::microseconds(100), hardware.ControlTimerId() }
        , input{ hardware.PhaseA(), hardware.PhaseB(), hardware.QuadratureEncoder() }
        , focController{ input, hardware.PwmThreePhaseOutput(), components }
        , terminalWithStorage(hardware.Terminal(), hardware.Tracer())
        , terminal(terminalWithStorage, focController)
    {}
}
