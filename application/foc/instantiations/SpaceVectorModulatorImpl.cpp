#include "application/foc/instantiations/SpaceVectorModulatorImpl.hpp"

namespace application
{
    FocOutput::Output SpaceVectorModulatorImpl::Generate(TwoVoltagePhase& voltagePhase)
    {
        return spaceVectorModulation.Generate(voltagePhase);
    }
}
