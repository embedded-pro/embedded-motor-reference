#include "application/foc/instantiations/SpaceVectorModulatorImpl.hpp"

namespace application
{
    std::tuple<Percent, Percent, Percent> SpaceVectorModulatorImpl::Generate(TwoVoltagePhase& voltagePhase)
    {
        auto duties = spaceVectorModulation.Generate(voltagePhase);

        return std::make_tuple(Percent(duties.a), Percent(duties.b), Percent(duties.c));
    }
}
