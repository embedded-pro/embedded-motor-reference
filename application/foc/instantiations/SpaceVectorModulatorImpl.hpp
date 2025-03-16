#ifndef APPLICATION_FOC_INSTANTIATIONS_HPP
#define APPLICATION_FOC_INSTANTIATIONS_HPP

#include "application/foc/FocWithTimer.hpp"
#include "numerical/controllers/SpaceVectorModulation.hpp"

namespace application
{
    class SpaceVectorModulatorImpl
        : public SpaceVectorModulator
    {
    public:
        SpaceVectorModulatorImpl() = default;

        FocOutput::Output Generate(TwoVoltagePhase& voltagePhase) override;

    private:
        controllers::SpaceVectorModulation<float> spaceVectorModulation;
    };
}

#endif
