#ifndef APPLICATION_FOC_INSTANTIATIONS_HPP
#define APPLICATION_FOC_INSTANTIATIONS_HPP

#include "application/foc/MotorFieldOrientedController.hpp"
#include "numerical/controllers/SpaceVectorModulation.hpp"

namespace application
{
    class SpaceVectorModulatorImpl
        : public SpaceVectorModulator
    {
    public:
        SpaceVectorModulatorImpl() = default;

        std::tuple<Percent, Percent, Percent> Generate(TwoVoltagePhase& voltagePhase) override;

    private:
        controllers::SpaceVectorModulation<float> spaceVectorModulation;
    };
}

#endif
