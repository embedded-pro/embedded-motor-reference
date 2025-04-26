#pragma once

#include "application/foc/FieldOrientedController.hpp"
#include "numerical/controllers/SpaceVectorModulation.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"
#include "numerical/math/TrigonometricFunctions.hpp"

namespace application
{
    class FieldOrientedControllerImpl
        : public FieldOrientedController
    {
    public:
        explicit FieldOrientedControllerImpl(math::TrigonometricFunctions<float>& trigFunctions);

        std::tuple<Percent, Percent, Percent> Calculate(controllers::Pid<float>& dPid, controllers::Pid<float>& qPid, const std::tuple<MilliVolt, MilliVolt, MilliVolt>& voltagePhases, std::optional<Degrees> position) override;

    private:
        controllers::Park<float> park;
        controllers::Clarke<float> clarke;
        controllers::SpaceVectorModulation<float> spaceVectorModulator;
    };
}
