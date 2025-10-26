#pragma once

#include "application/foc/implementations/SpaceVectorModulation.hpp"
#include "application/foc/implementations/TransformsClarkePark.hpp"
#include "application/foc/interfaces/Driver.hpp"
#include "application/foc/interfaces/FieldOrientedController.hpp"
#include "numerical/controllers/implementations/PidIncremental.hpp"
#include "numerical/math/TrigonometricFunctions.hpp"

namespace foc
{
    class FieldOrientedControllerTorqueImpl
        : public FieldOrientedControllerTorqueControl
    {
    public:
        explicit FieldOrientedControllerTorqueImpl(math::TrigonometricFunctions<float>& trigFunctions);

        void SetPoint(IdAndIqPoint setPoint) override;
        void SetTunings(Volts Vdc, const IdAndIqTunings& tunings) override;
        void Reset() override;
        PhasePwmDutyCycles Calculate(const PhaseCurrents& currentPhases, Radians& position) override;

    private:
        Park park;
        Clarke clarke;
        controllers::PidIncrementalSynchronous<float> dPid;
        controllers::PidIncrementalSynchronous<float> qPid;
        SpaceVectorModulation spaceVectorModulator;
    };

    class FieldOrientedControllerSpeedImpl
        : public FieldOrientedControllerSpeedControl
    {
    public:
        explicit FieldOrientedControllerSpeedImpl(math::TrigonometricFunctions<float>& trigFunctions);

        void SetPoint(RadiansPerSecond point) override;
        void SetTunings(Volts Vdc, const SpeedTunings& speedTuning, const IdAndIqTunings& torqueTunings) override;
        void Reset() override;
        PhasePwmDutyCycles Calculate(const PhaseCurrents& currentPhases, Radians& position) override;

    private:
        Park park;
        Clarke clarke;
        controllers::PidIncrementalSynchronous<float> speedPid;
        controllers::PidIncrementalSynchronous<float> dPid;
        controllers::PidIncrementalSynchronous<float> qPid;
        SpaceVectorModulation spaceVectorModulator;
    };
}
