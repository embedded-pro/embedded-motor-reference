#pragma once

#include "application/foc/interfaces/Driver.hpp"
#include "application/foc/interfaces/FieldOrientedController.hpp"
#include "numerical/controllers/SpaceVectorModulation.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"
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
        static constexpr float invSqrt3 = 0.577350269189626f;
        controllers::Park<float> park;
        controllers::Clarke<float> clarke;
        controllers::Pid<float> dPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } };
        controllers::Pid<float> qPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } };
        controllers::SpaceVectorModulation<float> spaceVectorModulator;
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
        static constexpr float invSqrt3 = 0.577350269189626f;
        controllers::Park<float> park;
        controllers::Clarke<float> clarke;
        controllers::Pid<float> speedPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } };
        controllers::Pid<float> dPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } };
        controllers::Pid<float> qPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } };
        controllers::SpaceVectorModulation<float> spaceVectorModulator;
    };
}
