#ifndef APPLICATION_FOC_WITH_TIMER_HPP
#define APPLICATION_FOC_WITH_TIMER_HPP

#include "application/foc/FieldOrientedControllerInterface.hpp"
#include "application/pid/Pid.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"
#include "numerical/math/TrigonometricFunctions.hpp"

namespace application
{
    class SpaceVectorModulator
    {
    public:
        using TwoVoltagePhase = controllers::TwoPhase<float>;

        virtual std::tuple<Percent, Percent, Percent> Generate(TwoVoltagePhase& voltagePhase) = 0;
    };

    class MotorFieldOrientedController
    {
    public:
        struct Components
        {
            math::TrigonometricFunctions<float>& trigonometricFunctions;
            SpaceVectorModulator& spaceVectorModulator;
            Pid& dPid;
            Pid& qPid;
        };

        using IdAndIqTunnings = std::pair<controllers::Pid<float>::Tunnings, controllers::Pid<float>::Tunnings>;
        using IdAndIqPoint = std::pair<float, float>;

        MotorFieldOrientedController(FieldOrientedControllerInterface& interface, Components& components);

        void SetTunnings(IdAndIqTunnings tunnings);
        void SetPoint(const IdAndIqPoint& point);
        virtual void Enable();
        void Disable();
        bool IsRunning() const;

    protected:
        FieldOrientedControllerInterface& interface;
        SpaceVectorModulator& spaceVectorModulator;
        Pid& dPid;
        Pid& qPid;
        controllers::Clarke<float> clarke;
        controllers::Park<float> park;
        bool enabled = false;
    };
}

#endif
