#include "application/foc/MotorFieldOrientedController.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"

namespace application
{
    MotorFieldOrientedController::MotorFieldOrientedController(FieldOrientedControllerInterface& interface, Components& components)
        : interface(interface)
        , spaceVectorModulator(components.spaceVectorModulator)
        , dPid(components.dPid)
        , qPid(components.qPid)
        , park(components.trigonometricFunctions)
    {
        interface.PhaseCurrentsReady([this](std::tuple<MilliVolt, MilliVolt, MilliVolt> voltagePhases, std::optional<Degrees> position)
            {
                auto threePhaseCurrent = controllers::ThreePhase<float>{ std::get<0>(voltagePhases).Value(), std::get<1>(voltagePhases).Value(), std::get<2>(voltagePhases).Value() };
                auto angle = position.value_or(Degrees(0.0f));
                auto idAndIq = park.Forward(clarke.Forward(threePhaseCurrent), angle.Value());
                auto twoPhaseVoltage = controllers::RotatingFrame<float>{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) };
                auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, angle.Value());

                this->interface.ThreePhasePwmOutput(spaceVectorModulator.Generate(voltageAlphaBeta));
            });
    }

    void MotorFieldOrientedController::SetTunnings(IdAndIqTunnings tunnings)
    {
        dPid.SetTunnings(tunnings.first);
        qPid.SetTunnings(tunnings.second);
    }

    void MotorFieldOrientedController::SetPoint(const IdAndIqPoint& point)
    {
        dPid.SetPoint(point.first);
        qPid.SetPoint(point.second);
    }

    void MotorFieldOrientedController::Enable()
    {
        enabled = true;
        dPid.Enable();
        qPid.Enable();
        interface.Start();
    }

    void MotorFieldOrientedController::Disable()
    {
        enabled = false;
        interface.Stop();
        dPid.Disable();
        qPid.Disable();
    }

    bool MotorFieldOrientedController::IsRunning() const
    {
        return enabled;
    }
}
