#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "numerical/math/CompilerOptimizations.hpp"

namespace
{
    static constexpr float invSqrt3 = 0.577350269189626f;
}

namespace foc
{
    FieldOrientedControllerTorqueImpl::FieldOrientedControllerTorqueImpl(math::TrigonometricFunctions<float>& trigFunctions)
        : park{ trigFunctions }
        , dPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } }
        , qPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } }
    {}

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerTorqueImpl::Reset()
    {
        dPid.Disable();
        qPid.Disable();

        dPid.Enable();
        qPid.Enable();
    }

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerTorqueImpl::SetPoint(IdAndIqPoint setPoint)
    {
        dPid.SetPoint(setPoint.first.Value());
        qPid.SetPoint(setPoint.second.Value());
    }

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerTorqueImpl::SetTunings(Volts Vdc, const IdAndIqTunings& tunings)
    {
        dPid.SetTunings({ tunings.first.kp / Vdc.Value(), tunings.first.ki / Vdc.Value(), tunings.first.kd / Vdc.Value() });
        qPid.SetTunings({ tunings.second.kp / Vdc.Value(), tunings.second.ki / Vdc.Value(), tunings.second.kd / Vdc.Value() });
    }

    OPTIMIZE_FOR_SPEED
    PhasePwmDutyCycles FieldOrientedControllerTorqueImpl::Calculate(const PhaseCurrents& currentPhases, Radians& position)
    {
        auto threePhaseCurrent = ThreePhase{ std::get<0>(currentPhases).Value(), std::get<1>(currentPhases).Value(), std::get<2>(currentPhases).Value() };
        auto angle = position.Value();
        auto idAndIq = park.Forward(clarke.Forward(threePhaseCurrent), angle);
        auto twoPhaseVoltage = RotatingFrame{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) };
        auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, angle);
        auto output = spaceVectorModulator.Generate(voltageAlphaBeta);

        return PhasePwmDutyCycles{ hal::Percent(static_cast<uint8_t>(output.a * 100.0f)), hal::Percent(static_cast<uint8_t>(output.b * 100.0f)), hal::Percent(static_cast<uint8_t>(output.c * 100.0f)) };
    }

    OPTIMIZE_FOR_SPEED
    FieldOrientedControllerSpeedImpl::FieldOrientedControllerSpeedImpl(math::TrigonometricFunctions<float>& trigFunctions)
        : park{ trigFunctions }
        , speedPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } }
        , dPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } }
        , qPid{ { 0.0f, 0.0f, 0.0f }, { -invSqrt3, invSqrt3 } }
    {}

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerSpeedImpl::SetPoint(RadiansPerSecond point)
    {
        speedPid.SetPoint(point.Value());
    }

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerSpeedImpl::SetTunings(Volts Vdc, const SpeedTunings& speedTuning, const IdAndIqTunings& torqueTunings)
    {
        speedPid.SetTunings({ speedTuning.kp / Vdc.Value(), speedTuning.ki / Vdc.Value(), speedTuning.kd / Vdc.Value() });
        dPid.SetTunings({ torqueTunings.first.kp / Vdc.Value(), torqueTunings.first.ki / Vdc.Value(), torqueTunings.first.kd / Vdc.Value() });
        qPid.SetTunings({ torqueTunings.second.kp / Vdc.Value(), torqueTunings.second.ki / Vdc.Value(), torqueTunings.second.kd / Vdc.Value() });
    }

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerSpeedImpl::Reset()
    {
        speedPid.Disable();
        dPid.Disable();
        qPid.Disable();

        speedPid.Enable();
        dPid.Enable();
        qPid.Enable();
    }

    OPTIMIZE_FOR_SPEED
    PhasePwmDutyCycles FieldOrientedControllerSpeedImpl::Calculate(const PhaseCurrents& currentPhases, Radians& position)
    {
        auto threePhaseCurrent = ThreePhase{ std::get<0>(currentPhases).Value(), std::get<1>(currentPhases).Value(), std::get<2>(currentPhases).Value() };
        auto angle = position.Value();

        qPid.SetPoint(speedPid.Process(0.0f)); // Assuming 0.0f as the current speed for simplicity

        auto idAndIq = park.Forward(clarke.Forward(threePhaseCurrent), angle);
        auto twoPhaseVoltage = RotatingFrame{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) };
        auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, angle);
        auto output = spaceVectorModulator.Generate(voltageAlphaBeta);

        return PhasePwmDutyCycles{ hal::Percent(static_cast<uint8_t>(output.a * 100.0f)), hal::Percent(static_cast<uint8_t>(output.b * 100.0f)), hal::Percent(static_cast<uint8_t>(output.c * 100.0f)) };
    }
}
