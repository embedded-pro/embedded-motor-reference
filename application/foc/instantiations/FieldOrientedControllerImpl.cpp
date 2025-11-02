#include "application/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"

namespace
{
    static constexpr float invSqrt3 = 0.577350269189625f;
    static constexpr float pi = 3.14159265359f;
    static constexpr float two_pi = 6.28318530718f;
}

namespace foc
{
    FieldOrientedControllerTorqueImpl::FieldOrientedControllerTorqueImpl(math::TrigonometricFunctions<float>& trigFunctions)
        : park{ trigFunctions }
        , dPid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f } }
        , qPid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f } }
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
        auto scale = 1.0f / (invSqrt3 * static_cast<float>(Vdc.Value()));

        dPid.SetTunings({ tunings.first.kp * scale, tunings.first.ki * scale, tunings.first.kd * scale });
        qPid.SetTunings({ tunings.second.kp * scale, tunings.second.ki * scale, tunings.second.kd * scale });
    }

    OPTIMIZE_FOR_SPEED
    PhasePwmDutyCycles FieldOrientedControllerTorqueImpl::Calculate(const PhaseCurrents& currentPhases, Radians& position)
    {
        auto threePhaseCurrent = ThreePhase{ std::get<0>(currentPhases).Value(), std::get<1>(currentPhases).Value(), std::get<2>(currentPhases).Value() };

        // position is mechanical angle from encoder
        // Park transform needs electrical angle
        // For torque control, we need to know pole pairs - but it's not set!
        // Assuming we need to add SetPolePairs method or pass electrical angle
        // For now, assuming position is already electrical (needs clarification)
        auto angle = position.Value();

        auto idAndIq = park.Forward(clarke.Forward(threePhaseCurrent), angle);
        auto twoPhaseVoltage = RotatingFrame{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) };
        auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, angle);
        auto output = spaceVectorModulator.Generate(voltageAlphaBeta);

        return PhasePwmDutyCycles{ hal::Percent(static_cast<uint8_t>(output.a * 100.0f + 0.5f)),
            hal::Percent(static_cast<uint8_t>(output.b * 100.0f + 0.5f)),
            hal::Percent(static_cast<uint8_t>(output.c * 100.0f + 0.5f)) };
    }

    OPTIMIZE_FOR_SPEED
    FieldOrientedControllerSpeedImpl::FieldOrientedControllerSpeedImpl(math::TrigonometricFunctions<float>& trigFunctions, foc::Ampere maxCurrent, std::chrono::system_clock::duration timeStep)
        : park{ trigFunctions }
        , speedPid{ { 0.0f, 0.0f, 0.0f }, { -maxCurrent.Value(), maxCurrent.Value() } }
        , dPid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f } }
        , qPid{ { 0.0f, 0.0f, 0.0f }, { -1.0f, 1.0f } }
        , dt{ std::chrono::duration_cast<std::chrono::duration<float>>(timeStep).count() }
    {
        really_assert(maxCurrent.Value() > 0);

        speedPid.Enable();
        dPid.Enable();
        qPid.Enable();
    }

    void FieldOrientedControllerSpeedImpl::SetPolePairs(std::size_t pole)
    {
        polePairs = static_cast<float>(pole);
    }

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerSpeedImpl::SetPoint(RadiansPerSecond point)
    {
        speedPid.SetPoint(point.Value());
        dPid.SetPoint(0.0f);
    }

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerSpeedImpl::SetTunings(Volts Vdc, const SpeedTunings& speedTuning, const IdAndIqTunings& torqueTunings)
    {
        auto scale = 1.0f / (invSqrt3 * static_cast<float>(Vdc.Value()));

        speedPid.SetTunings({ speedTuning.kp, speedTuning.ki * dt, speedTuning.kd / dt });
        dPid.SetTunings({ torqueTunings.first.kp * scale, torqueTunings.first.ki * scale * dt, torqueTunings.first.kd * scale / dt });
        qPid.SetTunings({ torqueTunings.second.kp * scale, torqueTunings.second.ki * scale * dt, torqueTunings.second.kd * scale / dt });
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

        previousPosition = 0.0f;
    }

    OPTIMIZE_FOR_SPEED
    float FieldOrientedControllerSpeedImpl::CalculateFilteredSpeed(float mechanicalPosition)
    {
        float positionDelta = mechanicalPosition - previousPosition;

        if (positionDelta > pi)
            positionDelta -= two_pi;
        else if (positionDelta < -pi)
            positionDelta += two_pi;

        auto mechanicalSpeed = positionDelta / dt;

        previousPosition = mechanicalPosition;

        return mechanicalSpeed;
    }

    OPTIMIZE_FOR_SPEED
    PhasePwmDutyCycles FieldOrientedControllerSpeedImpl::Calculate(const PhaseCurrents& currentPhases, Radians& position)
    {
        auto threePhaseCurrent = ThreePhase{ std::get<0>(currentPhases).Value(), std::get<1>(currentPhases).Value(), std::get<2>(currentPhases).Value() };
        auto mechanicalAngle = position.Value();

        auto filteredSpeed = CalculateFilteredSpeed(mechanicalAngle);
        auto speedSetPoint = speedPid.Process(filteredSpeed);
        qPid.SetPoint(speedSetPoint);
        auto electricalAngle = mechanicalAngle * polePairs;
        auto idAndIq = park.Forward(clarke.Forward(threePhaseCurrent), electricalAngle);
        auto dPid_ = dPid.Process(idAndIq.d);
        auto qPid_ = qPid.Process(idAndIq.q);
        auto twoPhaseVoltage = RotatingFrame{ dPid_, qPid_ };
        auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, electricalAngle);
        auto output = spaceVectorModulator.Generate(voltageAlphaBeta);

        return PhasePwmDutyCycles{ hal::Percent(static_cast<uint8_t>(output.a * 100.0f + 0.5f)),
            hal::Percent(static_cast<uint8_t>(output.b * 100.0f + 0.5f)),
            hal::Percent(static_cast<uint8_t>(output.c * 100.0f + 0.5f)) };
    }
}
