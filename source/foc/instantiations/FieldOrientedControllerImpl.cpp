#include "source/foc/instantiations/FieldOrientedControllerImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <numbers>

namespace
{
    constexpr float invSqrt3 = 0.577350269189625f;
    constexpr float pi = std::numbers::pi_v<float>;
    constexpr float two_pi = 2.0f * pi;

    OPTIMIZE_FOR_SPEED
    float PositionWithWrapAround(float position)
    {
        if (position > pi)
            position -= two_pi;
        else if (position < -pi)
            position += two_pi;

        return position;
    }
}

namespace foc
{
    FieldOrientedControllerTorqueImpl::FieldOrientedControllerTorqueImpl(math::TrigonometricFunctions<float>& trigFunctions)
        : trigFunctions{ trigFunctions }
    {}

    OPTIMIZE_FOR_SPEED
    void FieldOrientedControllerTorqueImpl::Reset()
    {
        dPid.Disable();
        qPid.Disable();

        dPid.Enable();
        qPid.Enable();
    }

    void FieldOrientedControllerTorqueImpl::SetPolePairs(std::size_t polePairs)
    {
        this->polePairs = static_cast<float>(polePairs);
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
        auto scale = 1.0f / (invSqrt3 * Vdc.Value());

        dPid.SetTunings({ tunings.first.kp * scale, tunings.first.ki * scale, tunings.first.kd * scale });
        qPid.SetTunings({ tunings.second.kp * scale, tunings.second.ki * scale, tunings.second.kd * scale });
    }

    OPTIMIZE_FOR_SPEED
    PhasePwmDutyCycles FieldOrientedControllerTorqueImpl::Calculate(const PhaseCurrents& currentPhases, Radians& position)
    {
        auto mechanicalAngle = position.Value();
        auto electricalAngle = mechanicalAngle * polePairs;

        auto cosTheta = trigFunctions.Cosine(electricalAngle);
        auto sinTheta = trigFunctions.Sine(electricalAngle);

        auto idAndIq = park.Forward(clarke.Forward(ThreePhase{ currentPhases.a.Value(), currentPhases.b.Value(), currentPhases.c.Value() }), cosTheta, sinTheta);
        auto output = spaceVectorModulator.Generate(park.Inverse(RotatingFrame{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) }, cosTheta, sinTheta));

        return PhasePwmDutyCycles{ hal::Percent(static_cast<uint8_t>(output.a * 100.0f + 0.5f)),
            hal::Percent(static_cast<uint8_t>(output.b * 100.0f + 0.5f)),
            hal::Percent(static_cast<uint8_t>(output.c * 100.0f + 0.5f)) };
    }

    OPTIMIZE_FOR_SPEED
    FieldOrientedControllerSpeedImpl::FieldOrientedControllerSpeedImpl(math::TrigonometricFunctions<float>& trigFunctions, foc::Ampere maxCurrent, std::chrono::system_clock::duration timeStep)
        : trigFunctions{ trigFunctions }
        , speedPid{ { 0.0f, 0.0f, 0.0f }, { -maxCurrent.Value(), maxCurrent.Value() } }
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
        auto scale = 1.0f / (invSqrt3 * Vdc.Value());

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
        auto mechanicalSpeed = PositionWithWrapAround(mechanicalPosition - previousPosition) / dt;

        previousPosition = mechanicalPosition;

        return mechanicalSpeed;
    }

    OPTIMIZE_FOR_SPEED
    PhasePwmDutyCycles FieldOrientedControllerSpeedImpl::Calculate(const PhaseCurrents& currentPhases, Radians& position)
    {
        auto mechanicalAngle = position.Value();
        auto electricalAngle = mechanicalAngle * polePairs;

        auto cosTheta = trigFunctions.Cosine(electricalAngle);
        auto sinTheta = trigFunctions.Sine(electricalAngle);

        qPid.SetPoint(speedPid.Process(CalculateFilteredSpeed(mechanicalAngle)));

        auto idAndIq = park.Forward(clarke.Forward(ThreePhase{ currentPhases.a.Value(), currentPhases.b.Value(), currentPhases.c.Value() }), cosTheta, sinTheta);
        auto output = spaceVectorModulator.Generate(park.Inverse(RotatingFrame{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) }, cosTheta, sinTheta));

        return PhasePwmDutyCycles{ hal::Percent(static_cast<uint8_t>(output.a * 100.0f + 0.5f)),
            hal::Percent(static_cast<uint8_t>(output.b * 100.0f + 0.5f)),
            hal::Percent(static_cast<uint8_t>(output.c * 100.0f + 0.5f)) };
    }
}
