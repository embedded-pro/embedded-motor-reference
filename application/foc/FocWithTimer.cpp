#include "application/foc/FocWithTimer.hpp"
#include "numerical/controllers/TransformsClarkePark.hpp"

namespace application
{
    FocWithTimer::FocWithTimer(FocInput& input, FocOutput& output, Components& components)
        : input(input)
        , output(output)
        , spaceVectorModulator(components.spaceVectorModulator)
        , dPid(components.dPid)
        , qPid(components.qPid)
        , timer(components.timerId)
        , sampleTime(components.samplingTime)
        , park(components.trigonometricFunctions)
    {
        really_assert(components.timerId != infra::systemTimerServiceId);
    }

    void FocWithTimer::SetTunnings(IdAndIqTunnings tunnings)
    {
        dPid.SetTunnings(tunnings.first);
        qPid.SetTunnings(tunnings.second);
    }

    void FocWithTimer::SetPoint(const IdAndIqPoint& point)
    {
        dPid.SetPoint(point.first);
        qPid.SetPoint(point.second);
    }

    void FocWithTimer::Enable()
    {
        dPid.Enable();
        qPid.Enable();

        timer.Start(sampleTime, [this]()
            {
                auto [threePhaseCurrent, angle] = input.Read();

                auto idAndIq = park.Forward(clarke.Forward(threePhaseCurrent), angle);
                auto twoPhaseVoltage = controllers::RotatingFrame<float>{ dPid.Process(idAndIq.d), qPid.Process(idAndIq.q) };
                auto voltageAlphaBeta = park.Inverse(twoPhaseVoltage, angle);
                auto pwmOutput = spaceVectorModulator.Generate(voltageAlphaBeta);

                output.Update(pwmOutput);
            });
    }

    void FocWithTimer::Disable()
    {
        timer.Cancel();
        output.Disable();
        dPid.Disable();
        qPid.Disable();
    }

    bool FocWithTimer::IsRunning() const
    {
        return timer.Armed();
    }
}
